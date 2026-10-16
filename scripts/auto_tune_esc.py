#!/usr/bin/env python3
"""FR (NE) stand auto-tune, Phases 0-2: offline guards, current-loop verify,
and speed-PI Kp hunting-ceiling scan via the RPM2 runtime GAIN_SET command.

Hardware layout: Nucleo H755 is the CAN sender (wheel_bench producer with
BENCH_TUNE), FR/NE ESC runs the rpm2-tune image, monitor_esc_power.py logs
the ESC over its own probe (read-only). Two USB cables, exclusive ownership
of both probes; battery ON; wheels in the air. No ESC flashing here.

CAN TX ACK / bench ARMED / RUNNING never prove ESC acceptance: every GAIN_SET
is verified against test_gain_set_result/test_speed_kp_gain in the monitor
stream, and every run is verified against ESC mci_state. STOP is sent on any
failure and before exit; a lost host/probe cannot guarantee a physical stop.

Analysis core (run_stats/verdicts/ki solver/selection) is pure and unit
tested; pyOCD imports lazily in the hardware layer only.
"""
import argparse
import datetime
import json
import math
import os
import pathlib
import re
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
MONITOR = ROOT / "scripts" / "monitor_esc_power.py"
NUCLEO_PROBE = "002200364142500620353451"
NUCLEO_TARGET = "stm32h755zitx"
WHEEL_NE = 1
ARM2, RUN2, STOP, TUNE = 0x41524D32, 0x52554E32, 0x53544F50, 0x47535431
BENCH_STATES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "STOPPING", 4: "FAULT"}
ARM_MAGIC = 0x52504D32  # RPM2
KP_DIV, KI_DIV = 1024, 16384
CURRENT_SCALE = 0.001836396986618638
# Baseline RPM2 numerators (audited free-air-stable) and scan defaults.
KP_BASELINE, KI_BASELINE = -18600, -72
KP_CANDIDATES = (-18600, -22528, -26624, -30720, -32767)  # 18.16/22/26/30/32 cts/unit
KP_CAP, KP_FLOOR = -24576, -18600  # skill window: 18.0 <= kp <= 24.0 cts/unit
SCAN_RPMS = (51, 250)
PHASE1_RPM = 150
SETTLE_S = 0.8
RUN_SECONDS = {51: 5.0, 250: 3.5, 150: 3.5}
HUNT_MAX_ZERO_STREAK_S = 0.3
HUNT_P2P_FACTOR = 3.0
IQ_ERR_PASS_A, ID_PASS_A = 0.05, 0.10

SESSION_OPTIONS = {
    "target_override": NUCLEO_TARGET, "connect_mode": "attach",
    "resume_on_disconnect": False, "auto_unlock": False,
    "no_config": True, "user_script": "/dev/null",
    "pack.debug_sequences.enable": False, "enable_swv": False,
    "cache.enable_memory": False, "cache.enable_register": False,
    "vector_catch": "none", "frequency": 1000000,
}
BENCH_SYMBOLS = ("wheel_bench_request", "wheel_bench_wheel", "wheel_bench_direction",
                 "wheel_bench_state", "wheel_bench_result", "wheel_bench_speed_rpm",
                 "wheel_bench_run_count", "wheel_bench_first_fault",
                 "wheel_bench_tune_kp", "wheel_bench_tune_ki")


# ---------------------------------------------------------------- analysis --
def parse_utc(text):
    return datetime.datetime.fromisoformat(text)


def quantized_target_rpm(rpm):
    """RPM_2_SPEED_UNIT truncates (rpm*10)/60 toward zero; 1 unit = 6 RPM."""
    return 6 * math.trunc(rpm * 10 / 60)


def load_samples(path):
    """Normalize monitor JSONL samples; invalid samples raise (bad evidence)."""
    samples = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            if record.get("type") != "sample":
                continue
            raw = record["raw"]
            samples.append({
                "utc": parse_utc(record["blocks"][-1]["utc_end"]),
                "rpm": raw["measured_rpm"], "state": raw["mci_state"],
                "iqref_counts": raw["iq_reference_counts"],
                "iq_counts": raw["measured_iq_counts"],
                "id_counts": raw["measured_id_counts"],
                "current_valid": bool(raw["current_valid"]),
                "stop_reason": raw["stop_reason"],
                "cal_result": raw["calibration_result"],
                "cal_valid": raw["calibration_valid"],
                "armed": raw["armed"],
                "gain_set_result": raw.get("gain_set_result"),
                "speed_kp_gain": raw.get("speed_kp_gain"),
                "speed_ki_gain": raw.get("speed_ki_gain"),
            })
    return samples


def slice_window(samples, start_utc, end_utc):
    return [s for s in samples if start_utc <= s["utc"] <= end_utc]


def _median(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return None
    mid = n // 2
    return ordered[mid] if n % 2 else (ordered[mid - 1] + ordered[mid]) / 2


def run_stats(window, target_rpm, settle_s=SETTLE_S):
    """Steady-window metrics for one run. Window must bracket the RUN command;
    samples outside mci_state==6 and the first settle_s are excluded. RPM
    samples are sparse (~20 Hz host reads of the ESC's 1 kHz telemetry), so
    streaks use realized sample spacing, not assumed period."""
    if not window:
        raise ValueError("empty run window")
    t0 = window[0]["utc"]
    steady = [s for s in window
              if s["state"] == 6 and (s["utc"] - t0).total_seconds() >= settle_s]
    if len(steady) < 4:
        return {"steady_samples": len(steady), "saw_run": any(s["state"] == 6 for s in window),
                "hunting": True, "hunt_reasons": ["insufficient steady RUN samples"]}
    gaps = [(b["utc"] - a["utc"]).total_seconds() for a, b in zip(steady, steady[1:])]
    rpms = [s["rpm"] for s in steady]
    reversals = sum(1 for a, b in zip(rpms, rpms[1:])
                    if a != 0 and b != 0 and (a < 0) != (b < 0))
    zero_streak, longest = 0.0, 0.0
    for s, gap in zip(steady, gaps + [0.0]):
        zero_streak = zero_streak + gap if s["rpm"] == 0 else 0.0
        longest = max(longest, zero_streak)
    target_q = quantized_target_rpm(target_rpm)
    iq_pairs = [(s["iq_counts"], s["iqref_counts"]) for s in steady if s["current_valid"]]
    ids = [abs(s["id_counts"]) for s in steady if s["current_valid"]]
    iq_err_a = [abs(iq - ref) * CURRENT_SCALE for iq, ref in iq_pairs]
    return {
        "steady_samples": len(steady), "saw_run": True,
        "target_rpm": target_rpm, "target_quantized_rpm": target_q,
        "rpm_median": _median(rpms), "rpm_p2p": max(rpms) - min(rpms),
        "sign_reversals": reversals, "max_zero_streak_s": round(longest, 3),
        "median_abs_track_err_rpm": _median([abs(r - target_q) for r in rpms]),
        "median_abs_iq_err_a": (_median(iq_err_a) if iq_err_a else None),
        "median_abs_id_a": (_median([v * CURRENT_SCALE for v in ids]) if ids else None),
        "iqref_p2p_a": (max(s["iqref_counts"] for s in steady)
                        - min(s["iqref_counts"] for s in steady)) * CURRENT_SCALE,
        "stop_reason_at_end": window[-1]["stop_reason"],
    }


def hunting_verdict(stats, target_rpm):
    """Free-air Hall-FIFO hunting signatures (P2 stand capture, seq 20):
    sign reversals, >0.3 s zero-RPM streaks while commanded, RPM p2p far
    beyond target magnitude, or insufficient RUN evidence."""
    reasons = list(stats.get("hunt_reasons", []))
    if stats.get("steady_samples", 0) >= 4:
        if stats["sign_reversals"] >= 2:
            reasons.append(f"sign reversals {stats['sign_reversals']} >= 2")
        if stats["max_zero_streak_s"] > HUNT_MAX_ZERO_STREAK_S:
            reasons.append(f"zero-RPM streak {stats['max_zero_streak_s']}s > "
                           f"{HUNT_MAX_ZERO_STREAK_S}s")
        if stats["rpm_p2p"] > HUNT_P2P_FACTOR * abs(target_rpm):
            reasons.append(f"RPM p2p {stats['rpm_p2p']} > "
                           f"{HUNT_P2P_FACTOR}x|{target_rpm}|")
    return (bool(reasons), reasons)


def current_loop_verdict(stats):
    reasons = []
    iq_err, id_a = stats.get("median_abs_iq_err_a"), stats.get("median_abs_id_a")
    if iq_err is None or id_a is None:
        return False, ["no valid current samples in steady window"]
    if iq_err > IQ_ERR_PASS_A:
        reasons.append(f"|Iq-Iqref| {iq_err:.3f}A > {IQ_ERR_PASS_A}A")
    if id_a > ID_PASS_A:
        reasons.append(f"|Id| {id_a:.3f}A > {ID_PASS_A}A")
    return (not reasons, reasons)


def kp_counts_per_unit(num, div=KP_DIV):
    return abs(num) / div


def ti_ms(kp_num, ki_num, kp_div=KP_DIV, ki_div=KI_DIV):
    """PI integral time constant in ms: ki is per (unit x ms), so Ti = kp/ki."""
    return (abs(kp_num) / kp_div) / (abs(ki_num) / ki_div)


def ki_numerator_for_breakaway(i_break_a, kp_num, u_units, t_break_ms=1200,
                               margin=1.3, ki_div=KI_DIV):
    """Smallest |Ki numerator| so a stalled wheel's Iqref reaches
    margin*I_break by t_break (500 ms ramp, integrator active from 250 ms).
    Iq(t)[cts] = kp*U + ki*U*(t-250); returns (numerator, ti_ms). Caller must
    still validate Ti in [200, 450] ms and verify on hardware."""
    kp = abs(kp_num) / KP_DIV
    need = 544.0 * margin * i_break_a
    ki = (need - kp * u_units) / (u_units * (t_break_ms - 250))
    ki = max(ki, 1.0 / ki_div)
    num = -math.ceil(ki * ki_div)
    return num, round(ti_ms(kp_num, num), 1)


def select_kp(per_candidate, candidates=KP_CANDIDATES,
              cap=KP_CAP, floor=KP_FLOOR):
    """per_candidate: {kp_num: {"hunting": bool, ...}} over the scan matrix.
    Rule: highest clean candidate with >=25% margin below the lowest hunting
    candidate, within [floor, cap]. Baseline hunting => abort (not a tuning
    problem). Never returns a candidate that hunted."""
    if KP_BASELINE not in per_candidate:
        raise ValueError("baseline must be scanned")
    clean = [c for c in candidates if not per_candidate[c]["hunting"]]
    if per_candidate[KP_BASELINE]["hunting"]:
        return {"chosen": None, "reason": "ABORT: baseline -18600 hunts on stand; "
                "suspect hardware/sensor/log setup, not gains"}
    hunting = sorted((abs(c) for c in candidates if per_candidate[c]["hunting"]))
    ceiling = 0.75 * hunting[0] if hunting else float("inf")
    ceiling = min(ceiling, abs(cap))
    eligible = [c for c in clean if abs(c) <= ceiling]
    if not eligible:
        return {"chosen": floor, "ceiling_counts_per_unit": ceiling / KP_DIV,
                "reason": "no clean candidate within 25% margin; keeping baseline"}
    chosen = max(eligible, key=abs)
    return {"chosen": chosen,
            "chosen_counts_per_unit": kp_counts_per_unit(chosen),
            "ceiling_counts_per_unit": ceiling / KP_DIV,
            "first_hunting": hunting[0] if hunting else None,
            "reason": "highest clean candidate within margin"}


# ------------------------------------------------------------ nucleo layer --
def find_nm():
    for cand in ("/tmp/opencode/cm7-toolchain/root/usr/bin/arm-none-eabi-nm",
                 "arm-none-eabi-nm"):
        if cand.startswith("/") and not os.path.exists(cand):
            continue
        return cand
    return None


def resolve_bench_addrs(elf_path):
    nm = find_nm()
    if nm is None:
        raise RuntimeError("arm-none-eabi-nm not found")
    out = subprocess.run([nm, str(elf_path)], capture_output=True, text=True,
                         timeout=30).stdout
    addrs = {}
    for line in out.splitlines():
        m = re.match(r"([0-9a-fA-F]+)\s+\w\s+(wheel_bench_\w+)$", line.strip())
        if m and m.group(2) in BENCH_SYMBOLS:
            addrs[m.group(2)] = int(m.group(1), 16)
    missing = [name for name in BENCH_SYMBOLS if name not in addrs]
    if missing:
        raise RuntimeError(f"Nucleo ELF lacks {missing}; rebuild/reflash an image "
                           "with the BENCH_TUNE wheel_bench before tuning")
    return addrs


class NucleoBench:
    """wheel_bench producer over a pyOCD attach session (no halt/reset)."""

    def __init__(self, target, addrs):
        self.t, self.a = target, addrs

    def read(self, name):
        return self.t.read32(self.a[name])

    def poke(self, name, value):
        self.t.write32(self.a[name], value & 0xFFFFFFFF)
        if self.read(name) != value & 0xFFFFFFFF:
            raise RuntimeError(f"readback mismatch on {name}")

    def request(self, code):
        self.t.write32(self.a["wheel_bench_request"], code)

    def wait_bench(self, expect, what, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read("wheel_bench_state") == expect:
                return
            time.sleep(0.02)
        raise RuntimeError(f"{what}: bench never reached {BENCH_STATES.get(expect)} "
                           f"(result={self.read('wheel_bench_result')}, "
                           f"fault={self.read('wheel_bench_first_fault')})")

    def select(self, wheel, direction):
        self.poke("wheel_bench_wheel", wheel)
        self.poke("wheel_bench_direction", direction & 0xFFFFFFFF)

    def arm(self):
        self.request(ARM2)
        self.wait_bench(1, "ARM")

    def tune(self, kp_num, ki_num):
        self.poke("wheel_bench_tune_kp", kp_num & 0xFFFFFFFF)
        self.poke("wheel_bench_tune_ki", ki_num & 0xFFFFFFFF)
        self.request(TUNE)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self.read("wheel_bench_result") == 6:
                return
            if self.read("wheel_bench_state") not in (1,):
                break
            time.sleep(0.02)
        raise RuntimeError(f"TUNE not queued (result={self.read('wheel_bench_result')})")

    def run(self, rpm_magnitude, direction):
        self.poke("wheel_bench_speed_rpm", rpm_magnitude)
        self.poke("wheel_bench_direction", direction & 0xFFFFFFFF)
        self.request(RUN2)
        self.wait_bench(2, "RUN")

    def stop(self):
        self.request(STOP)
        self.wait_bench(0, "STOP", timeout_s=5.0)


# -------------------------------------------------------------- esc monitor --
class EscMonitor:
    """monitor_esc_power.py subprocess; tail the JSONL for fresh samples."""

    def __init__(self, manifest, out_path, seconds, period=0.05):
        self.path = out_path
        self.proc = subprocess.Popen(
            [sys.executable, "-u", str(MONITOR), "--profile", "rpm2-tune",
             "--manifest", str(manifest), "--seconds", str(seconds),
             "--period", str(period), "--output", str(out_path)],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.offset = 0

    def _wait_header(self, timeout_s=60.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("ESC monitor exited early: "
                                   + self.proc.stderr.read().decode()[-500:])
            if self.path.exists() and self.path.stat().st_size > 0:
                with open(self.path) as fh:
                    if json.loads(fh.readline()).get("type") == "header":
                        return
            time.sleep(0.2)
        raise RuntimeError("ESC monitor never wrote a header (verify_live gate?)")

    def samples_since_last(self):
        with open(self.path) as fh:
            fh.seek(self.offset)
            lines = fh.readlines()
            self.offset = fh.tell()
        out = []
        for line in lines:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue  # partially written tail line; retried next poll
            if record.get("type") == "sample":
                out.append(record)
        return out

    def sync_to_end(self):
        """Discard buffered samples; next wait_for sees only fresh evidence."""
        self.samples_since_last()

    def latest_raw(self):
        samples = self.samples_since_last()
        return samples[-1]["raw"] if samples else None

    def wait_for(self, predicate, what, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for record in self.samples_since_last():
                if predicate(record["raw"]):
                    return record["raw"]
            if self.proc.poll() is not None:
                raise RuntimeError("ESC monitor exited while waiting for " + what)
            time.sleep(0.05)
        raise RuntimeError(f"timeout waiting for {what}")

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


# ------------------------------------------------------------ orchestration --
def open_nucleo_session():
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    probes = ConnectHelper.get_all_connected_probes(blocking=False,
                                                    unique_id=NUCLEO_PROBE)
    exact = [p for p in probes if p.unique_id == NUCLEO_PROBE]
    if len(exact) != 1:
        raise RuntimeError("expected exactly one exact-serial Nucleo probe")
    session = Session(exact[0], auto_open=False, options=dict(SESSION_OPTIONS))
    session.open()
    return session


def esc_prep(bench, esc, log):
    """Battery-ON state machine: fault-ack ARM, CAL ARM, then verified armed.
    Auto-CAL completion never arms or starts the motor (skill section B)."""
    def state_ok(raw):
        return raw["mci_state"] == 0 and raw["current_faults"] == 0
    log("prep: waiting for ESC IDLE/fault-free")
    try:
        esc.wait_for(state_ok, "ESC IDLE", timeout_s=3.0)
    except RuntimeError:
        log("prep: ARM #1 (fault acknowledge)")
        bench.arm()
        esc.wait_for(state_ok, "ESC IDLE after fault ack", timeout_s=5.0)
    def cal_ok(raw):
        return raw["calibration_result"] == 2 and raw["calibration_valid"] == 1
    try:
        esc.wait_for(cal_ok, "CAL 2/1", timeout_s=1.0)
    except RuntimeError:
        log("prep: ARM #2 (start auto-CAL)")
        bench.arm()
        esc.wait_for(lambda r: r["mci_state"] == 0 and cal_ok(r),
                     "auto-CAL completion", timeout_s=5.0)
    log("prep: ESC calibrated; per-run ARM verifies the armed magic")


def one_run(bench, esc, log, kp_num, ki_num, rpm, direction, run_s):
    """ARM -> GAIN_SET -> verify ESC acceptance -> RUN -> hold -> STOP.
    Every wait is synced so only evidence produced AFTER the command counts
    (GAIN_SET acceptance additionally requires the ESC's set-count to advance).
    Returns the UTC window bracketing the run for log slicing."""
    bench.select(WHEEL_NE, direction)
    esc.sync_to_end()
    bench.arm()
    esc.wait_for(lambda r: r["armed"] == ARM_MAGIC, "ESC armed magic", 3.0)
    prev = esc.latest_raw() or {}
    expect_count = (prev.get("gain_set_count") or 0) + 1
    esc.sync_to_end()
    bench.tune(kp_num, ki_num)
    esc.wait_for(lambda r: r.get("gain_set_count") == expect_count
                 and r.get("gain_set_result") == 1
                 and r.get("speed_kp_gain") == kp_num
                 and r.get("speed_ki_gain") == ki_num,
                 f"GAIN_SET acceptance kp={kp_num} ki={ki_num}", 3.0)
    esc.sync_to_end()
    start = datetime.datetime.now(datetime.timezone.utc)
    bench.run(abs(rpm), direction)
    esc.wait_for(lambda r: r["state"] == 6, "ESC RUN", 3.0)
    time.sleep(run_s)
    esc.sync_to_end()
    bench.stop()
    esc.wait_for(lambda r: r["state"] == 0, "ESC IDLE after STOP", 5.0)
    end = datetime.datetime.now(datetime.timezone.utc)
    time.sleep(0.3)  # bench re-ARM quiet
    log("run done", kp=kp_num, rpm=rpm * direction,
        window=f"{start.isoformat()}..{end.isoformat()}")
    return start, end


def analyze_run(samples, start, end, kp_num, ki_num, rpm_cmd):
    pad = datetime.timedelta(seconds=0.3)
    window = slice_window(samples, start - pad, end + pad)
    stats = run_stats(window, rpm_cmd)
    hunting, hunt_reasons = hunting_verdict(stats, rpm_cmd)
    current_ok, current_reasons = current_loop_verdict(stats)
    stats.update({"kp_num": kp_num, "ki_num": ki_num,
                  "kp_counts_per_unit": kp_counts_per_unit(kp_num),
                  "hunting": hunting, "hunt_reasons": hunt_reasons,
                  "current_loop_ok": current_ok, "current_reasons": current_reasons})
    return stats


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nucleo-elf", type=pathlib.Path,
                        default=ROOT / "CM7/build/Debug/sfurs_firmware_v2_CM7.elf")
    parser.add_argument("--esc-manifest", type=pathlib.Path,
                        help="rpm2-tune manifest (required unless --dry-run)")
    parser.add_argument("--kp-candidates", type=str,
                        default=",".join(str(c) for c in KP_CANDIDATES))
    parser.add_argument("--out-dir", type=pathlib.Path)
    parser.add_argument("--dry-run", action="store_true",
                        help="offline suites + plan only; no probe access")
    args = parser.parse_args(argv)
    candidates = tuple(int(c) for c in args.kp_candidates.split(","))
    for cand in candidates:
        if not -32767 <= cand <= -1000:
            parser.error(f"candidate {cand} outside GAIN_SET kp bounds")
    if not args.dry_run and args.esc_manifest is None:
        parser.error("--esc-manifest is required for hardware runs")

    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out_dir or pathlib.Path("/tmp/opencode") / f"auto-tune-{stamp}"
    out_dir.mkdir(parents=True, exist_ok=False)
    timeline_path = out_dir / "timeline.jsonl"
    report = {"started_utc": stamp, "candidates": list(candidates),
              "scan_rpms": list(SCAN_RPMS), "phase1_rpm": PHASE1_RPM,
              "out_dir": str(out_dir), "runs": []}

    def log(event, **fields):
        record = {"utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  "event": event, **fields}
        with open(timeline_path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(record) + "\n")
        print(f"[{record['utc']}] {event} " +
              " ".join(f"{k}={v}" for k, v in fields.items()), flush=True)

    log("phase0: offline guard suites")
    for suite in ("tests/test_fr_speed_tune.py",
                  "HW/B-G431B-ESC1/motor_config/test_reverse_speed_guards.py",
                  "HW/B-G431B-ESC1/motor_config/test_tune_gain_set_guards.py"):
        subprocess.run([sys.executable, str(ROOT / suite)], check=True,
                       capture_output=True)
    log("phase0: PASS")
    if args.dry_run:
        log("dry-run: planned runs",
            plan=[{"kp": c, "rpm": [r * d for r in SCAN_RPMS for d in (1, -1)]}
                  for c in candidates])
        print(f"dry-run OK; artifacts in {out_dir}")
        return 0

    session, esc, bench = None, None, None
    try:
        session = open_nucleo_session()
        bench = NucleoBench(session.target, resolve_bench_addrs(args.nucleo_elf))
        total_s = 180 + (2 + len(candidates) * len(SCAN_RPMS) * 2) * 15
        esc = EscMonitor(args.esc_manifest, out_dir / "esc-monitor.jsonl", total_s)
        esc._wait_header()
        log("monitor: image verified, sampling")
        esc_prep(bench, esc, log)

        log("phase1: current-loop verify", rpm=PHASE1_RPM)
        phase1 = []
        for direction in (1, -1):
            start, end = one_run(bench, esc, log, KP_BASELINE, KI_BASELINE,
                                 PHASE1_RPM, direction, RUN_SECONDS[PHASE1_RPM])
            phase1.append((start, end, direction))
        esc_samples = load_samples(esc.path)
        for start, end, direction in phase1:
            stats = analyze_run(esc_samples, start, end, KP_BASELINE, KI_BASELINE,
                                PHASE1_RPM * direction)
            stats["phase"] = 1
            report["runs"].append(stats)
            if not stats["current_loop_ok"] or stats["hunting"]:
                raise RuntimeError(f"Phase 1 failed dir={direction}: "
                                   f"{stats['current_reasons']} {stats['hunt_reasons']}")
        log("phase1: PASS")

        log("phase2: Kp hunting-ceiling scan")
        per_candidate = {}
        for kp_num in candidates:
            runs = []
            for rpm in SCAN_RPMS:
                for direction in (1, -1):
                    start, end = one_run(bench, esc, log, kp_num, KI_BASELINE,
                                         rpm, direction, RUN_SECONDS[rpm])
                    runs.append((start, end, rpm * direction))
            esc_samples = load_samples(esc.path)  # re-read full log per candidate
            hunting_any, reasons = False, []
            for start, end, rpm_cmd in runs:
                stats = analyze_run(esc_samples, start, end, kp_num, KI_BASELINE, rpm_cmd)
                stats["phase"] = 2
                report["runs"].append(stats)
                hunting_any |= stats["hunting"]
                reasons += stats["hunt_reasons"]
            per_candidate[kp_num] = {"hunting": hunting_any, "reasons": reasons}
            log("candidate done", kp=kp_num, hunting=hunting_any)
        # Leave the ESC at audited baseline numerators (best effort; the
        # values are RAM-only and revert at the next reset regardless).
        try:
            bench.select(WHEEL_NE, 1)
            bench.arm()
            bench.tune(KP_BASELINE, KI_BASELINE)
            log("restore: baseline numerators re-applied")
        except Exception as exc:
            log("restore: FAILED (RAM-only; reset reverts)", error=str(exc))
        report["selection"] = select_kp(per_candidate, candidates)
        report["per_candidate"] = {str(k): v for k, v in per_candidate.items()}
        log("selection", **report["selection"])
    except KeyboardInterrupt:
        report["aborted"] = "interrupted"
        raise
    except Exception as exc:
        report["error"] = str(exc)
        log("ERROR", error=str(exc))
        raise
    finally:
        if bench is not None:
            try:
                bench.stop()
            except Exception:
                pass
        if session is not None:
            session.close()
        if esc is not None:
            esc.close()
        report["ended_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        with open(out_dir / "report.json", "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2)
        print(f"report: {out_dir / 'report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
