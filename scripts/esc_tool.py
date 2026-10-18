#!/usr/bin/env python3
"""ESC setup through the Nucleo streaming IK image (drive must be stopped).

  esc_tool.py status                    print all four wheels once
  esc_tool.py align FR|BR|BL|FL|all     measure + apply Hall phase shift (RAM)
  esc_tool.py set WHEEL KEY VALUE       KEY: kp, ki (speed PI), current_kp,
                                        current_ki (Id+Iq PI), shift (deg),
                                        iqmax_ma, ff_static_ma, ff_kinetic_ma
                                        (friction feed-forward) (RAM)
  esc_tool.py capture WHEEL [--step-ma 3000] [--out f.json]
                                        current-loop step test: R, L, step
                                        response, PI gains for 0.5/0.8/1 kHz
  esc_tool.py spin WHEEL --rpm 60 [--seconds 3] [--kp N --ki N] [--log f.jsonl]
                                        one wheel at a speed setpoint, telemetry
                                        logged; speed-loop check (motion!)
  esc_tool.py pulse [--axis vy] [--speed 0.2] [--seconds 1.5] [--reps 3]
                    [--kp N --ki N] [--ff-static MA --ff-kinetic MA] [--log f.jsonl]
                                        scripted ground drive out and back,
                                        per-wheel breakaway time (motion!)
  esc_tool.py stop                      broadcast STOP; also clears a fault whose
                                        cause is gone (FAULT_OVER -> IDLE)

align turns the wheel about 1/4 revolution forward and back at 2 A: battery
ON, wheels off the ground. Results are lost on ESC reset; bake them into the
per-wheel build with build_cli.py --hall-shift. capture holds the rotor with
d-axis current (1 A, then the step): it may click into position but makes no
torque; battery ON. spin streams SPEED_WRITE to one wheel only; --kp/--ki
are set in RAM first and the firmware defaults (drive_parameters.h) restored
afterwards; STOP on exit, and the ESC stops itself 0.25 s after the stream.
pulse drives the whole robot through the Nucleo (same path as keyboard_drive):
each rep goes out along --axis for --seconds and back, so it ends near where
it started; gains and friction feed-forward are set in RAM on all four wheels
and the drive_parameters.h defaults restored afterwards. Kill it (Ctrl-C) or switch the battery off to
stop: the Nucleo brakes 0.3 s after the heartbeat stops.
"""
import argparse
import json
import pathlib
import re
import statistics
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import capture_analysis  # noqa: E402
import ik_link  # noqa: E402

NAMES = {name: (i, sel) for i, (name, _esc, sel) in enumerate(ik_link.WHEELS)}
NAMES.update({esc: (i, sel) for i, (_name, esc, sel) in enumerate(ik_link.WHEELS)})


def wheels(arg):
    if arg.lower() == "all":
        return [(name, i, sel) for i, (name, _e, sel) in enumerate(ik_link.WHEELS)]
    key = arg.upper()
    if key not in NAMES:
        raise SystemExit(f"unknown wheel {arg}; use FR BR BL FL (or NE SE SW NW) or all")
    return [(key, *NAMES[key])]


def fresh_row(link, index, field, after, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        row = link.rows()[index]
        if row and row[field] != after:
            return row
        time.sleep(0.02)
    raise RuntimeError("no telemetry reply from this ESC (powered? CAN connected?)")


def show(before, rows):
    for (name, esc, _), r0, r in zip(ik_link.WHEELS, before, rows):
        if r is None or not r["status_count"] or (r0 and r0["status_count"] == r["status_count"]):
            print(f"{name}/{esc}: no reply")  # rows keep the last values; only a new count is live
            continue
        app = f"{r['app_crc']:08X}/{r['app_size']}" if r["ident_count"] else "?"
        print(f"{name}/{esc}: {r['mc_state']:<10s} rpm {r['rpm']:5d}  Iq {r['iq_a']:5.2f} A  "
              f"Vbus {r['vbus_v']:5.2f} V  faults {','.join(r['fault_names']) or '-'}  "
              f"hall shift {r['hall_shift_deg']} deg ({'known' if r['aligned'] else 'NOT ALIGNED'})  "
              f"align {r['align']}  stop {r['stop_reason']}  app {app}")


def align(link, name, index, selector):
    before = link.rows()[index]
    count = before["diag_count"] if before else 0
    link.send_raw(ik_link.HALL_ALIGN | selector, ik_link.HALL_ALIGN_MAGIC)
    print(f"{name}: aligning...", flush=True)
    time.sleep(0.3)
    deadline = time.monotonic() + 8.0
    row = fresh_row(link, index, "diag_count", count)
    while row["align_result"] == 5 or row["aligning"]:
        if time.monotonic() > deadline:
            raise RuntimeError(f"{name}: alignment did not finish")
        time.sleep(0.1)
        row = link.rows()[index]
    print(f"{name}: result {row['align']}, hall shift {row['hall_shift_deg']} deg "
          f"(was {before['hall_shift_deg'] if before else '?'})")
    return row


def capture(link, name, selector, args):
    info, samples = link.capture(selector, args.step_ma)
    a = capture_analysis.analyze(info, samples)
    if args.out:
        args.out.write_text(json.dumps(dict(wheel=name, step_ma=args.step_ma, info=info,
                                            analysis=a, samples=samples)))
    rise = f"{a['rise_us']:.0f} us" if a["rise_us"] is not None else "-"
    settle = f"{a['settle_us']:.0f} us" if a["settle_us"] is not None else "not settled"
    print(f"{name}: Vbus {a['vbus_v']:.2f} V, Id {a['hold_a']:.2f} -> {a['step_a']:.2f} A")
    print(f"  R {a['r_ohm'] * 1e3:.0f} mOhm  L {a['l_h'] * 1e6:.0f} uH  tau {a['tau_ms']:.2f} ms  "
          f"PWM delay {a['delay_periods']} periods")
    print(f"  step: rise 10-90% {rise}, overshoot {a['overshoot_pct']:.1f}%, "
          f"settle {settle} (band {a['settle_band_a'] * 1e3:.0f} mA), "
          f"steady error {a['steady_error_a'] * 1e3:.0f} mA, noise {a['noise_a'] * 1e3:.0f} mA rms, "
          f"{a['disturbance_samples']} disturbed samples later")
    for fc in (500, 800, 1000):
        kp, ki = capture_analysis.pi_gains(a["r_ohm"], a["l_h"], a["vbus_v"], fc_hz=fc)
        print(f"  for {fc} Hz: current_kp {kp} current_ki {ki}")
    return a


def firmware_defaults():
    """RAM-tunable values the ESC boots with, from drive_parameters.h."""
    text = (pathlib.Path(__file__).resolve().parents[1] /
            "HW/B-G431B-ESC1/motor_config/Inc/drive_parameters.h").read_text()
    get = lambda name: int(re.search(rf"#define {name}\s+(\d+)", text)[1])
    return dict(kp=get("PID_SPEED_KP_DEFAULT"), ki=get("PID_SPEED_KI_DEFAULT"),
                ff_static_ma=get("FRICTION_FF_STATIC_MA_DEFAULT"),
                ff_kinetic_ma=get("FRICTION_FF_KINETIC_MA_DEFAULT"),
                speed_rev_filter=get("SPEED_REV_FILTER_DEFAULT"),
                **{key: get(name) for key, name in DSC_DEFAULTS.items()})


# CONFIG key -> drive_parameters.h default, startup speed loop (drive_speed_ctrl.h)
DSC_DEFAULTS = dict(dsc_enable="DSC_ENABLE_DEFAULT", dsc_ff_run_ma="DSC_FF_RUN_MA_DEFAULT",
                    dsc_ff_per_krpm_ma="DSC_FF_PER_KRPM_MA_DEFAULT",
                    dsc_start_from_ma="DSC_START_FROM_MA_DEFAULT",
                    dsc_start_cap_ma="DSC_START_CAP_MA_DEFAULT",
                    dsc_start_slope_ma="DSC_START_SLOPE_MA_DEFAULT",
                    dsc_start_hold_ms="DSC_START_HOLD_MS_DEFAULT", dsc_pcap_ma="DSC_PCAP_MA_DEFAULT",
                    dsc_pcap_ms="DSC_PCAP_MS_DEFAULT", dsc_pcap_updates="DSC_PCAP_UPDATES_DEFAULT",
                    dsc_pcap_fallback_ms="DSC_PCAP_FALLBACK_MS_DEFAULT",
                    dsc_hold_decay_ma="DSC_HOLD_DECAY_MA_DEFAULT", dsc_watch_ms="DSC_WATCH_MS_DEFAULT",
                    dsc_preload_pct="DSC_PRELOAD_PCT_DEFAULT")


def speed_defaults():
    d = firmware_defaults()
    return d["kp"], d["ki"]


def wait_idle(link, indices, timeout=3.0):
    """Block until the given wheels report IDLE: CONFIG_SET is dropped in any
    other state, and STOP -> IDLE takes ~0.7 s after a parked or aborted run."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rows = link.rows()
        if all(rows[i] and rows[i]["mc_state"] == "IDLE" for i in indices):
            return True
        time.sleep(0.05)
    return False


def spin(link, name, index, selector, args):
    """Speed step on one wheel; returns the steady-state summary."""
    if abs(args.rpm) > 300 or not 0 < args.seconds <= 10:
        raise SystemExit("spin: |rpm| <= 300 and 0 < seconds <= 10")
    kp0, ki0 = speed_defaults()
    gains = args.kp is not None or args.ki is not None
    kp = kp0 if args.kp is None else args.kp
    ki = ki0 if args.ki is None else args.ki
    rows = []
    try:
        if gains:
            link.send_raw(*ik_link.config_frame(selector, "kp", kp))
            link.send_raw(*ik_link.config_frame(selector, "ki", ki))
        word = ((args.ramp_ms & 0xFFFF) << 16) | (args.rpm & 0xFFFF)
        t0, last_cmd, last_count = time.monotonic(), -1.0, None
        while (now := time.monotonic() - t0) < args.seconds + 0.6:
            if now < args.seconds and now - last_cmd >= 0.05:
                link.send_raw(ik_link.SPEED_WRITE | selector, word)
                last_cmd = now
            r = link.rows()[index]
            if r and r["status_count"] != last_count:  # one entry per fresh reply
                last_count = r["status_count"]
                rows.append(dict(t=round(now, 4), cmd=args.rpm if now < args.seconds else 0,
                                 **{k: r[k] for k in ("rpm", "iq_a", "iq_ref_a", "mc_state",
                                                      "fault_names", "stop_reason")}))
            time.sleep(0.005)
    finally:
        for _ in range(3):
            try:
                link.send_raw(ik_link.MOTOR_STOP | 0x0F, 0)
            except RuntimeError:
                pass
        if gains:
            if not wait_idle(link, [index]):
                print(f"{name}: not IDLE; speed gains may not be restored (esc_tool.py set)")
            link.send_raw(*ik_link.config_frame(selector, "kp", kp0))
            link.send_raw(*ik_link.config_frame(selector, "ki", ki0))
        if args.log:
            args.log.write_text("".join(json.dumps(r) + "\n" for r in rows))
    steady = [r for r in rows if args.ramp_ms / 1000 + 0.5 < r["t"] < args.seconds]
    if not steady:
        print(f"{name}: no telemetry during the run")
        return None
    rpm = [r["rpm"] for r in steady]
    ref = [r["iq_ref_a"] for r in steady]
    s = dict(wheel=name, cmd=args.rpm, kp=kp, ki=ki, rpm_mean=statistics.mean(rpm),
             rpm_sd=statistics.pstdev(rpm), wrong_sign=sum(1 for v in rpm if v * args.rpm < 0),
             samples=len(rpm), iq_ref_mean=statistics.mean(ref), iq_ref_peak=max(abs(x) for x in ref),
             faults=sorted({f for r in rows for f in r["fault_names"]}))
    print(f"{name}: cmd {args.rpm} rpm (kp {kp} ki {ki}): mean {s['rpm_mean']:.1f} sd {s['rpm_sd']:.1f} "
          f"wrong-sign {s['wrong_sign']}/{s['samples']} | Iq ref mean {s['iq_ref_mean']:.2f} A "
          f"peak {s['iq_ref_peak']:.2f} A | faults {','.join(s['faults']) or '-'}")
    return s


BREAKAWAY_RPM = 20


def breakaways(log, min_target=40, window=1.0):
    """Per start (all targets 0 -> any non-zero): seconds until each wheel whose
    target reaches min_target RPM turns >= BREAKAWAY_RPM the right way (None if
    not within window). log entries: t, target_rpm[4], rpm[4] (None = no reply)."""
    starts, idle = [], True
    for k, e in enumerate(log):
        moving = any(abs(x) >= 1 for x in e["target_rpm"])
        if idle and moving:
            span = [x for x in log[k:] if x["t"] - e["t"] <= window]
            times = {}
            for wi, (name, _esc, _sel) in enumerate(ik_link.WHEELS):
                if max(abs(x["target_rpm"][wi]) for x in span) < min_target:
                    continue
                times[name] = next((x["t"] - e["t"] for x in span if x["rpm"][wi] is not None
                                    and abs(x["rpm"][wi]) >= BREAKAWAY_RPM
                                    and x["rpm"][wi] * x["target_rpm"][wi] > 0), None)
            starts.append(times)
        idle = not moving
    return starts


def breakaway_summary(starts):
    per = {name: [s[name] for s in starts if name in s] for name, _e, _s in ik_link.WHEELS}
    spreads = [max(v) - min(v) for s in starts
               if len(v := [t for t in s.values() if t is not None]) >= 2]
    out = {}
    for name, times in per.items():
        moved = sorted(t for t in times if t is not None)
        out[name] = dict(starts=len(times), moved=len(moved),
                         median_s=statistics.median(moved) if moved else None,
                         worst_s=moved[-1] if moved else None)
    out["spread_median_s"] = statistics.median(spreads) if spreads else None
    out["spread_worst_s"] = max(spreads) if spreads else None
    return out


RUNAWAY_RPM = 200     # pulse targets are ~50 RPM; beyond this a wheel is unloaded or limit-cycling
STALL_AMPS, STALL_S = 4.5, 0.6


class PulseGuard:
    """Abort reason for a pulse log entry, or None: a wheel far above any
    target (lost contact / limit cycle), or one held at high current without
    turning (blocked)."""

    def __init__(self):
        self.stall_since = [None] * 4

    def check(self, e):
        for w, (name, _esc, _sel) in enumerate(ik_link.WHEELS):
            rpm, iq = e["rpm"][w], e["iq_ref_a"][w]
            if rpm is None or iq is None:
                continue
            if abs(rpm) > RUNAWAY_RPM:
                return f"{name} at {rpm} rpm"
            if rpm == 0 and abs(iq) >= STALL_AMPS:
                self.stall_since[w] = self.stall_since[w] if self.stall_since[w] is not None else e["t"]
                if e["t"] - self.stall_since[w] >= STALL_S:
                    return f"{name} stalled at {iq:.1f} A"
            else:
                self.stall_since[w] = None
        return None


def pulse_plan(twists, seconds, pause=1.0, lead=0.5):
    """[(start, end, twist)] for moves of `seconds` each, `pause` s still between
    them, the first after `lead` s; and the total duration."""
    plan, t = [], lead
    for twist in twists:
        plan.append((t, t + seconds, tuple(twist)))
        t += seconds + pause
    return plan, t


def run_pulse(link, plan, duration, settings, defaults):
    """Drive `plan` through the Nucleo with per-wheel RAM `settings` (four
    {key: value} dicts), then restore `defaults` on every wheel once they are
    IDLE. Returns (log, abort reason or None, monotonic time of log t = 0)."""
    before = link.rows()
    time.sleep(0.3)
    dead = [name for (name, _e, _s), r0, r in zip(ik_link.WHEELS, before, link.rows())
            if r is None or r0 is None or r["status_count"] == r0["status_count"]]
    if dead:  # rows keep the last values; only a moving reply count is live
        raise SystemExit(f"pulse: no live reply from {', '.join(dead)} (battery on? CAN?)")
    log, last_counts, guard, aborted = [], [None] * 4, PulseGuard(), None
    t0 = time.monotonic()
    try:
        for (_name, _i, sel), values in zip(wheels("all"), settings):
            for key, value in values.items():
                link.send_raw(*ik_link.config_frame(sel, key, value))
        link.write("accel_rpmps", 0)
        t0, next_cmd = time.monotonic(), 0.0
        while (now := time.monotonic() - t0) < duration:
            if now >= next_cmd:
                twist = next((tw for start, end, tw in plan if start <= now < end), (0, 0, 0))
                link.drive(*twist)
                next_cmd = now + 0.02
            status, rows = link.status(), link.rows()
            counts = [r["status_count"] if r else None for r in rows]
            if counts != last_counts:  # an entry whenever any wheel replied
                last_counts = counts
                log.append(dict(t=round(now, 4), twist=twist, target_rpm=status["target_rpm"],
                                rpm=[r["rpm"] if r else None for r in rows],
                                iq_ref_a=[round(r["iq_ref_a"], 3) if r else None for r in rows],
                                mc_state=[r["mc_state"] if r else None for r in rows],
                                faults=[r["fault_names"] if r else None for r in rows],
                                vbus_v=[r["vbus_v"] if r else None for r in rows]))
                aborted = guard.check(log[-1])
                if aborted:
                    break
    finally:
        link.write("enable", 0)
        deadline = time.monotonic() + 3.0
        while link.read("state") != 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        if not wait_idle(link, range(4)):
            print("pulse: not all wheels IDLE; defaults may not be restored (esc_tool.py set)")
        keys = {k for values in settings for k in values}
        for _name, _i, sel in wheels("all"):
            for key in keys:
                link.send_raw(*ik_link.config_frame(sel, key, defaults[key]))
    return log, aborted, t0


def pulse(link, args):
    if not 0 < args.speed <= 0.3 or not 0 < args.seconds <= 3 or not 1 <= args.reps <= 10:
        raise SystemExit("pulse: 0 < speed <= 0.3 m/s, 0 < seconds <= 3, 1 <= reps <= 10")
    defaults = firmware_defaults()
    asked = dict(kp=args.kp, ki=args.ki, ff_static_ma=args.ff_static, ff_kinetic_ma=args.ff_kinetic)
    # Send every tunable, not only the non-default ones: the ESCs may still hold
    # values from an earlier run whose restore did not land.
    used = {k: defaults[k] if v is None else v for k, v in asked.items()}
    v = round(args.speed * 1000)
    out = (v, 0, 0) if args.axis == "vx" else (0, v, 0)
    plan, duration = pulse_plan([tw for _ in range(args.reps)
                                 for tw in (out, tuple(-x for x in out))], args.seconds)
    log, aborted, _t0 = run_pulse(link, plan, duration, [used] * 4, defaults)
    if args.log:
        args.log.write_text("".join(json.dumps(e) + "\n" for e in log))
    s = breakaway_summary(breakaways(log))
    faults = sorted({f for e in log for fs in e["faults"] if fs for f in fs})
    vbus = [x for e in log for x in e["vbus_v"] if x]
    if aborted:
        print(f"ABORTED: {aborted}")
    print(f"pulse {args.axis} {args.speed} m/s x{args.reps} (kp {used['kp']} ki {used['ki']} "
          f"ff {used['ff_static_ma']}/{used['ff_kinetic_ma']} mA): "
          f"faults {','.join(faults) or '-'}, Vbus min {min(vbus) if vbus else 0:.2f} V")
    ms = lambda x: f"{x * 1000:.0f}" if x is not None else "-"
    for name, _e, _s in ik_link.WHEELS:
        w = s[name]
        print(f"  {name}: moved {w['moved']}/{w['starts']}, to {BREAKAWAY_RPM} rpm median "
              f"{ms(w['median_s'])} ms worst {ms(w['worst_s'])} ms")
    print(f"  spread between wheels: median {ms(s['spread_median_s'])} ms "
          f"worst {ms(s['spread_worst_s'])} ms")
    return s


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", type=pathlib.Path, default=ik_link.DEFAULT_ELF)
    parser.add_argument("--sn", default=ik_link.DEFAULT_SN)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("stop")
    p = sub.add_parser("align")
    p.add_argument("wheel")
    p = sub.add_parser("set")
    p.add_argument("wheel")
    p.add_argument("key", choices=sorted(ik_link.CONFIG_KEYS))
    p.add_argument("value", type=int)
    p = sub.add_parser("capture")
    p.add_argument("wheel")
    p.add_argument("--step-ma", type=int, default=3000)
    p.add_argument("--out", type=pathlib.Path)
    p = sub.add_parser("spin")
    p.add_argument("wheel")
    p.add_argument("--rpm", type=int, required=True)
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--ramp-ms", type=int, default=300)
    p.add_argument("--kp", type=int)
    p.add_argument("--ki", type=int)
    p.add_argument("--log", type=pathlib.Path)
    p = sub.add_parser("pulse")
    p.add_argument("--axis", choices=("vx", "vy"), default="vy", help="vy forward (default), vx right")
    p.add_argument("--speed", type=float, default=0.2, help="m/s, at most 0.3")
    p.add_argument("--seconds", type=float, default=1.5)
    p.add_argument("--reps", type=int, default=3)
    p.add_argument("--kp", type=int)
    p.add_argument("--ki", type=int)
    p.add_argument("--ff-static", type=int, help="friction feed-forward at standstill, mA")
    p.add_argument("--ff-kinetic", type=int, help="friction feed-forward once moving, mA")
    p.add_argument("--log", type=pathlib.Path)
    args = parser.parse_args(argv)
    session, link = ik_link.Link.open(args.sn, args.elf)
    try:
        if args.command == "status":
            before = link.rows()
            time.sleep(0.3)
            show(before, link.rows())
        elif args.command == "stop":
            link.send_raw(ik_link.MOTOR_STOP | 0x0F, 0)
        elif args.command == "align":
            results = [align(link, name, i, sel) for name, i, sel in wheels(args.wheel)]
            print("build flags: " + "  ".join(
                f"{name}:--hall-shift {r['hall_shift_deg']}"
                for (name, _i, _s), r in zip(wheels(args.wheel), results) if r["align_result"] == 1))
        elif args.command == "capture":
            for name, _i, sel in wheels(args.wheel):
                capture(link, name, sel, args)
        elif args.command == "spin":
            targets = wheels(args.wheel)
            if len(targets) != 1:
                raise SystemExit("spin: one wheel at a time")
            spin(link, *targets[0], args)
        elif args.command == "pulse":
            pulse(link, args)
        elif args.command == "set":
            for name, _i, sel in wheels(args.wheel):
                link.send_raw(*ik_link.config_frame(sel, args.key, args.value))
                print(f"{name}: {args.key} = {args.value} (RAM)")
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
