#!/usr/bin/env python3
"""Discrete, single-axis IK keyboard drive (pyOCD attach, no halt/reset).

W/S: forward/backward, A/D: left/right, Q/E: CCW+/CW- yaw.
SPACE: STOP, ESC or Ctrl-C: STOP and quit. P: ARM-only preparation for
existing auto-CAL; wait independently and explicitly start later. Readiness
is NOT verified. No latch, simultaneous-key tracking, or running updates.

Curses has NO key-up events: repeated keys refresh a 0.6 s timeout. The last
movement can last 0.6 s after release (plus transport/firmware latency).
Direction changes STOP and discard repeats until 0.6 s of movement-key
silence, host IDLE, then a NEW keypress. There is no automatic restart/retry.
CAN TX ACK/host RUNNING do not prove ESC acceptance or physical motion.
STOP is best effort; a lost host/probe cannot guarantee a physical stop.
Telemetry is read at ~10 Hz; missing/stale replies are never measured zero.
--monitor-only ignores control keys and NEVER writes, including exit STOP.
Both modes need exclusive Nucleo probe ownership (no concurrent TUI).
JSONL logging is mandatory, with an exclusive fresh /tmp/opencode default.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf"
DEFAULT_SN = "002200364142500620353451"
TARGET = "stm32h755zitx"
ABI = 0x494B0002
PREPARE, START, STOP = 1, 2, 3
IDLE, BUSY, RUNNING, STOPPING, FAULT = range(5)
NONE, SENT, REJECTED, TX_FAULT, BAD_TARGET = range(5)
STATES = dict(enumerate(("IDLE", "BUSY", "RUNNING", "STOPPING", "FAULT")))
RESULTS = dict(enumerate(("NONE", "SENT", "REJECTED", "TX_FAULT", "BAD_TARGET")))
HOLD_TIMEOUT = 0.6
MAX_LINEAR_SPEED = 5.0
TRANSACTION_TIMEOUT = 5.0
FIELDS = ("abi", "vx_mmps", "vy_mmps", "omega_mradps", "request",
          "sequence", "completed", "state", "result", "rpm", "tx_count",
          "ack_count", "fault", "telem", "telem_now")
MOTION_KEYS = frozenset("wasdqe")
WHEELS = (("FR", "NE", 1), ("BR", "SE", 2), ("BL", "SW", 4), ("FL", "NW", 8))
TELEM_FIELDS = ("generation", "rpm", "state", "hall", "hall_changes", "speed_tick",
                "iq_counts", "id_counts", "flags", "peak_l1_counts", "current_tick",
                "speed_count", "current_count")
TELEM_PERIOD = 0.1
STALE_MS = 500
COUNTS_PER_AMP = 544
FLAG_BITS = {"profile": 0x8000, "armed": 0x4000, "everrun": 0x2000,
             "active": 0x1000, "cal": 0x0800}


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


class JsonlLog:
    """Exclusive, flushed JSONL; any write failure aborts normal operation."""

    def __init__(self, path, elf, monitor_only=False):
        digest = hashlib.sha256(pathlib.Path(elf).read_bytes()).hexdigest()
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = pathlib.Path(path) if path else pathlib.Path(
            f"/tmp/opencode/ik-telemetry-{stamp}-{os.getpid()}.jsonl")
        self.stream = self.path.open("x", encoding="utf-8")
        try:
            self.write("session", elf_path=str(pathlib.Path(elf).resolve()), elf_sha256=digest,
                       abi=ABI, abi_version=2, monitor_only=monitor_only,
                       telemetry_period_s=TELEM_PERIOD, stale_after_ms=STALE_MS,
                       wheels=[dict(wheel=w, compass=c, selector=s) for w, c, s in WHEELS],
                       units={"twist": "mm/s, mm/s, mrad/s", "rpm": "motor RPM",
                              "age": "ms", "current_raw": "SDK counts",
                              "current_a": "firmware-scaled A", "counts_per_amp": COUNTS_PER_AMP},
                       notes=["IK geometry is provisional; x right, y forward, yaw CCW+.",
                              "544 counts/A is firmware scaling; offsets not independently validated.",
                              "peak_l1_counts is a per-run sample peak, not instantaneous physical current.",
                              "Vbus/Iq reference unavailable over current CAN",
                              "TX ACK/host RUNNING do not prove target acceptance or motion.",
                              "No key-up events; release timeout is 0.6 s plus transport latency."])
        except BaseException:
            self.stream.close()
            raise

    def write(self, event, **data):
        try:
            self.stream.write(json.dumps(dict(event=event, monotonic=time.monotonic(),
                                              utc=utc_now(), **data), allow_nan=False) + "\n")
            self.stream.flush()
        except Exception as exc:
            raise RuntimeError(f"Telemetry logging failed ({self.path}): {exc}") from exc

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stream.close()


def decode_telem(words, identity, tick, elapsed_ms=0, minimum_ages=None):
    wheel, compass, selector = identity
    row = dict(wheel=wheel, compass=compass, selector=selector, available=words is not None,
               raw=None, rpm=None, state=None, hall=None, hall_changes=None,
               iq_counts=None, id_counts=None, peak_l1_counts=None, flags=None,
               speed_age_ms=None, current_age_ms=None, speed_fresh=False, current_fresh=False,
               iq_a=None, id_a=None, current_valid=False, invalid_reasons=["unavailable"])
    if words is None:
        return row
    raw = dict(zip(TELEM_FIELDS, words))
    for name in ("rpm", "iq_counts", "id_counts"):
        raw[name] = signed32(raw[name])
    row["raw"] = raw
    for group in ("speed", "current"):
        age = (((tick - raw[group + "_tick"]) & 0xFFFFFFFF) + elapsed_ms
               if raw[group + "_count"] else None)
        if age is not None and minimum_ages is not None:
            age = max(age, minimum_ages.get(group, 0))
        row[group + "_age_ms"] = age
        row[group + "_fresh"] = age is not None and age <= STALE_MS
    if raw["speed_count"]:
        for name in ("rpm", "state", "hall", "hall_changes"):
            row[name] = raw[name]
    if raw["current_count"]:
        for name in ("iq_counts", "id_counts", "peak_l1_counts"):
            row[name] = raw[name]
        row["flags"] = {name: bool(raw["flags"] & mask) for name, mask in FLAG_BITS.items()}
        row["flags"]["faults"] = raw["flags"] & 0x07FF
    reasons = [group + " no reply/stale" for group in ("speed", "current")
               if not row[group + "_fresh"]]
    if row["state"] != 6:
        reasons.append("motor not RUN(6)")
    flags = row["flags"] or {}
    reasons.extend(name + " unset" for name in ("profile", "cal", "active") if not flags.get(name))
    if flags.get("faults"):
        reasons.append("faults")
    row["invalid_reasons"] = reasons
    row["current_valid"] = not reasons
    if row["current_valid"]:
        row["iq_a"], row["id_a"] = (raw[name] / COUNTS_PER_AMP for name in ("iq_counts", "id_counts"))
    return row


def resolve_addrs(elf):
    """Resolve every ABI word strictly from the selected ELF; no fallback RAM."""
    elf = pathlib.Path(elf)
    if not elf.is_file():
        raise RuntimeError(f"IK ELF not found: {elf}; build/flash the IK image first")
    nm = shutil.which("arm-none-eabi-nm") or shutil.which("nm")
    fallback = pathlib.Path("/tmp/opencode/cm7-toolchain/root/usr/bin/arm-none-eabi-nm")
    if nm is None and fallback.is_file():
        nm = str(fallback)
    if nm is None:
        raise RuntimeError("nm is required to resolve the explicit IK ELF")
    output = subprocess.run([nm, "--defined-only", str(elf)], check=True,
                            capture_output=True, text=True, timeout=30).stdout
    addrs = {}
    for line in output.splitlines():
        match = re.fullmatch(r"([0-9a-fA-F]+)\s+\w\s+ik_drive_(\w+)", line.strip())
        if match and match[2] in FIELDS:
            name, address = match[2], int(match[1], 16)
            if name in addrs or address % 4:
                raise RuntimeError(f"Invalid/duplicate IK symbol: ik_drive_{name}")
            addrs[name] = address
    missing = set(FIELDS) - addrs.keys()
    if missing:
        raise RuntimeError("Not a complete IK ELF; missing: " +
                           ", ".join("ik_drive_" + name for name in sorted(missing)))
    return addrs


def flash_segments(elf):
    """Use physical/load addresses, including RAM .data's flash initializer."""
    segments = []
    for segment in elf.iter_segments():
        if segment["p_type"] != "PT_LOAD" or not segment["p_filesz"]:
            continue
        address, size = int(segment["p_paddr"]), int(segment["p_filesz"])
        if address < 0x08200000 and address + size > 0x08000000:
            if address < 0x08000000 or address + size > 0x08200000:
                raise RuntimeError("ELF flash load segment crosses the H755 flash range")
            data = bytes(segment.data())
            if len(data) != size:
                raise RuntimeError("Truncated ELF flash load segment")
            segments.append((address, data))
    if not segments:
        raise RuntimeError("ELF has no file-backed H755 flash PT_LOAD segments")
    return segments


def load_flash_segments(path):
    try:
        from elftools.elf.elffile import ELFFile
    except ImportError as exc:
        raise RuntimeError("pyelftools is required for full flashed-image verification") from exc
    with open(path, "rb") as stream:
        return flash_segments(ELFFile(stream))


def signed32(value):
    return value - 0x100000000 if value & 0x80000000 else value


class Drive:
    """One mailbox transaction; only STOP may supersede an incomplete request."""

    def __init__(self, target, addrs, logger=None, read_only=False):
        self.target, self.addrs = target, addrs
        self.verified = False
        self.logger, self.read_only = logger, read_only
        self._cleanup = False
        self._logged_pending = None
        self._telem_at = float("-inf")
        self._telem_rows = [None] * 4
        self._telem_tick = 0
        self._telem_errors = [None] * 4
        self._telem_progress = [{} for _ in WHEELS]

    def event(self, kind, **data):
        if self.logger is not None:
            try:
                self.logger.write(kind, **data)
            except Exception as exc:
                if not self._cleanup:
                    raise
                # A broken log must not prevent the final best-effort STOP.
                print(f"Cleanup log unavailable: {exc}", file=sys.stderr)

    def read(self, name):
        return self.target.read32(self.addrs[name])

    def verify(self, elf):
        self.verified = False
        for address, expected in load_flash_segments(elf):
            for offset in range(0, len(expected), 1024):
                chunk = expected[offset:offset + 1024]
                actual = bytes(self.target.read_memory_block8(address + offset, len(chunk)))
                if actual != chunk:
                    raise RuntimeError(
                        f"Flash mismatch at 0x{address + offset:08x}; reflash the selected "
                        f"IK ELF ({elf}) before using this TUI. No writes performed.")
        if self.read("abi") != ABI:
            raise RuntimeError("IK ABI mismatch; reflash the selected IK ELF. No writes performed.")
        self.verified = True

    def submit(self, command, vector=(0, 0, 0)):
        if self.read_only:
            raise RuntimeError("Monitor-only is read-only; all writes forbidden")
        if not self.verified:
            raise RuntimeError("Firmware identity has not been verified; writes forbidden")
        if command not in (PREPARE, START, STOP):
            raise ValueError("Unknown IK request")
        sequence = self.read("sequence")
        if command != STOP:
            if self.read("completed") != sequence or self.read("state") != IDLE:
                raise RuntimeError("IK mailbox not idle; only STOP is allowed")
        if len(vector) != 3 or any(type(v) is not int or not -(1 << 31) <= v < (1 << 31)
                                   for v in vector):
            raise ValueError("IK inputs must be three signed 32-bit integers")
        sequence = ((sequence + 1) & 0xFFFFFFFF) or 1
        self.event("mailbox_submission", command=command, sequence=sequence, vector=vector,
                   supersedes=self._logged_pending)
        if command == START:
            for name, value in zip(("vx_mmps", "vy_mmps", "omega_mradps"), vector):
                self.target.write32(self.addrs[name], value & 0xFFFFFFFF)
        self.target.write32(self.addrs["request"], command)
        self.target.flush()
        # Sequence commits LAST. Firmware never clears request, avoiding a
        # CPU request-clear racing with an SWD STOP publication.
        self.target.write32(self.addrs["sequence"], sequence)
        self.target.flush()
        self._logged_pending = (sequence, command)
        self.event("mailbox_committed", command=command, sequence=sequence, vector=vector)
        return sequence

    def poll(self):
        # Read completed before result/state, so a newer completion cannot be
        # paired with output sampled before that transaction finished.
        status = {"completed": self.read("completed")}
        status.update((name, self.read(name)) for name in
                      ("sequence", "request", "state", "result", "tx_count", "ack_count", "fault"))
        status["rpm"] = [signed32(self.target.read32(self.addrs["rpm"] + 4 * i))
                         for i in range(4)]
        status["mailbox_twist"] = [signed32(self.read(name)) for name in
                                   ("vx_mmps", "vy_mmps", "omega_mradps")]
        if self._logged_pending and status["completed"] == self._logged_pending[0]:
            self.event("mailbox_outcome", command=self._logged_pending[1], status=status)
            self._logged_pending = None
        return status

    def poll_telem(self, now=None):
        """10 Hz coherent rows, independent of the fast mailbox poll.

        Cached ages advance on host monotonic time, including between failed
        polls. A torn/error row replaces the old row with unavailable, never 0.
        """
        now = time.monotonic() if now is None else now
        if now - self._telem_at >= TELEM_PERIOD:
            self._telem_at = now
            rows, errors = [], []
            for i in range(4):
                words, error = None, "torn snapshot"
                try:
                    base = self.addrs["telem"] + i * 52
                    for _ in range(3):
                        sample = self.target.read_memory_block32(base, 13)
                        generation = self.target.read32(base)
                        if len(sample) == 13 and sample[0] == generation and not generation & 1:
                            words, error = list(sample), None
                            break
                except Exception as exc:
                    error = str(exc)
                rows.append(words)
                errors.append(error)
            try:
                self._telem_tick = self.read("telem_now")
            except Exception as exc:
                rows, errors = [None] * 4, [str(exc)] * 4
            self._telem_rows, self._telem_errors = rows, errors
        elapsed = max(0, now - self._telem_at) * 1000
        rows = []
        for index, (words, identity) in enumerate(zip(self._telem_rows, WHEELS)):
            minimum_ages = {}
            if words is not None:
                raw = dict(zip(TELEM_FIELDS, words))
                for group in ("speed", "current"):
                    signature = (raw[group + "_count"], raw[group + "_tick"])
                    if signature[0] == 0:
                        self._telem_progress[index].pop(group, None)
                        continue
                    anchor = self._telem_progress[index].get(group)
                    if anchor is None or anchor[0] != signature:
                        initial_age = ((self._telem_tick - signature[1]) & 0xFFFFFFFF) + elapsed
                        anchor = (signature, now, initial_age)
                        self._telem_progress[index][group] = anchor
                    # Re-reading unchanged RAM is not a new reply. Keep wall
                    # time aging even if the MCU clock freezes and later resumes.
                    minimum_ages[group] = anchor[2] + max(0, now - anchor[1]) * 1000
            rows.append(decode_telem(words, identity, self._telem_tick, elapsed, minimum_ages))
        for row, error in zip(rows, self._telem_errors):
            row["read_error"] = error
        return dict(snapshot_monotonic=self._telem_at, now_tick=self._telem_tick, rows=rows)


class Controller:
    """Nonblocking keyboard tick; no queued motion, retries, or automatic ARM."""

    def __init__(self, drive, speed=0.2, yaw=0.5, monitor_only=False):
        self.drive = drive
        self.monitor_only = monitor_only
        speed, yaw = round(speed * 1000), round(yaw * 1000)
        # Firmware body frame: x right, y forward (not ROS x-forward).
        self.vectors = {"w": (0, speed, 0), "s": (0, -speed, 0),
                        "a": (-speed, 0, 0), "d": (speed, 0, 0),
                        "q": (0, 0, yaw), "e": (0, 0, -yaw)}
        self.active_key = None
        self.requested = (0, 0, 0)
        self.last_motion = self.last_prepare = float("-inf")
        self.blocked = False
        self.pending = None
        self.status = {}
        self.initialized = False
        self.message = "P: ARM-only; wait for auto-CAL independently. Readiness NOT verified."

    def issue(self, command, now, vector=(0, 0, 0)):
        self.pending = (self.drive.submit(command, vector), command, now)

    def stop(self, now, reason):
        self.active_key, self.requested = None, (0, 0, 0)
        self.blocked, self.last_motion = True, now
        if self.pending is None or self.pending[1] != STOP:
            self.issue(STOP, now)
        self.message = reason + "; release movement keys for 0.6 s, then NEW keypress."

    def tick(self, keys, now):
        keys = [key.lower() for key in keys]
        self.status = status = self.drive.poll()
        if self.monitor_only:
            self.message = "READ-ONLY: movement/P/SPACE ignored; no startup or exit STOP."
            return not any(key in ("\x1b", "\x03") for key in keys)
        if status["state"] == FAULT or status["fault"]:
            raise RuntimeError(f"Host FAULT {status['fault']}; no automatic retry")
        if self.pending is not None:
            sequence, command, sent_at = self.pending
            if status["completed"] == sequence:
                if status["result"] != SENT:
                    raise RuntimeError("IK transaction " + RESULTS.get(status["result"], "UNKNOWN") +
                                       "; no automatic retry")
                self.pending = None
                if command == PREPARE:
                    self.message = "ARM TX ACK only; wait for auto-CAL. Readiness NOT verified."
                elif command == STOP:
                    self.message = "STOP TX ACK only; physical stop NOT verified. Release then NEW key."
            elif now - sent_at >= TRANSACTION_TIMEOUT:
                self.drive.event("mailbox_timeout", sequence=sequence, command=command)
                raise RuntimeError("IK transaction timed out; no automatic retry")
        if not self.initialized:
            self.initialized = True
            if status["state"] != IDLE or status["completed"] != status["sequence"]:
                self.stop(now, "STOP existing host operation")
        if any(key in ("\x1b", "\x03") for key in keys):
            return False
        if self.active_key and self.pending is None and status["state"] != RUNNING:
            raise RuntimeError("Host left RUNNING; no automatic restart")
        if self.active_key and now - self.last_motion >= HOLD_TIMEOUT:
            self.stop(now, "Movement repeat timeout: STOP")
        has_motion = any(key in MOTION_KEYS for key in keys)
        # Only an EMPTY movement-key tick can unlock. A buffered repeat batch
        # can never both unlock and start, even after a long UI/probe delay.
        if (self.blocked and not has_motion and now - self.last_motion >= HOLD_TIMEOUT
                and status["state"] == IDLE and self.pending is None):
            self.blocked = False
        for key in keys:
            if key == " ":
                self.stop(now, "SPACE: STOP")
            elif key == "p":
                fresh = now - self.last_prepare >= HOLD_TIMEOUT
                self.last_prepare = now
                if fresh and self.active_key is None and self.pending is None and status["state"] == IDLE:
                    self.issue(PREPARE, now)
                    self.blocked, self.last_motion = True, now
                    self.message = "ARM-only pending; never automatically starts. Readiness NOT verified."
            elif key in MOTION_KEYS:
                self.last_motion = now
                if self.blocked:
                    continue
                if self.active_key:
                    if key != self.active_key:
                        self.stop(now, "Direction change: STOP (new direction discarded)")
                elif self.pending is None and status["state"] == IDLE:
                    self.requested = self.vectors[key]
                    self.issue(START, now, self.requested)
                    self.active_key = key
                    self.message = "START requested; host/bus status is NOT physical motion evidence."
                else:
                    self.blocked = True  # Busy keypresses must not become delayed STARTs.
        return True


def finish_stop(drive, report=print, timeout=TRANSACTION_TIMEOUT,
                clock=time.monotonic, sleep=time.sleep):
    """One best-effort STOP, bounded completion wait; never claim wheels stopped."""
    if drive.read_only:
        report("Monitor-only: no STOP or other writes performed.")
        return
    was_cleanup, drive._cleanup = drive._cleanup, True
    sequence = None
    try:
        sequence = drive.submit(STOP)
        deadline = clock() + timeout
        while clock() < deadline:
            status = drive.poll()
            if status["completed"] == sequence:
                drive.event("cleanup_stop_outcome", sequence=sequence, status=status)
                if status["result"] == SENT and status["state"] == IDLE and not status["fault"]:
                    report("STOP transaction completed: CAN TX ACK only; physical stop NOT verified.")
                else:
                    report(f"STOP unconfirmed: state={status['state']} result={status['result']} "
                           f"fault={status['fault']}; physical stop NOT verified.")
                return
            sleep(0.05)
        drive.event("cleanup_stop_failed", sequence=sequence, reason="timeout",
                    error="STOP completion timed out")
        report("STOP completion timed out; physical stop NOT verified.")
    except (Exception, KeyboardInterrupt) as exc:
        drive.event("cleanup_stop_failed", sequence=sequence, reason="transport_or_interrupt",
                    error=str(exc))
        report(f"STOP failed: {exc}; physical stop NOT verified. No retry.")
    finally:
        drive._cleanup = was_cleanup


def operate(drive, elf, body, report=print, monitor_only=False):
    if monitor_only:
        drive.read_only = True
    try:
        drive.verify(elf)
        drive.event("firmware_verified", abi=ABI, read_only=drive.read_only)
        body(drive)
    finally:
        # An incorrect ELF/ABI must cause ZERO writes, including cleanup STOP.
        if drive.verified and not drive.read_only:
            finish_stop(drive, report)


def telemetry_lines(row, command):
    identity = f"{row['wheel']}/{row['compass']}/{row['selector']}"
    if not row["available"]:
        return [f"{identity} CMD={command:+d} MEASURED unavailable ({row['read_error']})",
                "  Speed/current reply ages unavailable; firmware-scaled Iq/Id INVALID",
                "  Raw telemetry unavailable (not zero)"]
    def value(name):
        return "--" if row[name] is None else str(row[name])
    def age(group):
        ms = row[group + "_age_ms"]
        return "NO REPLY" if ms is None else f"{ms:.0f}ms {'fresh' if row[group + '_fresh'] else 'STALE'}"
    flags = row["flags"]
    flag_text = ("NO REPLY" if flags is None else
                 " ".join(f"{key}={int(flags[key])}" for key in FLAG_BITS) + f" faults=0x{flags['faults']:03x}")
    amps = (f"{row['iq_a']:+.3f}/{row['id_a']:+.3f} A" if row["current_valid"] else "INVALID")
    state = "RUN(6)" if row["state"] == 6 else value("state")
    return [f"{identity} CMD={command:+d} MEASURED={value('rpm')}rpm state={state} "
            f"hall={value('hall')} changes={value('hall_changes')} speed#{row['raw']['speed_count']} {age('speed')}",
            f"  firmware-scaled Iq/Id={amps} raw={value('iq_counts')}/{value('id_counts')} counts "
            f"peakL1={value('peak_l1_counts')} current#{row['raw']['current_count']} {age('current')}",
            "  " + flag_text]


def run_ui(screen, drive, speed, yaw, monitor_only=False):
    import curses
    controller = Controller(drive, speed, yaw, monitor_only=monitor_only)
    screen.nodelay(True)
    curses.flushinp()  # Discard keys entered during flash verification.
    last_sample = float("-inf")
    while True:
        keys = []
        for _ in range(128):
            key = screen.getch()
            if key == -1:
                break
            if 0 <= key < 256:
                keys.append(chr(key))
        if keys:
            drive.event("keys", keys=keys, monitor_only=monitor_only)
        if not controller.tick(keys, time.monotonic()):
            return
        status = controller.status
        now = time.monotonic()
        telemetry = drive.poll_telem(now)
        if now - last_sample >= TELEM_PERIOD:
            drive.event("sample", requested_twist=None if monitor_only else controller.requested,
                        host_status=status, telemetry=telemetry, monitor_only=monitor_only)
            last_sample = now
        vx, vy, omega = controller.requested
        lines = ["IK " + ("READ-ONLY MONITOR" if monitor_only else "discrete keyboard drive") +
                 " | pyOCD attach (no reset/halt)",
                 ("Movement/P/SPACE ignored; NO prepare/start/STOP, even on exit." if monitor_only else
                  "W/S forward/back  A/D left/right  Q/E CCW+/CW- (single axis only)"),
                 ("ESC/Ctrl-C quit | exclusive Nucleo probe; no concurrent TUI" if monitor_only else
                  "SPACE STOP | ESC/Ctrl-C quit | P ARM-only auto-CAL preparation"),
                 "NO key-up: motion lasts up to 0.6 s + latency. Direction change: STOP, release 0.6 s, NEW key.",
                 "TX ACK / host RUNNING != ESC acceptance, readiness, or actual motion.",
                 ("No local twist requested (read-only); mailbox twist=" + str(status["mailbox_twist"])
                  if monitor_only else
                  f"Requested: vx={vx / 1000:+.3f} m/s vy={vy / 1000:+.3f} m/s yaw={omega / 1000:+.3f} rad/s"),
                 f"Host {STATES.get(status['state'], '?')} / {RESULTS.get(status['result'], '?')} "
                 f"tx/ack={status['tx_count']}/{status['ack_count']} fault={status['fault']} "
                 f"seq/done={status['sequence']}/{status['completed']} "
                 f"{'INHIBITED' if controller.blocked else ''}",
                 controller.message]
        for row, command in zip(telemetry["rows"], status["rpm"]):
            lines.extend(telemetry_lines(row, command))
        lines.extend(["544 counts/A: firmware-scaled; offsets not independently validated. peakL1: per-run sample peak counts.",
                      "Vbus/Iq reference unavailable over current CAN; peakL1 NOT instantaneous physical/battery current.",
                      f"Log: {drive.logger.path}" if drive.logger else ""])
        screen.erase()
        height, width = screen.getmaxyx()
        for row, line in enumerate(lines[:max(0, height - 1)]):
            if width > 1:
                screen.addnstr(row, 0, line, width - 1)
        screen.refresh()
        time.sleep(0.02)


def bounded_float(low, high):
    def parse(text):
        value = float(text)
        if not math.isfinite(value) or not low <= value <= high:
            raise argparse.ArgumentTypeError(f"must be in {low}..{high}")
        return value
    return parse


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", type=pathlib.Path, default=DEFAULT_ELF,
                        help="exact flashed IK ELF; all flash PT_LOAD bytes are verified before writes")
    parser.add_argument("--sn", default=DEFAULT_SN)
    parser.add_argument("--speed", type=bounded_float(0.05, MAX_LINEAR_SPEED), default=0.2,
                        help="translation m/s (0.05..5.0); requires matching high-cap firmware")
    parser.add_argument("--yaw", type=bounded_float(0.1, 1.0), default=0.5, help="yaw rad/s (0.1..1.0)")
    parser.add_argument("--log", type=pathlib.Path,
                        help="exclusive new JSONL path (default: fresh /tmp/opencode/ik-telemetry-UTC-PID.jsonl)")
    parser.add_argument("--monitor-only", action="store_true",
                        help="read-only: ignore controls; no prepare/start/STOP, even on exit; exclusive probe required")
    parser.add_argument("--self-test", action="store_true", help="hardware-free unittest suite (no pyOCD import)")
    args = parser.parse_args(argv)
    if args.self_test:
        import unittest
        suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"), pattern="test_keyboard_drive.py")
        if not suite.countTestCases():
            raise RuntimeError("tests/test_keyboard_drive.py is required for --self-test")
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    addrs = resolve_addrs(args.elf)
    from pyocd.core.helpers import ConnectHelper
    import curses
    with JsonlLog(args.log, args.elf, args.monitor_only) as logger:
        print(f"Telemetry log: {logger.path}", flush=True)
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=args.sn, connect_mode="attach", target_override=TARGET, blocking=False)
        if session is None:
            raise RuntimeError(f"No probe {args.sn}; no firmware writes performed")
        with session:
            operate(Drive(session.target, addrs, logger, read_only=args.monitor_only), args.elf,
                    lambda drive: curses.wrapper(run_ui, drive, args.speed, args.yaw, args.monitor_only),
                    monitor_only=args.monitor_only)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("Interrupted; see STOP confirmation above (if firmware was verified).")
    except Exception as exc:
        sys.exit(str(exc))
