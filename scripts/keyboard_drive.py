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
"""
import argparse
import math
import pathlib
import re
import shutil
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "CM7/build/IK/sfurs_firmware_v2_CM7.elf"
DEFAULT_SN = "002200364142500620353451"
TARGET = "stm32h755zitx"
ABI = 0x494B0001
PREPARE, START, STOP = 1, 2, 3
IDLE, BUSY, RUNNING, STOPPING, FAULT = range(5)
NONE, SENT, REJECTED, TX_FAULT, BAD_TARGET = range(5)
STATES = dict(enumerate(("IDLE", "BUSY", "RUNNING", "STOPPING", "FAULT")))
RESULTS = dict(enumerate(("NONE", "SENT", "REJECTED", "TX_FAULT", "BAD_TARGET")))
HOLD_TIMEOUT = 0.6
TRANSACTION_TIMEOUT = 5.0
FIELDS = ("abi", "vx_mmps", "vy_mmps", "omega_mradps", "request",
          "sequence", "completed", "state", "result", "rpm", "tx_count",
          "ack_count", "fault")
MOTION_KEYS = frozenset("wasdqe")


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

    def __init__(self, target, addrs):
        self.target, self.addrs = target, addrs
        self.verified = False

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
        if command == START:
            for name, value in zip(("vx_mmps", "vy_mmps", "omega_mradps"), vector):
                self.target.write32(self.addrs[name], value & 0xFFFFFFFF)
        sequence = ((sequence + 1) & 0xFFFFFFFF) or 1
        self.target.write32(self.addrs["request"], command)
        self.target.flush()
        # Sequence commits LAST. Firmware never clears request, avoiding a
        # CPU request-clear racing with an SWD STOP publication.
        self.target.write32(self.addrs["sequence"], sequence)
        self.target.flush()
        return sequence

    def poll(self):
        # Read completed before result/state, so a newer completion cannot be
        # paired with output sampled before that transaction finished.
        status = {"completed": self.read("completed")}
        status.update((name, self.read(name)) for name in
                      ("sequence", "request", "state", "result", "tx_count", "ack_count", "fault"))
        status["rpm"] = [signed32(self.target.read32(self.addrs["rpm"] + 4 * i))
                         for i in range(4)]
        return status


class Controller:
    """Nonblocking keyboard tick; no queued motion, retries, or automatic ARM."""

    def __init__(self, drive, speed=0.2, yaw=0.5):
        self.drive = drive
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
    try:
        sequence = drive.submit(STOP)
        deadline = clock() + timeout
        while clock() < deadline:
            status = drive.poll()
            if status["completed"] == sequence:
                if status["result"] == SENT and status["state"] == IDLE and not status["fault"]:
                    report("STOP transaction completed: CAN TX ACK only; physical stop NOT verified.")
                else:
                    report(f"STOP unconfirmed: state={status['state']} result={status['result']} "
                           f"fault={status['fault']}; physical stop NOT verified.")
                return
            sleep(0.05)
        report("STOP completion timed out; physical stop NOT verified.")
    except (Exception, KeyboardInterrupt) as exc:
        report(f"STOP failed: {exc}; physical stop NOT verified. No retry.")


def operate(drive, elf, body, report=print):
    try:
        drive.verify(elf)
        body(drive)
    finally:
        # An incorrect ELF/ABI must cause ZERO writes, including cleanup STOP.
        if drive.verified:
            finish_stop(drive, report)


def run_ui(screen, drive, speed, yaw):
    import curses
    controller = Controller(drive, speed, yaw)
    screen.nodelay(True)
    curses.flushinp()  # Discard keys entered during flash verification.
    while True:
        keys = []
        for _ in range(128):
            key = screen.getch()
            if key == -1:
                break
            if 0 <= key < 256:
                keys.append(chr(key))
        if not controller.tick(keys, time.monotonic()):
            return
        status = controller.status
        vx, vy, omega = controller.requested
        lines = ["IK discrete keyboard drive | pyOCD attach (no reset/halt)",
                 "W/S forward/back  A/D left/right  Q/E CCW+/CW- (single axis only)",
                 "SPACE STOP | ESC/Ctrl-C quit | P ARM-only auto-CAL preparation",
                 "NO key-up tracking: motion may last 0.6 s after release + transport latency.",
                 "Direction changes STOP: release 0.6 s, then NEW keypress. No auto-restart.",
                 "TX ACK / host RUNNING != ESC acceptance, readiness, or actual motion.",
                 f"Requested: vx={vx / 1000:+.3f} m/s vy={vy / 1000:+.3f} m/s yaw={omega / 1000:+.3f} rad/s",
                 "Commanded motor RPM FR BR BL FL: " + " ".join(f"{rpm:+d}" for rpm in status["rpm"]),
                 f"Host {STATES.get(status['state'], '?')} / {RESULTS.get(status['result'], '?')} "
                 f"tx/ack={status['tx_count']}/{status['ack_count']} fault={status['fault']}",
                 f"Sequence/completed={status['sequence']}/{status['completed']} "
                 f"{'INHIBITED: release movement keys' if controller.blocked else 'new key allowed when IDLE'}",
                 controller.message]
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
    parser.add_argument("--speed", type=bounded_float(0.05, 0.3), default=0.2, help="translation m/s (0.05..0.3)")
    parser.add_argument("--yaw", type=bounded_float(0.1, 1.0), default=0.5, help="yaw rad/s (0.1..1.0)")
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
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.sn, connect_mode="attach", target_override=TARGET, blocking=False)
    if session is None:
        raise RuntimeError(f"No probe {args.sn}; no firmware writes performed")
    with session:
        operate(Drive(session.target, addrs), args.elf,
                lambda drive: curses.wrapper(run_ui, drive, args.speed, args.yaw))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("Interrupted; see STOP confirmation above (if firmware was verified).")
    except Exception as exc:
        sys.exit(str(exc))
