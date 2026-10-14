#!/usr/bin/env python3
"""Curses TUI hold-to-run control of one wheel via the Nucleo CAN sender.

Transport is a persistent pyOCD debug session (attach: no halt, no reset),
so reads/writes take milliseconds -- no CLI subprocesses, no verify races.

  1/2/3/4  select NE / SE / SW / NW
  f / r    direction forward (+) / reverse (-)
  + / -    speed setting 10..120 RPM (SE/SW/NW only)
  SPACE    hold to run, release to stop
  t        toggle run latch (for long runs)
  s        stop now
  q / ctrl-c  stop and quit (STOP is always sent on exit)

Speeds: NE reverse is firmware-locked to -100 RPM (ESC exact-match gate);
NE forward is legacy +30. The +/- setting applies to SE/SW/NW (stock ESC
firmware, 500 ms ramp). Single wheel at a time. The ESC must already be
calibrated; this tool only drives the Nucleo side.
"""
import argparse
import curses
import os
import pathlib
import re
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_SN = "002200364142500620353451"
TARGET = "stm32h755zitx"
VERSION = "v4-pyocd"

ARM2 = 0x41524D32
RUN2 = 0x52554E32
STOP = 0x53544F50

WHEELS = {"1": (1, "NE"), "2": (2, "SE"), "3": (4, "SW"), "4": (8, "NW")}
SPEED_MIN, SPEED_MAX, SPEED_DEF = 10, 120, 30
STATES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "STOPPING", 4: "FAULT"}

FALLBACK = {"request": 0x20000134, "wheel": 0x20000138,
            "direction": 0x2000013C, "state": 0x20000140,
            "result": 0x20000144, "run_count": 0x20000148,
            "tx_count": 0x20000154, "tx_ack": 0x20000158,
            "first_fault": 0x2000015C, "speed_rpm": None}


def find_nm():
    for cand in ("/tmp/opencode/cm7-toolchain/root/usr/bin/arm-none-eabi-nm",
                 "arm-none-eabi-nm"):
        if cand.startswith("/") and not os.path.exists(cand):
            continue
        return cand
    return None


def resolve_addrs():
    addrs = dict(FALLBACK)
    elf = ROOT / "CM7/build/Debug/sfurs_firmware_v2_CM7.elf"
    nm = find_nm()
    if nm is None or not elf.exists():
        return addrs, False, addrs.get("speed_rpm") is not None
    try:
        out = subprocess.run([nm, str(elf)], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return addrs, False, False
    found = speed = 0
    for line in out.splitlines():
        m = re.match(r"([0-9a-fA-F]+)\s+\w\s+wheel_bench_(\w+)$", line.strip())
        if not m:
            continue
        name = m.group(2)
        if name in addrs:
            addrs[name] = int(m.group(1), 16)
            found += 1
        if name == "speed_rpm":
            addrs["speed_rpm"] = int(m.group(1), 16)
            speed += 1
    return addrs, found > 0, speed > 0


def eff_speed(wheel, direction, setting):
    """Mirror of the sender: effective RPM for a selection."""
    if wheel == 1 and direction < 0:
        return -100, True
    if wheel == 1:
        return 30, True
    mag = max(SPEED_MIN, min(SPEED_MAX, setting))
    return (mag if direction > 0 else -mag), False


class Bench:
    """Thin wrapper over a pyOCD target (or fake for self-test)."""

    def __init__(self, target, addrs):
        self.t = target
        self.addrs = addrs

    def read(self, addr):
        return self.t.read32(addr)

    def poke(self, addr, value):
        self.t.write32(addr, value & 0xFFFFFFFF)
        if self.read(addr) != value & 0xFFFFFFFF:
            raise RuntimeError(f"readback mismatch @0x{addr:08X}")

    def _wait(self, addr, expect, what, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read(addr) == expect:
                return
            time.sleep(0.05)
        raise RuntimeError(f"{what} had no effect; retry / check FAULT")

    def start(self, wheel, direction, speed_addr=None, speed=None):
        """ARM+RUN; returns poll dict. Raises with the reason on rejection."""
        a = self.addrs
        if speed_addr is not None and speed is not None:
            self.poke(speed_addr, speed)
        self.poke(a["wheel"], wheel)
        self.poke(a["direction"], direction & 0xFFFFFFFF)
        self.t.write32(a["request"], ARM2)
        try:
            self._wait(a["state"], 1, "ARM")
        except RuntimeError:
            res = self.read(a["result"])
            raise RuntimeError(
                f"ARM rejected (result={res}); 2 s cooldown? hold SPACE "
                "again shortly / check FAULT")
        self.t.write32(a["request"], RUN2)
        self._wait(a["state"], 2, "RUN")
        return self.poll()

    def stop(self):
        self.t.write32(self.addrs["request"], STOP)

    def poll(self):
        a = self.addrs
        return {"state": self.read(a["state"]),
                "result": self.read(a["result"]),
                "runs": self.read(a["run_count"]),
                "tx": self.read(a["tx_count"]),
                "ack": self.read(a["tx_ack"]),
                "fault": self.read(a["first_fault"])}


class UI:
    def __init__(self, stdscr, bench, addrs, speed_ok):
        self.s = stdscr
        self.bench = bench
        self.addrs = addrs
        self.speed_ok = speed_ok
        self.wheel, self.wname = 1, "NE"
        self.direction = -1
        self.speed = SPEED_DEF
        self.log = []
        self.status = {}
        self.status_age = 9e9
        self.running = False
        self.latched = False
        self.last_space = 0.0
        self._poll_t = 0.0

    def msg(self, text):
        self.log.append(f"{time.strftime('%H:%M:%S')} {text}")
        del self.log[:-200]

    def refresh(self, poll):
        if poll:
            try:
                self.status = self.bench.poll()
                self._poll_t = time.monotonic()
                self.status_age = 0.0
            except Exception as e:  # probe hiccup: show, keep going
                self.msg(f"poll failed: {e}")
        else:
            self.status_age = time.monotonic() - self._poll_t
        rpm, locked = eff_speed(self.wheel, self.direction, self.speed)
        st = self.status.get("state")
        h, w = self.s.getmaxyx()
        self.s.erase()
        self.s.addstr(0, 0, (f"WHEEL JOG {VERSION}  pyOCD attach  "
                             f"probe {self.bench.sn[:8]}..")[:w - 1])
        self.s.addstr(1, 0, (f"wheel [{self.wname}]  dir [{self.direction:+d}]  "
                             f"set [{self.speed if self.speed_ok else '--'}] RPM  "
                             f"eff [{rpm:+d} RPM{' LOCKED' if locked else ''}]")[:w - 1])
        self.s.addstr(2, 0, (f"bench: {STATES.get(st, '?')} "
                             f"res={self.status.get('result')} "
                             f"runs={self.status.get('runs')} "
                             f"tx/ack={self.status.get('tx')}/{self.status.get('ack')} "
                             f"fault={self.status.get('fault')} "
                             f"({self.status_age:.1f}s ago)"
                             f"{'  HELD' if self.running else ''}"
                             f"{' LATCH' if self.latched else ''}")[:w - 1])
        for i, ln in enumerate(self.log[-(h - 6):]):
            self.s.addstr(4 + i, 0, ln[:w - 1])
        self.s.addstr(h - 2, 0, ("1-4 wheel  f/r dir  +/- speed  SPACE hold-run  "
                                 "t latch  s stop  q quit")[:w - 1])
        self.s.refresh()

    def set_speed(self, delta):
        if not self.speed_ok:
            self.msg("speed config n/a; sender defaults apply")
            return
        self.speed = max(SPEED_MIN, min(SPEED_MAX, self.speed + delta))
        try:
            self.bench.poke(self.addrs["speed_rpm"], self.speed)
            self.msg(f"speed set {self.speed} RPM")
        except Exception as e:
            self.msg(f"speed poke failed: {e}")

    def speed_arg(self):
        if self.speed_ok and not (self.wheel == 1):
            return self.addrs["speed_rpm"], self.speed
        return None, None

    def do_start(self):
        try:
            addr, val = self.speed_arg()
            self.status = self.bench.start(self.wheel, self.direction,
                                           addr, val)
            self._poll_t = time.monotonic()
            self.status_age = 0.0
            rpm, _ = eff_speed(self.wheel, self.direction, self.speed)
            self.msg(f"RUNNING {self.wname} {self.direction:+d} ({rpm:+d} RPM)")
            self.running = True
            self.last_space = time.monotonic()
        except Exception as e:
            self.msg(f"start rejected: {e}")

    def do_stop(self, why):
        try:
            self.bench.stop()
            self.msg(f"STOP ({why})")
        except Exception as e:
            self.msg(f"STOP failed: {e}")
        self.running = False
        self.latched = False

    def run(self):
        try:
            s = self.bench.poll()
        except Exception as e:
            self.msg(f"initial poll failed: {e}")
            s = {"state": -1}
        self.status, self._poll_t, self.status_age = s, time.monotonic(), 0.0
        if s.get("state") == 4:
            self.msg(f"FAULT latched ({s.get('fault')}); reset Nucleo first")
        elif s.get("state") == 1:
            try:
                self.bench.stop()
                self.msg("leftover ARM disarmed")
            except Exception as e:
                self.msg(f"disarm failed: {e}")
        else:
            self.msg(f"ready: {STATES.get(s.get('state'), '?')} "
                     f"runs={s.get('runs')}")
        self.s.nodelay(True)
        last_poll = 0.0
        try:
            while True:
                now = time.monotonic()
                try:
                    key = self.s.getch()
                except curses.error:
                    key = -1
                while key != -1:
                    ch = chr(key) if 0 <= key < 256 else ""
                    if ch == "q":
                        if self.running or self.latched:
                            self.do_stop("quit")
                        return
                    elif ch in WHEELS:
                        if self.running or self.latched:
                            self.do_stop("wheel change")
                        self.wheel, self.wname = WHEELS[ch]
                        rpm, locked = eff_speed(self.wheel, self.direction,
                                                self.speed)
                        self.msg(f"wheel {self.wname} -> {rpm:+d} RPM"
                                 f"{' (locked)' if locked else ''}")
                    elif ch == "f":
                        self.direction = 1
                        self.msg("dir +1")
                    elif ch == "r":
                        self.direction = -1
                        self.msg("dir -1")
                    elif ch in ("+", "="):
                        self.set_speed(10)
                    elif ch in ("-", "_"):
                        self.set_speed(-10)
                    elif ch == " ":
                        self.last_space = now
                        if not self.running and not self.latched:
                            try:
                                st = self.bench.poll()
                            except Exception as e:
                                self.msg(f"poll failed: {e}")
                                st = {"state": -1}
                            self.status = st
                            if st["state"] == 4:
                                self.msg("FAULT latched; reset Nucleo first")
                            elif st["state"] != 0:
                                self.msg(f"busy ({STATES.get(st['state'], '?')})")
                            else:
                                self.do_start()
                    elif ch == "t":
                        if self.running or self.latched:
                            self.do_stop("unlatch")
                        else:
                            self.latched = True
                            self.last_space = now
                            self.do_start()
                            if self.running:
                                self.msg("latched: runs until t/s/q")
                                self.running = False
                    elif ch == "s":
                        if self.running or self.latched:
                            self.do_stop("key")
                    try:
                        key = self.s.getch()
                    except curses.error:
                        key = -1
                held = (self.running and not self.latched
                        and now - self.last_space < 0.8)
                if self.running and not held and not self.latched:
                    self.do_stop("released")
                want_poll = (now - last_poll > (0.8 if self.running or
                                                self.latched else 2.5))
                if want_poll:
                    last_poll = now
                    self.refresh(True)
                else:
                    self.refresh(False)
                time.sleep(0.05)
        except KeyboardInterrupt:
            if self.running or self.latched:
                self.msg("ctrl-c -> STOP")
                try:
                    self.do_stop("ctrl-c")
                except Exception as e:
                    self.msg(f"STOP on ctrl-c failed: {e}")


class FakeTarget:
    """In-memory bench for --self-test (no hardware)."""

    def __init__(self):
        self.mem = {v: 0 for v in FALLBACK.values() if v is not None}

    def read32(self, addr):
        return self.mem.get(addr, 0)

    def write32(self, addr, value):
        a = FALLBACK
        self.mem[addr] = value & 0xFFFFFFFF
        if addr == a["request"]:
            if value == ARM2 and self.mem[a["state"]] == 0:
                self.mem[a["state"]] = 1
                self.mem[a["result"]] = 1
            elif value == RUN2 and self.mem[a["state"]] == 1:
                self.mem[a["state"]] = 2
                self.mem[a["run_count"]] += 1
            elif value == STOP:
                self.mem[a["state"]] = 0
                self.mem[a["result"]] = 3
            else:
                self.mem[a["result"]] = 4


def self_test():
    assert eff_speed(1, -1, 100) == (-100, True)
    assert eff_speed(1, 1, 100) == (30, True)
    assert eff_speed(2, 1, 100) == (100, False)
    assert eff_speed(4, -1, 500) == (-120, False)
    assert eff_speed(2, 1, 5) == (10, False)
    b = Bench(FakeTarget(), dict(FALLBACK))
    st = b.start(1, -1)
    assert st["state"] == 2 and st["runs"] == 1, st
    assert b.read(FALLBACK["state"]) == 2
    b.stop()
    assert b.read(FALLBACK["state"]) == 0
    b.poke(FALLBACK["wheel"], 2)
    assert b.read(FALLBACK["wheel"]) == 2
    print("SELF-TEST OK")


def main():
    ap = argparse.ArgumentParser(description="curses wheel jog TUI (pyOCD)")
    ap.add_argument("--sn", default=DEFAULT_SN)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    addrs, via_nm, speed_ok = resolve_addrs()
    from pyocd.core.helpers import ConnectHelper
    import logging
    logging.disable(logging.CRITICAL)
    try:
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=args.sn, connect_mode="attach", target_override=TARGET)
    except Exception as e:
        sys.exit(f"no probe {args.sn}: {e}")
    with session as board:
        bench = Bench(board.target, addrs)
        try:
            s = bench.poll()
        except Exception as e:
            sys.exit(f"probe read failed: {e}")
        print(f"bench: {STATES.get(s['state'], '?')} runs={s['runs']} "
              f"tx/ack={s['tx']}/{s['ack']} fault={s['fault']} "
              f"(addrs via {'nm' if via_nm else 'fallback'})")
        curses.wrapper(lambda scr: UI(scr, bench, addrs, speed_ok).run())
    print("quit (probe released).")


if __name__ == "__main__":
    main()
