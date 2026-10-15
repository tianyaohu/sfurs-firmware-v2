#!/usr/bin/env python3
"""Curses TUI hold-to-run control of one wheel via the Nucleo CAN sender.

Transport is a persistent pyOCD debug session (attach: no halt, no reset),
so reads/writes take milliseconds -- no CLI subprocesses, no verify races.

  1/2/3/4  select NE / SE / SW / NW
  f / r    direction forward (+) / reverse (-)
  + / -    speed setting 10..150 RPM (SE/SW/NW only)
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
SPEED_MIN, SPEED_MAX, SPEED_DEF = 10, 150, 30
STATES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "STOPPING", 4: "FAULT"}
# ESC motor states, copied from firmware-v2
# HW/B-G431B-ESC1/motor_config/Inc/mc_interface.h MCI_State_t.
ESC_STATES = {0: "IDLE", 12: "ICLWAIT", 16: "CHARGE_BOOT_CAP",
              17: "OFFSET_CALIB", 2: "ALIGNMENT", 4: "START",
              19: "SWITCH_OVER", 6: "RUN", 8: "STOP", 10: "FAULT_NOW",
              11: "FAULT_OVER", 20: "WAIT_STOP_MOTOR",
              21: "OTF_DETECTION", 22: "OTF_BRAKE"}
ESC_STALE_MS = 3000
ESC_NEVER = 0xFFFFFFFF

FALLBACK = {"request": 0x20000134, "wheel": 0x20000138,
            "direction": 0x2000013C, "state": 0x20000140,
            "result": 0x20000144, "run_count": 0x20000148,
            "tx_count": 0x20000154, "tx_ack": 0x20000158,
            "first_fault": 0x2000015C, "speed_rpm": None,
            "esc_rpm": 0x20000160, "esc_state": 0x20000164,
            "esc_age_ms": 0x20000168}


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
        return addrs, False, addrs.get("speed_rpm") is not None, False
    try:
        out = subprocess.run([nm, str(elf)], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return addrs, False, False, False
    found = speed = esc = 0
    # nm suffix -> Bench key. Symbols like tx_ack_count/request_count/
    # first_fault_tick must not collide with the shorter keys.
    aliases = {"tx_ack_count": "tx_ack"}
    for line in out.splitlines():
        m = re.match(r"([0-9a-fA-F]+)\s+\w\s+wheel_bench_(\w+)$", line.strip())
        if not m:
            continue
        name = aliases.get(m.group(2), m.group(2))
        if name in addrs:
            addrs[name] = int(m.group(1), 16)
            found += 1
        if name == "speed_rpm":
            addrs["speed_rpm"] = int(m.group(1), 16)
            speed += 1
        if name in ("esc_rpm", "esc_state", "esc_age_ms"):
            esc += 1
    return addrs, found > 0, speed > 0, esc == 3


def eff_speed(wheel, direction, setting):
    """Mirror of the sender. setting 0 = firmware default.
    Returns (rpm, note) with note in {"", "DEF"}."""
    if wheel == 1:
        if direction < 0:
            if not setting:
                return -100, "DEF"
            return -max(30, min(300, setting)), ""
        if not setting:
            return 30, "DEF"
        return max(30, min(300, setting)), ""
    if not setting:
        return (30 if direction > 0 else -30), "DEF"
    mag = max(SPEED_MIN, min(SPEED_MAX, setting))
    return (mag if direction > 0 else -mag), ""


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

    def start(self, wheel, direction):
        """ARM+RUN; returns poll dict. Raises with the reason on rejection."""
        a = self.addrs
        self.poke(a["wheel"], wheel)
        self.poke(a["direction"], direction & 0xFFFFFFFF)
        self.t.write32(a["request"], ARM2)
        try:
            self._wait(a["state"], 1, "ARM")
        except RuntimeError:
            res = self.read(a["result"])
            raise RuntimeError(
                f"ARM rejected (result={res}); 0.25 s cooldown? hold SPACE "
                "again shortly / check FAULT (confirm wheel stopped first)")
        self.t.write32(a["request"], RUN2)
        self._wait(a["state"], 2, "RUN")
        return self.poll()

    def stop(self):
        self.t.write32(self.addrs["request"], STOP)

    def poll(self):
        a = self.addrs
        d = {"state": self.read(a["state"]),
             "result": self.read(a["result"]),
             "runs": self.read(a["run_count"]),
             "tx": self.read(a["tx_count"]),
             "ack": self.read(a["tx_ack"]),
             "fault": self.read(a["first_fault"])}
        if a.get("esc_rpm") is None or a.get("esc_state") is None \
                or a.get("esc_age_ms") is None:
            d.update({"esc_rpm": None, "esc_state": None, "esc_age": None})
        else:
            raw = self.read(a["esc_rpm"]) & 0xFFFF
            d.update({"esc_rpm": raw - 0x10000 if raw & 0x8000 else raw,
                      "esc_state": self.read(a["esc_state"]) & 0xFF,
                      "esc_age": self.read(a["esc_age_ms"])})
        return d

    def esc_line(self, poll, wheel):
        """Tier-1 ESC status line. Display only; selection-gated."""
        if poll.get("esc_age") is None:
            return "esc: n/a (addrs unresolved)"
        if wheel not in (1, 2):
            return "esc: -- (NE/SE only)"
        age = poll["esc_age"]
        if age == ESC_NEVER:
            return "esc: no-reply"
        name = ESC_STATES.get(poll["esc_state"],
                              f"S{poll['esc_state']}")
        rpm = poll["esc_rpm"]
        if age > ESC_STALE_MS:
            return (f"esc: STALE {name} rpm={rpm:+d} "
                    f"({age / 1000:.1f}s ago)")
        return f"esc: {name} rpm={rpm:+d} ({age / 1000:.1f}s ago)"


class UI:
    def __init__(self, stdscr, bench, sn, addrs, speed_ok, esc_ok=True):
        self.s = stdscr
        self.bench = bench
        self.sn = sn
        self.addrs = addrs
        self.speed_ok = speed_ok
        self.esc_ok = esc_ok
        self.wheel, self.wname = 1, "NE"
        self.direction = -1
        self.speed = 200  # session default; +/- makes it explicit
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
        rpm, note = eff_speed(self.wheel, self.direction, self.speed)
        st = self.status.get("state")
        h, w = self.s.getmaxyx()
        self.s.erase()
        self.s.addstr(0, 0, (f"WHEEL JOG {VERSION}  pyOCD attach  "
                             f"probe {self.sn[:8]}..")[:w - 1])
        set_txt = "DEF" if not self.speed else str(self.speed)
        self.s.addstr(1, 0, (f"wheel [{self.wname}]  dir [{self.direction:+d}]  "
                             f"set [{set_txt if self.speed_ok else '--'}] RPM  "
                             f"eff [{rpm:+d} RPM{' '+note if note else ''}]")[:w - 1])
        self.s.addstr(2, 0, (f"bench: {STATES.get(st, '?')} "
                             f"res={self.status.get('result')} "
                             f"runs={self.status.get('runs')} "
                             f"tx/ack={self.status.get('tx')}/{self.status.get('ack')} "
                             f"fault={self.status.get('fault')} "
                             f"({self.status_age:.1f}s ago)"
                             f"{'  HELD' if self.running else ''}"
                             f"{' LATCH' if self.latched else ''}")[:w - 1])
        esc_txt = ("esc: n/a (addrs unresolved)" if not self.esc_ok
                   else self.bench.esc_line(self.status, self.wheel))
        self.s.addstr(3, 0, esc_txt[:w - 1])
        for i, ln in enumerate(self.log[-(h - 7):]):
            self.s.addstr(5 + i, 0, ln[:w - 1])
        self.s.addstr(h - 2, 0, ("1-4 wheel  f/r dir  +/- speed  SPACE hold-run  "
                                 "t latch  s stop  q quit")[:w - 1])
        self.s.refresh()

    def set_speed(self, delta):
        if not self.speed_ok:
            self.msg("speed config n/a; sender defaults apply")
            return
        lo = 30 if self.wheel == 1 else SPEED_MIN
        hi = 300 if self.wheel == 1 else SPEED_MAX
        base = abs(eff_speed(self.wheel, self.direction, self.speed)[0])
        self.speed = max(lo, min(hi, base + delta))
        try:
            self.bench.poke(self.addrs["speed_rpm"], self.speed)
            self.msg(f"speed set {self.speed} RPM")
        except Exception as e:
            self.msg(f"speed poke failed: {e}")

    def do_start(self):
        try:
            # Speed setting already poked by +/- and persists in the sender.
            self.status = self.bench.start(self.wheel, self.direction)
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
        if self.speed_ok:
            try:
                self.bench.poke(self.addrs["speed_rpm"], self.speed)
                self.msg(f"session default speed {self.speed} RPM")
            except Exception as e:
                self.msg(f"default speed poke failed: {e}")
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
        self.mem[FALLBACK["esc_age_ms"]] = ESC_NEVER  # no reply yet

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

    def inject_esc(self, rpm, state, age_ms):
        """Fake a 0x280|ID reply landing in the sender mirror words."""
        a = FALLBACK
        self.mem[a["esc_rpm"]] = rpm & 0xFFFF
        self.mem[a["esc_state"]] = state & 0xFF
        self.mem[a["esc_age_ms"]] = age_ms & 0xFFFFFFFF


def self_test():
    assert eff_speed(1, -1, 0) == (-100, "DEF")
    assert eff_speed(1, -1, 100) == (-100, "")
    assert eff_speed(1, -1, 500) == (-300, "")
    assert eff_speed(1, -1, 5) == (-30, "")
    assert eff_speed(1, 1, 0) == (30, "DEF")
    assert eff_speed(1, 1, 200) == (200, "")
    assert eff_speed(1, 1, 500) == (300, "")
    assert eff_speed(2, 1, 0) == (30, "DEF")
    assert eff_speed(2, 1, 100) == (100, "")
    assert eff_speed(4, -1, 500) == (-150, "")
    assert eff_speed(2, 1, 5) == (10, "")
    b = Bench(FakeTarget(), dict(FALLBACK))
    st = b.start(1, -1)
    assert st["state"] == 2 and st["runs"] == 1, st
    assert b.read(FALLBACK["state"]) == 2
    b.stop()
    assert b.read(FALLBACK["state"]) == 0
    b.poke(FALLBACK["wheel"], 2)
    assert b.read(FALLBACK["wheel"]) == 2
    # Tier-1 ESC mirror: injection path, signed rpm, names, stale/no-reply.
    ft = FakeTarget()
    eb = Bench(ft, dict(FALLBACK))
    p = eb.poll()
    assert p["esc_age"] == ESC_NEVER, p
    assert eb.esc_line(p, 1) == "esc: no-reply"
    assert eb.esc_line(p, 4) == "esc: -- (NE/SE only)"
    ft.inject_esc(201, 0, 600)
    p = eb.poll()
    assert (p["esc_rpm"], p["esc_state"], p["esc_age"]) == (201, 0, 600), p
    assert eb.esc_line(p, 1) == "esc: IDLE rpm=+201 (0.6s ago)", \
        eb.esc_line(p, 1)
    ft.inject_esc(-30, 6, 0)
    p = eb.poll()
    assert eb.esc_line(p, 2) == "esc: RUN rpm=-30 (0.0s ago)"
    ft.inject_esc(0, 10, 4200)
    p = eb.poll()
    assert eb.esc_line(p, 1) == \
        "esc: STALE FAULT_NOW rpm=+0 (4.2s ago)", eb.esc_line(p, 1)
    ft.inject_esc(0, 99, 100)
    assert eb.esc_line(eb.poll(), 1) == "esc: S99 rpm=+0 (0.1s ago)"
    ft.inject_esc(-1, 11, 3100)
    p = eb.poll()
    assert p["esc_rpm"] == -1 and p["esc_state"] == 11, p
    assert eb.esc_line(p, 1) == \
        "esc: STALE FAULT_OVER rpm=-1 (3.1s ago)", eb.esc_line(p, 1)
    # UI smoke: stub screen exercises refresh paths (caught a .sn bug).
    ui_addrs = dict(FALLBACK)
    scr = StubScreen()
    ui = UI(scr, Bench(FakeTarget(), ui_addrs), "0022sn",
            ui_addrs, True)
    ui.msg("hello")
    ui.set_speed(10)
    ui.refresh(True)
    assert scr.cells[(3, 0)] == "esc: no-reply", scr.cells[(3, 0)]
    ui.refresh(False)
    print("SELF-TEST OK")


class StubScreen:
    """Minimal curses stand-in for UI smoke testing."""

    def __init__(self):
        self.cells = {}

    def getmaxyx(self):
        return 24, 80

    def erase(self):
        pass

    def addstr(self, y, x, text):
        self.cells[(y, x)] = text

    def refresh(self):
        pass

    def nodelay(self, flag):
        pass

    def getch(self):
        return ord("q")


def main():
    ap = argparse.ArgumentParser(description="curses wheel jog TUI (pyOCD)")
    ap.add_argument("--sn", default=DEFAULT_SN)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    addrs, via_nm, speed_ok, esc_ok = resolve_addrs()
    from pyocd.core.helpers import ConnectHelper
    import logging
    logging.disable(logging.CRITICAL)
    try:
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=args.sn, connect_mode="attach",
            target_override=TARGET, blocking=False)
    except KeyboardInterrupt:
        sys.exit("interrupted while waiting for probe")
    except Exception as e:
        sys.exit(f"no probe {args.sn}: {e}\n"
                 "Check USB, replug the board, and make sure no other "
                 "TUI/CLI holds it (one session at a time).")
    with session as board:
        bench = Bench(board.target, addrs)
        try:
            s = bench.poll()
        except Exception as e:
            sys.exit(f"probe read failed: {e}")
        print(f"bench: {STATES.get(s['state'], '?')} runs={s['runs']} "
              f"tx/ack={s['tx']}/{s['ack']} fault={s['fault']} "
              f"(addrs via {'nm' if via_nm else 'fallback'})")
        if (s.get("state") not in STATES or s.get("result") not in range(6)
                or (s.get("ack") or 0) > (s.get("tx") or 0)):
            sys.exit("implausible bench words: address map does not match "
                     "flashed firmware. Rebuild CM7, reflash, and rerun "
                     "(nm must resolve the flashed ELF).")
        curses.wrapper(lambda scr: UI(scr, bench, args.sn, addrs,
                                         speed_ok, esc_ok).run())
    print("quit (probe released).")


if __name__ == "__main__":
    main()
