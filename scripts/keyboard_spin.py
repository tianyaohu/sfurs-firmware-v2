#!/usr/bin/env python3
"""Curses TUI hold-to-run control of one wheel via the Nucleo CAN sender.

Hold SPACE (terminal key-repeat on) to run; release to STOP. All probe I/O
is batched into single CLI sessions (~0.1 s), so the UI stays responsive.

  1/2/3/4  select NE / SE / SW / NW
  f / r    direction forward (+) / reverse (-)
  + / -    speed setting 10..120 RPM (SE/SW/NW only)
  SPACE    hold to run, release to stop
  t        toggle run latch (for long runs)
  s        stop now
  q        stop and quit

Speeds: NE reverse is firmware-locked to -100 RPM (ESC exact-match gate);
NE forward is legacy +30. The +/- setting applies to SE/SW/NW (stock ESC
firmware, 500 ms ramp, sender STOP window 1000 ms). Single wheel at a time.
The ESC must already be calibrated; this tool only drives the Nucleo side.
All RAM access uses HOTPLUG (a normal connect would reset the target).
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
DEFAULT_CLI = (pathlib.Path.home() / "STMicroelectronics/STM32Cube"
               / "STM32CubeProgrammer/bin/STM32_Programmer_CLI")
VERSION = "v3"

ARM2 = 0x41524D32
RUN2 = 0x52554E32
STOP = 0x53544F50

WHEELS = {"1": (1, "NE"), "2": (2, "SE"), "3": (4, "SW"), "4": (8, "NW")}
SPEED_MIN, SPEED_MAX, SPEED_DEF = 10, 120, 30
STATES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "STOPPING", 4: "FAULT"}

# Fallback map (refreshed 2026-10-14; prefer nm resolution at runtime).
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
        return addrs, False, False
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


def parse_reads(output):
    """Map every 'addr : WORD' readback in CLI output."""
    return {int(a, 16): int(w, 16)
            for a, w in re.findall(r"(0x[0-9A-Fa-f]+)\s*:\s*([0-9A-Fa-f]{8})",
                                   output)}


def eff_speed(wheel, direction, setting):
    """Mirror of the sender: effective RPM for a selection."""
    if wheel == 1 and direction < 0:
        return -100, True
    if wheel == 1:
        return 30, True
    mag = max(SPEED_MIN, min(SPEED_MAX, setting))
    return (mag if direction > 0 else -mag), False


class Bench:
    def __init__(self, cli, sn, addrs):
        self.cli = str(cli)
        self.sn = sn
        self.addrs = addrs

    def _call(self, ops):
        """One probe connection for sticky writes/reads."""
        cmd = [self.cli, "-c", f"port=SWD sn={self.sn} mode=HOTPLUG"]
        for op in ops:
            if op[0] == "w":
                cmd += ["-w32", f"0x{op[1]:08X}", f"0x{op[2]:08X}"]
            else:
                cmd += ["-r32", f"0x{op[1]:08X}", "0x4"]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        except subprocess.SubprocessError as e:
            raise RuntimeError(f"CLI call failed: {e}")
        out = p.stdout + p.stderr
        low = out.lower().replace(" ", "")
        for op in ops:
            if op[0] == "w" and f"w@0x{op[1]:08x}" not in low:
                raise RuntimeError(f"write not issued @0x{op[1]:08X}:\n"
                                   + out[-500:])
        return parse_reads(out)

    def session(self, ops):
        """Run ops across minimal probe connections.

        Request words ("W") are consumed by the firmware within milliseconds,
        which trips the CLI's download-verify and aborts the rest of that
        call -- so each "W" goes alone in its own call and is confirmed by
        state effect, never by readback. Sticky writes/reads batch freely.
        op kinds: "w" sticky write, "r" read, "W" request write.
        """
        reads = {}
        chunk = []

        def flush():
            if chunk:
                reads.update(self._call(chunk))
                del chunk[:]

        for op in ops:
            if op[0] == "W":
                flush()
                self._call([("w", op[1], op[2])])
            else:
                chunk.append(op)
        flush()
        return reads

    def read(self, addr):
        return self.session([("r", addr)]).get(addr)

    def poke(self, addr, value):
        self.session([("w", addr, value)])
        if self.read(addr) != value & 0xFFFFFFFF:
            raise RuntimeError(f"readback mismatch @0x{addr:08X}")

    def poll(self):
        a = self.addrs
        r = self.session([("r", a[k]) for k in
                          ("state", "result", "run_count",
                           "tx_count", "tx_ack", "first_fault")])
        return {"state": r.get(a["state"]), "result": r.get(a["result"]),
                "runs": r.get(a["run_count"]), "tx": r.get(a["tx_count"]),
                "ack": r.get(a["tx_ack"]), "fault": r.get(a["first_fault"])}

    def start(self, wheel, direction):
        """ARM+RUN in minimal sessions; returns poll dict. Raises on reject."""
        a = self.addrs
        r = self.session([("w", a["wheel"], wheel),
                          ("w", a["direction"], direction & 0xFFFFFFFF),
                          ("W", a["request"], ARM2),
                          ("r", a["state"]), ("r", a["result"])])
        if r.get(a["state"]) != 1:
            raise RuntimeError(f"ARM rejected (state={r.get(a['state'])}, "
                               f"result={r.get(a['result'])}); 2 s cooldown? "
                               "hold SPACE again shortly / check FAULT")
        r = self.session([("W", a["request"], RUN2),
                          ("r", a["state"])])
        if r.get(a["state"]) != 2:
            raise RuntimeError("RUN rejected after ARM; retry")
        return self.poll()

    def stop(self):
        self.session([("W", self.addrs["request"], STOP)])


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
        self.busy = ""

    def msg(self, text):
        self.log.append(f"{time.strftime('%H:%M:%S')} {text}")
        del self.log[:-200]

    def refresh(self, poll):
        if poll:
            t0 = time.monotonic()
            try:
                self.status = self.bench.poll()
                self.status_age = 0.0
                self._poll_t = t0
            except RuntimeError as e:
                self.msg(f"poll failed: {e}")
        else:
            self.status_age = time.monotonic() - getattr(self, "_poll_t",
                                                          1e9) + 0.0 \
                if hasattr(self, "_poll_t") else 9e9
        rpm, locked = eff_speed(self.wheel, self.direction, self.speed)
        st = self.status.get("state")
        lines = [
            f"WHEEL JOG {VERSION}  probe {self.bench.sn[:8]}..  "
            f"ESC calibrated? (separate cable)  {self.busy}",
            f"wheel [{self.wname}]  dir [{self.direction:+d}]  "
            f"set [{self.speed if self.speed_ok else '--'}] RPM  "
            f"eff [{rpm:+d} RPM{' LOCKED' if locked else ''}]",
            f"bench: {STATES.get(st, '?')} res={self.status.get('result')} "
            f"runs={self.status.get('runs')} "
            f"tx/ack={self.status.get('tx')}/{self.status.get('ack')} "
            f"fault={self.status.get('fault')} "
            f"({self.status_age:.1f}s ago){'  HELD' if self.running else ''}"
            f"{' LATCH' if self.latched else ''}",
            "",
        ]
        h, w = self.s.getmaxyx()
        self.s.erase()
        for i, ln in enumerate(lines[:4]):
            self.s.addstr(i, 0, ln[:w - 1])
        for i, ln in enumerate(self.log[-(h - 7):]):
            self.s.addstr(4 + i, 0, ln[:w - 1])
        self.s.addstr(h - 2, 0, ("1-4 wheel  f/r dir  +/- speed  "
                                 "SPACE hold-run  t latch  s stop  q quit")[:w - 1])
        self.s.refresh()

    def set_speed(self, delta):
        if not self.speed_ok:
            self.msg("speed config n/a (nm failed); sender defaults apply")
            return
        self.speed = max(SPEED_MIN, min(SPEED_MAX, self.speed + delta))
        try:
            self.bench.poke(self.addrs["speed_rpm"], self.speed)
            self.msg(f"speed set {self.speed} RPM")
        except RuntimeError as e:
            self.msg(f"speed poke failed: {e}")

    def do_start(self):
        self.busy = "ARM+RUN..."
        self.refresh(False)
        try:
            self.status = self.bench.start(self.wheel, self.direction)
            self._poll_t = time.monotonic()
            self.status_age = 0.0
            rpm, _ = eff_speed(self.wheel, self.direction, self.speed)
            self.msg(f"RUNNING {self.wname} {self.direction:+d} ({rpm:+d} RPM)")
            self.running = True
        except RuntimeError as e:
            self.msg(f"start rejected: {e}")
        finally:
            self.busy = ""

    def do_stop(self, why):
        try:
            self.bench.stop()
            self.msg(f"STOP ({why})")
        except RuntimeError as e:
            self.msg(f"STOP failed: {e}")
        self.running = False
        self.latched = False

    def run(self):
        s = self.bench.poll()
        self.status, self._poll_t, self.status_age = s, time.monotonic(), 0.0
        if s["state"] == 4:
            self.msg(f"FAULT latched ({s['fault']}); reset Nucleo first")
        elif s["state"] == 1:
            self.bench.stop()
            self.msg("leftover ARM disarmed")
        else:
            self.msg(f"ready: {STATES.get(s['state'], '?')} runs={s['runs']}")
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
                        if self.running:
                            self.do_stop("quit")
                        return
                    elif ch in WHEELS:
                        if self.running:
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
                            st = self.bench.poll()
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
                if want_poll and not self.busy:
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
                except RuntimeError as e:
                    self.msg(f"STOP on ctrl-c failed: {e}")


def self_test():
    assert parse_reads("0x2000013C : 00000002") == {0x2000013C: 2}
    assert eff_speed(1, -1, 100) == (-100, True)
    assert eff_speed(1, 1, 100) == (30, True)
    assert eff_speed(2, 1, 100) == (100, False)
    assert eff_speed(4, -1, 500) == (-120, False)
    assert eff_speed(2, 1, 5) == (10, False)
    import tempfile as _tf
    with _tf.TemporaryDirectory() as tmp:
        stfile = pathlib.Path(tmp) / "state"
        stfile.write_text("0")
        fake = pathlib.Path(tmp) / "fakecli"
        fake.write_text(
            "#!/usr/bin/env python3\nimport pathlib, sys\n"
            f"st = pathlib.Path({str(stfile)!r})\n"
            "s = int(st.read_text())\n"
            "a = sys.argv[1:]\n"
            "i = 0\nout = []\n"
            "while i < len(a):\n"
            "    if a[i] == '-w32':\n"
            "        addr, val = a[i+1], int(a[i+2], 16)\n"
            "        out.append(f'w @{addr}: 0x{val:08x} [===] 100%')\n"
            "        out.append('Error: Failed to download data!')\n"
            "        if val == 0x41524D32: s = 1\n"
            "        elif val == 0x52554E32 and s == 1: s = 2\n"
            "        elif val == 0x53544F50: s = 0\n"
            "        i += 3\n"
            "    elif a[i] == '-r32':\n"
            "        out.append(f'{a[i+1]} : {s:08X}')\n"
            "        i += 3\n"
            "    else: i += 1\n"
            "st.write_text(str(s))\nprint('\\n'.join(out))\n")
        fake.chmod(0o755)
        addrs = dict(FALLBACK)
        b = Bench(str(fake), "sn", addrs)
        r = b.session([("w", addrs["wheel"], 1),
                       ("w", addrs["direction"], 0xFFFFFFFF),
                       ("W", addrs["request"], ARM2),
                       ("r", addrs["state"])])
        assert r[addrs["state"]] == 1, r
        st = b.start(1, -1)
        assert st["state"] == 2, st
        b.stop()
    print("SELF-TEST OK")


def main():
    ap = argparse.ArgumentParser(description="curses wheel jog TUI")
    ap.add_argument("--sn", default=DEFAULT_SN)
    ap.add_argument("--cli", default=str(DEFAULT_CLI))
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    if not os.path.exists(args.cli):
        sys.exit(f"CLI not found: {args.cli}")
    addrs, via_nm, speed_ok = resolve_addrs()
    bench = Bench(args.cli, args.sn, addrs)
    try:
        s = bench.poll()
    except RuntimeError as e:
        sys.exit(f"probe read failed: {e}\nIs cable on Nucleo {args.sn}?")
    print(f"bench: {STATES.get(s['state'], '?')} runs={s['runs']} "
          f"tx/ack={s['tx']}/{s['ack']} fault={s['fault']} "
          f"(addrs via {'nm' if via_nm else 'fallback'})")
    curses.wrapper(lambda scr: UI(scr, bench, addrs, speed_ok).run())
    print("quit.")


if __name__ == "__main__":
    main()
