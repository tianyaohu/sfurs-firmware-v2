#!/usr/bin/env python3
"""Hold-to-run keyboard control of one wheel through the Nucleo CAN sender.

Hold SPACE (with terminal key-repeat on) to run the selected wheel/direction;
release to STOP. No dependencies beyond Python 3 and STM32CubeProgrammer CLI.

  1/2/3/4  select NE / SE / SW / NW (front-right / rear-right / rear-left / front-left)
  f / r    direction forward (+) / reverse (-)
  SPACE    hold to run, release to stop
  s        stop now
  q        stop and quit

Speeds are firmware-fixed, not chosen here: NE reverse -100 RPM, everything
else +/-30 RPM, 500 ms ramp. Single wheel at a time (sender limit).
The ESC must already be calibrated (result 2 / valid 1) over its own cable;
this script only drives the Nucleo side. All Nucleo RAM access uses HOTPLUG:
a normal connect would reset the target and wipe bench state.

Each CLI call costs ~1-3 s (probe connect overhead), so press-to-motion is
several seconds and release-to-stop ~2 s. This is a bench jog tool, not a
real-time drive. Keep the motor battery ON and the wheel clear at all times.
"""
import argparse
import os
import pathlib
import re
import select
import subprocess
import sys
import termios
import time
import tty

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_SN = "002200364142500620353451"
DEFAULT_CLI = (pathlib.Path.home() / "STMicroelectronics/STM32Cube"
               / "STM32CubeProgrammer/bin/STM32_Programmer_CLI")

ARM2 = 0x41524D32
RUN2 = 0x52554E32
STOP = 0x53544F50

WHEELS = {"1": (1, "NE"), "2": (2, "SE"), "3": (4, "SW"), "4": (8, "NW")}
SPEEDS = {(1, 1): "+30", (1, -1): "-100", (2, 1): "+30", (2, -1): "-30",
          (4, 1): "+30", (4, -1): "-30", (8, 1): "+30", (8, -1): "-30"}
STATES = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "STOPPING", 4: "FAULT"}

# Fallback map (verified against ELF via nm at startup when available).
FALLBACK = {"request": 0x20000130, "wheel": 0x20000134,
            "direction": 0x20000138, "state": 0x2000013C,
            "result": 0x20000140, "run_count": 0x20000144,
            "tx_count": 0x20000150, "tx_ack": 0x20000154,
            "first_fault": 0x20000158}


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
        return addrs, False
    try:
        out = subprocess.run([nm, str(elf)], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return addrs, False
    found = 0
    for line in out.splitlines():
        m = re.match(r"([0-9a-fA-F]+)\s+\w\s+wheel_bench_(\w+)$", line.strip())
        if m and m.group(2) in addrs:
            addrs[m.group(2)] = int(m.group(1), 16)
            found += 1
    return addrs, found > 0


def parse_u32(output):
    """Extract the last 32-bit hex word from CLI readback text."""
    words = re.findall(r":\s*([0-9A-Fa-f]{8})\b", output)
    if not words:
        raise RuntimeError("no hex word in CLI output:\n" + output[-500:])
    return int(words[-1], 16)


class Bench:
    def __init__(self, cli, sn, addrs):
        self.cli = str(cli)
        self.sn = sn
        self._addrs = addrs

    def _run(self, *args):
        cmd = [self.cli, "-c", f"port=SWD sn={self.sn} mode=HOTPLUG", *args]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        except subprocess.SubprocessError as e:
            raise RuntimeError(f"CLI call failed: {e}")
        return p.stdout + p.stderr

    def read(self, addr):
        return parse_u32(self._run("-r32", f"0x{addr:08X}", "0x4"))

    def write(self, addr, value):
        out = self._run("-w32", f"0x{addr:08X}", f"0x{value:08X}")
        low = out.lower().replace(" ", "")
        if f"w@0x{addr:08x}" not in low:
            raise RuntimeError("write not issued:\n" + out[-500:])
        return out

    def poke(self, addr, value):
        """Write a sticky word and confirm by readback."""
        self.write(addr, value)
        got = self.read(addr)
        if got != value:
            raise RuntimeError(f"readback mismatch @0x{addr:08X}: "
                               f"wrote 0x{value:08X}, read 0x{got:08X}")

    def trigger(self, value, expect, timeout_s=8.0):
        """Write a consumed request word, confirm by state effect.

        The firmware clears request words within milliseconds, so the CLI's
        own download-verify can report failure even when the write landed.
        State is the ground truth.
        """
        self.write(self._addrs["request"], value)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read(self._addrs["state"]) == expect:
                return
            time.sleep(0.5)
        raise RuntimeError(f"request 0x{value:08X} had no effect "
                           f"(state != {expect})")

    def state(self, a=None):
        a = a or self._addrs
        return {k: self.read(v) for k, v in
                (("state", a["state"]), ("result", a["result"]),
                 ("runs", a["run_count"]), ("tx", a["tx_count"]),
                 ("ack", a["tx_ack"]), ("fault", a["first_fault"]))}


def drain_keys():
    """Return bytes currently waiting on stdin (non-blocking)."""
    data = b""
    while select.select([sys.stdin], [], [], 0)[0]:
        chunk = os.read(sys.stdin.fileno(), 64)
        if not chunk:
            break
        data += chunk
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sn", default=DEFAULT_SN)
    ap.add_argument("--cli", default=str(DEFAULT_CLI))
    ap.add_argument("--self-test", action="store_true",
                    help="check key map, parser and address resolution only")
    args = ap.parse_args()

    if args.self_test:
        assert parse_u32("0x2000013C : 00000002") == 2
        assert parse_u32("noise : DEADBEEF trailing") == 0xDEADBEEF
        assert SPEEDS[(1, -1)] == "-100" and SPEEDS[(8, 1)] == "+30"
        assert WHEELS["1"] == (1, "NE") and WHEELS["4"] == (8, "NW")
        addrs, via_nm = resolve_addrs()
        assert addrs["request"] == 0x20000130 and addrs["state"] == 0x2000013C
        # Fake probe: write-then-verify races the firmware, so confirm the
        # Bench verifies sticky words by readback and requests by effect.
        import tempfile as _tf
        with _tf.TemporaryDirectory() as tmp:
            st = pathlib.Path(tmp) / "st"
            st.write_text("0")
            fake = pathlib.Path(tmp) / "fakecli"
            fake.write_text(
                "#!/usr/bin/env python3\n"
                "import pathlib, sys\n"
                f"st = pathlib.Path({str(st)!r})\n"
                "a = sys.argv\n"
                "if '-w32' in a:\n"
                "    i = a.index('-w32'); addr, val = a[i+1], int(a[i+2], 16)\n"
                "    print(f'w @{addr}: 0x{val:08x}\\nDownloading...\\n"
                "[===========] 100%\\nError: Failed to download data!')\n"
                "    if val == 0x41524D32: st.write_text('1')\n"
                "    elif val == 0x52554E32: st.write_text('2')\n"
                "    elif val == 0x53544F50: st.write_text('0')\n"
                "elif '-r32' in a:\n"
                "    print(f\"{a[a.index('-r32')+1]} : {int(st.read_text()):08X}\")\n")
            fake.chmod(0o755)
            b = Bench(str(fake), "sn", dict(addrs))
            b.trigger(ARM2, expect=1, timeout_s=5.0)
            assert b.read(addrs["state"]) == 1
            b.trigger(RUN2, expect=2, timeout_s=5.0)
            b.write(addrs["request"], STOP)
            assert b.read(addrs["state"]) == 0
        print(f"SELF-TEST OK (addrs via {'nm' if via_nm else 'fallback'})")
        return

    if not os.path.exists(args.cli):
        sys.exit(f"CLI not found: {args.cli}")
    addrs, via_nm = resolve_addrs()
    bench = Bench(args.cli, args.sn, addrs)

    wheel, wname = 1, "NE"
    direction = -1
    print(f"addrs via {'nm' if via_nm else 'fallback'}; "
          f"wheel={wname} dir={direction:+d} speed={SPEEDS[(wheel, direction)]} RPM")
    print("Hold SPACE to run, release to stop. 1-4 wheel, f/r direction, s stop, q quit.")
    try:
        s = bench.state(addrs)
        print(f"bench: {STATES.get(s['state'], '?')} result={s['result']} "
              f"runs={s['runs']} tx/ack={s['tx']}/{s['ack']} fault={s['fault']}")
        if s["state"] == 4:
            print("FAULT latched (fault=%d). Reset the Nucleo (normal-mode "
                  "connect) before running." % s["fault"])
        elif s["state"] == 1:
            print("Leftover ARM from an earlier attempt; disarming to IDLE.")
            bench.write(addrs["request"], STOP)
    except RuntimeError as e:
        sys.exit(f"probe read failed: {e}\nIs cable on Nucleo {args.sn}?")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    running = False
    released = False
    stop_sent = 0.0
    last_space = 0.0
    try:
        tty.setraw(fd)
        while True:
            for ch in drain_keys().decode("utf-8", "ignore"):
                if ch == "q":
                    raise KeyboardInterrupt
                elif ch in WHEELS:
                    if running and not released:
                        bench.write(addrs["request"], STOP)
                        released = True
                        stop_sent = time.monotonic()
                    wheel, wname = WHEELS[ch]
                    print(f"\r\nwheel={wname} speed={SPEEDS[(wheel, direction)]} RPM")
                elif ch == "f":
                    direction = 1
                    print(f"\r\ndir=+1 speed={SPEEDS[(wheel, direction)]} RPM")
                elif ch == "r":
                    direction = -1
                    print(f"\r\ndir=-1 speed={SPEEDS[(wheel, direction)]} RPM")
                elif ch == " ":
                    last_space = time.monotonic()
                    released = False
                    if not running:
                        s = bench.state(addrs)
                        if s["state"] == 4:
                            print("\r\nFAULT latched; reset Nucleo first.")
                            continue
                        if s["state"] not in (0,):
                            print(f"\r\nbusy ({STATES.get(s['state'], '?')}); "
                                  "wait for IDLE.")
                            continue
                        print(f"\r\nARM+RUN {wname} {direction:+d} "
                              f"({SPEEDS[(wheel, direction)]} RPM)...")
                        bench.poke(addrs["wheel"], wheel)
                        bench.poke(addrs["direction"], direction & 0xFFFFFFFF)
                        bench.trigger(ARM2, expect=1)
                        bench.trigger(RUN2, expect=2)
                        running = True
                elif ch == "s":
                    if running and not released:
                        bench.write(addrs["request"], STOP)
                        released = True
                        stop_sent = time.monotonic()
                        print("\r\nSTOP sent.")
            if running and not released and time.monotonic() - last_space > 0.8:
                bench.write(addrs["request"], STOP)
                released = True
                stop_sent = time.monotonic()
                print("\r\nspace released -> STOP sent.")
            if running:
                try:
                    s = bench.state(addrs)
                    print(f"\r{STATES.get(s['state'], '?')} res={s['result']} "
                          f"runs={s['runs']} tx/ack={s['tx']}/{s['ack']}   ",
                          end="", flush=True)
                    if s["state"] == 0:
                        running = False
                        released = False
                        print("\r\nback to IDLE.")
                    elif released and time.monotonic() - stop_sent > 4.0:
                        bench.write(addrs["request"], STOP)
                        stop_sent = time.monotonic()
                except RuntimeError as e:
                    print(f"\r\npoll failed: {e}")
                    running = False
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\r\nquit.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if running:
            try:
                bench.write(addrs["request"], STOP)
                print("STOP sent on exit.")
            except RuntimeError:
                print("STOP on exit FAILED; STOP the bench manually.")


if __name__ == "__main__":
    main()
