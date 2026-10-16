#!/usr/bin/env python3
"""Keyboard omni drive over the Nucleo streaming IK image.

W/S forward/back, A/D left/right, Q/E rotate CCW/CW. Keys combine: the
latest press of each axis is held for --hold seconds (curses has no key-up
events; key auto-repeat keeps it alive). SPACE zeroes everything at once.
+/- change the speed. X or ESC quits (decelerate, then STOP).

The host streams setpoints at 50 Hz. If this program dies, the Nucleo
decelerates and stops after 0.3 s without a heartbeat, and each ESC stops
itself 0.25 s after the Nucleo goes quiet.
"""
import argparse
import curses
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import ik_link  # noqa: E402

PERIOD = 0.02
AXES = {"w": ("vy", 1), "s": ("vy", -1), "a": ("vx", -1), "d": ("vx", 1),
        "q": ("omega", 1), "e": ("omega", -1)}


class Keys:
    """Per-axis held direction with a release timeout."""

    def __init__(self, hold):
        self.hold = hold
        self.axes = {}  # axis -> (sign, expiry)

    def press(self, key, now):
        if key == " ":
            self.axes.clear()
        elif key in AXES:
            axis, sign = AXES[key]
            self.axes[axis] = (sign, now + self.hold)

    def vector(self, now, speed, yaw):
        self.axes = {a: v for a, v in self.axes.items() if v[1] > now}
        get = lambda axis: self.axes.get(axis, (0, 0))[0]
        return (round(get("vx") * speed * 1000), round(get("vy") * speed * 1000),
                round(get("omega") * yaw * 1000))


def wheel_lines(rows, status):
    lines = ["wheel  cmd/target rpm   rpm    Iq A  Iqref A  state            stop     "
             "Vbus   I2T  faults  hall  align"]
    for i, (name, esc, _) in enumerate(ik_link.WHEELS):
        r = rows[i]
        cmd = f"{status['rpm'][i]:5d}/{status['target_rpm'][i]:<5d}"
        if r is None or not r["status_count"]:
            lines.append(f"{name}/{esc}  {cmd}      no reply")
            continue
        age = (status["telem_now"] - r["status_tick"]) & 0xFFFFFFFF
        stale = " STALE" if age > 200 else ""
        flags = (("A" if r["active"] else "-") + ("D" if r["derated"] else "-") +
                 ("L" if r["aligning"] else "-") + ("H" if r["aligned"] else "!"))
        lines.append(f"{name}/{esc}  {cmd}  {r['rpm']:6d}  {r['iq_a']:6.2f}  {r['iq_ref_a']:6.2f}  "
                     f"{r['mc_state']:<10s} {flags}  {r['stop_reason']:<7s}  {r['vbus_v']:5.2f}  "
                     f"{r['i2t_percent']:3d}%  {','.join(r['fault_names']) or '-':<6s}  "
                     f"{r['hall_shift_deg']:4d}  {r['align']}{stale}")
    return lines


def run(screen, link, args, log):
    curses.curs_set(0)
    screen.nodelay(True)
    keys = Keys(args.hold)
    speed, yaw = args.speed, args.yaw
    next_tick = time.monotonic()
    rows, status, last_ui = [None] * 4, link.status(), 0.0
    quitting, quit_deadline = False, 0.0
    while True:
        now = time.monotonic()
        while (ch := screen.getch()) != -1:
            key = chr(ch).lower() if 0 <= ch < 256 else ""
            if key in ("x", "\x1b"):
                quitting, quit_deadline = True, now + 3.0
            elif key == "+":
                speed = min(args.max_speed, round(speed + 0.1, 2))
            elif key == "-":
                speed = max(0.1, round(speed - 0.1, 2))
            elif not quitting:
                keys.press(key, now)
        vector = (0, 0, 0) if quitting else keys.vector(now, speed, yaw)
        link.drive(*vector, enable=not quitting)
        if now - last_ui >= 0.1:
            last_ui = now
            status, rows = link.status(), link.rows()
            if log:
                log.write(json.dumps(dict(t=round(now, 3), vector=vector, status=status,
                                          rows=rows)) + "\n")
            lines = [f"speed {speed:.1f} m/s  yaw {yaw:.1f} rad/s   twist vx={vector[0]} "
                     f"vy={vector[1]} w={vector[2]}   nucleo {status['state_name']}  "
                     f"tx {status['tx_count']} fail {status['tx_fail']}",
                     "WASD move, QE rotate, SPACE zero, +/- speed, X quit", ""]
            lines += wheel_lines(rows, status)
            lines += ["", "flags: A active, D I2T derated, L aligning, H Hall shift known "
                      "(! = not aligned: wheel refuses to drive; run esc_tool.py align)"]
            if quitting:
                lines.append("stopping...")
            screen.erase()
            h, w = screen.getmaxyx()
            for y, line in enumerate(lines[:h - 1]):
                screen.addnstr(y, 0, line, w - 1)
            screen.refresh()
            if quitting and (status["state"] == 0 or now > quit_deadline):
                return
        next_tick += PERIOD
        time.sleep(max(0.0, next_tick - time.monotonic()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", type=pathlib.Path, default=ik_link.DEFAULT_ELF)
    parser.add_argument("--sn", default=ik_link.DEFAULT_SN)
    parser.add_argument("--speed", type=float, default=0.5, help="translation m/s (default 0.5)")
    parser.add_argument("--max-speed", type=float, default=2.5)
    parser.add_argument("--yaw", type=float, default=2.0, help="rotation rad/s (default 2.0)")
    parser.add_argument("--hold", type=float, default=0.6,
                        help="seconds a key stays held without repeats (default 0.6)")
    parser.add_argument("--accel", type=int, default=0,
                        help="wheel acceleration limit RPM/s (0 = firmware default 900)")
    parser.add_argument("--log", type=pathlib.Path, help="JSONL telemetry log")
    args = parser.parse_args(argv)
    session, link = ik_link.Link.open(args.sn, args.elf)
    log = open(args.log, "x") if args.log else None
    try:
        link.write("accel_rpmps", args.accel)
        curses.wrapper(run, link, args, log)
    finally:
        try:  # stop streaming; the Nucleo brakes and STOPs after the heartbeat ages out
            link.write("enable", 0)
        finally:
            session.close()
            if log:
                log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
