#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Plan or execute oblique out-and-back pulses through esc_tool's IK link.

Default: print the plan, no hardware access. --execute drives the robot!
Battery ON, wheels raised for the first run after firmware changes; otherwise
use a clear floor with an operator ready to stop. Read system-info/hardware.md first.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ik_link


def make_plan(angles, speeds, seconds, pause):
    if not angles or not speeds or not all(math.isfinite(x) for x in (*angles, *speeds, seconds, pause)):
        raise ValueError("plan values must be finite and nonempty")
    if not all(0 < s <= 0.3 for s in speeds) or not 0 < seconds <= 3 or pause < 0.5:
        raise ValueError("require speeds 0..0.3 m/s, seconds 0..3, pause >=0.5 s")
    plan = [dict(duration_s=pause, twist=[0, 0, 0])]
    for angle in angles:
        for speed in speeds:
            vx, vy = (round(1000 * speed * f(math.radians(angle))) for f in (math.cos, math.sin))
            for sign in (1, -1):
                plan.append(dict(duration_s=seconds, twist=[sign * vx, sign * vy, 0]))
                plan.append(dict(duration_s=pause, twist=[0, 0, 0]))
    return plan


def execute(args, plan):
    session, link = ik_link.Link.open(args.sn, args.elf)
    try:
        if link.read("state") != 0:
            raise RuntimeError("drive must be stopped before the sweep")
        rows = link.rows()
        if any(r is None or r["mc_state"] != "IDLE" or r["faults"] or not r["aligned"] for r in rows):
            raise RuntimeError("all four wheels must report IDLE, aligned, without faults")
        with args.out.open("w") as output:
            last_count = [None] * 4
            last_seen = [time.monotonic()] * 4
            for step in plan:
                deadline = time.monotonic() + step["duration_s"]
                while time.monotonic() < deadline:
                    tick = time.monotonic()
                    link.drive(*step["twist"])
                    status, rows = link.status(), link.rows()
                    if any(r and (r["faults"] or r["derated"]) for r in rows):
                        raise RuntimeError("wheel fault/derating; sweep stopped")
                    if any(r is None for r in rows):
                        raise RuntimeError("incomplete telemetry; sweep stopped")
                    for i, r in enumerate(rows):
                        if r["status_count"] != last_count[i]:
                            last_count[i], last_seen[i] = r["status_count"], time.monotonic()
                        if time.monotonic() - last_seen[i] > 0.2:
                            raise RuntimeError(f"stale {ik_link.WHEELS[i][0]} telemetry; sweep stopped")
                    row = dict(t=time.monotonic(), twist=step["twist"], target_rpm=status["target_rpm"],
                               rows=rows)
                    output.write(json.dumps(row, allow_nan=False) + "\n")
                    output.flush()
                    time.sleep(max(0, tick + 0.02 - time.monotonic()))
    finally:
        try:
            link.write("enable", 0)
            deadline = time.monotonic() + 3
            while link.read("state") != 0 and time.monotonic() < deadline:
                time.sleep(0.05)
            print("drive state:", ik_link.STATES.get(link.read("state"), "unknown"))
        finally:
            session.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--angles-deg", type=float, nargs="+", default=[11.25 + 22.5 * i for i in range(8)],
                   help="body directions; each also runs its opposite")
    p.add_argument("--speeds", type=float, nargs="+", default=[0.1, 0.2, 0.3], help="m/s")
    p.add_argument("--seconds", type=float, default=1.5)
    p.add_argument("--pause", type=float, default=1)
    p.add_argument("--out", type=Path)
    p.add_argument("--execute", action="store_true", help="connect probe and drive the robot")
    p.add_argument("--elf", type=Path, default=ik_link.DEFAULT_ELF)
    p.add_argument("--sn", default=ik_link.DEFAULT_SN)
    args = p.parse_args()
    plan = make_plan(args.angles_deg, args.speeds, args.seconds, args.pause)
    if not args.execute:
        print(json.dumps(dict(duration_s=sum(s["duration_s"] for s in plan), plan=plan), indent=2))
        return
    if args.out is None:
        p.error("--execute requires --out")
    execute(args, plan)


if __name__ == "__main__":
    main()
