#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyocd>=0.36", "opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6"]
# ///
"""Straight-line drive test, measured by the overhead camera.

  uv run scripts/move_test.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml --out DIR
      [--axis vy] [--distance 0.6] [--speed 0.2] [--reps 1] [--kp N --ki N] [--ff MA | --ff-static MA --ff-kinetic MA]
      [--dsc [--dsc-ff-run MA --dsc-cap MA --dsc-slope MA_PER_MS --dsc-pcap MA]] [--record [--record-pre N --record-delay MS]]

MOTION: drives `distance` along +axis, stops, then the same back (vy = forward, vx =
right); reps repeat the pair. Place the robot so the whole path stays in view, starting
`distance`/2 behind the image centre along the axis. Each move is judged on the camera:
distance vs commanded (the +-10 cm goal over 1 m), sideways drift and heading change.
Before driving, the planned end points (with --length-margin and +-cone-deg slack) must lie
inside the floor tags' polygon (the mat), predicted from the tag pose and --forward-deg.
--dsc switches the ESCs to the startup speed loop (ramp to the first Hall edge, then
running ff + PI; notes/ff-design.md) for this run; --record arms each ESC's 1 kHz speed-loop
recorder, which captures the first start (384 ms, --record-pre ms before it, or a window
--record-delay ms later) and is read back after the drive into DIR/record.json.
Writes DIR/result.json, DIR/drive.jsonl and DIR/drive.camera.jsonl.
"""
import argparse
import json
import math
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "rig")]
import esc_tool  # noqa: E402
import ik_link  # noqa: E402
import tune_ground as tg  # noqa: E402


def path_outside(start, stream, args, inside_floor):
    """Planned end points that fall outside the floor tags' polygon, with slack."""
    t, R = start
    x, y = R[:, 0], R[:, 1]  # tag axes, in the floor plane (floor-plane poses)
    axis_deg = args.forward_deg - (90.0 if args.axis == "vx" else 0.0)  # vx = right = forward - 90
    bad = []
    for sign, name in ((1, "out"), (-1, "back")):
        for cone in (-args.cone_deg, 0.0, args.cone_deg):
            a = math.radians(axis_deg + cone)
            end = t + 1000 * args.distance * args.length_margin * (math.cos(a) * x + math.sin(a) * y)
            if sign < 0:  # coming back from the nominal far end can overshoot past the start
                far = t + 1000 * args.distance * (math.cos(math.radians(axis_deg)) * x +
                                                  math.sin(math.radians(axis_deg)) * y)
                end = far - (end - t)
            if not inside_floor(end, stream.floor_centers, stream.basis):
                bad.append(f"{name} end at {cone:+.0f} deg")
    return bad


def record_summary(info, rows):
    """Hand-off time and current from one wheel's decoded recording. Only an
    observed STARTING -> RUNNING step counts: a delayed window starts with a
    stale "edge" reason and holds no breakaway current."""
    handoff = next((k for k in range(1, len(rows)) if rows[k - 1]["state"] == "starting"
                    and rows[k]["state"] == "running" and rows[k]["reason"] == "edge"), None)
    return dict(samples=info["samples"], trigger=info["trigger_index"],
                handoff_ms=None if handoff is None else handoff - info["trigger_index"],
                handoff_iq_a=None if handoff is None else round(rows[handoff - 1]["iq_ref_a"], 2),
                peak_rpm=max((abs(r["rpm"]) for r in rows), default=0))


def read_records(link, path):
    """Read every wheel's speed-loop recorder into `path`; returns a summary."""
    out, summary = {}, {}
    for name, _esc, sel in ik_link.WHEELS:
        try:
            info, samples = link.record_read(sel)
        except RuntimeError as e:
            summary[name] = str(e)
            continue
        rows = [ik_link.unpack_record(x) for x in samples]
        out[name] = dict(info=info, samples=rows)
        summary[name] = record_summary(info, rows)
        print(f"   record {name}: {summary[name]}")
    path.write_text(json.dumps(out))
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", type=pathlib.Path, required=True)
    p.add_argument("--calibration", type=pathlib.Path)
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--exposure", type=float, default=100)
    p.add_argument("--tag-height-mm", type=float, help="robot tag above the floor tags (default: estimated)")
    p.add_argument("--axis", choices=["vx", "vy"], default="vy", help="vy = forward, vx = right")
    p.add_argument("--distance", type=float, default=0.6, help="m per move")
    p.add_argument("--speed", type=float, default=0.2, help="m/s")
    p.add_argument("--reps", type=int, default=1, help="out-and-back pairs")
    p.add_argument("--pause", type=float, default=1.5)
    p.add_argument("--kp", type=int)
    p.add_argument("--ki", type=int)
    p.add_argument("--ff", type=int, help="friction feed-forward mA, static = kinetic, all wheels")
    p.add_argument("--ff-static", type=int, help="start (breakaway) feed-forward mA, all wheels; overrides --ff")
    p.add_argument("--ff-kinetic", type=int, help="running feed-forward mA, all wheels; overrides --ff")
    p.add_argument("--dsc", action="store_true", help="startup speed loop on all wheels")
    p.add_argument("--dsc-ff-run", type=int, help="running ff mA (default: firmware)")
    p.add_argument("--dsc-cap", type=int, help="startup ramp ceiling mA")
    p.add_argument("--dsc-slope", type=int, help="startup ramp mA per ms")
    p.add_argument("--dsc-pcap", type=int, help="positive-P cap after the hand-off, mA")
    p.add_argument("--record", action="store_true", help="arm the ESC speed-loop recorders")
    p.add_argument("--record-pre", type=int, default=50, help="recorder samples before the start")
    p.add_argument("--record-delay", type=int, default=0, help="recorder trigger delay ms")
    p.add_argument("--forward-deg", type=float, default=108.0,
                   help="robot +vy from the tag's x-axis toward its y-axis (measured 2026-10-17 run A)")
    p.add_argument("--length-margin", type=float, default=1.2, help="path check: overshoot factor")
    p.add_argument("--cone-deg", type=float, default=20.0, help="path check: heading slack")
    p.add_argument("--min-vbus", type=float, default=11.0)
    p.add_argument("--check", action="store_true", help="preflight and path check only, no motion")
    p.add_argument("--elf", type=pathlib.Path, default=ik_link.DEFAULT_ELF)
    p.add_argument("--sn", default=ik_link.DEFAULT_SN)
    args = p.parse_args()
    seconds = args.distance / args.speed if args.speed else 0
    if not 0 < args.speed <= 0.3 or not 0 < args.distance <= 1.0 or not 1 <= args.reps <= 3 or seconds > 5:
        raise SystemExit("0 < speed <= 0.3 m/s, 0 < distance <= 1 m, 1 <= reps <= 3, distance/speed <= 5 s")
    if args.no_camera == bool(args.calibration):
        raise SystemExit("give --calibration, or --no-camera")

    defaults = esc_tool.firmware_defaults()
    used = {**defaults, **{k: v for k, v in dict(
        kp=args.kp, ki=args.ki,
        ff_static_ma=args.ff if args.ff_static is None else args.ff_static,
        ff_kinetic_ma=args.ff if args.ff_kinetic is None else args.ff_kinetic,
        dsc_enable=1 if args.dsc else None, dsc_ff_run_ma=args.dsc_ff_run, dsc_start_cap_ma=args.dsc_cap,
        dsc_start_slope_ma=args.dsc_slope, dsc_pcap_ma=args.dsc_pcap).items() if v is not None}}
    if not args.dsc and any(x is not None for x in (args.dsc_ff_run, args.dsc_cap, args.dsc_slope, args.dsc_pcap)):
        raise SystemExit("--dsc-* options need --dsc")
    v = round(args.speed * 1000)
    out = (v, 0, 0) if args.axis == "vx" else (0, v, 0)
    back = tuple(-x for x in out)
    plan, duration = esc_tool.pulse_plan([out, back] * args.reps, seconds, pause=args.pause)
    args.out.mkdir(parents=True, exist_ok=True)
    result = dict(date=time.strftime("%Y-%m-%d %H:%M:%S"), args={k: str(x) for k, x in vars(args).items()},
                  settings=used)

    stream = None
    session, link = ik_link.Link.open(args.sn, args.elf)
    try:
        problems = tg.preflight(link, args.min_vbus)
        if not args.no_camera:
            from tag_stream import TagStream, inside_floor, still_pose
            stream = TagStream(args.calibration, args.camera, args.exposure, tag_height_mm=args.tag_height_mm).start()
            time.sleep(1.0)
            now = time.monotonic()
            print(f"camera: {stream.mode} mode ({stream.floor_seen} floor tags), tag height "
                  f"{'-' if stream.height_mm is None else f'{stream.height_mm:.0f} mm'}", flush=True)
            result["camera"] = dict(mode=stream.mode, floor_tags=stream.floor_seen, tag_height_mm=stream.height_mm)
            start = still_pose(stream.snapshot(), now - 1.0, now)
            if start is None:
                problems.append("camera: tag 0 not seen still")
            elif stream.basis is None:
                problems.append("camera: no floor plane (>= 3 floor tags), cannot check the path")
            else:
                outside = path_outside(start, stream, args, inside_floor)
                result["path_check"] = outside or "ok"
                if outside:
                    problems.append("path leaves the mat: " + "; ".join(outside))
        if problems:
            raise SystemExit("preflight failed:\n  " + "\n  ".join(problems))
        if args.check:
            print("preflight and path check passed (no motion: --check)")
            return
        loop = (f"startup loop ff_run {used['dsc_ff_run_ma']} cap {used['dsc_start_cap_ma']} "
                f"slope {used['dsc_start_slope_ma']} mA/ms pcap {used['dsc_pcap_ma']}" if args.dsc else
                f"ff {used['ff_static_ma']}/{used['ff_kinetic_ma']} mA")
        print(f"-- {args.distance:.2f} m {args.axis} at {args.speed} m/s ({seconds:.1f} s), kp {used['kp']} "
              f"ki {used['ki']} {loop}", flush=True)
        if args.record:
            for _name, _esc, sel in ik_link.WHEELS:
                link.record_start(sel, args.record_pre, args.record_delay)
        s = tg.stage(link, stream, "drive", plan, duration, [used] * 4, defaults, args.out)
        if args.record:
            result["record"] = read_records(link, args.out / "record.json")
    finally:
        session.close()
        if stream is not None:
            stream.stop()

    resp = tg.response(s["log"])
    s.pop("log")
    result.update(aborted=s["aborted"], faults=s["faults"], vbus_min=s["vbus_min"], response=resp)
    print(f"   aborted {s['aborted']}  faults {s['faults'] or '-'}  Vbus min {s['vbus_min']}")
    tg.show(resp)
    rows = []
    for tw, m in s["moves"] or []:
        if m is None:
            rows.append(dict(twist=list(tw), seen=False))
            print(f"   move {tw}: not seen still at both ends")
            continue
        err = m["distance_mm"] - m["commanded_mm"]
        rows.append(dict(twist=list(tw), seen=True, error_mm=round(err, 1), **{k: m[k] for k in (
            "distance_mm", "commanded_mm", "max_sideways_mm", "heading_change_deg", "direction_deg",
            "frames", "frames_outside_reliable")}))
        print(f"   move {tw}: {m['distance_mm']:.0f} mm of {m['commanded_mm']:.0f} ({err:+.0f}, "
              f"{err / m['commanded_mm']:+.1%})  sideways {m['max_sideways_mm']:.0f} mm  "
              f"heading {m['heading_change_deg']:+.1f} deg  dir {m['direction_deg']:+.0f} deg  "
              f"outside centre {m['frames_outside_reliable']}/{m['frames']} frames")
    result["moves"] = rows
    (args.out / "result.json").write_text(json.dumps(result, indent=1))
    print(f"   {args.out / 'result.json'}")


if __name__ == "__main__":
    main()
