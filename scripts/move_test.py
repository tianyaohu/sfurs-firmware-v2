#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyocd>=0.36", "opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6"]
# ///
"""Straight-line drive test, measured by the overhead camera.

  uv run scripts/move_test.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml --out DIR
      [--axis vy] [--distance 0.6] [--speed 0.2] [--reps 1] [--kp N --ki N] [--ff MA]

MOTION: drives `distance` along +axis, stops, then the same back (vy = forward, vx =
right); reps repeat the pair. Place the robot so the whole path stays in view, starting
`distance`/2 behind the image centre along the axis. Each move is judged on the camera:
distance vs commanded (the +-10 cm goal over 1 m), sideways drift and heading change.
Writes DIR/result.json, DIR/drive.jsonl and DIR/drive.camera.jsonl.
"""
import argparse
import json
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "rig")]
import esc_tool  # noqa: E402
import ik_link  # noqa: E402
import tune_ground as tg  # noqa: E402


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
    p.add_argument("--min-vbus", type=float, default=11.0)
    p.add_argument("--elf", type=pathlib.Path, default=ik_link.DEFAULT_ELF)
    p.add_argument("--sn", default=ik_link.DEFAULT_SN)
    args = p.parse_args()
    seconds = args.distance / args.speed if args.speed else 0
    if not 0 < args.speed <= 0.3 or not 0 < args.distance <= 1.0 or not 1 <= args.reps <= 3 or seconds > 5:
        raise SystemExit("0 < speed <= 0.3 m/s, 0 < distance <= 1 m, 1 <= reps <= 3, distance/speed <= 5 s")
    if args.no_camera == bool(args.calibration):
        raise SystemExit("give --calibration, or --no-camera")

    defaults = esc_tool.firmware_defaults()
    used = {**defaults, **{k: v for k, v in dict(kp=args.kp, ki=args.ki, ff_static_ma=args.ff,
                                                  ff_kinetic_ma=args.ff).items() if v is not None}}
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
            from tag_stream import TagStream, still_pose
            stream = TagStream(args.calibration, args.camera, args.exposure, tag_height_mm=args.tag_height_mm).start()
            time.sleep(1.0)
            now = time.monotonic()
            print(f"camera: {stream.mode} mode ({stream.floor_seen} floor tags), tag height "
                  f"{'-' if stream.height_mm is None else f'{stream.height_mm:.0f} mm'}", flush=True)
            result["camera"] = dict(mode=stream.mode, floor_tags=stream.floor_seen, tag_height_mm=stream.height_mm)
            if still_pose(stream.snapshot(), now - 1.0, now) is None:
                problems.append("camera: tag 0 not seen still")
        if problems:
            raise SystemExit("preflight failed:\n  " + "\n  ".join(problems))
        print(f"-- {args.distance:.2f} m {args.axis} at {args.speed} m/s ({seconds:.1f} s), kp {used['kp']} "
              f"ki {used['ki']} ff {used['ff_static_ma']}/{used['ff_kinetic_ma']} mA", flush=True)
        s = tg.stage(link, stream, "drive", plan, duration, [used] * 4, defaults, args.out)
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
