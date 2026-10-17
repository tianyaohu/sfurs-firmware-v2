#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyocd>=0.36", "opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6"]
# ///
"""Friction feed-forward tuning on the current surface, checked by the overhead camera.

  uv run scripts/tune_ground.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml --out DIR
  uv run scripts/tune_ground.py --no-camera --out DIR        ESC telemetry only

MOTION: the robot drives short out-and-back moves (+vy -vy +vx -vx) and ends near where it
started. Robot on the floor, centred under the camera with tag 0 flat on top, battery on,
an operator ready to switch it off. Every stage aborts on a runaway or stalled wheel.

1. preflight   all four ESCs live, IDLE, no faults, Vbus; camera sees the tag still
2. identify    feed-forward off: per wheel, Iq ref at breakaway (static friction) and
               while tracking (kinetic friction) on this surface; refuse if not repeatable
3. tune        ff_static = frac x breakaway, ff_kinetic = kfrac x kinetic, per wheel;
               lower frac while starts overshoot, raise it while they are slow
4. accept      camera: per move, direction error, sideways drift and heading change,
               baseline (stage 2) against the chosen values

Writes DIR/result.json (values per wheel, all metrics, pass/fail) and one telemetry log
per stage. Nothing is written to the ESCs beyond RAM; defaults are restored after each
stage. Bake accepted values with the per-wheel build afterwards.
"""
import argparse
import json
import math
import pathlib
import statistics
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "rig")]
import esc_tool  # noqa: E402
import ik_link  # noqa: E402

NAMES = [name for name, _esc, _sel in ik_link.WHEELS]


# ---- analysis (pure; tested offline) ----

def segments(log, min_target=40):
    """Starts in a pulse log: [(first index, end index)] for each run of entries with a
    non-zero target, per the Nucleo's slewed targets."""
    out, start = [], None
    for k, e in enumerate(log):
        moving = any(abs(x) >= 1 for x in e["target_rpm"])
        if moving and start is None:
            start = k
        elif not moving and start is not None:
            out.append((start, k))
            start = None
    if start is not None:
        out.append((start, len(log)))
    return [(a, b) for a, b in out if max(abs(x) for e in log[a:b] for x in e["target_rpm"]) >= min_target]


def friction_samples(log, min_target=40, track_tol=0.3):
    """Per wheel, from a feed-forward-off log: breakaway [A] = largest right-signed Iq ref
    up to the first Hall motion the commanded way, one per start; kinetic [A] = median
    right-signed Iq ref while tracking within track_tol in the second half of each move."""
    out = {n: dict(breakaway=[], kinetic=[]) for n in NAMES}
    for a, b in segments(log, min_target):
        seg = log[a:b]
        half = seg[0]["t"] + (seg[-1]["t"] - seg[0]["t"]) / 2
        for w, name in enumerate(NAMES):
            target = max((e["target_rpm"][w] for e in seg), key=abs)
            if abs(target) < min_target:
                continue
            sign = 1 if target > 0 else -1
            peak = 0.0
            for e in seg:
                rpm, iq = e["rpm"][w], e["iq_ref_a"][w]
                if rpm is None or iq is None:
                    continue
                peak = max(peak, sign * iq)
                if sign * rpm >= 1:
                    out[name]["breakaway"].append(peak)
                    break
            track = [sign * e["iq_ref_a"][w] for e in seg if e["t"] >= half and e["rpm"][w] is not None
                     and e["iq_ref_a"][w] is not None and e["target_rpm"][w]
                     and abs(e["rpm"][w] - e["target_rpm"][w]) <= track_tol * abs(e["target_rpm"][w])]
            if track:
                out[name]["kinetic"].append(statistics.median(track))
    return out


def spread(values):
    """Relative spread of repeated measurements: median absolute deviation / median."""
    if len(values) < 2:
        return math.inf
    m = statistics.median(values)
    return statistics.median(abs(v - m) for v in values) / m if m > 0 else math.inf


def overshoot(log, min_target=40, window=0.6):
    """Per wheel: peak right-signed rpm / target in the first `window` s of each start."""
    out = {n: [] for n in NAMES}
    for a, b in segments(log, min_target):
        seg = [e for e in log[a:b] if e["t"] - log[a]["t"] <= window]
        for w, name in enumerate(NAMES):
            target = max((e["target_rpm"][w] for e in log[a:b]), key=abs)
            if abs(target) < min_target:
                continue
            peak = max((e["rpm"][w] * (1 if target > 0 else -1) for e in seg if e["rpm"][w] is not None), default=0)
            out[name].append(peak / abs(target))
    return out


def response(log):
    """Start metrics: per wheel breakaway time median/worst and overshoot median/worst."""
    times = esc_tool.breakaway_summary(esc_tool.breakaways(log))
    over = overshoot(log)
    wheels = {n: dict(moved=times[n]["moved"], starts=times[n]["starts"], breakaway_s=times[n]["median_s"],
                      breakaway_worst_s=times[n]["worst_s"],
                      overshoot=statistics.median(over[n]) if over[n] else None,
                      overshoot_worst=max(over[n]) if over[n] else None) for n in NAMES}
    return dict(wheels=wheels, spread_s=times["spread_median_s"], spread_worst_s=times["spread_worst_s"])


def judge(resp, limits, baseline_overshoot=0.0):
    """(slow, overshooting) for a response: any wheel's median breakaway above the limit
    or not moving every start; any wheel's median overshoot above the limit (or above the
    feed-forward-off baseline, when the speed PI alone already overshoots that much)."""
    w = resp["wheels"].values()
    slow = any(x["moved"] < x["starts"] or x["breakaway_s"] is None or x["breakaway_s"] > limits["breakaway_s"]
               for x in w) or (resp["spread_s"] or 0) > limits["spread_s"]
    over = any((x["overshoot"] or 0) > max(limits["overshoot"], baseline_overshoot + 0.1) for x in w)
    return slow, over


def next_fraction(frac, slow, over, step_down=0.15, step_up=0.1, lo=0.2, hi=1.0):
    """Static feed-forward fraction for the next try; None when done or out of range."""
    if over:
        nxt = frac - step_down
    elif slow:
        nxt = frac + step_up
    else:
        return None
    nxt = round(min(hi, max(lo, nxt)), 3)
    return None if nxt == frac else nxt


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def accept_moves(moves, limits):
    """Camera acceptance over moves [(commanded twist, summary or None)]. The tag's mounting
    yaw is unknown, so the mean (direction measured - commanded) is removed first; each move
    is then judged on its own direction error, sideways drift and heading change, and the
    distance ratios on their spread (absolute scale belongs to the kinematics fit)."""
    seen = [(tw, s) for tw, s in moves if s is not None and s["distance_mm"] > 20]
    if not seen:
        return dict(ok=False, reason="no move seen still at both ends", moves=[])
    diffs = [math.radians(s["direction_deg"] - math.degrees(math.atan2(tw[1], tw[0]))) for tw, s in seen]
    mount = math.degrees(math.atan2(sum(map(math.sin, diffs)), sum(map(math.cos, diffs))))
    rows = []
    for tw, s in seen:
        cmd_mm = s.get("commanded_mm") or math.hypot(tw[0], tw[1])
        rows.append(dict(twist=list(tw), distance_mm=round(s["distance_mm"], 1),
                         ratio=round(s["distance_mm"] / cmd_mm, 3),
                         direction_error_deg=round(wrap_deg(s["direction_deg"] - math.degrees(
                             math.atan2(tw[1], tw[0])) - mount), 2),
                         sideways_mm=round(s["max_sideways_mm"], 1),
                         heading_change_deg=round(s["heading_change_deg"], 2)))
    ratios = [r["ratio"] for r in rows]
    worst = dict(direction_error_deg=max(abs(r["direction_error_deg"]) for r in rows),
                 heading_change_deg=max(abs(r["heading_change_deg"]) for r in rows),
                 sideways_per_m=max(r["sideways_mm"] / r["distance_mm"] * 1000 for r in rows),
                 ratio_spread=(max(ratios) - min(ratios)) / statistics.mean(ratios))
    ok = (worst["direction_error_deg"] <= limits["direction_deg"] and
          worst["heading_change_deg"] <= limits["heading_deg"] and
          worst["ratio_spread"] <= limits["ratio_spread"])
    return dict(ok=ok, mount_yaw_deg=round(mount, 2), worst={k: round(v, 3) for k, v in worst.items()},
                missing=len(moves) - len(seen), moves=rows)


# ---- hardware ----

def preflight(link, min_vbus):
    before = link.rows()
    time.sleep(0.3)
    rows = link.rows()
    problems = []
    for name, r0, r in zip(NAMES, before, rows):
        if r is None or r0 is None or r["status_count"] == r0["status_count"]:
            problems.append(f"{name}: no live reply")
        elif r["mc_state"] != "IDLE" or r["fault_names"]:
            problems.append(f"{name}: {r['mc_state']} faults {','.join(r['fault_names']) or '-'} (esc_tool.py stop)")
        elif r["vbus_v"] < min_vbus:
            problems.append(f"{name}: Vbus {r['vbus_v']:.2f} V < {min_vbus} V")
    return problems


def camera_moves(stream, t0, plan):
    from tag_stream import move_summary
    frames = stream.snapshot()
    out = []
    for k, (start, end, tw) in enumerate(plan):
        nxt = plan[k + 1][0] if k + 1 < len(plan) else end + 1.0
        s = move_summary(frames, (t0 + start - 0.4, t0 + start - 0.02), (t0 + nxt - 0.4, t0 + nxt - 0.02))
        if s is not None:
            s["commanded_mm"] = math.hypot(tw[0], tw[1]) * (end - start)
        out.append((tw, s))
    return out


def stage(link, stream, name, plan, duration, settings, defaults, outdir):
    print(f"-- {name}: " + " ".join(f"{n} {s['ff_static_ma']}/{s['ff_kinetic_ma']}" for n, s in zip(NAMES, settings)),
          flush=True)
    log, aborted, t0 = esc_tool.run_pulse(link, plan, duration, settings, defaults)
    (outdir / f"{name}.jsonl").write_text("".join(json.dumps(e) + "\n" for e in log))
    if stream is not None:
        time.sleep(0.2)
    moves = camera_moves(stream, t0, plan) if stream is not None else None
    faults = sorted({f for e in log for fs in e["faults"] if fs for f in fs})
    vbus = [x for e in log for x in e["vbus_v"] if x]
    return dict(name=name, settings=settings, aborted=aborted, faults=faults,
                vbus_min=min(vbus) if vbus else None, log=log, moves=moves)


def show(resp):
    ms = lambda x: "-" if x is None else f"{x * 1000:.0f}"
    for n, w in resp["wheels"].items():
        print(f"   {n}: moved {w['moved']}/{w['starts']}  breakaway {ms(w['breakaway_s'])} ms "
              f"(worst {ms(w['breakaway_worst_s'])})  overshoot x{w['overshoot'] or 0:.2f} "
              f"(worst x{w['overshoot_worst'] or 0:.2f})")
    print(f"   spread between wheels {ms(resp['spread_s'])} ms (worst {ms(resp['spread_worst_s'])})")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", type=pathlib.Path, required=True, help="directory for result.json and logs")
    p.add_argument("--calibration", type=pathlib.Path)
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--exposure", type=float, default=100)
    p.add_argument("--tag-mm", type=float, default=100)
    p.add_argument("--speed", type=float, default=0.2, help="m/s per move (>= 0.2: wheel targets >= 40 rpm)")
    p.add_argument("--seconds", type=float, default=0.8, help="per move")
    p.add_argument("--reps", type=int, default=2, help="(+vy -vy +vx -vx) cycles per stage")
    p.add_argument("--static-frac", type=float, default=0.8)
    p.add_argument("--kinetic-frac", type=float, default=0.8)
    p.add_argument("--max-tries", type=int, default=4)
    p.add_argument("--max-spread", type=float, default=0.2, help="breakaway MAD/median allowed")
    p.add_argument("--breakaway-ms", type=float, default=80)
    p.add_argument("--spread-ms", type=float, default=30)
    p.add_argument("--overshoot", type=float, default=1.3, help="peak/target allowed")
    p.add_argument("--min-vbus", type=float, default=11.0)
    p.add_argument("--elf", type=pathlib.Path, default=ik_link.DEFAULT_ELF)
    p.add_argument("--sn", default=ik_link.DEFAULT_SN)
    args = p.parse_args()
    if not 0 < args.speed <= 0.3 or not 0 < args.seconds <= 2 or not 1 <= args.reps <= 5:
        raise SystemExit("0 < speed <= 0.3 m/s, 0 < seconds <= 2, 1 <= reps <= 5")
    if args.no_camera == bool(args.calibration):
        raise SystemExit("give --calibration, or --no-camera")
    limits = dict(breakaway_s=args.breakaway_ms / 1000, spread_s=args.spread_ms / 1000, overshoot=args.overshoot,
                  direction_deg=3.0, heading_deg=2.0, ratio_spread=0.05)
    args.out.mkdir(parents=True, exist_ok=True)
    v = round(args.speed * 1000)
    twists = [(0, v, 0), (0, -v, 0), (v, 0, 0), (-v, 0, 0)] * args.reps
    plan, duration = esc_tool.pulse_plan(twists, args.seconds)
    defaults = esc_tool.firmware_defaults()
    result = dict(date=time.strftime("%Y-%m-%d %H:%M:%S"), args={k: str(x) for k, x in vars(args).items()},
                  limits=limits, stages=[])

    def save():
        (args.out / "result.json").write_text(json.dumps(result, indent=1))

    stream = None
    session, link = ik_link.Link.open(args.sn, args.elf)
    try:
        problems = preflight(link, args.min_vbus)
        if not args.no_camera:
            from tag_stream import TagStream, still_pose
            stream = TagStream(args.calibration, args.camera, args.exposure, tag_mm=args.tag_mm).start()
            time.sleep(2.0)
            frames = stream.snapshot()
            now = time.monotonic()
            recent = [f for f in frames if f[0] > now - 1.0]
            if still_pose(frames, now - 1.0, now) is None:
                problems.append("camera: tag 0 not seen still (in view? flat? lit?)")
            elif not all(r for _t, pose, r in recent if pose is not None):
                problems.append("camera: tag outside the calibrated centre; move the robot under the camera")
        if problems:
            raise SystemExit("preflight failed:\n  " + "\n  ".join(problems))

        base = {**defaults, "ff_static_ma": 0, "ff_kinetic_ma": 0}
        s = stage(link, stream, "identify", plan, duration, [dict(base)] * 4, defaults, args.out)
        resp = response(s["log"])
        fric = friction_samples(s["log"])
        s.pop("log")
        s["response"], s["friction"] = resp, fric
        result["stages"].append(s)
        save()
        print("   feed-forward off:")
        show(resp)
        if s["aborted"] or s["faults"]:
            raise SystemExit(f"identify: aborted {s['aborted']} faults {s['faults']}; not tuning")
        static, kinetic = {}, {}
        for n in NAMES:
            b, k = fric[n]["breakaway"], fric[n]["kinetic"]
            print(f"   {n}: breakaway {statistics.median(b) if b else 0:.2f} A (n {len(b)}, spread {spread(b):.0%})"
                  f"  kinetic {statistics.median(k) if k else 0:.2f} A (n {len(k)})")
            if spread(b) > args.max_spread:
                raise SystemExit(f"identify: {n} breakaway not repeatable (spread {spread(b):.0%} > "
                                 f"{args.max_spread:.0%}); check the wheel and surface before tuning")
            static[n], kinetic[n] = statistics.median(b), statistics.median(k) if k else 0.0
        baseline_over = max(w["overshoot"] or 0 for w in resp["wheels"].values())

        frac, best = args.static_frac, None
        for attempt in range(1, args.max_tries + 1):
            settings = [{**defaults, "ff_static_ma": round(1000 * frac * static[n]),
                         "ff_kinetic_ma": round(1000 * args.kinetic_frac * kinetic[n])} for n in NAMES]
            s = stage(link, stream, f"tune{attempt}", plan, duration, settings, defaults, args.out)
            resp = response(s["log"])
            s.pop("log")
            slow, over = judge(resp, limits, baseline_over)
            s.update(response=resp, static_frac=frac, slow=slow, overshooting=over)
            result["stages"].append(s)
            save()
            print(f"   static x{frac:.2f}: {'slow ' if slow else ''}{'overshooting ' if over else ''}"
                  f"{'ok' if not (slow or over or s['aborted']) else ''}{' ABORTED ' + s['aborted'] if s['aborted'] else ''}")
            show(resp)
            if s["faults"]:
                raise SystemExit(f"{s['name']}: faults {s['faults']}; stopping")
            if not (slow or over or s["aborted"]):
                best = s
                break
            frac = next_fraction(frac, slow, over or bool(s["aborted"]))
            if frac is None:
                break

        result["chosen"] = None if best is None else dict(
            static_frac=best["static_frac"], kinetic_frac=args.kinetic_frac,
            wheels={n: dict(ff_static_ma=st["ff_static_ma"], ff_kinetic_ma=st["ff_kinetic_ma"])
                    for n, st in zip(NAMES, best["settings"])})
        if stream is not None:
            result["camera"] = {st["name"]: accept_moves(st["moves"], limits) for st in result["stages"]
                                if st.get("moves") is not None}
            for st in result["stages"]:
                st["moves"] = [(list(tw), m) for tw, m in st["moves"] or []]
        save()
    finally:
        session.close()
        if stream is not None:
            stream.stop()

    print("== result")
    if result["chosen"] is None:
        print("   no setting met the limits; see result.json")
    else:
        print(f"   static x{result['chosen']['static_frac']:.2f} of breakaway, kinetic "
              f"x{args.kinetic_frac:.2f}: " + "  ".join(f"{n} {w['ff_static_ma']}/{w['ff_kinetic_ma']} mA"
                                                     for n, w in result["chosen"]["wheels"].items()))
    for name, cam in (result.get("camera") or {}).items():
        print(f"   camera {name}: {'PASS' if cam['ok'] else 'FAIL'} {cam.get('reason', '')}"
              f" worst {cam.get('worst')} (mount yaw {cam.get('mount_yaw_deg')} deg, missing {cam.get('missing')})")
    print(f"   {args.out / 'result.json'}")


if __name__ == "__main__":
    main()
