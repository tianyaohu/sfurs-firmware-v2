#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
# ///
"""Align vision/ESC logs, measure tracking and slip, fit per-wheel Twente friction.

  uv run src/tools/rig/rig_analyze.py --rig rig.jsonl --esc pulse.jsonl \
      --telemetry-origin-t 12345.0 --estimate-latency --out /tmp/analysis

Positive latency means vision arrives late: compare vision(t) to ESC(t-latency).
Units: body mm/s, rad/s; motor RPM and signed motor current A, FR BR BL FL.
"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ik_link import COUNTS_PER_AMP, WHEELS

ROOT = Path(__file__).resolve().parents[3]
NAMES = [w[0] for w in WHEELS]


def geometry_matrix():
    # ik_link defines wire order; the actual geometry lives in the firmware.
    source = (ROOT / "src/shared/src/rover_kinematics.c").read_text()
    table = source.split("rover_ik_provisional", 1)[1].split("};", 1)[0]
    rows = re.findall(r"\{\s*([-+\d.].*?)\}", table)
    geometry = [[float(v.strip().removesuffix("f")) for v in row.split(",")[:7]] for row in rows]
    if len(geometry) != 4:
        raise ValueError("cannot read the four firmware geometry rows")
    J = []
    for x, y, dx, dy, radius, ratio, sign in geometry:
        factor = 60 / (2 * np.pi * radius) * ratio * sign
        J.append(np.array([dx / 1000, dy / 1000, x * dy - y * dx]) * factor)
    return np.asarray(J)


def read_jsonl(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    if len(rows) < 5:
        raise ValueError(f"{path}: need at least five rows")
    times = np.array([r["t"] for r in rows], float)
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError(f"{path}: timestamps must be finite and strictly increasing")
    return rows


def wheel_values(row, field):
    if field in row:
        value = row[field]
    elif field in row.get("status", {}):
        value = row["status"][field]
    else:
        wheels = row.get("rows", [])
        if isinstance(wheels, dict):
            wheels = [wheels.get(name, wheels.get(esc)) for name, esc, _ in WHEELS]
        if not wheels:
            return np.full(4, np.nan)
        value = [w.get(field) if w else None for w in wheels]
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f"{field}: expected FR BR BL FL array of length four")
    return np.array([np.nan if v is None else v for v in value], float)


def effort_values(row, effort):
    values = wheel_values(row, effort)
    counts = wheel_values(row, effort.replace("_a", "_counts")) / COUNTS_PER_AMP
    if effort == "iq_ref_a":
        counts = np.where(np.isfinite(counts), counts, wheel_values(row, "iq_ref") / COUNTS_PER_AMP)
    return np.where(np.isfinite(values), values, counts)


def telemetry(rows, origin, J, effort):
    t = np.array([r["t"] + origin for r in rows])
    rpm = np.array([wheel_values(r, "rpm") for r in rows])
    target = np.array([wheel_values(r, "target_rpm") for r in rows])
    u = np.array([effort_values(r, effort) for r in rows])
    odom = rpm @ np.linalg.pinv(J).T
    cmd = []
    safe = []
    for r, ref in zip(rows, target):
        if "twist" in r or "vector" in r:
            c = np.array(r.get("twist", r.get("vector")), float) * [1, 1, 0.001]
        else:
            c = np.linalg.pinv(J) @ ref
        cmd.append(c)
        nested = r.get("rows", [])
        if isinstance(nested, dict):
            nested = list(nested.values())
        fault = any(r.get("faults", [])) or any(w and (w.get("faults") or w.get("derated")
                    or w.get("flags", 0) & 0x20) for w in nested)
        safe.append(not fault and not any(r.get("derated", [])))
    cmd = np.array(cmd)
    missing = ~np.isfinite(target).all(axis=1)
    target[missing] = cmd[missing] @ J.T
    return dict(t=t, rpm=rpm, target=target, effort=u, odom=odom, cmd=cmd,
                safe=np.asarray(safe))


def vision_velocity(rows, smooth_s=0.3, max_gap_s=0.15):
    """Local quadratic fits on real timestamps; never differentiate across a lost tag."""
    groups, group = [], []
    for r in rows:
        valid = r.get("detected", all(k in r for k in ("x_mm", "y_mm", "theta_rad")))
        if group and (not valid or r["t"] - group[-1]["t"] > max_gap_s):
            groups.append(group)
            group = []
        if valid:
            if not np.isfinite([r[k] for k in ("x_mm", "y_mm", "theta_rad")]).all():
                raise ValueError("nonfinite detected vision pose")
            group.append(r)
    if group:
        groups.append(group)
    ts, poses, velocities, runs = [], [], [], []
    for gi, group in enumerate(groups):
        if len(group) < 5:
            continue
        t = np.array([r["t"] for r in group])
        pose = np.array([[r[k] for k in ("x_mm", "y_mm", "theta_rad")] for r in group])
        pose[:, 2] = np.unwrap(pose[:, 2])
        for k, now in enumerate(t):
            mask = np.abs(t - now) <= smooth_s / 2
            if mask.sum() < 5:
                continue
            dt = t[mask] - now
            fit = np.linalg.lstsq(np.column_stack((np.ones(len(dt)), dt, dt ** 2)),
                                  pose[mask], rcond=None)[0]
            ts.append(now)
            poses.append(fit[0])
            c, s = np.cos(fit[0, 2]), np.sin(fit[0, 2])
            velocities.append([c * fit[1, 0] + s * fit[1, 1],
                               -s * fit[1, 0] + c * fit[1, 1], fit[1, 2]])
            runs.append(gi)
    if len(ts) < 10:
        raise ValueError("too few continuous detected poses for differentiation")
    return dict(t=np.array(ts), pose=np.array(poses), vel=np.array(velocities), run=np.array(runs))


def interpolate(t, values, query, max_gap_s, hold=False):
    values = np.asarray(values)
    one = values.ndim == 1
    values = values.reshape(len(t), -1)
    right = np.searchsorted(t, query, side="right").clip(1, len(t) - 1)
    left = right - 1
    weight = np.zeros(len(query)) if hold else (query - t[left]) / (t[right] - t[left])
    result = values[left] * (1 - weight[:, None]) + values[right] * weight[:, None]
    invalid = (query < t[0]) | (query > t[-1]) | (t[right] - t[left] > max_gap_s)
    result[invalid] = np.nan
    return result[:, 0] if one else result


def estimate_latency(vision, telem, center=0, search_s=0.5, max_gap_s=0.15):
    speed = np.linalg.norm(vision["vel"][:, :2], axis=1)
    wheel_speed = np.linalg.norm(telem["odom"][:, :2], axis=1)
    step = 0.005
    candidates = np.arange(center - search_s, center + search_s + step / 2, step)
    scores = []
    for offset in candidates:
        sampled = interpolate(telem["t"], wheel_speed, vision["t"] - offset, max_gap_s)
        mask = np.isfinite(sampled)
        minimum = max(20, int(0.6 * min(len(speed), len(wheel_speed))))
        if mask.sum() < minimum or np.std(sampled[mask]) < 5 or np.std(speed[mask]) < 5:
            scores.append(-np.inf)
        else:
            scores.append(float(np.corrcoef(speed[mask], sampled[mask])[0, 1]))
    best = int(np.argmax(scores))
    if not np.isfinite(scores[best]) or scores[best] < 0.3:
        raise ValueError("latency unidentifiable: need overlapping starts/stops with changing speed")
    outside = np.abs(candidates - candidates[best]) > 0.15
    second = max(np.array(scores)[outside], default=-np.inf)
    return float(candidates[best]), dict(correlation=scores[best], step_s=step,
                at_search_boundary=best in (0, len(candidates) - 1),
                ambiguous=bool(scores[best] - second < 0.02),
                search_center_s=center, search_radius_s=search_s)


def direction_basis(theta):
    sine = np.sin(theta)
    sine = np.where(np.abs(sine) < 0.001, 0, sine)
    return sine, np.sign(sine)


def steady_segments(data, args):
    t, vel, cmd = data["t"], data["vel"], data["cmd"]
    speed = np.linalg.norm(vel[:, :2], axis=1)
    # Differentiating the smoothed body velocity is sufficient for near-zero-yaw fits.
    accel = np.linalg.norm(np.gradient(vel[:, :2], t, axis=0), axis=1)
    change = np.r_[True, (np.linalg.norm(np.diff(cmd[:, :2], axis=0), axis=1) > 5)
                   | (np.abs(np.diff(cmd[:, 2])) > 0.02)
                   | (np.diff(data["run"]) != 0) | (np.diff(t) > args.max_gap_s)]
    start = np.maximum.accumulate(np.where(change, np.arange(len(t)), 0))
    valid = ((t - t[start] >= args.settle_s) & (speed >= args.min_speed_mmps)
             & (accel < args.max_accel_mmps2) & (np.abs(vel[:, 2]) < 0.15)
             & (np.abs(cmd[:, 2]) < 0.05) & data["safe"]
             & (np.max(np.abs(data["effort"]), axis=1) < args.current_limit_a * 0.95)
             & (np.max(np.abs(data["rpm"] - data["target"]), axis=1) < args.max_wheel_error_rpm)
             & (np.max(np.abs(data["slip"]), axis=1) < args.max_slip_rpm))
    theta = np.arctan2(vel[:, 1], vel[:, 0])
    bins = np.floor((theta % (2 * np.pi)) / (2 * np.pi) * args.direction_bins).astype(int)
    segments, active = [], []

    def finish(indices):
        if len(indices) < 5 or t[indices[-1]] - t[indices[0]] < args.min_segment_s:
            return
        ix = np.array(indices)
        direction = float(np.arctan2(np.sin(theta[ix]).mean(), np.cos(theta[ix]).mean()))
        segments.append(dict(start_t=float(t[ix[0]]), end_t=float(t[ix[-1]]), samples=len(ix),
                        direction_bin=int(bins[ix[0]]), theta_rad=direction,
                        speed_mmps=float(speed[ix].mean()),
                        rpm=data["rpm"][ix].mean(axis=0).tolist(),
                        target_rpm=data["target"][ix].mean(axis=0).tolist(),
                        vision_rpm=data["vision_rpm"][ix].mean(axis=0).tolist(),
                        effort_a=data["effort"][ix].mean(axis=0).tolist()))

    for i in range(len(t)):
        if active and (not valid[i] or change[i] or bins[i] != bins[active[-1]]):
            finish(active)
            active = []
        if valid[i]:
            active.append(i)
    finish(active)
    return segments


def fit_friction(segments):
    fits = {}
    for wi, name in enumerate(NAMES):
        if len(segments) < 4:
            fits[name] = dict(identifiable=False, reason="need at least four steady segments")
            continue
        theta = np.array([s["theta_rad"] for s in segments])
        sine, sign = direction_basis(theta)
        rpm = np.array([s["target_rpm"][wi] for s in segments])
        effort = np.array([s["effort_a"][wi] for s in segments])
        X = np.column_stack((sine, sign, rpm))
        scales = np.linalg.norm(X, axis=0)
        normalized = X / np.where(scales > 0, scales, 1)
        condition = float(np.linalg.cond(normalized))
        if np.linalg.matrix_rank(X) < 3 or condition > 1e4:
            fits[name] = dict(identifiable=False, reason="rank deficient direction/speed sweep")
            continue
        coeff = np.linalg.lstsq(X, effort, rcond=None)[0]
        pred = X @ coeff
        residual = effort - pred
        variance = float(residual @ residual / (len(X) - 3))
        ci = 1.96 * np.sqrt(np.maximum(0, np.diag(np.linalg.inv(X.T @ X)) * variance))
        total = float(np.sum((effort - effort.mean()) ** 2))
        intercepts = []
        for direction in sorted({s["direction_bin"] for s in segments}):
            ix = np.array([i for i, s in enumerate(segments) if s["direction_bin"] == direction])
            if len(ix) < 2 or np.ptp(rpm[ix]) < 5:
                continue
            slope, intercept = np.polyfit(rpm[ix], effort[ix], 1)
            intercepts.append(dict(direction_bin=direction, theta_rad=float(theta[ix].mean()),
                             slope_a_per_rpm=float(slope), intercept_a=float(intercept),
                             segments=len(ix)))
        fits[name] = dict(identifiable=True, a_a=float(coeff[0]), b_a=float(coeff[1]),
                          d_a_per_rpm=float(coeff[2]), ci95_halfwidth=ci.tolist(),
                          rmse_a=float(np.sqrt(np.mean(residual ** 2))),
                          r_squared=1 - float(residual @ residual) / total if total else None,
                          condition=condition, segments=len(X), direction_intercepts=intercepts)
    return fits


def analyze(rig, esc, args):
    J = geometry_matrix()
    vision = vision_velocity(rig, args.smooth_s, args.max_gap_s)
    telem = telemetry(esc, args.telemetry_origin_t, J, args.effort)
    latency, alignment = args.latency_s, None
    if args.estimate_latency:
        latency, alignment = estimate_latency(vision, telem, latency, args.lag_search_s, args.max_gap_s)
    query = vision["t"] - latency
    data = dict(t=vision["t"], pose=vision["pose"], vel=vision["vel"], run=vision["run"])
    for field in ("rpm", "effort", "target", "cmd", "odom"):
        data[field] = interpolate(telem["t"], telem[field], query, args.max_gap_s,
                                  hold=field in ("target", "cmd"))
    safe = interpolate(telem["t"], telem["safe"].astype(float), query, args.max_gap_s, hold=True)
    mask = np.isfinite(np.column_stack([data[k] for k in ("rpm", "cmd", "odom")])).all(axis=1)
    if mask.sum() < 10:
        raise ValueError("too few overlapping samples; check telemetry origin/latency and gaps")
    data = {k: v[mask] for k, v in data.items()}
    data["safe"] = safe[mask] == 1
    data["vision_rpm"] = data["vel"] @ J.T
    data["slip"] = data["rpm"] - data["vision_rpm"]
    segments = steady_segments(data, args)
    error = data["vel"] - data["cmd"]
    fits = fit_friction(segments)
    warnings = []
    if alignment and alignment["ambiguous"]:
        warnings.append("latency has competing peaks; use a nonperiodic speed pattern")
    if alignment and alignment["at_search_boundary"]:
        warnings.append("latency optimum is at search boundary; widen or move the search")
    if not all(f["identifiable"] for f in fits.values()):
        warnings.append("friction unidentifiable; collect multiple speeds and oblique directions")
    if any(r.get("source") == "video" and r.get("clock") == "host_monotonic" for r in rig):
        raise ValueError("video replay grab times cannot align telemetry; supply --video-start-t in tracker")
    summary = dict(samples=len(data["t"]), vision_detection_fraction=sum(bool(r.get("detected", True))
                   for r in rig) / len(rig), latency_s=latency, telemetry_origin_t=args.telemetry_origin_t,
                   alignment=alignment, effort_field=args.effort,
                   model="u_A = a_A*sin(theta) + b_A*sign(sin(theta)) + d_A_per_RPM*w_ref_RPM",
                   geometry_source="src/shared/src/rover_kinematics.c: rover_ik_provisional",
                   tracking_rmse=dict(zip(("vx_mmps", "vy_mmps", "omega_radps"),
                                      np.sqrt(np.mean(error ** 2, axis=0)).tolist())),
                   wheel_slip_rms_rpm=dict(zip(NAMES, np.sqrt(np.mean(data["slip"] ** 2, axis=0)).tolist())),
                   steady_segments=segments, friction=fits, warnings=warnings)
    return summary, data


def write_outputs(out, summary, data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    with (out / "aligned.jsonl").open("w") as f:
        for i, t in enumerate(data["t"]):
            row = dict(t=float(t), body_velocity=data["vel"][i].tolist(),
                       command=data["cmd"][i].tolist(), wheel_slip_rpm=data["slip"][i].tolist())
            f.write(json.dumps(row, allow_nan=False) + "\n")
    t = data["t"] - data["t"][0]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, (ax, label) in enumerate(zip(axes, ("vx (mm/s)", "vy (mm/s)", "yaw (rad/s)"))):
        ax.plot(t, data["cmd"][:, i], label="command")
        ax.plot(t, data["vel"][:, i], label="vision")
        ax.plot(t, data["odom"][:, i], alpha=0.6, label="wheel odometry")
        ax.set_ylabel(label)
        ax.grid()
    axes[0].legend()
    axes[-1].set_xlabel("aligned vision time (s)")
    fig.tight_layout()
    fig.savefig(out / "tracking.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].plot(data["pose"][:, 0], data["pose"][:, 1])
    axes[0].set(xlabel="floor x (mm)", ylabel="floor y (mm)", aspect="equal")
    for i, name in enumerate(NAMES):
        axes[1].plot(t, data["slip"][:, i], label=name)
    axes[1].set(xlabel="time (s)", ylabel="measured - vision-predicted wheel RPM")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(out / "slip.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    segments = summary["steady_segments"]
    for wi, (name, ax) in enumerate(zip(NAMES, axes.flat)):
        fit = summary["friction"][name]
        if fit["identifiable"]:
            theta = np.array([s["theta_rad"] for s in segments])
            u = np.array([s["effort_a"][wi] for s in segments])
            rpm = np.array([s["target_rpm"][wi] for s in segments])
            grid = np.linspace(-np.pi, np.pi, 401)
            sine, sign = direction_basis(grid)
            ax.scatter(theta, u - fit["d_a_per_rpm"] * rpm, s=12, label="segment: u-d*w_ref")
            ax.plot(grid, fit["a_a"] * sine + fit["b_a"] * sign, label="f(theta) fit")
            ax.legend(fontsize=8)
        else:
            ax.text(0.1, 0.5, "Insufficient direction/speed coverage", transform=ax.transAxes, fontsize=8)
        ax.set(title=name, xlabel="body driving direction (rad)", ylabel="friction current (A)")
    fig.tight_layout()
    fig.savefig(out / "friction.png", dpi=150)
    plt.close(fig)


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rig", type=Path, required=True)
    p.add_argument("--esc", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--telemetry-origin-t", type=float, default=0,
                   help="add this monotonic origin to elapsed pulse timestamps")
    p.add_argument("--latency-s", type=float, default=0, help="vision delay; also center for estimation")
    p.add_argument("--estimate-latency", action="store_true")
    p.add_argument("--lag-search-s", type=float, default=0.5)
    p.add_argument("--effort", choices=("iq_ref_a", "iq_a"), default="iq_ref_a")
    p.add_argument("--smooth-s", type=float, default=0.3)
    p.add_argument("--max-gap-s", type=float, default=0.15)
    p.add_argument("--settle-s", type=float, default=0.5)
    p.add_argument("--min-segment-s", type=float, default=0.4)
    p.add_argument("--min-speed-mmps", type=float, default=30)
    p.add_argument("--max-accel-mmps2", type=float, default=100)
    p.add_argument("--max-wheel-error-rpm", type=float, default=30)
    p.add_argument("--max-slip-rpm", type=float, default=30)
    p.add_argument("--current-limit-a", type=float, default=6)
    p.add_argument("--direction-bins", type=int, default=16)
    return p


def main():
    args = parser().parse_args()
    summary, data = analyze(read_jsonl(args.rig), read_jsonl(args.esc), args)
    write_outputs(args.out, summary, data)
    print(f"{summary['samples']} aligned samples; latency {summary['latency_s']:.3f} s; "
          f"{len(summary['steady_segments'])} steady segments")
    for name, fit in summary["friction"].items():
        if fit["identifiable"]:
            print(f"{name}: a={fit['a_a']:.5f} A b={fit['b_a']:.5f} A "
                  f"d={fit['d_a_per_rpm']:.6f} A/RPM; RMSE {fit['rmse_a']:.4f} A "
                  f"R2 {fit['r_squared']:.4f}")
        else:
            print(f"{name}: {fit['reason']}")
    for warning in summary["warnings"]:
        print("warning:", warning)


if __name__ == "__main__":
    main()
