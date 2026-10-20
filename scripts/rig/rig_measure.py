#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6", "reportlab>=4"]
# ///
"""Quick displacement check: how far did one AprilTag move? No floor survey needed.

Uses only the lens calibration and the printed tag size. Press r to set the reference
pose; the displacement then prints once per second. q quits. With --log, every frame after
r is recorded and a drive summary is printed on quit: distance, sideways drift from a straight
line, heading change, and pass/fail against --target-mm. With --known-mm, press m after each
move to record an accuracy trial; quitting prints errors per direction.

  uv run scripts/rig/rig_measure.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml
  uv run scripts/rig/rig_measure.py --calibration ... --log /tmp/drive.jsonl --target-mm 500
"""
import argparse
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from rig_common import april_dictionary, camera_model, open_source


def tag_pose(corners, K, dist, tag_mm):
    """Tag center (mm, camera frame) and rotation matrix from its four TL, TR, BR, BL corners."""
    h = tag_mm / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, np.float64).reshape(4, 1, 2), K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return tvec.ravel(), cv2.Rodrigues(rvec)[0]


def average_pose(poses):
    """Median position and mean rotation (projected back onto SO(3)) of a still tag.

    A single small tag's orientation is noisy by several degrees; using one frame's tilted
    plane shortened every in-plane distance by ~1% in synthetic tests.
    """
    t = np.median([p[0] for p in poses], axis=0)
    u, _, vt = np.linalg.svd(np.mean([p[1] for p in poses], axis=0))
    R = u @ vt
    if np.linalg.det(R) < 0:
        R = u @ np.diag([1, 1, -1]) @ vt
    return t, R


def floor_basis(centers):
    """Rows u, v, n of a floor frame from >=3 floor-tag centers (camera frame, mm).

    n is the plane normal toward the camera; u is the camera x-axis (image right) projected
    onto the plane; v points toward image down. A plane through tags ~0.8 m apart is far less
    sensitive than one small tag's own tilt (a sheet on carpet tilted ~6 deg gave +35 mm
    out-of-plane and skewed real trials).
    """
    c = np.asarray(centers, float)
    centroid = c.mean(axis=0)
    n = np.linalg.svd(c - centroid)[2][-1]
    if n @ centroid > 0:
        n = -n
    u = np.array([1.0, 0, 0]) - n[0] * n
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    if v[1] < 0:
        v = -v
    return np.array([u, v, n])


def plane_pose(corners, K, dist, basis, origin, height_mm=0.0):
    """Tag pose from its corner pixels intersected with the floor plane (raised by height_mm).

    Avoids single-tag depth: four tags on one floor scattered +-12 mm (~1% of range) around
    their best plane, which leaked into oblique-view distances. Pixel rays are good to ~1 px.
    Returns (center, R) like tag_pose, with R's x-axis along the tag's top edge and z = n.
    """
    n = basis[2]
    rays = cv2.undistortPoints(np.asarray(corners, np.float64).reshape(4, 1, 2), K, dist).reshape(4, 2)
    rays = np.hstack([rays, np.ones((4, 1))])
    pts = rays * ((n @ origin + height_mm) / (rays @ n))[:, None]  # TL, TR, BR, BL on the plane
    x = (pts[1] - pts[0]) + (pts[2] - pts[3])
    x -= (x @ n) * n
    x /= np.linalg.norm(x)
    return pts.mean(axis=0), np.column_stack([x, np.cross(n, x), n])


def displacement(ref, cur, basis=None):
    """In-plane distance (mm) and yaw change (deg) of the tag relative to the reference pose.

    Measured in the floor plane when `basis` (from floor_basis) is given, else in the reference
    tag's own plane. Single-tag depth is the noisiest axis and a tag on a floor moves within
    the plane anyway. d[2] is the out-of-plane part, a flatness check.
    """
    (t0, R0), (t1, R1) = ref, cur
    B = R0.T if basis is None else basis
    d = B @ (t1 - t0)  # x, y in plane, z normal
    a0, a1 = B @ R0[:, 0], B @ R1[:, 0]  # tag x-axes in the plane frame
    yaw = np.degrees(np.arctan2(a1[1], a1[0]) - np.arctan2(a0[1], a0[0]))
    return float(np.hypot(d[0], d[1])), float((yaw + 180) % 360 - 180), d


def direction_label(d):
    """Nearest in-plane axis the move went along: +x, -x, +y or -y. With floor tags, +x is
    image right and +y image down; without them, the reference tag's own axes."""
    if abs(d[0]) >= abs(d[1]):
        return "+x" if d[0] > 0 else "-x"
    return "+y" if d[1] > 0 else "-y"


def trial_report(trials, known_mm):
    """Lines summarising accuracy trials (distance_mm, d) against a known distance."""
    lines = [f"{'#':>2} {'dir':>3} {'measured':>9} {'error':>7} {'out-of-plane':>12}"]
    for i, (dist_mm, d) in enumerate(trials, 1):
        lines.append(f"{i:>2} {direction_label(d):>3} {dist_mm:9.1f} {dist_mm - known_mm:+7.1f} {d[2]:+12.1f}")
    errors = np.array([t[0] - known_mm for t in trials])
    lines.append(f"known {known_mm:g} mm, {len(trials)} trials: mean error {errors.mean():+.1f} mm "
                 f"({100 * errors.mean() / known_mm:+.1f}%), worst {errors[np.argmax(abs(errors))]:+.1f} mm")
    for label in ("+x", "-x", "+y", "-y"):
        e = [t[0] - known_mm for t in trials if direction_label(t[1]) == label]
        if e:
            lines.append(f"  {label}: {len(e)} trial(s), mean error {np.mean(e):+.1f} mm")
    return lines


def path_summary(path, end, target_mm=None, tolerance_mm=None):
    """Straight-drive metrics from per-frame (x, y) in the reference plane, mm, and the
    averaged final displacement `end` = (distance, yaw_deg, d)."""
    distance, yaw, d = end
    xy = np.array([[r["x_mm"], r["y_mm"]] for r in path]) if path else np.zeros((0, 2))
    line = d[:2] / max(distance, 1e-9)
    lateral = xy @ np.array([-line[1], line[0]]) if len(xy) else np.zeros(1)
    out = dict(distance_mm=distance, heading_change_deg=yaw, end_x_mm=float(d[0]), end_y_mm=float(d[1]),
               end_out_of_plane_mm=float(d[2]), max_sideways_mm=float(np.max(np.abs(lateral))),
               frames=len(path), frames_outside_reliable=int(sum(not r["reliable"] for r in path)))
    if target_mm is not None:
        out["error_mm"] = distance - target_mm
        if tolerance_mm is not None:
            out["pass"] = bool(abs(out["error_mm"]) <= tolerance_mm)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--exposure", type=float, default=100)
    p.add_argument("--tag-id", type=int, default=0)
    p.add_argument("--tag-mm", type=float, default=100, help="black outer square, as measured")
    p.add_argument("--average", type=int, default=15, help="frames averaged per pose (tag held still)")
    p.add_argument("--log", type=Path, help="record every frame after r; print a drive summary on quit")
    p.add_argument("--target-mm", type=float, help="commanded distance, for the summary")
    p.add_argument("--tolerance-mm", type=float, default=20)
    p.add_argument("--known-mm", type=float, help="true distance for accuracy trials (press m after each move)")
    p.add_argument("--floor-ids", type=int, nargs="+", default=[10, 11, 12, 13],
                   help="floor tags that define the measuring plane (>=3 must be visible at r)")
    p.add_argument("--floor-tag-mm", type=float, default=120)
    p.add_argument("--tag-height-mm", type=float, default=0,
                   help="height of the measured tag above the floor tags (e.g. robot top)")
    p.add_argument("--max-out-of-plane-mm", type=float, default=5,
                   help="refuse trials when the tag left the reference plane by more (tilted or lifted)")
    p.add_argument("--reliable-px", type=float, default=250,
                   help="calibration is trusted within this radius of the image center")
    args = p.parse_args()
    _, K, dist = camera_model(args.calibration)
    detector = cv2.aruco.ArucoDetector(april_dictionary())
    cap = open_source(args.camera, None, args.exposure)
    recent, ref, path, trials = deque(maxlen=args.average), None, [], []
    floor = {i: deque(maxlen=args.average) for i in args.floor_ids}
    basis = origin = None  # floor plane; frozen when the reference is set
    last_print = 0.0

    def averaged():
        return average_pose(list(recent))

    try:
        while True:
            ok, image = cap.read()
            if not ok:
                break
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            found = ids is not None and args.tag_id in ids.ravel()
            seen = set() if ids is None else set(ids.ravel().tolist())
            for fid, q in floor.items():
                if fid in seen:
                    pose = tag_pose(corners[list(ids.ravel()).index(fid)], K, dist, args.floor_tag_mm)
                    if pose is not None:
                        q.append(pose[0])
                else:
                    q.clear()
            if ref is None:
                full = [np.median(q, axis=0) for q in floor.values() if len(q) == q.maxlen]
                new_basis = floor_basis(full) if len(full) >= 3 else None
                if (new_basis is None) != (basis is None):
                    recent.clear()  # don't mix plane and single-tag poses
                basis, origin = new_basis, (np.mean(full, axis=0) if new_basis is not None else None)
            if found:
                c = corners[list(ids.ravel()).index(args.tag_id)]
                pose = (plane_pose(c, K, dist, basis, origin, args.tag_height_mm) if basis is not None
                        else tag_pose(c, K, dist, args.tag_mm))
                if pose is not None:
                    recent.append(pose)
                    if ref is not None and args.log:
                        _, frame_yaw, fd = displacement(ref, pose, basis)
                        center = c.reshape(4, 2).mean(axis=0)
                        path.append(dict(t=time.monotonic(), x_mm=float(fd[0]), y_mm=float(fd[1]),
                                         out_of_plane_mm=float(fd[2]), yaw_deg=frame_yaw,
                                         reliable=bool(np.hypot(*(center - [K[0, 2], K[1, 2]])) <= args.reliable_px)))
                cv2.aruco.drawDetectedMarkers(image, [c], np.array([[args.tag_id]]))
            else:
                recent.clear()
            lines = []
            if recent:
                t, _ = averaged()
                lines.append(f"tag {args.tag_id}: {t[2] / 1000:.2f} m from camera, "
                             + ("floor-plane mode" if basis is not None else "single-tag mode (no floor tags)"))
                if ref is not None:
                    dist_mm, yaw, d = displacement(ref, averaged(), basis)
                    lines.append(f"moved {dist_mm:6.1f} mm   turned {yaw:+6.1f} deg")
                    if abs(d[2]) > args.max_out_of_plane_mm:
                        lines.append(f"NOT FLAT: out-of-plane {d[2]:+.0f} mm, reading unreliable")
                    elif args.known_mm:
                        lines.append(f"press m to record trial {len(trials) + 1}")
                    if time.monotonic() - last_print > 1:
                        last_print = time.monotonic()
                        print(f"moved {dist_mm:6.1f} mm (x {d[0]:+6.1f}, y {d[1]:+6.1f}, "
                              f"out-of-plane {d[2]:+5.1f}) turned {yaw:+5.1f} deg", flush=True)
            else:
                lines.append(f"tag {args.tag_id} not visible")
            if ref is None:
                lines.append("hold still, press r to set reference")
            for i, text in enumerate(lines):
                cv2.putText(image, text, (20, 40 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow("rig_measure (r=reference, q=quit)", image)
            key = cv2.waitKey(1) & 255
            if key == ord("q"):
                break
            if key == ord("m") and args.known_mm and ref is not None and len(recent) == recent.maxlen:
                dist_mm, _, d = displacement(ref, averaged(), basis)
                if dist_mm < 0.5 * args.known_mm:
                    print(f"not recorded: only {dist_mm:.1f} mm moved; move the tag first", flush=True)
                    continue
                if abs(d[2]) > args.max_out_of_plane_mm:
                    # Session data: out-of-plane -15 mm read 315 mm on a 300 mm move; +6 mm read 294.
                    print(f"not recorded: out-of-plane {d[2]:+.1f} mm (tag tilted or lifted); flatten it and press m again",
                          flush=True)
                    continue
                trials.append((dist_mm, d))
                print(f"trial {len(trials)}: {direction_label(d)} measured {dist_mm:.1f} mm, error "
                      f"{dist_mm - args.known_mm:+.1f} mm, out-of-plane {d[2]:+.1f} mm", flush=True)
            if key == ord("r") and len(recent) == recent.maxlen:
                ref = averaged()
                print("measuring plane: " + ("floor tags (ray-plane positions)" if basis is not None
                                             else "moving tag's own plane (no floor tags; tilt- and depth-sensitive)"),
                      flush=True)
                path.clear()
                print(f"reference set: {ref[0][2] / 1000:.3f} m from camera", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()
    if trials:
        print("\n".join(trial_report(trials, args.known_mm)))
    if args.log and ref is not None:
        if len(recent) < recent.maxlen:
            print("drive summary skipped: tag was not visible and still at the end")
            return
        summary = path_summary(path, displacement(ref, averaged(), basis), args.target_mm, args.tolerance_mm)
        with args.log.open("w") as f:
            for row in path:
                f.write(json.dumps(row) + "\n")
            f.write(json.dumps(dict(summary=summary)) + "\n")
        print("drive summary: " + ", ".join(f"{k} {v:.1f}" if isinstance(v, float) else f"{k} {v}"
                                              for k, v in summary.items()))
        if summary["frames_outside_reliable"]:
            print(f"WARNING: {summary['frames_outside_reliable']} frames outside the calibrated center; "
                  "distances there are less trustworthy")
        if abs(summary["end_out_of_plane_mm"]) > 10:
            print("WARNING: tag ended >10 mm off the reference plane (tilted or lifted?)")


if __name__ == "__main__":
    main()
