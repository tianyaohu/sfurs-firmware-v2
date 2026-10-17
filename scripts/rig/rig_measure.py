#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6", "reportlab>=4"]
# ///
"""Quick displacement check: how far did one AprilTag move? No floor survey needed.

Uses only the lens calibration and the printed tag size. Press r to set the reference
pose; the displacement then prints once per second. q quits. With --log, every frame after
r is recorded and a drive summary is printed on quit: distance, sideways drift from a straight
line, heading change, and pass/fail against --target-mm.

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


def displacement(ref, cur):
    """In-plane distance (mm) and yaw change (deg) of the tag relative to the reference pose.

    Measured in the reference tag's plane: single-tag depth is the noisiest axis, and a tag
    on a floor moves within that plane anyway. Also returns the out-of-plane part as a check.
    """
    (t0, R0), (t1, R1) = ref, cur
    d = R0.T @ (t1 - t0)  # displacement in reference tag axes: x, y in plane, z normal
    rel = R0.T @ R1
    return float(np.hypot(d[0], d[1])), float(np.degrees(np.arctan2(rel[1, 0], rel[0, 0]))), d


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
    p.add_argument("--reliable-px", type=float, default=250,
                   help="calibration is trusted within this radius of the image center")
    args = p.parse_args()
    _, K, dist = camera_model(args.calibration)
    detector = cv2.aruco.ArucoDetector(april_dictionary())
    cap = open_source(args.camera, None, args.exposure)
    recent, ref, path = deque(maxlen=args.average), None, []
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
            if found:
                c = corners[list(ids.ravel()).index(args.tag_id)]
                pose = tag_pose(c, K, dist, args.tag_mm)
                if pose is not None:
                    recent.append(pose)
                    if ref is not None and args.log:
                        _, frame_yaw, fd = displacement(ref, pose)
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
                lines.append(f"tag {args.tag_id}: {t[2] / 1000:.2f} m from camera")
                if ref is not None:
                    dist_mm, yaw, d = displacement(ref, averaged())
                    lines.append(f"moved {dist_mm:6.1f} mm   turned {yaw:+6.1f} deg")
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
            if key == ord("r") and len(recent) == recent.maxlen:
                ref = averaged()
                path.clear()
                print(f"reference set: {ref[0][2] / 1000:.3f} m from camera", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()
    if args.log and ref is not None:
        if len(recent) < recent.maxlen:
            print("drive summary skipped: tag was not visible and still at the end")
            return
        summary = path_summary(path, displacement(ref, averaged()), args.target_mm, args.tolerance_mm)
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
