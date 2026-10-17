#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6", "reportlab>=4"]
# ///
"""Quick displacement check: how far did one AprilTag move? No floor survey needed.

Uses only the lens calibration and the printed tag size. Press r to set the reference
pose; the displacement then prints once per second. q quits.

  uv run scripts/rig/rig_measure.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml
"""
import argparse
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


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--exposure", type=float, default=100)
    p.add_argument("--tag-id", type=int, default=0)
    p.add_argument("--tag-mm", type=float, default=100, help="black outer square, as measured")
    p.add_argument("--average", type=int, default=15, help="frames averaged per pose (tag held still)")
    args = p.parse_args()
    _, K, dist = camera_model(args.calibration)
    detector = cv2.aruco.ArucoDetector(april_dictionary())
    cap = open_source(args.camera, None, args.exposure)
    recent, ref = deque(maxlen=args.average), None
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
                print(f"reference set: {ref[0][2] / 1000:.3f} m from camera", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
