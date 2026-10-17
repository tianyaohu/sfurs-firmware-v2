#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6"]
# ///
"""tag_stream still-window selection and per-move metrics on synthetic poses.
Run: uv run scripts/rig/test_tag_stream.py -v"""
import math
import unittest

import numpy as np

from tag_stream import move_summary, still_pose


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def frames_for(path, fps=30, noise_mm=0.5, seed=1):
    """path(t) -> (x_mm, y_mm, yaw_deg) of a tag 1000 mm below the camera, in camera axes."""
    rng = np.random.default_rng(seed)
    out = []
    for k in range(int(6 * fps)):
        t = k / fps
        x, y, yaw = path(t)
        out.append((t, (np.array([x, y, 1000.0]) + rng.normal(0, noise_mm, 3), rot_z(yaw)), True))
    return out


def straight(t, start=1.0, end=2.0, dx=160.0, dy=0.0, bulge=0.0, turn=0.0):
    u = min(1.0, max(0.0, (t - start) / (end - start)))
    n = math.hypot(dx, dy)
    side = bulge * math.sin(math.pi * u)  # perpendicular to the move
    return dx * u - dy / n * side, dy * u + dx / n * side, turn * u


class Still(unittest.TestCase):
    def test_still_window_and_motion_rejected(self):
        frames = frames_for(straight)
        self.assertIsNotNone(still_pose(frames, 0.5, 0.95))
        self.assertIsNone(still_pose(frames, 1.2, 1.6))  # moving
        self.assertIsNone(still_pose(frames, 7.0, 8.0))  # no frames

    def test_lost_tag(self):
        frames = [(t, None, False) if 0.5 <= t <= 1.0 else (t, p, r) for t, p, r in frames_for(straight)]
        self.assertIsNone(still_pose(frames, 0.5, 0.95))


class Move(unittest.TestCase):
    def test_straight_move(self):
        s = move_summary(frames_for(straight), (0.5, 0.95), (2.5, 2.95), 160.0)
        self.assertAlmostEqual(s["distance_mm"], 160.0, delta=1.0)
        self.assertAlmostEqual(s["direction_deg"], 0.0, delta=0.5)
        self.assertLess(s["max_sideways_mm"], 3.0)
        self.assertAlmostEqual(s["heading_change_deg"], 0.0, delta=0.3)

    def test_bulge_and_turn_measured_in_tag_frame(self):
        frames = frames_for(lambda t: straight(t, dx=0.0, dy=160.0, bulge=12.0, turn=4.0))
        s = move_summary(frames, (0.5, 0.95), (2.5, 2.95))
        self.assertAlmostEqual(abs(s["direction_deg"]), 90.0, delta=1.0)
        self.assertAlmostEqual(s["max_sideways_mm"], 12.0, delta=2.0)
        self.assertAlmostEqual(abs(s["heading_change_deg"]), 4.0, delta=0.3)


if __name__ == "__main__":
    unittest.main()
