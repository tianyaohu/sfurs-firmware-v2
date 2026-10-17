#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "matplotlib>=3.8", "PyYAML>=6", "reportlab>=4"]
# ///
"""Offline integration/regression tests. Run: uv run scripts/rig/test_rig.py -v"""
import json
import contextlib
import io
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path

import cv2
import numpy as np

from rig_analyze import (analyze, direction_basis, fit_friction, geometry_matrix,
                         interpolate, parser, read_jsonl, telemetry, vision_velocity)
from rig_calibrate import make_board
from rig_common import april_dictionary, tag_corners
from rig_track import Tracker
import rig_sweep

HERE = Path(__file__).resolve().parent


def camera_scene():
    K = np.array([[900., 0, 640], [0, 895, 360], [0, 0, 1]])
    dist = np.array([-0.30, 0.10, 0.001, -0.001, -0.02])
    R = cv2.Rodrigues(np.array([0.06, -0.04, 0.025]))[0] @ np.diag([1., -1., -1.])
    C = np.array([120., -80., 2000.])
    rv, tv = cv2.Rodrigues(R)[0], -R @ C
    calibration = dict(K=K.tolist(), dist=dist.tolist(), image_size=[1280, 720])
    floor = dict(tags={str(i + 10): tag_corners(center, 120).tolist()
                      for i, center in enumerate([[-850, -440], [850, -440], [850, 440], [-850, 440]])})
    yy, xx = np.indices((720, 1280), dtype=np.float32)
    uv = cv2.undistortPoints(np.stack((xx, yy), axis=-1).reshape(-1, 1, 2), K, dist, P=K)
    mapping = uv.reshape(720, 1280, 2)

    def render(body_xy, theta, include_floor=True, robot=True):
        image = np.full((720, 1280), 255, np.uint8)

        def marker(i, xyz):
            dst = cv2.projectPoints(np.asarray(xyz, float), rv, tv, K, None)[0].reshape(4, 2).astype(np.float32)
            src = np.array([[0, 0], [399, 0], [399, 399], [0, 399]], np.float32)
            H = cv2.getPerspectiveTransform(src, dst)
            tag = cv2.aruco.generateImageMarker(april_dictionary(), i, 400)
            warped = cv2.warpPerspective(tag, H, (1280, 720), borderValue=255)
            np.minimum(image, warped, out=image)

        if include_floor:
            for i, xy in floor["tags"].items():
                marker(int(i), np.column_stack((xy, np.zeros(4))))
        if robot:
            c, s = np.cos(theta), np.sin(theta)
            tag_xy = np.asarray(body_xy) + np.array([[c, -s], [s, c]]) @ [15., -8.]
            xy = tag_corners(tag_xy, 100, theta + 0.15)
            marker(0, np.column_stack((xy, np.full(4, 150))))
        return cv2.remap(image, mapping[:, :, 0], mapping[:, :, 1], cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=255)

    return calibration, floor, render, (K, dist, rv, tv, C)


def synthetic_logs():
    rng = np.random.default_rng(82)
    J = geometry_matrix()
    a = np.array([0.35, 0.45, -0.30, -0.40])
    b = np.array([0.20, -0.15, 0.25, -0.18])
    d = np.array([0.012, 0.015, 0.011, 0.013])
    plan = [(0.8, np.zeros(3))]
    elapsed = 0.8
    for direction in rng.permutation(16):
        angle = direction * 2 * np.pi / 16 + np.pi / 64
        for speed in rng.permutation([100, 180, 280]):
            elapsed += 2.0
            plan.append((elapsed, np.array([speed * np.cos(angle), speed * np.sin(angle), 0.])))
            elapsed += rng.uniform(0.5, 0.9)
            plan.append((elapsed, np.zeros(3)))
    t = np.arange(0, elapsed, 0.01)
    ends = np.array([p[0] for p in plan])
    cmd = np.array([plan[min(np.searchsorted(ends, x), len(plan) - 1)][1] for x in t])
    vel = np.zeros_like(cmd)
    for i in range(1, len(t)):
        vel[i] = vel[i - 1] + (cmd[i] - vel[i - 1]) * (1 - np.exp(-0.01 / 0.08))
    position = np.vstack((np.zeros(3), np.cumsum((vel[1:] + vel[:-1]) * 0.005, axis=0)))
    target = cmd @ J.T
    rpm = vel @ J.T + rng.normal(0, 0.1, (len(t), 4))
    theta = np.arctan2(vel[:, 1], vel[:, 0])
    # Ground truth written out independently of rig_analyze.direction_basis so a bug there cannot cancel out.
    sine = np.sin(theta)
    sign = np.sign(sine)
    effort = target * d + sine[:, None] * a + sign[:, None] * b + rng.normal(0, 0.015, (len(t), 4))
    esc = [dict(t=float(x), twist=(cmd[i] * [1, 1, 1000]).tolist(),
                rpm=rpm[i].tolist(), target_rpm=target[i].tolist(), iq_ref_a=effort[i].tolist(),
                faults=[[], [], [], []]) for i, x in enumerate(t)]
    delay, origin = 0.125, 1000.
    vt = np.arange(0.01, t[-1] - 0.1, 1 / 30)
    poses = np.column_stack([np.interp(vt, t, position[:, i]) for i in range(3)])
    poses[:, :2] += rng.normal(0, 0.08, (len(vt), 2))
    rig = [dict(t=float(x + origin + delay), detected=not 12 < x < 12.8, frame=i,
                x_mm=float(poses[i, 0]), y_mm=float(poses[i, 1]), theta_rad=0.) for i, x in enumerate(vt)]
    return rig, esc, (a, b, d), origin, delay


class RigTests(unittest.TestCase):
    def test_geometry_matches_firmware_targets(self):
        # Known answer from ground-runs/run1-0.3.jsonl: vector [0, 300, 0] mm/s -> firmware target_rpm.
        np.testing.assert_allclose(geometry_matrix() @ [0, 300, 0], [76, 72, -72, -76], atol=1)

    def test_measure_displacement_known_moves(self):
        from rig_measure import average_pose, displacement, tag_pose
        K, dist, h = np.array([[870., 0, 640], [0, 870, 360], [0, 0, 1]]), np.zeros(5), 50
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
        rng = np.random.default_rng(3)

        def pose(x, y, yaw_deg):
            R = cv2.Rodrigues(np.array([np.pi, 0, 0]))[0] @ cv2.Rodrigues(np.array([0, 0, np.radians(yaw_deg)]))[0]
            px = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], np.array([x, y, 1500.]), K, dist)[0].reshape(4, 2)
            return average_pose([tag_pose(px + rng.normal(0, 0.3, (4, 2)), K, dist, 100) for _ in range(15)])

        moved, turned, _ = displacement(pose(-150, -100, 0), pose(150, 100, -45))
        self.assertLess(abs(moved - np.hypot(300, 200)), 3)
        self.assertLess(abs(turned + 45), 1)

    def test_drive_summary_straightness(self):
        from rig_measure import path_summary
        ys = np.linspace(0, 500, 51)
        path = [dict(x_mm=float(12 * np.sin(np.pi * y / 500)), y_mm=float(y), reliable=y < 400) for y in ys]
        s = path_summary(path, (500.0, 3.0, np.array([0.0, 500.0, 1.0])), target_mm=490, tolerance_mm=20)
        self.assertAlmostEqual(s["max_sideways_mm"], 12, places=6)
        self.assertAlmostEqual(s["error_mm"], 10)
        self.assertTrue(s["pass"])
        self.assertEqual(s["frames_outside_reliable"], 11)

    def test_trial_report_directions(self):
        from rig_measure import direction_label, trial_report
        self.assertEqual([direction_label(np.array(v)) for v in ([5, 1, 0], [-5, 1, 0], [1, 5, 0], [1, -5, 0])],
                         ["+x", "-x", "+y", "-y"])
        lines = trial_report([(212.0, np.array([212., 0, 1])), (208.0, np.array([0., -208, 0]))], 210)
        self.assertIn("mean error +0.0 mm", lines[3])
        self.assertIn("+x: 1 trial(s), mean error +2.0 mm", "\n".join(lines))

    def test_distorted_video_end_to_end(self):
        calibration, floor, render, _ = camera_scene()
        with tempfile.TemporaryDirectory(prefix="rig-test-") as tmp:
            tmp = Path(tmp)
            (tmp / "camera.json").write_text(json.dumps(calibration))
            (tmp / "floor.json").write_text(json.dumps(floor))
            video = cv2.VideoWriter(str(tmp / "scene.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                    30, (1280, 720), isColor=False)
            self.assertTrue(video.isOpened())
            truth = []
            for i in range(48):
                xy = np.array([450 * np.cos(i / 12), 200 * np.sin(i / 10)])
                theta = 2.7 + i / 50
                truth.append((xy, theta))
                video.write(render(xy, theta, include_floor=i == 0, robot=i != 23))
            video.release()
            result = subprocess.run([sys.executable, str(HERE / "rig_track.py"),
                "--calibration", str(tmp / "camera.json"), "--floor", str(tmp / "floor.json"),
                "--video", str(tmp / "scene.avi"), "--video-start-t", "1000",
                "--tag-height-mm", "150", "--tag-offset-mm", "15", "-8", "--tag-yaw-rad", "0.15",
                "--out", str(tmp / "rig.jsonl")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            rows = read_jsonl(tmp / "rig.jsonl")
            errors, angles = [], []
            self.assertEqual(len(rows), 48)
            self.assertFalse(rows[23]["detected"])
            for row, (xy, theta) in zip(rows, truth):
                if row["frame"] == 23:
                    continue
                self.assertTrue(row["detected"], row)
                errors.append(np.linalg.norm(np.array([row["x_mm"], row["y_mm"]]) - xy))
                angles.append(abs(np.arctan2(np.sin(row["theta_rad"] - theta), np.cos(row["theta_rad"] - theta))))
                self.assertAlmostEqual(row["t"], 1000 + row["frame"] / 30, places=8)
            rms = float(np.sqrt(np.mean(np.array(errors) ** 2)))
            yaw_rms = float(np.sqrt(np.mean(np.array(angles) ** 2)))
            print(f"\nTRACK: {len(errors)}/47 visible poses; xy RMS {rms:.4f} mm, max {max(errors):.4f} mm; "
                  f"yaw RMS {yaw_rms:.6f} rad, max {max(angles):.6f} rad", flush=True)
            self.assertLess(rms, 4)
            self.assertLess(max(errors), 7)
            self.assertLess(yaw_rms, 0.025)

    def test_homography_height_and_missing_floor(self):
        calibration, floor, render, model = camera_scene()
        K, dist, rv, tv, C = model
        points = np.array([[-800, -450, 0], [800, -450, 0], [800, 450, 0], [-800, 450, 0]], float)
        pixels = cv2.projectPoints(points, rv, tv, K, dist)[0].reshape(4, 2)
        config = dict(image_points_px=pixels.tolist(), floor_points_mm=points[:, :2].tolist(),
                      camera_center_mm=C.tolist())
        tracker = Tracker(calibration, config, tag_height_mm=150, tag_offset_mm=(15, -8), tag_yaw_rad=0.15)
        row = tracker.process(render([300, -100], 0.4, include_floor=False), 1, 0)
        self.assertTrue(row["detected"])
        self.assertLess(np.linalg.norm(np.array([row["x_mm"], row["y_mm"]]) - [300, -100]), 4)
        config.pop("camera_center_mm")
        with self.assertRaisesRegex(ValueError, "camera_center_mm"):
            Tracker(calibration, config, tag_height_mm=150)
        row = Tracker(calibration, floor).process(render([0, 0], 0, include_floor=False), 1, 0)
        self.assertFalse(row["detected"])
        self.assertEqual(row["quality"]["reason"], "floor_unavailable")
        with self.assertRaisesRegex(ValueError, "differs"):
            tracker.process(np.zeros((480, 640), np.uint8), 2, 1)

    def test_analyze_end_to_end(self):
        rig, esc, coefficients, origin, delay = synthetic_logs()
        with tempfile.TemporaryDirectory(prefix="rig-analysis-") as tmp:
            tmp = Path(tmp)
            for name, rows in (("rig", rig), ("esc", esc)):
                (tmp / f"{name}.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
            result = subprocess.run([sys.executable, str(HERE / "rig_analyze.py"), "--rig", str(tmp / "rig.jsonl"),
                "--esc", str(tmp / "esc.jsonl"), "--out", str(tmp / "analysis"),
                "--telemetry-origin-t", str(origin), "--estimate-latency"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            summary = json.loads((tmp / "analysis/summary.json").read_text())
            print("\nANALYZE: " + result.stdout.strip(), flush=True)
            self.assertLessEqual(abs(summary["latency_s"] - delay), 0.015)
            self.assertGreaterEqual(len(summary["steady_segments"]), 40)
            actual = []
            for wi, name in enumerate(("FR", "BR", "BL", "FL")):
                fit = summary["friction"][name]
                self.assertTrue(fit["identifiable"])
                self.assertGreaterEqual(len(fit["direction_intercepts"]), 14)
                self.assertGreater(fit["r_squared"], 0.99)
                actual.append([fit["a_a"], fit["b_a"], fit["d_a_per_rpm"]])
            delta = np.abs(np.array(actual) - np.array(coefficients).T)
            print(f"RECOVERY: max |a error|={delta[:, 0].max():.6f} A, "
                  f"|b error|={delta[:, 1].max():.6f} A, |d error|={delta[:, 2].max():.8f} A/RPM", flush=True)
            self.assertTrue(np.all(delta < [0.02, 0.02, 0.0003]), delta)
            for name in ("tracking.png", "slip.png", "friction.png", "aligned.jsonl"):
                self.assertGreater((tmp / "analysis" / name).stat().st_size, 1000)

    def test_gaps_yaw_wrap_and_time_validation(self):
        rows = [dict(t=i / 30, detected=not 20 <= i <= 30, x_mm=i * 3., y_mm=0.,
                     theta_rad=float(np.angle(np.exp(1j * (3 + i / 100))))) for i in range(60)]
        v = vision_velocity(rows)
        self.assertEqual(len(set(v["run"])), 2)
        self.assertLess(np.max(np.abs(v["vel"][:, 2] - 0.3)), 1e-8)
        sampled = interpolate(np.array([0., 0.1, 1., 1.1]), np.arange(4.), np.array([0.05, 0.5, 1.05]), 0.15)
        self.assertTrue(np.isnan(sampled[1]))
        np.testing.assert_allclose(sampled[[0, 2]], [0.5, 2.5])
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "bad.jsonl"
            p.write_text('{"t": 1}\n' * 5)
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                read_jsonl(p)

    def test_tracking_and_slip_units_at_rotated_heading(self):
        J = geometry_matrix()
        times = np.arange(0, 5, 1 / 30)
        rig = [dict(t=float(t), detected=True, x_mm=0., y_mm=100 * t,
                    theta_rad=np.pi / 2) for t in times]
        args = parser().parse_args(["--rig", "unused", "--esc", "unused", "--out", "unused"])
        for wi in range(4):
            esc = []
            for t in times:
                faults = [[], [], [], []]
                if 1 <= t <= 3:
                    faults[wi] = ["overcurrent"]
                esc.append(dict(t=float(t), twist=[90, 0, 0], rpm=(J @ [110, 0, 0]).tolist(),
                                target_rpm=(J @ [90, 0, 0]).tolist(), iq_ref_a=[0.5] * 4, faults=faults))
            summary, data = analyze(rig, esc, args)
            self.assertAlmostEqual(summary["tracking_rmse"]["vx_mmps"], 10, places=6)
            self.assertLess(summary["tracking_rmse"]["vy_mmps"], 1e-6)
            np.testing.assert_allclose(data["slip"], np.tile(J @ [10, 0, 0], (len(data["t"]), 1)), atol=1e-6)
            self.assertTrue(summary["steady_segments"])
            for segment in summary["steady_segments"]:
                self.assertTrue(segment["end_t"] < 1 or segment["start_t"] > 3)

    def test_real_log_schema_and_all_wheel_counts(self):
        J = geometry_matrix()
        # Check the motor signs and units against a hand-computed forward command.
        expected = np.array([0.8660254, 0.8191520, -0.8191520, -0.8660254]) * 0.1 * 60 / (2 * np.pi * 0.03275)
        np.testing.assert_allclose(J @ [0, 100, 0], expected, rtol=1e-6)
        rows = [dict(t=i * 0.02, rows={name: dict(rpm=expected[k], iq_ref_counts=544 * (k + 1))
                       for k, name in enumerate(("FR", "BR", "BL", "FL"))}, twist=[0, 100, 0]) for i in range(8)]
        data = telemetry(rows, 100, J, "iq_ref_a")
        np.testing.assert_allclose(data["effort"], np.tile([1, 2, 3, 4], (8, 1)))
        np.testing.assert_allclose(data["odom"], np.tile([0, 100, 0], (8, 1)), atol=1e-6)
        legacy = [dict(t=r["t"], vector=r["twist"], status=dict(target_rpm=expected.tolist()),
                       rows=list(r["rows"].values())) for r in rows]
        old = telemetry(legacy, 0, J, "iq_ref_a")
        np.testing.assert_allclose(old["cmd"], np.tile([0, 100, 0], (8, 1)), atol=1e-6)
        np.testing.assert_allclose(old["target"], np.tile(expected, (8, 1)))
        self.assertFalse(any(f["identifiable"] for f in fit_friction([]).values()))
        segment = dict(theta_rad=np.pi / 2, target_rpm=expected.tolist(), effort_a=[1, 2, 3, 4], direction_bin=4)
        self.assertFalse(any(f["identifiable"] for f in fit_friction([segment] * 8).values()))

    def test_sweep_plan_and_stop_paths_all_wheels(self):
        plan = rig_sweep.make_plan([11.25 + 22.5 * i for i in range(8)], [0.1, 0.2, 0.3], 1.5, 1)
        self.assertEqual(sum(s["duration_s"] for s in plan), 121)
        moving = [s["twist"] for s in plan if any(s["twist"])]
        self.assertEqual(len(moving), 48)
        for i in range(0, len(moving), 2):
            np.testing.assert_array_equal(moving[i], -np.array(moving[i + 1]))
        with self.assertRaises(ValueError):
            rig_sweep.make_plan([0], [0.5], 1, 1)

        class Clock:
            now = 100.

            def monotonic(self):
                return self.now

            def sleep(self, seconds):
                self.now += seconds

        class Link:
            def __init__(self, failure, wheel):
                self.failure, self.wheel = failure, wheel
                self.state, self.calls, self.disabled = 0, 0, False

            def read(self, field):
                return self.state

            def rows(self):
                self.calls += 1
                rows = [dict(mc_state="IDLE", aligned=True, faults=0, derated=False,
                             status_count=self.calls) for _ in range(4)]
                if self.failure == "stale":
                    rows[self.wheel]["status_count"] = 0
                if self.calls > 1 and self.failure in ("faults", "derated"):
                    rows[self.wheel][self.failure] = 1
                return rows

            def status(self):
                return dict(target_rpm=[0] * 4)

            def drive(self, *twist):
                self.state = 1

            def write(self, field, value):
                self.disabled, self.state = True, 0

        for failure in (None, "faults", "derated", "stale"):
            for wi in range(4):
                with self.subTest(failure=failure, wheel=wi), tempfile.TemporaryDirectory() as tmp:
                    link = Link(failure, wi)
                    session = SimpleNamespace(closed=False)
                    session.close = lambda: setattr(session, "closed", True)
                    args = SimpleNamespace(sn="fake", elf="fake", out=Path(tmp) / "sweep.jsonl")
                    with patch.object(rig_sweep.ik_link.Link, "open", return_value=(session, link)), \
                            patch.object(rig_sweep, "time", Clock()), contextlib.redirect_stdout(io.StringIO()):
                        if failure:
                            with self.assertRaises(RuntimeError):
                                rig_sweep.execute(args, [dict(duration_s=0.5, twist=[100, 0, 0])])
                        else:
                            rig_sweep.execute(args, [dict(duration_s=0.5, twist=[100, 0, 0])])
                            records = read_jsonl(args.out)
                            self.assertGreaterEqual(records[0]["t"], 100)
                            self.assertEqual(len(records[0]["rows"]), 4)
                    self.assertTrue(link.disabled)
                    self.assertTrue(session.closed)

    def test_print_and_calibration_solve(self):
        rng = np.random.default_rng(71)
        args = type("Args", (), dict(cols=7, rows=5, square_mm=30, marker_mm=22))()
        board = make_board(args)
        board_image = board.generateImage((1260, 900), marginSize=0)
        K = np.array([[900., 0, 640], [0, 895, 360], [0, 0, 1]])
        obj = np.array([[0, 0, 0], [210, 0, 0], [210, 150, 0], [0, 150, 0]], float)
        with tempfile.TemporaryDirectory(prefix="rig-calibration-") as tmp:
            tmp = Path(tmp)
            for i in range(20):
                rv = rng.uniform([-0.5, -0.5, -0.3], [0.5, 0.5, 0.3])
                tv = rng.uniform([-170, -120, 370], [70, 40, 700])
                dst = cv2.projectPoints(obj, rv, tv, K, None)[0].reshape(4, 2).astype(np.float32)
                H = cv2.getPerspectiveTransform(np.array([[0, 0], [1259, 0], [1259, 899], [0, 899]], np.float32), dst)
                image = cv2.warpPerspective(board_image, H, (1280, 720), borderValue=255)
                cv2.imwrite(str(tmp / f"view-{i}.png"), image)
            for command in (("board", "--out", str(tmp / "board")),
                            ("solve", "--images", str(tmp / "view-*.png"), "--out", str(tmp / "camera.json"))):
                result = subprocess.run([sys.executable, str(HERE / "rig_calibrate.py"), *command], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            camera = json.loads((tmp / "camera.json").read_text())
            self.assertLess(camera["rms_px"], 0.6)
            np.testing.assert_allclose(np.diag(np.array(camera["K"]))[:2], [900, 895], rtol=0.025)
            print(f"\nCALIBRATE: {camera['views']} views; RMS {camera['rms_px']:.4f} px; "
                  f"fx {camera['K'][0][0]:.3f}, fy {camera['K'][1][1]:.3f} px", flush=True)
            calibration, floor, _, _ = camera_scene()
            (tmp / "floor.json").write_text(json.dumps(floor))
            result = subprocess.run([sys.executable, str(HERE / "rig_track.py"), "--floor", str(tmp / "floor.json"),
                                     "--print-tags", str(tmp / "tags")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(len(list((tmp / "tags").glob("*.pdf"))), 5)


if __name__ == "__main__":
    unittest.main()
