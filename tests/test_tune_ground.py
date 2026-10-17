#!/usr/bin/env python3
"""tune_ground analysis on synthetic pulse logs (no hardware, no camera)."""
import math
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import tune_ground as tg  # noqa: E402

LIMITS = dict(breakaway_s=0.08, spread_s=0.03, overshoot=1.3, direction_deg=3.0, heading_deg=2.0,
              ratio_spread=0.05)


def start_log(breakaway_a, kinetic_a, target=(50, 48, -48, -50), ramp_a_per_s=4.0, t_start=0.5,
              move_s=0.8, step=0.01, peak_rpm=None):
    """One move: each wheel's Iq ref ramps until it reaches breakaway_a[i], then the wheel
    turns, (optionally peaks at peak_rpm[i]) and tracks at kinetic_a[i]."""
    log, t = [], 0.0
    moved_at = [None] * 4
    while t < t_start + move_s + 0.5:
        moving = t_start <= t < t_start + move_s
        tgt = list(target) if moving else [0] * 4
        rpm, iq = [0] * 4, [0.0] * 4
        for i in range(4):
            if not moving:
                continue
            sign = 1 if target[i] > 0 else -1
            ramp = (t - t_start) * ramp_a_per_s
            if moved_at[i] is None and ramp >= breakaway_a[i]:
                moved_at[i] = t
            if moved_at[i] is None:
                iq[i] = sign * ramp
            else:
                dt = t - moved_at[i]
                peak = (peak_rpm or [abs(x) for x in target])[i]
                rpm[i] = sign * round(peak if dt < 0.1 else abs(target[i]))
                iq[i] = sign * kinetic_a[i]
        log.append(dict(t=round(t, 4), target_rpm=tgt, rpm=rpm, iq_ref_a=iq))
        t += step
    return log


def joined(*logs):
    out, offset = [], 0.0
    for log in logs:
        out += [dict(e, t=round(e["t"] + offset, 4)) for e in log]
        offset = out[-1]["t"] + 0.01
    return out


class Friction(unittest.TestCase):
    def test_breakaway_and_kinetic_per_wheel(self):
        log = joined(start_log([1.0, 1.5, 2.0, 2.5], [0.3, 0.4, 0.5, 0.6]),
                     start_log([1.1, 1.4, 2.1, 2.4], [0.3, 0.4, 0.5, 0.6], target=(-50, -48, 48, 50)))
        f = tg.friction_samples(log)
        for name, want, kin in zip(tg.NAMES, (1.05, 1.45, 2.05, 2.45), (0.3, 0.4, 0.5, 0.6)):
            self.assertEqual(len(f[name]["breakaway"]), 2)
            self.assertAlmostEqual(sum(f[name]["breakaway"]) / 2, want, delta=0.06)
            self.assertAlmostEqual(f[name]["kinetic"][0], kin, places=3)

    def test_wheel_that_never_moves_gives_no_breakaway(self):
        f = tg.friction_samples(start_log([1.0, 9.0, 1.0, 1.0], [0.3] * 4))
        self.assertEqual(f[tg.NAMES[1]]["breakaway"], [])
        self.assertEqual(len(f[tg.NAMES[0]]["breakaway"]), 1)

    def test_spread(self):
        self.assertAlmostEqual(tg.spread([1.0, 1.0, 1.0]), 0.0)
        self.assertAlmostEqual(tg.spread([1.0, 2.0, 3.0]), 0.5)
        self.assertEqual(tg.spread([1.0]), math.inf)


class Response(unittest.TestCase):
    def test_overshoot_ratio(self):
        log = start_log([0.2] * 4, [0.3] * 4, peak_rpm=[150, 48, 48, 50])
        over = tg.overshoot(log)
        self.assertAlmostEqual(over[tg.NAMES[0]][0], 3.0)
        self.assertAlmostEqual(over[tg.NAMES[1]][0], 1.0)

    def test_judge_and_next_fraction(self):
        fast_over = tg.response(start_log([0.1] * 4, [0.3] * 4, peak_rpm=[150, 144, 144, 150]))
        slow, over = tg.judge(fast_over, LIMITS)
        self.assertEqual((slow, over), (False, True))
        self.assertEqual(tg.next_fraction(0.8, slow, over), 0.65)
        slow_ok = tg.response(start_log([2.0] * 4, [0.3] * 4))  # 0.5 s to break away
        self.assertEqual(tg.judge(slow_ok, LIMITS), (True, False))
        self.assertEqual(tg.next_fraction(0.8, True, False), 0.9)
        good = tg.response(start_log([0.1] * 4, [0.3] * 4))
        self.assertEqual(tg.judge(good, LIMITS), (False, False))
        self.assertIsNone(tg.next_fraction(0.8, False, False))
        self.assertIsNone(tg.next_fraction(1.0, True, False))  # already at the top

    def test_baseline_overshoot_is_tolerated(self):
        resp = tg.response(start_log([0.1] * 4, [0.3] * 4, peak_rpm=[80, 77, 77, 80]))  # x1.6
        self.assertTrue(tg.judge(resp, LIMITS)[1])
        self.assertFalse(tg.judge(resp, LIMITS, baseline_overshoot=1.6)[1])


def summary(direction_deg, distance=160.0, sideways=2.0, heading=0.5):
    return dict(distance_mm=distance, direction_deg=direction_deg, max_sideways_mm=sideways,
                heading_change_deg=heading, commanded_mm=160.0)


class Camera(unittest.TestCase):
    def test_mount_yaw_removed_and_pass(self):
        mount = 30.0
        moves = [((0, 200, 0), summary(90 + mount)), ((0, -200, 0), summary(-90 + mount + 1)),
                 ((200, 0, 0), summary(0 + mount - 1)), ((-200, 0, 0), summary(180 + mount))]
        a = tg.accept_moves(moves, LIMITS)
        self.assertTrue(a["ok"])
        self.assertAlmostEqual(a["mount_yaw_deg"], mount, delta=0.5)
        self.assertLessEqual(a["worst"]["direction_error_deg"], 1.1)

    def test_fails_on_curve_heading_and_scale_spread(self):
        base = [((0, 200, 0), summary(90)), ((0, -200, 0), summary(-90)),
                ((200, 0, 0), summary(0)), ((-200, 0, 0), summary(180))]
        curved = list(base)
        curved[2] = ((200, 0, 0), summary(10))
        self.assertFalse(tg.accept_moves(curved, LIMITS)["ok"])
        turned = list(base)
        turned[0] = ((0, 200, 0), summary(90, heading=5))
        self.assertFalse(tg.accept_moves(turned, LIMITS)["ok"])
        short = list(base)
        short[1] = ((0, -200, 0), summary(-90, distance=140))
        self.assertFalse(tg.accept_moves(short, LIMITS)["ok"])

    def test_missing_moves(self):
        a = tg.accept_moves([((0, 200, 0), None)], LIMITS)
        self.assertFalse(a["ok"])


if __name__ == "__main__":
    unittest.main()
