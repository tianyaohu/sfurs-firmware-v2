#!/usr/bin/env python3
"""esc_tool breakaway metrics on synthetic pulse logs (no hardware)."""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import esc_tool  # noqa: E402

WHEELS = [name for name, _esc, _sel in esc_tool.ik_link.WHEELS]


def log_for(delays, target=(50, -50, 50, -50), t_start=0.5, step=0.01, end=2.0):
    """One start at t_start; wheel i reaches 30 rpm (right sign) after delays[i] s."""
    log, t = [], 0.0
    while t < end:
        moving = t >= t_start
        tgt = list(target) if moving else [0] * 4
        rpm = [(30 if tgt[i] > 0 else -30) if delays[i] is not None and moving
               and t - t_start >= delays[i] else 0 for i in range(4)]
        log.append(dict(t=round(t, 4), target_rpm=tgt, rpm=rpm))
        t += step
    return log


class Breakaway(unittest.TestCase):
    def test_each_wheel_time_and_spread(self):
        starts = esc_tool.breakaways(log_for([0.10, 0.20, 0.30, 0.40]))
        self.assertEqual(len(starts), 1)
        for name, want in zip(WHEELS, (0.10, 0.20, 0.30, 0.40)):
            self.assertAlmostEqual(starts[0][name], want, places=2)
        s = esc_tool.breakaway_summary(starts)
        self.assertAlmostEqual(s["spread_median_s"], 0.30, places=2)
        for name in WHEELS:
            self.assertEqual((s[name]["starts"], s[name]["moved"]), (1, 1))

    def test_stalled_wheel_and_wrong_sign(self):
        log = log_for([0.1, None, 0.1, 0.1])
        for e in log:  # FL turns the wrong way
            e["rpm"][3] = -e["rpm"][3]
        starts = esc_tool.breakaways(log)
        for name in (WHEELS[1], WHEELS[3]):
            self.assertIsNone(starts[0][name])
        s = esc_tool.breakaway_summary(starts)
        self.assertEqual(s[WHEELS[1]]["moved"], 0)
        self.assertAlmostEqual(s["spread_median_s"], 0.0, places=3)

    def test_small_target_wheels_ignored_and_no_reply(self):
        log = log_for([0.1, 0.1, 0.1, 0.1], target=(50, 10, 50, 50))
        for e in log:
            e["rpm"][2] = None if e["t"] < 0.9 else e["rpm"][2]
        starts = esc_tool.breakaways(log)
        self.assertNotIn(WHEELS[1], starts[0])
        self.assertAlmostEqual(starts[0][WHEELS[2]], 0.4, places=2)

    def test_two_starts(self):
        a = log_for([0.1] * 4, end=1.5)
        b = [dict(e, t=e["t"] + 1.5) for e in log_for([0.2] * 4, target=(-50, 50, -50, 50))]
        starts = esc_tool.breakaways(a + b)
        self.assertEqual(len(starts), 2)
        self.assertAlmostEqual(starts[1][WHEELS[0]], 0.2, places=2)


if __name__ == "__main__":
    unittest.main()
