#!/usr/bin/env python3
"""capture_analysis against a simulated RL plant under the MCSDK digital PI."""
import math
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import capture_analysis as ca  # noqa: E402


def simulate(r, ell, vbus, kp, ki, hold_a=1.0, step_a=3.0, v_off=0.15, delay=1, n=512, pre=64):
    """Discrete plant: the Vd computed at sample k drives the current from
    k+delay on; dead time adds a constant offset; integers like the ESC."""
    t = 1.0 / ca.PWM_HZ
    vs = ca.volts_per_s16(vbus)
    cur = hold_a
    integral = round(((r * hold_a + v_off) / vs) * ca.TF_KIDIV)  # settled at the hold
    pending = [round((r * hold_a + v_off) / vs)] * delay
    out = []
    for k in range(n):
        ref = step_a if k >= pre else hold_a
        meas = round(cur * ca.COUNTS_PER_AMP)
        err = round(ref * ca.COUNTS_PER_AMP) - meas
        integral += ki * err
        vd = max(-32767, min(32767, (kp * err) // ca.TF_KPDIV + integral // ca.TF_KIDIV))
        out.append(dict(id=meas, iq=0, vd=vd, vq=0, id_ref=round(ref * ca.COUNTS_PER_AMP), angle=0))
        pending.append(vd)
        applied = pending.pop(0) * vs
        cur += t / ell * (applied - r * cur - v_off)
    return dict(vbus_centivolts=round(vbus * 100), status=2, samples=n), out


class CaptureAnalysisTest(unittest.TestCase):
    def check(self, r, ell, kp, ki, delay=1):
        info, samples = simulate(r, ell, 12.1, kp, ki, delay=delay)
        a = ca.analyze(info, samples)
        self.assertAlmostEqual(a["r_ohm"], r, delta=0.03 * r)
        self.assertAlmostEqual(a["l_h"], ell, delta=0.1 * ell)
        self.assertEqual(a["delay_periods"], delay)
        self.assertAlmostEqual(a["hold_a"], 1.0, delta=0.01)
        self.assertAlmostEqual(a["step_a"], 3.0, delta=0.01)
        self.assertLess(abs(a["steady_error_a"]), 0.02)
        return a

    def test_generic_profile_motor(self):
        a = self.check(0.22, 110e-6, 3391, 3343)
        self.assertIsNotNone(a["rise_us"])

    def test_heavier_motor_slow_loop(self):
        a = self.check(0.8, 600e-6, 3391, 3343, delay=2)
        self.assertGreater(a["rise_us"], 200)  # the generic gains are slow on this motor

    def test_gains_roundtrip_bandwidth(self):
        kp, ki = ca.pi_gains(0.22, 110e-6, 12.0, fc_hz=1100)
        self.assertAlmostEqual(kp, 3391, delta=60)  # what Workbench chose for 12 V
        self.assertAlmostEqual(ki, 3343, delta=60)
        self.assertAlmostEqual(ca.loop_bandwidth_hz(kp, 110e-6, 12.0), 1100, delta=20)

    def test_tuned_gains_give_clean_step(self):
        r, ell = 0.8, 600e-6
        kp, ki = ca.pi_gains(r, ell, 12.1, fc_hz=800)
        info, samples = simulate(r, ell, 12.1, kp, ki, delay=1)
        a = ca.analyze(info, samples)
        self.assertLess(a["overshoot_pct"], 25)
        self.assertLess(a["rise_us"], 1000)

    def test_rejects_flat_capture(self):
        info, samples = simulate(0.3, 1e-4, 12.0, 3391, 3343, step_a=1.0)
        with self.assertRaises(ValueError):
            ca.analyze(info, samples)


if __name__ == "__main__":
    unittest.main()
