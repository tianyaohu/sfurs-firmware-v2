#!/usr/bin/env python3
"""Hardware-free tests for the FR stand auto-tune orchestrator."""
import datetime
import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("auto_tune_esc",
                                              ROOT / "scripts" / "auto_tune_esc.py")
at = importlib.util.module_from_spec(spec)
spec.loader.exec_module(at)

T0 = datetime.datetime(2026, 10, 15, 13, 0, 0, tzinfo=datetime.timezone.utc)


def make_window(rpm_series, state=6, iq_track=0.0, id_counts=0.0,
                iqref_counts=-300.0, period_s=0.05):
    """Synthetic steady-run window: 0.5 s pre-roll IDLE then RUN samples."""
    window = []
    t = T0
    for _ in range(10):  # 0.5 s IDLE pre-roll (excluded by settle)
        window.append({"utc": t, "rpm": 0, "state": 0, "iqref_counts": 0.0,
                       "iq_counts": 0.0, "id_counts": 0.0, "current_valid": False,
                       "stop_reason": 2, "cal_result": 2, "cal_valid": 1,
                       "armed": 0, "gain_set_result": 1, "speed_kp_gain": -18600,
                       "speed_ki_gain": -72})
        t += datetime.timedelta(seconds=period_s)
    for rpm in rpm_series:
        window.append({"utc": t, "rpm": rpm, "state": state,
                       "iqref_counts": iqref_counts,
                       "iq_counts": iqref_counts + iq_track,
                       "id_counts": id_counts, "current_valid": True,
                       "stop_reason": 0, "cal_result": 2, "cal_valid": 1,
                       "armed": at.ARM_MAGIC, "gain_set_result": 1,
                       "speed_kp_gain": -18600, "speed_ki_gain": -72})
        t += datetime.timedelta(seconds=period_s)
    return window


class MathTests(unittest.TestCase):
    def test_quantized_target_truncates_toward_zero(self):
        self.assertEqual(at.quantized_target_rpm(51), 48)
        self.assertEqual(at.quantized_target_rpm(-51), -48)
        self.assertEqual(at.quantized_target_rpm(150), 150)
        self.assertEqual(at.quantized_target_rpm(250), 246)
        self.assertEqual(at.quantized_target_rpm(29), 24)
        self.assertEqual(at.quantized_target_rpm(-29), -24)

    def test_ti_ms_matches_skill_table(self):
        # Baseline: Ti = 4.13 s; 12x Ki (balanced): Ti = 344 ms.
        self.assertAlmostEqual(at.ti_ms(-18600, -72), 4133, delta=2)
        self.assertAlmostEqual(at.ti_ms(-18600, -864), 344, delta=1)
        self.assertAlmostEqual(at.ti_ms(-18600, -1152), 258, delta=1)

    def test_ki_solver_inverts_the_stall_formula(self):
        # Iq(t) = kp*U + ki*U*(t-250); solver must clear margin*I_break.
        i_break, u_units, t_break, margin = 0.65, 8, 1200, 1.3
        num, ti = at.ki_numerator_for_breakaway(i_break, -18600, u_units,
                                                t_break, margin)
        kp = 18600 / 1024
        ki = abs(num) / 16384
        iq_at_break_a = (kp * u_units + ki * u_units * (t_break - 250)) / 544.0
        self.assertGreaterEqual(iq_at_break_a, margin * i_break)
        self.assertLess(iq_at_break_a, margin * i_break + 0.01)  # tightest ceil
        self.assertTrue(200 <= ti <= 450)
        self.assertEqual(num, -678)
        # A larger breakaway demand needs a proportionally larger Ki.
        num2, _ = at.ki_numerator_for_breakaway(2.0, -18600, 8)
        self.assertLess(num2, num)  # more negative = stronger


class StatsTests(unittest.TestCase):
    def test_clean_run_passes(self):
        window = make_window([48, 49, 47, 48, 48, 49, 47, 48, 48, 47] * 4,
                             iq_track=8.0, id_counts=12.0)
        stats = at.run_stats(window, 51)
        self.assertEqual(stats["target_quantized_rpm"], 48)
        self.assertEqual(stats["sign_reversals"], 0)
        self.assertEqual(stats["max_zero_streak_s"], 0.0)
        self.assertLessEqual(stats["median_abs_track_err_rpm"], 2)
        hunting, reasons = at.hunting_verdict(stats, 51)
        self.assertFalse(hunting, reasons)
        ok, reasons = at.current_loop_verdict(stats)
        self.assertTrue(ok, reasons)

    def test_p2_style_hunting_detected(self):
        # Mirrors FR P2 seq 20: -36..+90 swings plus a 1.1 s zero streak.
        series = [-36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 90, 45, -20, 60, -36, 80, 12, -30, 66, 0, 40, -25]
        stats = at.run_stats(make_window(series), 29)
        hunting, reasons = at.hunting_verdict(stats, 29)
        self.assertTrue(hunting)
        self.assertGreaterEqual(stats["sign_reversals"], 2)
        self.assertGreater(stats["max_zero_streak_s"], 0.3)
        self.assertGreaterEqual(len(reasons), 2)

    def test_single_sign_cross_is_not_hunting_but_p2p_is(self):
        series = [48] * 20 + [-48] * 3 + [48] * 17  # one there-and-back blip
        stats = at.run_stats(make_window(series), 51)
        hunting, _ = at.hunting_verdict(stats, 51)
        self.assertTrue(hunting)  # 96 RPM p2p > 3x51 would be False; check rule
        # p2p = 96 <= 3*51=153 and reversals == 2 -> flagged by reversal rule
        self.assertEqual(stats["sign_reversals"], 2)

    def test_insufficient_evidence_is_hunting(self):
        stats = at.run_stats(make_window([48, 49]), 51)
        self.assertTrue(at.hunting_verdict(stats, 51)[0])
        self.assertIn("insufficient", stats["hunt_reasons"][0])

    def test_current_loop_verdict_bounds(self):
        stats = at.run_stats(make_window([48] * 30, iq_track=60.0), 51)
        ok, reasons = at.current_loop_verdict(stats)
        self.assertFalse(ok)  # 60 counts = 0.110 A > 0.05 A
        stats = at.run_stats(make_window([48] * 30, iq_track=8.0, id_counts=70.0), 51)
        ok, reasons = at.current_loop_verdict(stats)
        self.assertFalse(ok)  # 70 counts = 0.129 A > 0.10 A
        stats = at.run_stats(make_window([48] * 30, iq_track=8.0, id_counts=12.0), 51)
        self.assertTrue(at.current_loop_verdict(stats)[0])
        # No valid current samples -> cannot pass.
        window = make_window([48] * 30)
        for s in window:
            s["current_valid"] = False
        self.assertFalse(at.current_loop_verdict(at.run_stats(window, 51))[0])


class SelectionTests(unittest.TestCase):
    def per(self, hunting):
        return {c: {"hunting": c in hunting} for c in at.KP_CANDIDATES}

    def test_no_hunting_caps_at_24(self):
        sel = at.select_kp(self.per(set()))
        self.assertEqual(sel["chosen"], -22528)  # highest candidate <= 24.0
        self.assertEqual(sel["chosen_counts_per_unit"], 22.0)

    def test_margin_below_first_hunting(self):
        sel = at.select_kp(self.per({-30720, -32767}))  # hunts at 30
        self.assertEqual(sel["chosen"], -22528)  # ceiling 0.75*30 = 22.5
        self.assertEqual(sel["first_hunting"], 30720)

    def test_hunting_at_22_keeps_baseline_with_warning(self):
        sel = at.select_kp(self.per({-22528, -26624, -30720, -32767}))
        self.assertEqual(sel["chosen"], at.KP_FLOOR)
        self.assertIn("baseline", sel["reason"])

    def test_baseline_hunting_aborts(self):
        sel = at.select_kp(self.per({-18600, -22528}))
        self.assertIsNone(sel["chosen"])
        self.assertIn("ABORT", sel["reason"])

    def test_missing_baseline_raises(self):
        with self.assertRaises(ValueError):
            at.select_kp({-22528: {"hunting": False}})


class NucleoBenchTests(unittest.TestCase):
    ARMED, RUNNING, IDLE = 1, 2, 0

    class FakeTarget:
        """Emulates the wheel_bench volatile contract (request consume)."""
        def __init__(self, addrs):
            self.mem = {a: 0 for a in addrs.values()}
            self.writes = []

        def read32(self, addr):
            return self.mem.get(addr, 0)

        def write32(self, addr, value):
            self.mem[addr] = value & 0xFFFFFFFF
            self.writes.append((addr, value))
            if addr == ADDRS["wheel_bench_request"]:
                code = value
                self.mem[ADDRS["wheel_bench_request"]] = 0
                if code == at.ARM2:
                    self.mem[ADDRS["wheel_bench_state"]] = 1
                    self.mem[ADDRS["wheel_bench_result"]] = 1
                elif code == at.TUNE:
                    if self.mem[ADDRS["wheel_bench_state"]] == 1:
                        self.mem[ADDRS["wheel_bench_result"]] = 6
                    else:
                        self.mem[ADDRS["wheel_bench_result"]] = 4
                elif code == at.RUN2:
                    self.mem[ADDRS["wheel_bench_state"]] = 2
                    self.mem[ADDRS["wheel_bench_result"]] = 2
                elif code == at.STOP:
                    self.mem[ADDRS["wheel_bench_state"]] = 0
                    self.mem[ADDRS["wheel_bench_result"]] = 3

    def setUp(self):
        global ADDRS
        ADDRS = {name: 0x20001000 + i * 4 for i, name in enumerate(at.BENCH_SYMBOLS)}
        self.target = self.FakeTarget(ADDRS)
        self.bench = at.NucleoBench(self.target, ADDRS)

    def test_arm_tune_run_stop_sequence_and_packing(self):
        self.bench.select(1, 1)
        self.bench.arm()
        self.bench.tune(-22528, -864)
        # poke() writes int32 sign-extended as u32.
        self.assertEqual(self.target.mem[ADDRS["wheel_bench_tune_kp"]],
                         0xFFFFA800)
        self.assertEqual(self.target.mem[ADDRS["wheel_bench_tune_ki"]],
                         0xFFFFFCA0)
        self.bench.run(250, 1)
        self.assertEqual(self.target.mem[ADDRS["wheel_bench_speed_rpm"]], 250)
        self.bench.stop()
        self.assertEqual(self.target.mem[ADDRS["wheel_bench_state"]], 0)

    def test_tune_rejected_raises(self):
        self.bench.select(1, 1)
        with self.assertRaisesRegex(RuntimeError, "TUNE not queued"):
            self.bench.tune(-22528, -864)  # never armed -> result 4

    def test_poke_readback_mismatch_raises(self):
        def bad_write(addr, value):
            pass  # write silently lost
        self.target.write32 = bad_write
        with self.assertRaisesRegex(RuntimeError, "readback mismatch"):
            self.bench.poke("wheel_bench_wheel", 1)


class SampleIoTests(unittest.TestCase):
    def test_load_samples_and_slice_window(self):
        records = [
            {"type": "header", "schema": 1},
            {"type": "sample", "sequence": 0,
             "blocks": [{"utc_end": "2026-10-15T13:00:00.000000+00:00"}],
             "raw": {"measured_rpm": 0, "mci_state": 0, "iq_reference_counts": 0,
                     "measured_iq_counts": 0, "measured_id_counts": 0,
                     "current_valid": 0, "stop_reason": 2, "calibration_result": 2,
                     "calibration_valid": 1, "armed": 0}},
            {"type": "sample", "sequence": 1,
             "blocks": [{"utc_end": "2026-10-15T13:00:01.000000+00:00"}],
             "raw": {"measured_rpm": 48, "mci_state": 6, "iq_reference_counts": -300,
                     "measured_iq_counts": -290, "measured_id_counts": 5,
                     "current_valid": 1, "stop_reason": 0, "calibration_result": 2,
                     "calibration_valid": 1, "armed": at.ARM_MAGIC,
                     "gain_set_result": 1, "speed_kp_gain": -22528,
                     "speed_ki_gain": -864}},
        ]
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as fh:
            for record in records:
                fh.write(json.dumps(record) + "\n")
            path = fh.name
        samples = at.load_samples(path)
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[1]["speed_kp_gain"], -22528)
        self.assertEqual(samples[1]["armed"], at.ARM_MAGIC)
        start = datetime.datetime(2026, 10, 15, 13, 0, 0, 500000,
                                  tzinfo=datetime.timezone.utc)
        end = start + datetime.timedelta(seconds=2)
        window = at.slice_window(samples, start, end)
        self.assertEqual(len(window), 1)
        self.assertEqual(window[0]["rpm"], 48)
        pathlib.Path(path).unlink()

    def test_gain_acceptance_predicate_uses_count(self):
        # Stale result==1 with the previous count must not prove acceptance.
        prev_count = 3
        expect = prev_count + 1
        stale = {"gain_set_count": 3, "gain_set_result": 1,
                 "speed_kp_gain": -22528, "speed_ki_gain": -72}
        fresh = {"gain_set_count": 4, "gain_set_result": 1,
                 "speed_kp_gain": -22528, "speed_ki_gain": -72}
        pred = (lambda r: r.get("gain_set_count") == expect
                and r.get("gain_set_result") == 1
                and r.get("speed_kp_gain") == -22528
                and r.get("speed_ki_gain") == -72)
        self.assertFalse(pred(stale))
        self.assertTrue(pred(fresh))


if __name__ == "__main__":
    unittest.main()
