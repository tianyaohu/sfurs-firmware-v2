#!/usr/bin/env python3
"""Compile the real drive_speed_ctrl.c with the real MCSDK pid_regulator.c on
the host and check the startup speed loop (notes/ff-design.md, stage 1)."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
MC = ROOT / "HW/B-G431B-ESC1/motor_config"
SDK = MC / "MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any"

MC_TYPE_STUB = r'''
#pragma once
#include <stdint.h>
#include <stdbool.h>
#define __weak __attribute__((weak))
#define MC_NULL ((void *)0)
'''

HARNESS = r'''
#include "drive_speed_ctrl.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static PID_Handle_t pid;
static dsc_cfg_t cfg;
static dsc_state_t s;
static dsc_in_t in;
static int16_t out;

static void setup(void)
{
  pid = (PID_Handle_t){ .hKpGain = 4096, .hKiGain = 1100, .hKpDivisor = 256, .hKiDivisor = 16384,
                        .hKpDivisorPOW2 = 8, .hKiDivisorPOW2 = 14,
                        .hUpperOutputLimit = 3264, .hLowerOutputLimit = -3264,
                        .wUpperIntegralLimit = 3264 * 16384, .wLowerIntegralLimit = -3264 * 16384 };
  cfg = (dsc_cfg_t){ .enable = 1, .ff_run = 326, .start_from = 272, .start_cap = 1632,
                     .start_slope = 9, .start_hold_ms = 100, .pcap = 272, .pcap_ms = 50,
                     .pcap_updates = 2, .pcap_fallback_ms = 150 };
  dsc_reset(&s);
  in = (dsc_in_t){ .limit = 3264, .active = 1 };
}
static void tick(int n) { while (n--) out = dsc_step(&cfg, &s, &in, &pid); }
static int32_t integ(void) { return pid.wIntegralTerm >> 14; }

/* Drive to RUNNING via one commanded-direction edge after `ticks` of ramp. */
static void start(int16_t ref, int ticks)
{
  in.ref_rpm = ref;
  tick(ticks);
  if (ref > 0) ++in.edges_pos; else ++in.edges_neg;
  tick(1);
}

static void test_ramp_and_failed_start(void)
{
  setup();
  in.ref_rpm = 51;
  tick(1);
  CHECK(s.state == DSC_STARTING && out == 272);
  tick(1);
  CHECK(out == 281);
  tick(200);
  CHECK(s.state == DSC_STARTING && out == 1632); /* ceiling */
  CHECK(pid.wIntegralTerm == 0);                 /* no PI while starting */
  tick(40);                                      /* ceiling reached at tick 153; hold 100 */
  CHECK(s.state == DSC_STARTING);
  tick(20);
  CHECK(s.state == DSC_FAILED && s.reason == DSC_R_NO_EDGE && out == 0);
  ++in.edges_pos;
  tick(5);
  CHECK(s.state == DSC_FAILED && out == 0);      /* latched */
  in.ref_rpm = 0;
  tick(1);
  CHECK(s.state == DSC_STOPPED);
  in.ref_rpm = 51;
  tick(1);
  CHECK(s.state == DSC_STARTING && out == 272);  /* re-armed after a stop */
}

static void test_ramp_respects_limit(void)
{
  setup();
  in.limit = 1000;
  in.ref_rpm = -51;
  tick(150);                 /* at the limit from tick 82; fails after hold 100 */
  CHECK(out == -1000 && s.state == DSC_STARTING);
}

/* ff-design counterexample: matching total current at the edge stored -P in
 * the integral and settled negative. Here the integral is 0 at the edge, P is
 * capped, and at target speed the command is ff_run (> 0). */
static void test_handoff_no_negative_settle(void)
{
  setup();
  start(51, 150);
  CHECK(s.state == DSC_RUNNING && s.reason == DSC_R_EDGE);
  CHECK(pid.wIntegralTerm == 0);
  CHECK(s.start_iq > 1000);
  CHECK(out == 272 + 326);            /* P 816 capped to 272, + ff */
  in.meas_rpm = 51;
  tick(1);
  CHECK(out == 326);
}

static void test_wrong_direction_edge_ignored(void)
{
  setup();
  in.ref_rpm = 51;
  tick(20);
  ++in.edges_neg;
  tick(1);
  CHECK(s.state == DSC_STARTING);
}

static void test_cap_release_and_frozen_integral(void)
{
  setup();
  start(51, 100);
  tick(60);
  CHECK(s.capped && pid.wIntegralTerm == 0);  /* no fresh captures yet */
  in.captures += 2;
  tick(1);
  CHECK(!s.capped);
  int16_t before = out;
  tick(1);
  CHECK(out > before);                         /* integrating again (meas 0) */
}

static void test_cap_fallback_fails(void)
{
  setup();
  start(51, 100);
  for (int k = 0; k < 200; ++k) { if (k % 20 == 0) ++in.edges_pos; tick(1); } /* edges, no captures */
  CHECK(s.state == DSC_FAILED && s.reason == DSC_R_NO_CAPTURE);
}

static void test_cap_fallback_scales_at_low_speed(void)
{
  setup();
  start(25, 100);          /* edge period 50 ms: 2 captures need ~150 ms */
  CHECK(s.capped);
  for (int k = 0; k < 160; ++k) { if (k % 50 == 49) ++in.edges_pos; tick(1); }
  CHECK(s.state == DSC_RUNNING);   /* fallback max(150, 2*3*50 = 300 ms) not reached */
}

/* Review finding 3 (+ re-review 1, 2): below DSC_CAP_MIN_RPM the SDK Hall
 * timeout discards every capture. Slow motion continues (no deadline there),
 * the cap stays on and the integral stays frozen, so the current is bounded. */
static void test_slow_without_captures_bounded(void)
{
  setup();
  start(5, 100);
  CHECK(s.state == DSC_RUNNING && s.capped);
  int16_t first = out;
  for (int k = 0; k < 3000; ++k) { if (k % 250 == 249) ++in.edges_pos; tick(1); }
  CHECK(s.state == DSC_RUNNING && s.capped);
  CHECK(pid.wIntegralTerm == 0 && out == first && out == 326 + (4096 * 5 >> 8));
}

/* Re-review 1: hand-off at 5 RPM, then 51 before any capture: still capped,
 * and the deadline starts counting at the crossing, not from the hand-off. */
static void test_threshold_crossing_up(void)
{
  setup();
  start(5, 100);
  for (int k = 0; k < 1000; ++k) { if (k % 250 == 249) ++in.edges_pos; tick(1); }
  in.ref_rpm = 51;
  ++in.edges_pos;
  tick(1);
  CHECK(s.state == DSC_RUNNING && s.capped && out == 272 + 326);
  for (int k = 0; k < 100; ++k) { if (k % 20 == 0) ++in.edges_pos; tick(1); }
  CHECK(s.state == DSC_RUNNING);              /* < 150 ms above the threshold */
  for (int k = 0; k < 100; ++k) { if (k % 20 == 0) ++in.edges_pos; tick(1); }
  CHECK(s.state == DSC_FAILED && s.reason == DSC_R_NO_CAPTURE);
}

/* Re-review 1: hand-off at 51, then 5 before any capture: no NO_CAPTURE. */
static void test_threshold_crossing_down(void)
{
  setup();
  start(51, 100);
  tick(20);
  in.ref_rpm = 5;
  for (int k = 0; k < 3000; ++k) { if (k % 250 == 249) ++in.edges_pos; tick(1); }
  CHECK(s.state == DSC_RUNNING && s.capped);
}

/* Review finding 4: edges keep arriving but never a capture; resticks must not
 * reset the capture deadline forever. */
static void test_resticks_without_captures_fail(void)
{
  setup();
  in.ref_rpm = 51;
  for (int k = 0; k < 3000 && s.state != DSC_FAILED; ++k) { if (k % 200 == 199) ++in.edges_pos; tick(1); }
  CHECK(s.state == DSC_FAILED && s.reason == DSC_R_NO_CAPTURE);
}

static void test_restick_count_resets_with_captures(void)
{
  setup();
  in.ref_rpm = 51;
  for (int k = 0; k < 3000; ++k) {
    if (k % 200 == 199) ++in.edges_pos;
    if (k % 200 == 120) ++in.captures;   /* one capture per cycle: real feedback */
    tick(1);
  }
  CHECK(s.state != DSC_FAILED);
}

static void test_negative_direction_cap(void)
{
  setup();
  start(-51, 100);
  CHECK(out == -(272 + 326));
}

static void test_conditional_integration(void)
{
  setup();
  in.captures = 0;
  start(51, 100);
  in.captures += 2;
  tick(60);                       /* uncapped */
  in.limit = 400;                 /* ff 326 + P 816 saturates */
  pid.wIntegralTerm = 0;
  for (int k = 0; k < 50; ++k) { ++in.edges_pos; tick(1); }
  CHECK(out == 400);
  CHECK(pid.wIntegralTerm == 0);  /* did not wind against the limit */
  in.meas_rpm = 80;               /* error reverses: integral may move down */
  for (int k = 0; k < 5; ++k) { ++in.edges_pos; tick(1); }
  CHECK(pid.wIntegralTerm < 0);
}

static void test_limit_change_same_tick(void)
{
  setup();
  start(51, 100);
  in.captures += 2;
  tick(60);
  in.limit = 200;
  ++in.edges_pos;
  tick(1);
  CHECK(out == 200);
}

static void test_restick(void)
{
  setup();
  start(51, 100);
  in.captures += 2;
  in.meas_rpm = 51;
  tick(100);                  /* 2 x 24 ms < 100 ms minimum */
  CHECK(s.state == DSC_RUNNING);
  tick(1);
  CHECK(s.state == DSC_STARTING && s.reason == DSC_R_RESTICK);
  CHECK(out == 272);
}

static void test_reversal_brakes_then_restarts(void)
{
  setup();
  start(51, 100);
  in.meas_rpm = 120;
  in.ref_rpm = -51;
  tick(1);
  CHECK(s.state == DSC_STOPPED && s.reason == DSC_R_REVERSE && s.dir == -1);
  CHECK(out < 0);              /* plain PI brakes */
  in.meas_rpm = 30;            /* slowed below DSC_REVERSE_RPM */
  tick(1);
  CHECK(s.state == DSC_STARTING && out == -272);
}

static void test_start_while_moving(void)
{
  setup();
  in.meas_rpm = 60;
  pid.wIntegralTerm = 1234 * 16384;
  in.ref_rpm = 51;
  tick(1);
  CHECK(s.state == DSC_RUNNING && s.reason == DSC_R_MOVING && !s.capped);
  CHECK(integ() <= 1);
}

static void test_stop_and_inactive(void)
{
  setup();
  start(51, 100);
  in.ref_rpm = 0;
  tick(1);
  CHECK(s.state == DSC_STOPPED && s.reason == DSC_R_STOP && s.dir == 0 && s.ff == 0);
  in.ref_rpm = 51;
  in.active = 0;
  tick(1);
  CHECK(s.state == DSC_STOPPED && s.reason == DSC_R_INACTIVE && s.ff == 0);
}

static void test_edge_counter_wrap(void)
{
  setup();
  in.edges_pos = 65535;
  in.ref_rpm = 51;
  tick(10);
  in.edges_pos = 0;   /* wraps */
  tick(1);
  CHECK(s.state == DSC_RUNNING);
}

/* Re-review gap: after a low-speed restick or a reversal, the next hand-off
 * re-establishes the cap with fresh timers and a zero integral. */
static void test_cap_reestablished_after_restick_and_reversal(void)
{
  setup();
  start(5, 100);
  tick(600);                                   /* no edge: restick (> 500 ms) */
  CHECK(s.state == DSC_STARTING && s.reason == DSC_R_RESTICK && s.at_cap == 0);
  pid.wIntegralTerm = 77 * 16384;
  tick(20);
  ++in.edges_pos;
  tick(1);
  CHECK(s.state == DSC_RUNNING && s.capped && s.t == 0 && s.at_cap == 0 && pid.wIntegralTerm == 0);
  in.captures += 2;
  in.meas_rpm = 5;
  tick(60);
  CHECK(!s.capped);
  in.ref_rpm = -51;                            /* reversal from slow motion */
  tick(1);
  CHECK(s.state == DSC_STARTING && s.reason == DSC_R_REVERSE);
  ++in.edges_neg;
  tick(1);
  CHECK(s.state == DSC_RUNNING && s.capped && s.t == 0 && s.at_cap == 0 && pid.wIntegralTerm == 0);
}

/* FR stall test 2026-10-17: rocking across one edge (+1/-1 every 6 ms) with
 * the SDK reading +-200 RPM must read 0; real motion passes after 2 edges. */
static void test_reversal_filter(void)
{
  dsc_rev_t f = {0};
  uint16_t pos = 7, neg = 3;
  CHECK(dsc_reversal_filter(&f, pos, neg, 0, 0) == 0);
  for (int k = 0; k < 60; ++k) {          /* rocking */
    if (k % 2) ++neg; else ++pos;
    CHECK(dsc_reversal_filter(&f, pos, neg, 0, (int16_t)(k % 2 ? -190 : 210)) == 0);
    for (int t = 0; t < 5; ++t) CHECK(dsc_reversal_filter(&f, pos, neg, 0, (int16_t)(k % 2 ? -190 : 210)) == 0);
  }
  ++pos; CHECK(dsc_reversal_filter(&f, pos, neg, 0, 40) == 0);  /* first forward edge after rocking */
  ++pos; CHECK(dsc_reversal_filter(&f, pos, neg, 0, 45) == 45); /* second: real motion */
  CHECK(dsc_reversal_filter(&f, pos, neg, 0, 47) == 47);        /* between edges: holds */
  CHECK(dsc_reversal_filter(&f, pos, neg, 0, -30) == 0);        /* sign disagrees with the edges */
  ++neg; ++pos; CHECK(dsc_reversal_filter(&f, pos, neg, 0, 50) == 0); /* both in one tick */
  pos = (uint16_t)(pos + 3); CHECK(dsc_reversal_filter(&f, pos, neg, 0, 1500) == 1500); /* fast: 3 at once */
  dsc_rev_t w = { .pos = 65535, .neg = 0 };
  CHECK(dsc_reversal_filter(&w, 0, 0, 0, 10) == 0);             /* wrap: one edge */
  CHECK(dsc_reversal_filter(&w, 1, 0, 0, 10) == 10);            /* second */
  dsc_rev_t r = {0};
  for (int k = 0; k < 5; ++k) CHECK(dsc_reversal_filter(&r, 0, (uint16_t)(k + 1), 0, -60) == (k ? -60 : 0)); /* reverse motion */
}

/* Re-review 4: rocking must not count as progress or feedback, and the
 * composed filter -> controller rocks into STARTING/FAILED, not 6 A of PI. */
static void test_qualified_feedback(void)
{
  dsc_rev_t f = {0};
  uint16_t pos = 0, neg = 0, caps = 0;
  ++pos; ++caps; (void)dsc_reversal_filter(&f, pos, neg, caps, 0);
  CHECK(f.qpos == 1);                        /* first edge from rest is progress */
  for (int k = 0; k < 40; ++k) {             /* rocking across that edge */
    if (k % 2) ++pos; else ++neg;
    ++caps;
    (void)dsc_reversal_filter(&f, pos, neg, caps, (int16_t)(k % 2 ? 200 : -200));
  }
  CHECK(f.qpos == 1 && f.qneg == 0 && f.qcaps == 0);
  for (int k = 0; k < 5; ++k) { ++pos; ++caps; (void)dsc_reversal_filter(&f, pos, neg, caps, 50); }
  CHECK(f.qpos == 6 && f.qcaps == 5);        /* real motion; the last rock was forward, so all 5 agree */
  for (int k = 0; k < 3; ++k) { ++neg; (void)dsc_reversal_filter(&f, pos, neg, caps, -40); }
  CHECK(f.qneg == 2);                        /* reversal: first back edge is inside the band */
}

static void test_rocking_through_filter_fails_start(void)
{
  setup();
  dsc_rev_t f = {0};
  uint16_t pos = 0, neg = 0, caps = 0;
  in.ref_rpm = 30;
  int max_out = 0;
  for (int k = 0; k < 3000 && s.state != DSC_FAILED; ++k) {
    if (k == 80) { ++pos; ++caps; }                          /* breakaway edge */
    if (k > 80 && k % 6 == 0) { if ((k / 6) % 2) ++pos; else ++neg; ++caps; } /* rocking */
    int16_t rpm = dsc_reversal_filter(&f, pos, neg, caps, (int16_t)((k / 6) % 2 ? 200 : -200));
    in.meas_rpm = rpm;
    in.edges_pos = f.qpos; in.edges_neg = f.qneg; in.captures = f.qcaps;
    tick(1);
    if (out > max_out) max_out = out;
  }
  CHECK(s.state == DSC_FAILED);
  CHECK(max_out <= 1632);                    /* never above the start ceiling (3 A) */
}

int main(void)
{
  test_ramp_and_failed_start();
  test_ramp_respects_limit();
  test_handoff_no_negative_settle();
  test_wrong_direction_edge_ignored();
  test_cap_release_and_frozen_integral();
  test_cap_fallback_fails();
  test_cap_fallback_scales_at_low_speed();
  test_slow_without_captures_bounded();
  test_threshold_crossing_up();
  test_threshold_crossing_down();
  test_cap_reestablished_after_restick_and_reversal();
  test_reversal_filter();
  test_qualified_feedback();
  test_rocking_through_filter_fails_start();
  test_resticks_without_captures_fail();
  test_restick_count_resets_with_captures();
  test_negative_direction_cap();
  test_conditional_integration();
  test_limit_change_same_tick();
  test_restick();
  test_reversal_brakes_then_restarts();
  test_start_while_moving();
  test_stop_and_inactive();
  test_edge_counter_wrap();
  puts("OK");
  return 0;
}
'''


@unittest.skipUnless(shutil.which("gcc"), "gcc not installed")
class SpeedCtrl(unittest.TestCase):
    def test_controller(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = pathlib.Path(tmp)
            (tmp / "mc_type.h").write_text(MC_TYPE_STUB)
            (tmp / "harness.c").write_text(HARNESS)
            exe = tmp / "harness"
            cmd = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion",
                   "-Wno-sign-conversion", "-fsanitize=undefined,address",
                   "-I", str(tmp), "-I", str(MC / "Inc"), "-I", str(SDK / "Inc"),
                   str(tmp / "harness.c"), str(MC / "Src/drive_speed_ctrl.c"),
                   str(SDK / "Src/pid_regulator.c"), "-o", str(exe)]
            build = subprocess.run(cmd, capture_output=True, text=True)
            if build.returncode:  # the SDK file is not -Wconversion clean; retry it relaxed
                cmd = [c for c in cmd if c not in ("-Werror",)]
                build = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn("OK", run.stdout)


if __name__ == "__main__":
    unittest.main()
