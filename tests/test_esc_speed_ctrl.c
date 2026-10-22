#include "drive_speed_ctrl.h"
#include <stdio.h>
#include <stdlib.h>

#include "check.h"

static PID_Handle_t pid;
static dsc_cfg_t cfg;
static dsc_state_t s;
static dsc_in_t in;
static int16_t out;
static int raw_own; /* test sets in.raw_* itself */

static void setup(void)
{
  pid = (PID_Handle_t){ .hKpGain = 4096, .hKiGain = 1100, .hKpDivisor = 256, .hKiDivisor = 16384,
                        .hKpDivisorPOW2 = 8, .hKiDivisorPOW2 = 14,
                        .hUpperOutputLimit = 3264, .hLowerOutputLimit = -3264,
                        .wUpperIntegralLimit = 3264 * 16384, .wLowerIntegralLimit = -3264 * 16384 };
  cfg = (dsc_cfg_t){ .enable = 1, .ff_run = 326, .start_from = 272, .start_cap = 1632,
                     .start_slope = 9, .start_hold_ms = 100, .pcap = 272, .pcap_ms = 50,
                     .pcap_updates = 2, .pcap_fallback_ms = 150, .hold_decay = 0 };
  s = (dsc_state_t){0};       /* dsc_reset keeps the learned breakaway */
  raw_own = 0;
  in = (dsc_in_t){ .limit = 3264, .active = 1 };
}
static void tick(int n)
{
  while (n--) {
    if (!raw_own) { in.raw_pos = in.edges_pos; in.raw_neg = in.edges_neg; }
    out = dsc_step(&cfg, &s, &in, &pid);
  }
}
static int32_t integ(void) { return pid.wIntegralTerm >> 14; }
/* hand-off hold the integral should carry: breakaway current above ff_run */
static int32_t hold(void) { const int32_t h = s.start_iq - s.dir * 326; return s.dir * h > 0 ? h : 0; }

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

/* Mat 2026-10-17: the first edge on the ground is often wind-up; dropping to
 * ff_run re-stuck every wheel. The integral holds the hand-off current above
 * ff_run (P is not subtracted, so the ff-design counterexample - P stored as
 * -I, settling negative - cannot happen), P capped on top. */
static void test_handoff_holds_breakaway(void)
{
  setup();
  start(51, 150);
  CHECK(s.state == DSC_RUNNING && s.reason == DSC_R_EDGE);
  CHECK(s.start_iq > 1000 && integ() == hold() && hold() > 0);
  CHECK(out == 272 + s.start_iq);     /* P 816 capped to 272, + hold + ff */
  in.meas_rpm = 30;
  tick(1);
  CHECK(s.holding && integ() == hold()); /* below target: hold kept */
  in.meas_rpm = 51;
  tick(1);
  CHECK(!s.holding && pid.wIntegralTerm == 0 && out == 326); /* at target: hold dropped, ff only */
  setup();
  cfg.start_from = 200;
  start(51, 1);                       /* hand-off below ff_run: no negative hold */
  CHECK(s.start_iq < 326 && pid.wIntegralTerm == 0);
}

static void test_hold_decays_while_capped(void)
{
  setup();
  cfg.hold_decay = 4;
  start(51, 100);
  const int32_t h = integ();
  CHECK(h > 400);
  tick(10);
  CHECK(integ() == h - 40);           /* 4 counts per tick */
  tick(400);                          /* edges, no captures: stays capped until it fails */
  CHECK(s.state == DSC_FAILED || pid.wIntegralTerm == 0);
  setup();
  cfg.hold_decay = 8;
  start(-51, 100);
  CHECK(integ() < -400 && integ() > -8 * 120);
  for (int k = 0; k < 120; ++k) { if (k % 20 == 0) ++in.edges_neg; tick(1); }
  CHECK(s.state == DSC_RUNNING && s.capped && pid.wIntegralTerm == 0); /* decayed to 0, not past */
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
  CHECK(s.capped && integ() == hold());      /* no fresh captures yet: frozen */
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
  CHECK(integ() == hold() && out == first && out == s.start_iq + (4096 * 5 >> 8));
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
  CHECK(s.state == DSC_RUNNING && s.capped && out == 272 + s.start_iq);
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
  CHECK(out == s.start_iq - 272);
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
  CHECK(s.start_iq > 1000 && out == s.start_iq);  /* ramp resumes at the hand-off current */
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
  tick(510);                                   /* no edge: restick (> 500 ms) */
  CHECK(s.state == DSC_STARTING && s.reason == DSC_R_RESTICK && s.at_cap == 0);
  pid.wIntegralTerm = 77 * 16384;
  tick(20);
  ++in.edges_pos;
  tick(1);
  CHECK(s.state == DSC_RUNNING && s.capped && s.t == 0 && s.at_cap == 0 && integ() == hold());
  in.captures += 2;
  in.meas_rpm = 5;
  tick(60);
  CHECK(!s.capped);
  in.ref_rpm = -51;                            /* reversal from slow motion */
  tick(1);
  CHECK(s.state == DSC_STARTING && s.reason == DSC_R_REVERSE);
  ++in.edges_neg;
  tick(1);
  CHECK(s.state == DSC_RUNNING && s.capped && s.t == 0 && s.at_cap == 0 && integ() == hold());
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

/* Re-review 5: after a long forward run the band sits at the far position; a
 * slow reverse start must still hand off on its first edge once rebased. */
static void test_slow_restart_after_long_run(void)
{
  setup();
  dsc_rev_t f = {0};
  uint16_t pos = 0, neg = 0, caps = 0;
  for (int k = 0; k < 100; ++k) { ++pos; ++caps; (void)dsc_reversal_filter(&f, pos, neg, caps, 50); }
  ++pos; ++neg;                             /* pending idle motion, not yet consumed */
  in.ref_rpm = -5;
  for (int k = 0; k < 400 && s.state != DSC_RUNNING; ++k) {
    if (k == 100) ++neg;                    /* first reverse edge of the attempt */
    if (k > 10 && k < 60 && k % 10 == 0) ++pos; /* old-direction coasting after the request */
    const uint8_t was = s.state;
    (void)dsc_reversal_filter(&f, pos, neg, caps, 0);
    in.edges_pos = f.qpos; in.edges_neg = f.qneg; in.captures = f.qcaps;
    in.raw_pos = pos; in.raw_neg = neg; raw_own = 1;
    tick(1);
    if (s.state == DSC_STARTING && was != DSC_STARTING) dsc_rev_rebase(&f, pos, neg, caps);
  }
  CHECK(s.state == DSC_RUNNING && s.reason == DSC_R_EDGE);
  dsc_rev_t o = { .net = (1L << 30), .hi = (1L << 30), .lo = (1L << 30) - 1 };
  (void)dsc_reversal_filter(&o, 1, 0, 0, 0);  /* origin moves, band kept */
  CHECK(o.net == 0 && o.hi == 0 && o.lo == -1 && o.qpos == 1);
}

static void rocking_through_filter(int16_t ref, int *ms, int *max_out)
{
  setup();
  dsc_rev_t f = {0};
  uint16_t pos = 0, neg = 0, caps = 0;
  in.ref_rpm = ref;
  *max_out = 0;
  int k = 0;
  for (; k < 6000 && s.state != DSC_FAILED; ++k) {
    if (k == 80) { ++pos; ++caps; }
    if (k > 80 && k % 6 == 0) { if ((k / 6) % 2) ++pos; else ++neg; ++caps; }
    int16_t rpm = dsc_reversal_filter(&f, pos, neg, caps, (int16_t)((k / 6) % 2 ? 200 : -200));
    in.meas_rpm = rpm;
    in.edges_pos = f.qpos; in.edges_neg = f.qneg; in.captures = f.qcaps;
    in.raw_pos = pos; in.raw_neg = neg; raw_own = 1;
    const uint8_t was = s.state;
    tick(1);
    if (s.state == DSC_STARTING && was != DSC_STARTING) dsc_rev_rebase(&f, pos, neg, caps);
    if (out > *max_out) *max_out = out;
  }
  *ms = k;
}

/* Re-review 7: rocking below DSC_CAP_MIN_RPM must also end, in bounded time. */
static void test_slow_rocking_fails_in_bounded_time(void)
{
  int ms, max_out;
  rocking_through_filter(5, &ms, &max_out);
  CHECK(s.state == DSC_FAILED && ms < 3000 && max_out <= 1632);
  rocking_through_filter(30, &ms, &max_out);
  CHECK(s.state == DSC_FAILED && ms < 3000 && max_out <= 1632);
}

/* Genuine 5 RPM motion (edge every 250 ms) after a raw hand-off inside the
 * band: no restick, no failure over 3 s. */
static void test_slow_motion_no_false_restick(void)
{
  setup();
  dsc_rev_t f = {0};
  uint16_t pos = 0, neg = 0, caps = 0;
  ++pos; (void)dsc_reversal_filter(&f, pos, neg, caps, 0);  /* band at +1 */
  in.ref_rpm = -5;
  int resticks = 0;
  for (int k = 0; k < 3000; ++k) {
    if (k % 250 == 100) ++neg;
    const uint8_t was = s.state;
    (void)dsc_reversal_filter(&f, pos, neg, caps, 0);
    in.edges_pos = f.qpos; in.edges_neg = f.qneg; in.captures = f.qcaps;
    in.raw_pos = pos; in.raw_neg = neg; raw_own = 1;
    tick(1);
    if (s.state == DSC_STARTING && was != DSC_STARTING) {
      dsc_rev_rebase(&f, pos, neg, caps);
      if (s.reason == DSC_R_RESTICK) ++resticks;
    }
  }
  CHECK(s.state == DSC_RUNNING && resticks == 0);
}

/* Re-review 8: genuine stick-slip (one forward edge per 600 ms at +5 RPM, no
 * captures) gains ground every cycle and must not fail. */
static void test_stick_slip_progress_never_fails(void)
{
  setup();
  dsc_rev_t f = {0};
  uint16_t pos = 0, neg = 0, caps = 0;
  in.ref_rpm = 5;
  for (int k = 0; k < 6000; ++k) {
    if (k % 600 == 80) ++pos;
    const uint8_t was = s.state;
    (void)dsc_reversal_filter(&f, pos, neg, caps, 0);
    in.edges_pos = f.qpos; in.edges_neg = f.qneg; in.captures = f.qcaps;
    in.raw_pos = pos; in.raw_neg = neg; raw_own = 1;
    tick(1);
    if (s.state == DSC_STARTING && was != DSC_STARTING) dsc_rev_rebase(&f, pos, neg, caps);
    CHECK(s.state != DSC_FAILED);
  }
}

/* Re-review 9: drives a raw-edge trace through filter + controller; returns
 * 1 if it ever FAILED. */
static int stick_slip_fails(int16_t ref, uint16_t pos0, uint16_t neg0, int coast_edges)
{
  dsc_rev_t f = { .pos = pos0, .neg = neg0 };
  uint16_t pos = pos0, neg = neg0, caps = 0;
  in.ref_rpm = ref;
  for (int k = 0; k < 6000; ++k) {
    if (k < coast_edges * 10 && k % 10 == 5) { if (ref < 0) ++pos; else ++neg; } /* old-way coasting */
    if (k % 600 == 80) { if (ref > 0) ++pos; else ++neg; }
    const uint8_t was = s.state;
    (void)dsc_reversal_filter(&f, pos, neg, caps, 0);
    in.edges_pos = f.qpos; in.edges_neg = f.qneg; in.captures = f.qcaps;
    in.raw_pos = pos; in.raw_neg = neg; raw_own = 1;
    tick(1);
    if (s.state == DSC_STARTING && was != DSC_STARTING) dsc_rev_rebase(&f, pos, neg, caps);
    if (s.state == DSC_FAILED) return 1;
  }
  return 0;
}

static void test_progress_after_long_run_and_reversal(void)
{
  setup();
  in.edges_pos = in.raw_pos = 40000;          /* long forward history, counters past int16 */
  raw_own = 1;
  CHECK(!stick_slip_fails(5, 40000, 0, 0));
  setup();
  CHECK(!stick_slip_fails(-5, 100, 0, 6));    /* reversal after 6 coasting forward edges */
  /* long same-direction run (> 32767 edges) then slow stick-slip */
  setup();
  in.ref_rpm = 100; raw_own = 1;
  uint16_t p = 0;
  dsc_rev_t f = {0};
  for (int k = 0; k < 40000; ++k) {
    p = (uint16_t)(p + 1);
    (void)dsc_reversal_filter(&f, p, 0, (uint16_t)k, 100);
    in.edges_pos = f.qpos; in.captures = f.qcaps; in.raw_pos = p; in.meas_rpm = 100;
    tick(1);
  }
  CHECK(s.state == DSC_RUNNING);
  in.meas_rpm = 0;
  CHECK(!stick_slip_fails(5, p, 0, 0));
}

/* Re-review 10: raw counter wrap through STARTING, and the rebase boundary. */
static void test_progress_wrap_and_rebase(void)
{
  setup();
  in.ref_rpm = 5; raw_own = 1;
  in.raw_pos = 65534; in.edges_pos = 65534;
  tick(5);
  CHECK(s.state == DSC_STARTING);
  in.raw_pos = 65535; tick(1);                 /* hand-off edge */
  CHECK(s.state == DSC_RUNNING && s.anchored && s.best == s.along);
  in.raw_pos = 0; tick(1);                     /* wraps: progress */
  CHECK(s.progressed && s.along == 2);
  setup();
  s.dir = 1; s.anchored = 1; s.along = (1L << 30) + 1; s.best = (1L << 30) + 10;
  in.ref_rpm = 51; in.meas_rpm = 51; raw_own = 1; s.state = DSC_RUNNING;
  s.rp = in.raw_pos; s.rn = in.raw_neg;
  tick(1);                                     /* past 2^30, behind best: origin moves, gap kept */
  CHECK(s.along == 0 && s.best == 9);
  s.anchored = 0; s.along = (1L << 30) + 1; s.best = -(1L << 30);
  tick(1);
  CHECK(s.along == 0 && s.best == 0);          /* unanchored: no stale gap */
}

/* watch_ms: an edge freezes the ramp; only further motion under that constant
 * current hands off (wind-up stops, a free wheel keeps going). */
static void test_watch_confirms_motion(void)
{
  setup();
  cfg.watch_ms = 40;
  in.ref_rpm = 51;
  tick(50);
  ++in.edges_pos;
  tick(1);                            /* wind-up edge: ramp freezes */
  CHECK(s.state == DSC_STARTING && s.watching == DSC_W_CONFIRM);
  const int16_t frozen = out;
  tick(38);
  CHECK(out == frozen && s.state == DSC_STARTING);
  tick(3);
  CHECK(s.watching == DSC_W_NONE && out > frozen); /* it stopped: ramp resumes */
  ++in.edges_pos;
  tick(1);                            /* next wind-up edge, new watch */
  CHECK(s.watching == DSC_W_CONFIRM);
  ++in.edges_pos;
  tick(1);                            /* within DSC_WATCH_MIN_MS: rotor settling */
  CHECK(s.state == DSC_STARTING);
  tick(5);
  ++in.edges_pos;
  tick(1);                            /* still going under constant current */
  CHECK(s.state == DSC_RUNNING && s.reason == DSC_R_EDGE);
  CHECK(s.learned[0] == s.start_iq && s.learned[1] == 0);
}

static void test_watch_ignores_rocking(void)
{
  setup();
  cfg.watch_ms = 40;
  in.ref_rpm = 51;
  tick(20);
  ++in.edges_pos;
  tick(1);
  for (int k = 0; k < 6; ++k) {       /* back and forth across the same edge */
    if (k % 2) ++in.edges_pos; else ++in.edges_neg;
    tick(5);
  }
  CHECK(s.state == DSC_STARTING);
}

static void test_preload_from_learned(void)
{
  setup();
  cfg.watch_ms = 40;
  cfg.preload_pct = 70;
  in.ref_rpm = 51;
  tick(1);
  CHECK(out == 272);                  /* nothing learned yet */
  tick(99);
  ++in.edges_pos;
  tick(5);
  ++in.edges_pos;
  tick(1);
  CHECK(s.state == DSC_RUNNING && integ() > 0);
  const int32_t pre = s.learned[0] * 70 / 100;
  in.ref_rpm = 0;
  tick(1);
  CHECK(s.state == DSC_STOPPED && pid.wIntegralTerm == 0 && out == 0); /* no held current on a stop */
  dsc_reset(&s);                      /* the app resets at every stop */
  CHECK(pre > 272 && s.learned[0] * 70 / 100 == pre);
  in.ref_rpm = 51;
  tick(1);
  CHECK(s.state == DSC_STARTING && out == pre && s.watching == DSC_W_SETTLE);
  ++in.edges_pos;
  tick(9);                            /* the step's wind-up settles: held 10 ms */
  CHECK(out == pre && s.state == DSC_STARTING);
  tick(1);
  CHECK(out == pre + 9);
  in.ref_rpm = 0;
  tick(1);
  in.ref_rpm = -51;
  tick(1);
  CHECK(out == -272);                 /* other direction: nothing learned */
}

/* VESC-style low-speed estimate: 1250 / max(since last edge, last interval). */
static void test_edge_rpm(void)
{
  dsc_rev_t f = {0};
  uint16_t pos = 0;
  CHECK(dsc_edge_rpm(&f) == 0);
  for (int k = 0; k < 100; ++k) { if (k % 25 == 0) ++pos; dsc_reversal_filter(&f, pos, 0, 0, 0); }
  CHECK(f.period == 25 && dsc_edge_rpm(&f) == 50);  /* 25 ms per edge */
  for (int k = 0; k < 74; ++k) dsc_reversal_filter(&f, pos, 0, 0, 0);
  CHECK(dsc_edge_rpm(&f) == 1250 / 100);              /* no edge: falls as 1/t */
  ++pos; dsc_reversal_filter(&f, pos, 0, 0, 0);
  CHECK(dsc_edge_rpm(&f) == 12);                      /* that slow interval */
  dsc_reversal_filter(&f, pos, 1, 0, 0);              /* reversal: not two agreeing */
  CHECK(dsc_edge_rpm(&f) == 0);
  dsc_rev_rebase(&f, pos, 1, 0);
  CHECK(f.period == 0 && dsc_edge_rpm(&f) == 0);
  pos = (uint16_t)(pos + 3); dsc_reversal_filter(&f, pos, 1, 0, 0); /* 3 edges in 1 ms */
  CHECK(f.period == 1 && dsc_edge_rpm(&f) == 1250);
}

static void test_controller(void)
{
  test_ramp_and_failed_start();
  test_ramp_respects_limit();
  test_handoff_holds_breakaway();
  test_hold_decays_while_capped();
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
  test_slow_restart_after_long_run();
  test_slow_rocking_fails_in_bounded_time();
  test_slow_motion_no_false_restick();
  test_stick_slip_progress_never_fails();
  test_progress_after_long_run_and_reversal();
  test_progress_wrap_and_rebase();
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
  test_watch_confirms_motion();
  test_watch_ignores_rocking();
  test_preload_from_learned();
  test_edge_rpm();
  puts("OK");
}

int main(void)
{
  test_controller();
  return 0;
}
