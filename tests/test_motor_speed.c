/* Speed loop (motor_speed.c) against technical-development-plans/esc-command-v2/E-speed-loop.md. Inputs are Hall
 * counter values and MCSDK fields as drive_app samples them; no motor or wheel model. Each case asserts a
 * production decision (rounding, a threshold, a timer) with the exact fixed-point value E specifies. */
#include "motor_speed.h"
#include "check.h"

#include <string.h>

static const struct motor_speed_config dflt = { .kp = 1024, .ki = 553, .stall_ramp_ma_per_s = 10000,
                                                .stall_ms = 500 };
static struct motor_speed s;
static struct motor_speed_inputs in;
static struct motor_speed_outputs out;

static void setup_cfg(const struct motor_speed_config *cfg)
{
  memset(&in, 0, sizeof in);
  in.mode = MOTOR_SPEED_RUNNING;
  in.limit_counts = 3264; /* IQMAX 6 A */
  motor_speed_init(&s, cfg, in.edges_pos, in.edges_neg);
}

static void setup(void) { setup_cfg(&dflt); }
static void tick(void) { motor_speed_tick(&s, &in, &out); }
static void ticks(unsigned n) { while (n--) tick(); }
static void edge(int dir, unsigned n)
{
  if (dir > 0) in.edges_pos = (uint16_t)(in.edges_pos + n);
  else in.edges_neg = (uint16_t)(in.edges_neg + n);
}
/* New setpoint and the full 20-tick interpolation, wheel not moving. */
static void settle(int16_t rpm, int16_t ff_ma)
{
  motor_speed_set(&s, rpm, ff_ma);
  ticks(MOTOR_PROTOCOL_PERIOD_MS);
  CHECK_EQ(out.ref_rpm, rpm);
}

/* P = (Kp*e)>>8 (floor), e clamped to +-5952, P clamped to +-L; L < 0 reads as 0. */
static void p_term(void)
{
  struct motor_speed_config c = dflt;
  c.ki = 0;
  setup_cfg(&c);
  settle(100, 0);
  CHECK_EQ(out.p_counts, 400);
  CHECK_EQ(out.iq_ref_counts, 400);
  c.kp = 100;
  setup_cfg(&c);
  settle(3, 0);
  CHECK_EQ(out.p_counts, 1); /* 300 >> 8 */
  setup_cfg(&c);
  settle(-3, 0);
  CHECK_EQ(out.p_counts, -2); /* -300 >> 8 floors */
  c.kp = 1;
  setup_cfg(&c);
  settle(32000, 0);
  CHECK_EQ(out.p_counts, 23); /* e clamped: 5952 >> 8, not 32000 >> 8 = 125 */
  setup_cfg(&c);
  settle(-32000, 0);
  CHECK_EQ(out.p_counts, -24); /* -5952 >> 8 */
  c.kp = 32767; /* largest gain at the largest error: no int32 overflow, clamped to L */
  setup_cfg(&c);
  settle(32000, 0);
  CHECK_EQ(out.p_counts, 3264);
  CHECK_EQ(out.iq_ref_counts, 3264);
  CHECK(out.saturated);
  in.limit_counts = -5;
  tick();
  CHECK_EQ(out.p_counts, 0);
  CHECK_EQ(out.iq_ref_counts, 0);
}

/* (b): one tick adds Ki*e = 553*e in Q14; a gain change clears I, other config keeps it. */
static void ki_step(void)
{
  setup();
  settle(100, 0);
  int32_t before = s.i_q14;
  tick();
  CHECK_EQ(s.i_q14 - before, 55300);
  CHECK_EQ(out.i_counts, s.i_q14 >> 14);
  CHECK_EQ(out.iq_ref_counts, 400 + (s.i_q14 >> 14));
  struct motor_speed_config c = dflt;
  c.stall_ms = 900;
  c.stall_ramp_ma_per_s = 5000;
  motor_speed_configure(&s, &c);
  CHECK(s.i_q14 != 0);
  c.ki = 600;
  motor_speed_configure(&s, &c);
  CHECK_EQ(s.i_q14, 0);
  tick();
  c.kp = 1000;
  CHECK(s.i_q14 != 0);
  motor_speed_configure(&s, &c);
  CHECK_EQ(s.i_q14, 0);
}

/* Saturated high: I frozen while e > 0, integrates while e < 0 (moving back inside); mirrored for r < 0.
 * ff 1000 mA = 544 counts against L = 100 keeps the total saturated whatever I is. */
static void conditional_integration(void)
{
  for (int dir = -1; dir <= 1; dir += 2) {
    setup();
    in.limit_counts = 100;
    settle((int16_t)(dir * 100), (int16_t)(dir * 1000));
    CHECK_EQ(out.ff_counts, dir * 544);
    CHECK(out.saturated);
    CHECK_EQ(out.iq_ref_counts, dir * 100);
    int32_t before = s.i_q14;
    tick(); /* e = r - 0 pushes further out: frozen */
    CHECK(out.saturated);
    CHECK_EQ(s.i_q14, before);
    /* One edge per 11 ms (~114 RPM, under the 115 RPM bleed line), MCSDK averaging 110 RPM: e = -10, small
     * enough that I stays clear of its +-L clamp while ff keeps the total saturated. */
    in.mcsdk_buffer_full = 1U;
    in.mcsdk_avg_rpm = (int16_t)(dir * 110);
    for (unsigned n = 0; n < 6; ++n) {
      edge(dir, 1U);
      tick();
      ticks(10);
    }
    CHECK(out.valid);
    CHECK_EQ(out.w_hat_rpm, dir * 110);
    before = s.i_q14;
    tick(); /* no edge this tick, so no bleed: e = -10 along dir pulls back in */
    CHECK(out.saturated);
    CHECK_EQ(out.w_hat_rpm, dir * 110);
    CHECK_EQ(s.i_q14 - before, 553 * dir * -10);
  }
  /* Inside the limit both ways: plain integration. */
  setup();
  settle(-100, 0);
  int32_t before = s.i_q14;
  tick();
  CHECK(!out.saturated);
  CHECK_EQ(s.i_q14 - before, -55300);
}

/* A new L acts on P, I and the output in the tick it arrives. */
static void limit_same_tick(void)
{
  setup();
  settle(100, 0);
  ticks(50);
  CHECK(out.iq_ref_counts > 400);
  in.limit_counts = 50;
  tick();
  CHECK_EQ(out.p_counts, 50);
  CHECK_EQ(out.iq_ref_counts, 50);
  CHECK(s.i_q14 <= 50 * 16384);
  in.limit_counts = 3264;
  tick();
  CHECK(out.iq_ref_counts > 50);
}

/* (a): starts when slip reaches 4 edges (reference travel sum |r| >= 4*1250), steps 89129 Q14 per tick at
 * 10 A/s, stops on progress, pauses for |r| < 10 or |w_hat| >= 45, clamps at L - p - ff; slip clamps at 4;
 * rocking makes no progress. */
static void stall_ramp(void)
{
  setup();
  motor_speed_set(&s, 125, 0);
  int32_t travel = 0, before = 0;
  unsigned t = 0;
  for (;;) {
    before = s.i_q14;
    tick();
    ++t;
    travel += out.ref_rpm;
    if (out.stall_ramp) break;
    CHECK(travel < 4 * 1250);
    CHECK(t < 1000);
  }
  CHECK(travel >= 4 * 1250);
  CHECK_EQ(s.i_q14 - before, 89129);
  for (int n = 0; n < 50; ++n) {
    before = s.i_q14;
    tick();
    CHECK(out.stall_ramp);
    CHECK_EQ(s.i_q14 - before, 89129);
  }
  /* Slip has been clamped at 4 edges for a long time: one progress edge brings it to 4 - 1 + 0.1, and the
   * ramp is back after exactly 9 more ticks (an unclamped deficit would never let it stop). */
  edge(1, 1U);
  tick();
  CHECK(!out.stall_ramp);
  for (int n = 1; n <= 9; ++n) {
    tick();
    CHECK_EQ(out.stall_ramp, n == 9);
  }
  /* Rocking: back one edge, forward one, repeatedly: never a new furthest position. */
  for (int n = 0; n < 40; ++n) {
    edge(n % 2 ? 1 : -1, 1U);
    before = s.i_q14;
    tick();
    CHECK(out.stall_ramp);
    CHECK_EQ(s.i_q14 - before, 89129);
    CHECK(!out.valid);
  }
  CHECK_EQ(in.edges_pos - in.edges_neg, 1);
  edge(-1, 1U); /* rocking ends behind its furthest position */
  tick();
  /* Pause: |r| < 10 (same sign, so nothing is cleared), then back. */
  motor_speed_set(&s, 9, 0);
  ticks(MOTOR_PROTOCOL_PERIOD_MS);
  CHECK_EQ(out.ref_rpm, 9);
  before = s.i_q14;
  tick();
  CHECK(!out.stall_ramp);
  CHECK(s.i_q14 != 0);
  motor_speed_set(&s, 125, 0);
  tick();
  CHECK_EQ(out.ref_rpm, 14);
  CHECK(out.stall_ramp);
  /* Pause: back-driven at >= 45 RPM (two backward edges 20 ms apart: 62 RPM). */
  edge(-1, 1U);
  tick();
  ticks(19);
  edge(-1, 1U);
  tick();
  CHECK(out.valid);
  CHECK_EQ(out.w_hat_rpm, -62);
  CHECK(!out.stall_ramp);

  /* Clamp: with Ki 0 the integrator comes only from the ramp and stops exactly at L - p - ff. */
  struct motor_speed_config c = dflt;
  c.ki = 0;
  setup_cfg(&c);
  in.limit_counts = 600;
  settle(125, 0); /* p = 500 */
  int ramped = 0;
  for (int n = 0; n < 400; ++n) {
    tick();
    ramped |= out.stall_ramp;
    CHECK(s.i_q14 <= 100 * 16384);
  }
  CHECK(ramped);
  CHECK_EQ(s.i_q14, 100 * 16384);
  CHECK_EQ(out.iq_ref_counts, 600);
  /* Same with ff along r: 1000 mA = 544 counts, cap = 600 - 500 - 544 < 0: the ramp adds nothing. */
  setup_cfg(&c);
  in.limit_counts = 600;
  settle(125, 1000);
  ticks(400);
  CHECK_EQ(s.i_q14, 0);
  /* Mirrored. */
  setup_cfg(&c);
  in.limit_counts = 600;
  settle(-125, 0);
  ticks(400);
  CHECK_EQ(s.i_q14, -100 * 16384);
  /* Step from the configured slope: 2 A/s and 30 A/s. */
  c.stall_ramp_ma_per_s = 2000;
  setup_cfg(&c);
  settle(400, 0);
  for (int n = 0; n < 20 && !out.stall_ramp; ++n) tick();
  CHECK(out.stall_ramp);
  before = s.i_q14;
  tick();
  CHECK_EQ(s.i_q14 - before, 17826);
  c.stall_ramp_ma_per_s = 30000;
  motor_speed_configure(&s, &c);
  before = s.i_q14;
  tick();
  CHECK_EQ(s.i_q14 - before, 267387);
}

/* (c) threshold, exact: 1250*k/(span+1) >= 1.15*|r|. Edge at t0, then k edges at t0 + span. Returns whether
 * the second edge tick bled, and checks the halving. */
static int bleed_at(int16_t r, unsigned span, unsigned k)
{
  setup();
  in.limit_counts = 20000; /* P alone must not saturate at 1000 RPM, or the stall ramp holds I at 0 */
  settle(r, 0);
  ticks(5);
  const int dir = r > 0 ? 1 : -1;
  edge(dir, 1U);
  tick();
  CHECK(!out.bleed);
  ticks(span - 1U);
  edge(dir, k);
  const int32_t before = s.i_q14;
  CHECK(dir * before > 0);
  tick();
  if (out.bleed) CHECK_EQ(s.i_q14, before / 2);
  return out.bleed;
}

static void bleed(void)
{
  CHECK(bleed_at(108, 9, 1));   /* 1250/10 = 125 >= 124.2 */
  CHECK(!bleed_at(109, 9, 1));  /* 125 < 125.35 */
  CHECK(bleed_at(120, 8, 1));   /* 1250/9 = 138.9 >= 138 */
  CHECK(!bleed_at(121, 8, 1));
  CHECK(bleed_at(1000, 24, 23));  /* exactly 1.15x: 1250*23/25 = 1150 */
  CHECK(!bleed_at(1001, 24, 23));
  CHECK(!bleed_at(500, 7, 20));   /* run spans < 8 ms: not judged */
  CHECK(bleed_at(-108, 9, 1));
  CHECK(!bleed_at(-109, 9, 1));

  /* Odd negative I halves toward zero: -2k-1 -> -k, not -k-1. */
  setup();
  settle(-108, 0);
  ticks(5);
  s.i_q14 |= 1; /* force odd: the rule, not the history, is under test */
  edge(-1, 1U);
  tick();
  ticks(8);
  edge(-1, 1U);
  int32_t before = s.i_q14;
  CHECK(before < 0 && (before & 1));
  tick();
  CHECK(out.bleed);
  CHECK_EQ(s.i_q14, before / 2);
  CHECK(s.i_q14 != (before >> 1));

  /* I against the command: no bleed even on a fast progress edge. Held first so the stall ramp builds a large
   * I along r: it bleeds while positive, then the 625 RPM error drives it negative. */
  setup();
  settle(108, 0);
  ticks(400);
  int fired = 0, against = 0;
  for (int n = 0; n < 200; ++n) {
    if (n % 2 == 0) edge(1, 1U); /* 625 RPM */
    before = s.i_q14;
    tick();
    fired |= out.bleed;
    if (n % 2 == 0 && n > 20 && before < 0) {
      ++against;
      CHECK(!out.bleed);
    }
  }
  CHECK(fired);
  CHECK(against > 10);

  /* Rocking breaks the run: a forward edge after a backward one is not a progress edge. */
  setup();
  settle(108, 0);
  edge(1, 1U);
  tick();
  ticks(4);
  edge(-1, 1U);
  tick();
  ticks(4);
  edge(1, 1U); /* back to the furthest position: no progress */
  tick();
  edge(1, 1U); /* new run starts here */
  before = s.i_q14;
  tick();
  CHECK(!out.bleed);
}

/* Edge times of a wheel tracking r with every sector 6 % short, seen at the next 1 ms tick, sweeping the phase:
 * the bleed must never fire. A run at 1.5x |r| with the same generator must. Wheel held first so the stall
 * ramp builds a positive I (the bleed rule is live). */
static unsigned sectors(int16_t r, unsigned pct, unsigned phase_us)
{
  setup();
  motor_speed_set(&s, r, 0);
  ticks(400);
  CHECK(s.i_q14 > 0);
  const uint32_t sector_us = 1250000U / (uint32_t)r * pct / 100U;
  uint32_t next_us = phase_us;
  unsigned bled = 0;
  for (uint32_t t = 0; t < 300U; ++t) {
    unsigned n = 0;
    while (next_us / 1000U + 1U == t + 1U) { ++n; next_us += sector_us; }
    if (n) edge(1, n);
    tick();
    bled += out.bleed;
  }
  /* Tracking runs: I stayed along r, so "never fired" was a live test, not a disabled rule. Over-speed runs are
   * meant to pull I down. */
  CHECK(pct < 90U || s.i_q14 > 0);
  return bled;
}

static void bleed_quantization(void)
{
  const int16_t speeds[] = { 50, 120, 400 };
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned phase = 0; phase < 1000U; phase += 37U) CHECK_EQ(sectors(speeds[i], 94, phase), 0);
    CHECK(sectors(speeds[i], 66, 500) > 0);
  }
}

/* Reversal of r: r = 0 clears I/ff/slip; the first tick of the other sign also clears validity. */
static void reversal_clears(void)
{
  setup();
  settle(100, 1000);
  edge(1, 1U);
  tick();
  ticks(9);
  edge(1, 1U);
  tick();
  ticks(3); /* a progress edge pays off up to one edge of slip; reference travel builds it again */
  CHECK(out.valid);
  CHECK(s.i_q14 > 0);
  CHECK(s.slip > 0);
  CHECK_EQ(out.ff_counts, 544);
  motor_speed_set(&s, -100, -1000);
  ticks(9);
  CHECK(out.ref_rpm > 0);
  tick();
  CHECK_EQ(out.ref_rpm, 0);
  CHECK_EQ(s.i_q14, 553 * (0 - out.w_hat_rpm));
  CHECK_EQ(s.slip, 0);
  CHECK_EQ(out.ff_counts, 0);
  CHECK(out.valid);
  tick();
  CHECK_EQ(out.ref_rpm, -10);
  CHECK(!out.valid);
  CHECK_EQ(s.i_q14, 553 * -10);
  CHECK_EQ(s.slip, 10);
  CHECK_EQ(out.ff_counts, -27);
  CHECK_EQ(s.back_edges, 0);
}

/* ff: 50 mA per tick toward the target along r, toward 0 against it; mA -> counts rounds half away from 0. */
static void ff_slew(void)
{
  for (int dir = -1; dir <= 1; dir += 2) {
    setup();
    motor_speed_set(&s, (int16_t)(dir * 100), (int16_t)(dir * 1000));
    const int32_t want[] = { 27, 54, 82, 109, 136 }; /* 50, 100, 150, 200, 250 mA */
    for (unsigned n = 0; n < 5; ++n) { tick(); CHECK_EQ(out.ff_counts, dir * want[n]); }
    ticks(15);
    CHECK_EQ(out.ff_counts, dir * 544);
    tick();
    CHECK_EQ(out.ff_counts, dir * 544);
    motor_speed_set(&s, (int16_t)(dir * 100), (int16_t)(dir * -1000)); /* now opposing: slews to 0 */
    tick();
    CHECK_EQ(out.ff_counts, dir * 517); /* 950 mA */
    ticks(19);
    CHECK_EQ(out.ff_counts, 0);
    tick();
    CHECK_EQ(out.ff_counts, 0);
    motor_speed_set(&s, (int16_t)(dir * 100), (int16_t)dir);
    tick();
    CHECK_EQ(out.ff_counts, dir); /* 1 mA: (544 + 500)/1000 */
  }
  setup();
  settle(0, 1000); /* no direction, no ff */
  CHECK_EQ(out.ff_counts, 0);
}

/* HOLD: Iq 0, I = ff = slip = 0, estimate still runs; RUNNING again starts with I = 0 plus one Ki step. */
static void hold(void)
{
  setup();
  settle(100, 1000);
  ticks(100);
  CHECK(s.i_q14 != 0);
  settle(0, 0);
  in.mode = MOTOR_SPEED_HOLD;
  tick();
  CHECK_EQ(out.iq_ref_counts, 0);
  CHECK_EQ(out.p_counts, 0);
  CHECK_EQ(out.i_counts, 0);
  CHECK_EQ(out.ff_counts, 0);
  CHECK_EQ(s.i_q14, 0);
  CHECK_EQ(s.ff_ma, 0);
  CHECK_EQ(s.slip, 0);
  CHECK_EQ(s.stall_count_ms, 0);
  CHECK(out.standstill);
  edge(1, 1U);
  tick();
  ticks(9);
  edge(1, 1U);
  tick();
  CHECK_EQ(out.w_hat_rpm, 125);
  CHECK(!out.standstill);
  CHECK_EQ(out.iq_ref_counts, 0);
  ticks(30); /* stale: 0 again */
  CHECK(out.standstill);
  in.mode = MOTOR_SPEED_RUNNING;
  motor_speed_set(&s, 100, 0);
  tick();
  CHECK_EQ(out.ref_rpm, 5);
  CHECK_EQ(s.i_q14, 553 * 5);
  CHECK_EQ(out.iq_ref_counts, 20);
}

/* Counts ticks with the stall timer running until the fault; returns that count. */
static unsigned until_fault(unsigned limit)
{
  unsigned counted = 0;
  for (unsigned n = 0; n < limit; ++n) {
    tick();
    counted += out.stall_timer;
    if (out.stall_fault) return counted;
  }
  return 0;
}

/* STALL: |iq| >= 0.8 L, |r| >= 10, no progress for stall_ms; pauses below 0.8 L; resets on progress, |r| < 10,
 * sign flip, leaving RUNNING. */
static void stall_timer(void)
{
  struct motor_speed_config c = dflt;
  c.stall_ms = 200;
  setup_cfg(&c);
  in.limit_counts = 100;
  motor_speed_set(&s, 100, 0);
  CHECK_EQ(until_fault(1000), 200);
  tick();
  CHECK(out.stall_fault); /* stays until a reset */

  setup_cfg(&c);
  in.limit_counts = 100;
  settle(100, 0);
  ticks(100);
  CHECK(out.stall_timer && !out.stall_fault);
  in.limit_counts = 3264; /* iq ~ 400-700 < 0.8 L: paused, not reset */
  for (int n = 0; n < 50; ++n) { tick(); CHECK(!out.stall_timer); }
  in.limit_counts = 100;
  const uint16_t held = s.stall_count_ms;
  CHECK(held >= 100);
  CHECK_EQ(until_fault(1000), 200U - held);

  setup_cfg(&c);
  in.limit_counts = 100;
  settle(100, 0);
  ticks(150);
  edge(1, 1U); /* progress */
  tick();
  CHECK_EQ(s.stall_count_ms, 0);
  CHECK_EQ(until_fault(1000), 200);

  setup_cfg(&c);
  in.limit_counts = 100;
  settle(100, 0);
  ticks(150);
  motor_speed_set(&s, 9, 0); /* |r| < 10 resets */
  ticks(MOTOR_PROTOCOL_PERIOD_MS);
  CHECK_EQ(s.stall_count_ms, 0);
  CHECK(!out.stall_timer);
  motor_speed_set(&s, -100, 0); /* through 0: sign flip */
  CHECK_EQ(until_fault(1000), 200);

  setup_cfg(&c);
  in.limit_counts = 100;
  settle(100, 0);
  ticks(150);
  in.mode = MOTOR_SPEED_HOLD;
  tick();
  CHECK(!out.stall_fault);
  in.mode = MOTOR_SPEED_RUNNING;
  CHECK_EQ(until_fault(1000), 200);

  setup();
  in.limit_counts = 100;
  motor_speed_set(&s, -100, 0);
  CHECK_EQ(until_fault(2000), 500);
}

/* Estimate: validity, edge-interval speed, MCSDK average from 6 edges, 1/t bound, stale window, wrap, HALL_LOST. */
static void estimator(void)
{
  setup();
  in.edges_pos = 65534; /* wraps during the run */
  motor_speed_init(&s, &dflt, in.edges_pos, in.edges_neg);
  edge(1, 1U);
  tick();
  CHECK(!out.valid);
  CHECK_EQ(out.w_hat_rpm, 0);
  ticks(9);
  edge(1, 1U);
  tick();
  CHECK(out.valid);
  CHECK_EQ(out.w_hat_rpm, 125); /* 1250 * 1 / 10 */
  CHECK_EQ(in.edges_pos, 0);
  ticks(4);
  edge(1, 1U);
  tick();
  CHECK_EQ(out.w_hat_rpm, 166); /* 1250 * 2 / 15 */
  in.mcsdk_avg_rpm = 130;
  in.mcsdk_buffer_full = 1U;
  for (int n = 0; n < 2; ++n) { ticks(4); edge(1, 1U); tick(); }
  CHECK_EQ(out.w_hat_rpm, 1250 * 4 / 25); /* 5 edges over 10 + 5 + 5 + 5 ms: still timed here */
  ticks(4);
  edge(1, 1U);
  tick();
  CHECK_EQ(out.w_hat_rpm, 130); /* 6 edges and a full buffer: MCSDK */
  in.mcsdk_buffer_full = 0U;
  tick();
  CHECK_EQ(out.w_hat_rpm, 1250 * 5 / 30); /* buffer not full: back to edge timing over the 30 ms run */
  in.mcsdk_buffer_full = 1U;
  in.mcsdk_avg_rpm = -130; /* MCSDK disagrees with the edge direction */
  tick();
  CHECK(!out.valid);
  CHECK_EQ(out.w_hat_rpm, 0);
  in.mcsdk_avg_rpm = 0; /* no opinion is no disagreement */
  tick();
  CHECK(out.valid);

  /* Direction change restarts the run; both directions in one tick leave no run. */
  edge(-1, 1U);
  tick();
  CHECK(!out.valid);
  ticks(4);
  edge(-1, 1U);
  tick();
  CHECK(out.valid);
  CHECK_EQ(out.w_hat_rpm, -250);
  edge(-1, 1U);
  edge(1, 1U);
  tick();
  CHECK(!out.valid);

  /* 1/t bound after MCSDK's 500 RPM. */
  setup();
  in.mcsdk_avg_rpm = 500;
  in.mcsdk_buffer_full = 1U;
  for (int n = 0; n < 6; ++n) { edge(1, 1U); tick(); tick(); }
  CHECK_EQ(out.w_hat_rpm, 500); /* since 1: bound 1250 */
  tick();
  CHECK_EQ(out.w_hat_rpm, 500); /* since 2: 625 */
  tick();
  CHECK_EQ(out.w_hat_rpm, 416); /* since 3 */
  ticks(7);
  CHECK_EQ(out.w_hat_rpm, 125); /* since 10 */

  /* Stale window clamp(2 * period, 20, 100): period 2 -> 20 ms, 30 -> 60 ms, 60 -> 100 ms. */
  const unsigned period[] = { 2, 30, 60 }, window[] = { 20, 60, 100 };
  for (unsigned i = 0; i < 3; ++i) {
    setup();
    for (int n = 0; n < 3; ++n) { edge(1, 1U); tick(); ticks(period[i] - 1U); }
    ticks(window[i] - period[i] + 1U);
    CHECK(out.valid);
    CHECK_EQ(out.w_hat_rpm, (int32_t)(1250U / window[i]) < (int32_t)(1250U / period[i])
                                ? (int32_t)(1250U / window[i]) : (int32_t)(1250U / period[i]));
    tick();
    CHECK(!out.valid);
    CHECK_EQ(out.w_hat_rpm, 0);
    edge(1, 1U); /* a run starts over: one edge is not enough */
    tick();
    CHECK(!out.valid);

    /* HALL_LOST: invalid code longer than the same window. */
    setup();
    if (period[i] > 2U) for (int n = 0; n < 2; ++n) { edge(1, 1U); tick(); ticks(period[i] - 1U); }
    const unsigned w = period[i] > 2U ? window[i] : 20U;
    in.hall_code_invalid = 1U;
    ticks(w);
    CHECK(!out.hall_lost);
    tick();
    CHECK(out.hall_lost);
    in.hall_code_invalid = 0U;
    tick();
    CHECK(!out.hall_lost);
  }

  /* Standstill threshold 45 RPM; a fresh start (OFF -> RUNNING) needs new edges. */
  setup();
  for (int n = 0; n < 2; ++n) { edge(1, 1U); tick(); ticks(27); }
  edge(1, 1U);
  tick();
  CHECK_EQ(out.w_hat_rpm, 44); /* 1250 * 2 / 56 */
  CHECK(out.standstill);
  setup();
  for (int n = 0; n < 2; ++n) { edge(1, 1U); tick(); ticks(26); }
  edge(1, 1U);
  tick();
  CHECK_EQ(out.w_hat_rpm, 46);
  CHECK(!out.standstill);
  in.mode = MOTOR_SPEED_OFF;
  tick();
  CHECK(out.valid); /* the estimate runs with the output off */
  in.mode = MOTOR_SPEED_RUNNING;
  tick();
  CHECK(!out.valid);
}

/* C5b: linear over exactly 20 ticks; a setpoint mid-ramp restarts from the present r; OFF holds r at 0. */
static void interpolation(void)
{
  setup();
  motor_speed_set(&s, 100, 0);
  for (int k = 1; k <= 20; ++k) { tick(); CHECK_EQ(out.ref_rpm, 5 * k); }
  tick();
  CHECK_EQ(out.ref_rpm, 100);
  setup();
  motor_speed_set(&s, 100, 0);
  ticks(10);
  CHECK_EQ(out.ref_rpm, 50);
  motor_speed_set(&s, -100, 0);
  tick();
  CHECK_EQ(out.ref_rpm, 43); /* 50 + (-150 * 1) / 20, truncated */
  ticks(17);
  CHECK_EQ(out.ref_rpm, -85); /* k = 18: 50 - 135 */
  tick();
  CHECK_EQ(out.ref_rpm, -92); /* k = 19: 50 - 142 (-142.5 truncated) */
  tick();
  CHECK_EQ(out.ref_rpm, -100);
  setup();
  in.mode = MOTOR_SPEED_OFF;
  motor_speed_set(&s, 100, 0); /* programmed with the start, before RUNNING */
  ticks(3);
  CHECK_EQ(out.ref_rpm, 0);
  CHECK_EQ(out.iq_ref_counts, 0);
  in.mode = MOTOR_SPEED_RUNNING;
  tick();
  CHECK_EQ(out.ref_rpm, 5);
  in.mode = MOTOR_SPEED_OFF;
  tick();
  CHECK_EQ(out.ref_rpm, 0);
  CHECK_EQ(s.i_q14, 0);
}

/* One instance per wheel controller: four interleaved instances match four solo runs. */
static void four_instances(void)
{
  struct motor_speed w[4];
  struct motor_speed_outputs o[4], solo[4][60];
  const int16_t sp[4] = { 120, -80, 300, -10 };
  for (int k = 0; k < 4; ++k) {
    setup();
    motor_speed_set(&s, sp[k], (int16_t)(sp[k] * 3));
    for (int t = 0; t < 60; ++t) { if (t % (k + 3) == 0) edge(sp[k], 1U); tick(); solo[k][t] = out; }
  }
  struct motor_speed_inputs win[4];
  for (int k = 0; k < 4; ++k) {
    memset(&win[k], 0, sizeof win[k]);
    win[k].mode = MOTOR_SPEED_RUNNING;
    win[k].limit_counts = 3264;
    motor_speed_init(&w[k], &dflt, 0, 0);
    motor_speed_set(&w[k], sp[k], (int16_t)(sp[k] * 3));
  }
  for (int t = 0; t < 60; ++t)
    for (int k = 0; k < 4; ++k) {
      if (t % (k + 3) == 0) {
        if (sp[k] > 0) ++win[k].edges_pos;
        else ++win[k].edges_neg;
      }
      motor_speed_tick(&w[k], &win[k], &o[k]);
      CHECK(memcmp(&o[k], &solo[k][t], sizeof o[k]) == 0);
    }
}

/* (a) with a limit that drops while stuck (I2T): the integrator comes back within L - p - ff at once. */
static void stall_ramp_limit_drop(void)
{
  for (int dir = -1; dir <= 1; dir += 2) {
    setup();
    settle((int16_t)(dir * 120), 0);
    ticks(600);
    CHECK(out.stall_ramp && dir * out.i_counts > 1632);
    in.limit_counts = 1632; /* 3 A */
    tick();
    CHECK(out.stall_ramp);
    CHECK(dir * out.i_counts <= 1632 - dir * (out.p_counts + out.ff_counts));
    CHECK(dir * out.i_counts >= 0);
  }
}

struct test_case {
  const char *name;
  void (*run)(void);
};

int main(int argc, char **argv)
{
  const struct test_case cases[] = {
    { "p_term", p_term }, { "ki_step", ki_step }, { "conditional_integration", conditional_integration },
    { "limit_same_tick", limit_same_tick }, { "stall_ramp", stall_ramp }, { "bleed", bleed },
    { "bleed_quantization", bleed_quantization }, { "reversal_clears", reversal_clears },
    { "ff_slew", ff_slew }, { "hold", hold }, { "stall_timer", stall_timer }, { "estimator", estimator },
    { "interpolation", interpolation }, { "four_instances", four_instances },
    { "stall_ramp_limit_drop", stall_ramp_limit_drop },
  };
  unsigned ran = 0;
  for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
    if (argc > 1 && strcmp(argv[1], cases[i].name) != 0) continue;
    cases[i].run();
    printf("%s: PASS\n", cases[i].name);
    ++ran;
  }
  if (!ran) { fprintf(stderr, "unknown case\n"); return 2; }
  return 0;
}
