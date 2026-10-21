/* esc_ops.c on synthetic pulse logs (no hardware). Ports tests/test_esc_tool_pulse.py. */
#include <math.h>
#include <string.h>

#include "check.h"
#include "esc_ops.h"

#define MAX_LOG 1024
static esc_log_entry_t logbuf[2 * MAX_LOG];
static size_t log_len;

/* One start at t_start; wheel i reaches 30 rpm (right sign) after delay[i] s; negative delay = never. */
static size_t log_for(const double delay[4], const int target[4], double t_start, double end, double t_offset,
                      esc_log_entry_t *out)
{
  size_t n = 0;
  for (double t = 0.0; t < end; t += 0.01, n++) {
    memset(&out[n], 0, sizeof out[n]);
    bool moving = t >= t_start;
    out[n].t = round((t + t_offset) * 1e4) / 1e4;
    for (int w = 0; w < 4; w++) {
      out[n].target_rpm[w] = moving ? target[w] : 0;
      out[n].rpm_ok[w] = out[n].iq_ok[w] = true;
      bool turning = delay[w] >= 0 && moving && t - t_start >= delay[w];
      out[n].rpm[w] = turning ? (target[w] > 0 ? 30 : -30) : 0;
    }
  }
  return n;
}

static const int T_ALT[4] = {50, -50, 50, -50};

static void test_each_wheel_time_and_spread(void)
{
  const double d[4] = {0.10, 0.20, 0.30, 0.40};
  log_len = log_for(d, T_ALT, 0.5, 2.0, 0, logbuf);
  esc_start_t starts[4];
  CHECK_EQ(esc_breakaways(logbuf, log_len, 40, 1.0, starts, 4), 1);
  for (int w = 0; w < 4; w++) {
    CHECK(starts[0].moved[w]);
    CHECK_NEAR(starts[0].seconds[w], d[w], 0.005);
  }
  esc_breakaway_summary_t s;
  esc_breakaway_summary(starts, 1, &s);
  CHECK(s.has_spread);
  CHECK_NEAR(s.spread_median_s, 0.30, 0.005);
  for (int w = 0; w < 4; w++) CHECK(s.wheel[w].starts == 1 && s.wheel[w].moved == 1);
}

static void test_stalled_wheel_and_wrong_sign(void)
{
  const double d[4] = {0.1, -1, 0.1, 0.1};
  log_len = log_for(d, T_ALT, 0.5, 2.0, 0, logbuf);
  for (size_t i = 0; i < log_len; i++) logbuf[i].rpm[3] = -logbuf[i].rpm[3]; /* FL turns the wrong way */
  esc_start_t starts[4];
  CHECK_EQ(esc_breakaways(logbuf, log_len, 40, 1.0, starts, 4), 1);
  CHECK(!starts[0].moved[1] && !starts[0].moved[3]);
  CHECK(starts[0].moved[0] && starts[0].moved[2]);
  esc_breakaway_summary_t s;
  esc_breakaway_summary(starts, 1, &s);
  CHECK_EQ(s.wheel[1].moved, 0);
  CHECK(s.has_spread);
  CHECK_NEAR(s.spread_median_s, 0.0, 0.001); /* only the two healthy wheels count */
}

static void test_small_target_wheels_ignored_and_no_reply(void)
{
  const double d[4] = {0.1, 0.1, 0.1, 0.1};
  const int target[4] = {50, 10, 50, 50};
  log_len = log_for(d, target, 0.5, 2.0, 0, logbuf);
  for (size_t i = 0; i < log_len; i++)
    if (logbuf[i].t < 0.9) logbuf[i].rpm_ok[2] = false; /* no reply before 0.9 s */
  esc_start_t starts[4];
  CHECK_EQ(esc_breakaways(logbuf, log_len, 40, 1.0, starts, 4), 1);
  CHECK(!starts[0].present[1]); /* 10 rpm target is below min_target */
  CHECK(starts[0].moved[2]);
  CHECK_NEAR(starts[0].seconds[2], 0.4, 0.005);
}

static void test_two_starts(void)
{
  const double a[4] = {0.1, 0.1, 0.1, 0.1}, b[4] = {0.2, 0.2, 0.2, 0.2};
  const int t2[4] = {-50, 50, -50, 50};
  size_t na = log_for(a, T_ALT, 0.5, 1.5, 0, logbuf);
  size_t nb = log_for(b, t2, 0.5, 2.0, 1.5, logbuf + na);
  esc_start_t starts[4];
  CHECK_EQ(esc_breakaways(logbuf, na + nb, 40, 1.0, starts, 4), 2);
  CHECK_NEAR(starts[1].seconds[0], 0.2, 0.005);
}

static esc_log_entry_t entry(double t, const int *rpm, const double *iq)
{
  esc_log_entry_t e;
  memset(&e, 0, sizeof e);
  e.t = t;
  for (int w = 0; w < 4; w++) {
    e.rpm[w] = rpm[w];
    e.iq_ref_a[w] = iq[w];
    e.rpm_ok[w] = e.iq_ok[w] = true;
  }
  return e;
}

static void test_fault_aborts(void)
{
  esc_guard_t g;
  esc_guard_init(&g);
  char why[96];
  const int z[4] = {0};
  const double zi[4] = {0};
  esc_log_entry_t e = entry(0.0, z, zi);
  CHECK(!esc_guard_check(&g, &e, why, sizeof why));
  const int r[4] = {0, 50, 50, 50};
  const double i1[4] = {0, 1, 1, 1};
  e = entry(0.1, r, i1);
  strcpy(e.faults[0], "speed_fdbk");
  CHECK(esc_guard_check(&g, &e, why, sizeof why));
  CHECK(strstr(why, "fault speed_fdbk"));
}

static void test_runaway(void)
{
  esc_guard_t g;
  esc_guard_init(&g);
  char why[96];
  const int r1[4] = {60, -60, 60, 0};
  const double i1[4] = {1, 1, 1, 0};
  esc_log_entry_t e = entry(0.0, r1, i1);
  e.rpm_ok[3] = e.iq_ok[3] = false; /* no reply from FL */
  CHECK(!esc_guard_check(&g, &e, why, sizeof why));
  const int r2[4] = {60, -390, 60, 0};
  const double i2[4] = {1, 6, 1, 0};
  e = entry(0.1, r2, i2);
  CHECK(esc_guard_check(&g, &e, why, sizeof why));
  CHECK(strstr(why, "BR"));
}

static void set_targets(esc_log_entry_t *e, int v0, int v1, int v2, int v3)
{
  e->target_rpm[0] = v0, e->target_rpm[1] = v1, e->target_rpm[2] = v2, e->target_rpm[3] = v3;
}

static void test_runaway_limit_scales_with_target(void)
{
  char why[96];
  esc_guard_t g;
  const double one[4] = {1, 1, 1, 1};
  esc_guard_init(&g);
  const int a[4] = {480, -480, 480, -480};
  esc_log_entry_t e = entry(0.0, a, one);
  set_targets(&e, 250, -250, 250, -250);
  CHECK(!esc_guard_check(&g, &e, why, sizeof why)); /* 2 x 250 */
  const int b[4] = {520, -480, 480, -480};
  e = entry(0.1, b, one);
  set_targets(&e, 250, -250, 250, -250);
  CHECK(esc_guard_check(&g, &e, why, sizeof why));
  CHECK(strstr(why, "limit 500"));

  esc_guard_init(&g);
  const int c[4] = {300, 300, 300, 300};
  e = entry(0.0, c, one);
  set_targets(&e, 250, 250, 250, 250);
  CHECK(!esc_guard_check(&g, &e, why, sizeof why));
  const int d[4] = {450, 450, 450, 450};
  e = entry(0.9, d, one);
  set_targets(&e, 0, 0, 0, 0);
  CHECK(!esc_guard_check(&g, &e, why, sizeof why)); /* coasting: the target still counts */
  const int f[4] = {250, 250, 250, 250};
  e = entry(2.0, f, one);
  set_targets(&e, 0, 0, 0, 0);
  CHECK(esc_guard_check(&g, &e, why, sizeof why)); /* 1 s later: only the floor */
  CHECK(strstr(why, "limit 200"));
}

static void test_stall_needs_time(void)
{
  esc_guard_t g;
  esc_guard_init(&g);
  char why[96];
  const int z[4] = {0};
  const double hot[4] = {0, 5.0, 0, 0}, cold[4] = {0, -5.0, 0, 0};
  for (int k = 0; k < 6; k++) {
    esc_log_entry_t e = entry(k * 0.1, z, hot);
    CHECK(!esc_guard_check(&g, &e, why, sizeof why));
  }
  const int turn[4] = {0, 10, 0, 0};
  esc_log_entry_t e = entry(0.55, turn, hot);
  CHECK(!esc_guard_check(&g, &e, why, sizeof why)); /* turned: reset */
  for (int k = 0; k < 6; k++) {
    e = entry(0.6 + k * 0.1, z, cold);
    CHECK(!esc_guard_check(&g, &e, why, sizeof why));
  }
  e = entry(1.2, z, cold);
  CHECK(esc_guard_check(&g, &e, why, sizeof why));
  CHECK(strstr(why, "stalled"));
}

static void test_pulse_plan(void)
{
  const int32_t tw[4][3] = {{0, 200, 0}, {0, -200, 0}, {0, 200, 0}, {0, -200, 0}};
  esc_move_t plan[4];
  double total = esc_pulse_plan(tw, 4, 1.5, 1.0, 0.5, plan);
  CHECK_NEAR(total, 0.5 + 4 * 2.5, 1e-9);
  CHECK_NEAR(plan[1].start, 3.0, 1e-9);
  int32_t cur[3];
  esc_plan_twist(plan, 4, 0.2, cur);
  CHECK(cur[0] == 0 && cur[1] == 0 && cur[2] == 0);
  esc_plan_twist(plan, 4, 1.0, cur);
  CHECK_EQ(cur[1], 200);
  esc_plan_twist(plan, 4, 3.2, cur);
  CHECK_EQ(cur[1], -200);
  esc_plan_twist(plan, 4, 2.2, cur); /* the pause between moves */
  CHECK(cur[0] == 0 && cur[1] == 0);
}

int main(void)
{
  test_each_wheel_time_and_spread();
  test_stalled_wheel_and_wrong_sign();
  test_small_target_wheels_ignored_and_no_reply();
  test_two_starts();
  test_fault_aborts();
  test_runaway();
  test_runaway_limit_scales_with_target();
  test_stall_needs_time();
  test_pulse_plan();
  printf("ok\n");
  return 0;
}
