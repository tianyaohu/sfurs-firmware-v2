/* Replay fixed telemetry traces at the public mailbox boundary. No plant model. */
#include "check.h"
#include "friction_run.h"
#include "pulse_run.h"
#include <string.h>

static double clock_s;
static unsigned tick, mode, disables, speed_commands, current_commands;
static bool enabled, freeze, io_failure, fault_latch, stop_failure, scheduling_gap;
static int bad_wheel, problem, direction;
static volatile sig_atomic_t cancelled;
static int32_t last_current[4], peak_current[4];
static friction_limits_t limits;

static void reset(void)
{
  clock_s = 0; tick = mode = disables = speed_commands = current_commands = 0;
  enabled = freeze = io_failure = fault_latch = stop_failure = scheduling_gap = false;
  bad_wheel = -1; problem = 0; direction = 1; cancelled = 0;
  memset(last_current, 0, sizeof last_current); memset(peak_current, 0, sizeof peak_current);
  limits = (friction_limits_t){11, 300, 0.3, &cancelled};
}

double __wrap_pulse_now_s(void) { return clock_s; }
void __wrap_pulse_sleep_s(double s)
{
  clock_s += scheduling_gap ? 0.2 : s;
  if (!freeze) tick++;
  if (problem == 6 && tick >= 8) cancelled = SIGINT;
}
int __wrap_ik_set_mode(ik_link_t *l, unsigned m) { (void)l; CHECK(!enabled); mode = m; return 0; }
int __wrap_ik_drive(ik_link_t *l, int32_t x, int32_t y, int32_t w, bool enable)
{
  (void)l; CHECK(x == 0 && w == 0); if (y) { CHECK_EQ(y, 200); CHECK_EQ(mode, IK_MODE_SPEED); speed_commands++; }
  enabled = enable; if (!enable) disables++;
  return 0;
}
int __wrap_ik_drive_current(ik_link_t *l, const int32_t ma[4], bool enable)
{
  (void)l; memcpy(last_current, ma, sizeof last_current);
  for (int w = 0; w < 4; w++) {
    CHECK(abs(ma[w]) <= 4000);
    if (abs(ma[w]) > peak_current[w]) peak_current[w] = abs(ma[w]);
    if (ma[w]) CHECK_EQ(mode, IK_MODE_CURRENT);
  }
  current_commands++; enabled = enable; if (!enable) disables++;
  return 0;
}
int __wrap_ik_write(ik_link_t *l, ik_sym_t sym, uint32_t value)
{
  (void)l; CHECK_EQ(sym, IK_ENABLE); CHECK_EQ(value, 0); enabled = false; disables++; return 0;
}
int __wrap_ik_read(ik_link_t *l, ik_sym_t sym, uint32_t *value)
{
  (void)l; CHECK_EQ(sym, IK_STATE); *value = stop_failure ? 2 : enabled; return 0;
}
int __wrap_ik_status(ik_link_t *l, ik_status_t *s)
{
  (void)l;
  if (io_failure && tick >= 8) return -1;
  memset(s, 0, sizeof *s); s->armed = tick >= 2 ? 15 : 0;
  s->fault = fault_latch && tick >= 8 ? 1 : 0;
  for (int w = 0; w < 4; w++) {
    s->rpm[w] = problem == 9 ? 100 + (int)tick : 100 + w;
    s->target_rpm[w] = problem == 9 ? 200 : 100 + w;
  }
  return 0;
}
int __wrap_ik_check_fault(ik_link_t *l, uint32_t fault)
{
  if (fault) { strcpy(l->err, "drive fault"); return -1; } return 0;
}
int __wrap_ik_link_rows(ik_link_t *l, ik_link_row_t r[4])
{
  (void)l;
  for (int w = 0; w < 4; w++) {
    r[w] = (ik_link_row_t){.valid = true, .reply_count = tick, .lifecycle = MOTOR_LC_RUNNING,
      .outcome = MOTOR_OUT_APPLIED, .rpm = direction * (100 + w), .iq_ma = direction * (300 + w * 50)};
    if (mode == IK_MODE_CURRENT && tick < 10) r[w].rpm = 0;
    if (w == bad_wheel && tick >= 8) {
      if (problem == 1) r[w].rpm = 301;
      if (problem == 3) r[w].reply_count = 7;
      if (problem == 4) r[w].valid = false;
      if (problem == 5) r[w].outcome = MOTOR_OUT_REJECTED;
    }
  }
  return 0;
}
int __wrap_ik_rows(ik_link_t *l, ik_row_t r[4])
{
  (void)l;
  for (int w = 0; w < 4; w++) {
    r[w] = (ik_row_t){.valid = true, .status_count = tick, .state = 0, .vbus_v = 12};
    if (enabled && w == bad_wheel && tick >= 8) {
      if (problem == 2) r[w].vbus_v = 10.9;
      if (problem == 7) r[w].faults = 1;
      if (problem == 8) r[w].status_count = 7;
    }
  }
  return 0;
}

static void stopped(void)
{
  CHECK(!enabled); CHECK(disables >= 2);
  for (int w = 0; w < 4; w++) CHECK_EQ(last_current[w], 0);
}

static void speed_collection(void)
{
  reset(); ik_link_t l = {0}; friction_point_t out[4];
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), 0);
  CHECK(speed_commands > 10); CHECK_EQ(mode, IK_MODE_SPEED); stopped();
  for (int w = 0; w < 4; w++) {
    CHECK_EQ(out[w].verdict, FRICTION_STEADY); CHECK(out[w].samples >= 5 && out[w].samples < 40);
    CHECK_EQ(out[w].rpm, 100 + w); CHECK_EQ(out[w].iq_ma, 300 + w * 50);
  }
}

static void abort_guards(void)
{
  for (int w = 0; w < 4; w++) for (int why = 1; why <= 8; why++) {
    reset(); bad_wheel = w; problem = why;
    ik_link_t l = {0}; friction_point_t out[4];
    CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), -1);
    stopped();
    if (why != 6) CHECK(strstr(l.err, IK_WHEEL_POS[w]));
  }
  reset(); scheduling_gap = true;
  ik_link_t l = {0}; friction_point_t out[4];
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), -1); stopped();
  CHECK(strstr(l.err, "scheduling gap") || strstr(l.err, "stale"));
  reset(); io_failure = true;
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), -1); stopped();
  reset(); fault_latch = true;
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), -1); stopped();
  reset(); stop_failure = true;
  CHECK_EQ(friction_stop(&l), -1); CHECK(strstr(l.err, "cannot confirm")); stopped();
}

static void breakaway_ramps(void)
{
  for (int w = 0; w < 5; w++) for (int sign = -1; sign <= 1; sign += 2) {
    reset(); direction = sign;
    ik_link_t l = {0}; friction_breakaway_t out[4];
    unsigned selected = w == 4 ? 15 : 1U << w;
    CHECK_EQ(friction_breakaway_run(&l, selected, sign, 2000, 4000, 20, &limits, out), 0);
    CHECK_EQ(mode, IK_MODE_CURRENT); stopped();
    for (int j = 0; j < 4; j++) {
      CHECK_EQ(out[j].moved, !!(selected & (1U << j)));
      if (out[j].moved) {
        CHECK(out[j].command_ma * sign >= 360 && out[j].command_ma * sign <= 440);
        CHECK_EQ(out[j].measured_ma, sign * (300 + j * 50));
      } else CHECK_EQ(peak_current[j], 0);
    }
  }
  reset(); direction = -1;
  ik_link_t l = {0}; friction_breakaway_t out[4];
  CHECK_EQ(friction_breakaway_run(&l, 15, 1, 2000, 400, 20, &limits, out), 0); stopped();
  for (int w = 0; w < 4; w++) { CHECK(!out[w].moved); CHECK(out[w].attempted && out[w].capped); }
  for (int w = 0; w < 4; w++) {
    reset(); bad_wheel = w; problem = 1;
    CHECK_EQ(friction_breakaway_run(&l, 15, 1, 2000, 4000, 20, &limits, out), -1); stopped();
  }
}

static void rejected_segments(void)
{
  reset(); direction = -1;
  ik_link_t l = {0}; friction_point_t out[4];
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), 0); stopped();
  for (int w = 0; w < 4; w++) { CHECK_EQ(out[w].verdict, FRICTION_UNSTEADY); CHECK(out[w].rpm < 0); }
  reset(); problem = 9;
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), 0); stopped();
  for (int w = 0; w < 4; w++) { CHECK_EQ(out[w].verdict, FRICTION_TOO_FEW); CHECK_EQ(out[w].samples, 0); }
}

static void invalid_limits(void)
{
  reset(); ik_link_t l = {0}; friction_point_t out[4]; friction_breakaway_t b[4];
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 6, &limits, out), -1);
  limits.min_vbus_v = 7;
  CHECK_EQ(friction_speed_move(&l, (int32_t[]){0, 200, 0}, 1, &limits, out), -1);
  reset(); CHECK_EQ(friction_breakaway_run(&l, 16, 1, 2000, 4000, 20, &limits, b), -1);
  CHECK_EQ(friction_breakaway_run(&l, 15, 0, 2000, 4000, 20, &limits, b), -1);
  CHECK_EQ(friction_breakaway_run(&l, 15, 1, 2000, 6001, 20, &limits, b), -1);
  CHECK_EQ(friction_breakaway_run(&l, 15, 1, 1, 4000, 20, &limits, b), -1);
  CHECK_EQ(disables, 0); CHECK_EQ(current_commands, 0);
}

int main(void)
{
  speed_collection(); abort_guards(); breakaway_ramps(); rejected_segments(); invalid_limits();
  puts("friction_run: 5 test groups passed (all four wheels)");
  return 0;
}
