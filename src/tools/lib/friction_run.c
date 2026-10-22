#include "friction_run.h"
#include "pulse_run.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SAMPLES 256

typedef struct {
  uint32_t replies[4], statuses[4];
  double reply_at[4], status_at[4];
  double last_command;
} health_t;

static int failure(ik_link_t *l, const char *text)
{
  snprintf(l->err, sizeof l->err, "%s", text);
  return -1;
}

int friction_stop(ik_link_t *l)
{
  int32_t zero[4] = {0};
  int rc = ik_drive_current(l, zero, false);
  /* A partial mailbox write must still attempt to drop enable. */
  if (ik_write(l, IK_ENABLE, 0)) rc = -1;
  double deadline = pulse_now_s() + 4;
  for (;;) {
    uint32_t state;
    ik_row_t rows[4];
    if (ik_read(l, IK_STATE, &state) || ik_rows(l, rows)) return -1;
    bool idle = state == 0;
    for (int w = 0; w < 4; w++) idle &= rows[w].valid && rows[w].state == 0 && rows[w].faults == 0;
    if (idle) return rc;
    if (pulse_now_s() >= deadline) return failure(l, "stop: cannot confirm STOPPED and all four IDLE/no faults");
    pulse_sleep_s(0.02);
  }
}

static void pace(double command_at)
{
  double remaining = command_at + 0.02 - pulse_now_s();
  if (remaining > 0) pulse_sleep_s(remaining);
}

static int limits_ok(const friction_limits_t *a)
{
  return a && isfinite(a->min_vbus_v) && a->min_vbus_v >= 8 &&
         isfinite(a->max_rpm) && a->max_rpm > 0 && a->max_rpm <= 600 &&
         isfinite(a->transient_s) && a->transient_s >= 0;
}

static int interrupted(ik_link_t *l, const friction_limits_t *a)
{
  return a->interrupted && *a->interrupted ? failure(l, "interrupted; stopping") : 0;
}

static int health_begin(ik_link_t *l, health_t *h)
{
  ik_link_row_t r[4]; ik_row_t t[4];
  if (ik_link_rows(l, r) || ik_rows(l, t)) return -1;
  double now = pulse_now_s();
  memset(h, 0, sizeof *h);
  h->last_command = now;
  for (int w = 0; w < 4; w++) {
    if (!r[w].valid || !t[w].valid) return failure(l, "unreadable baseline telemetry");
    h->replies[w] = r[w].reply_count; h->statuses[w] = t[w].status_count;
    h->reply_at[w] = h->status_at[w] = now;
  }
  return 0;
}

static int health_read(ik_link_t *l, health_t *h, const friction_limits_t *a,
                        ik_status_t *status, ik_link_row_t r[4], bool fresh[4])
{
  ik_row_t t[4];
  if (interrupted(l, a) || ik_status(l, status) || (status->fault && ik_check_fault(l, status->fault)) ||
      ik_link_rows(l, r) || ik_rows(l, t)) return -1;
  double now = pulse_now_s();
  if (now - h->last_command > 0.12) return failure(l, "heartbeat scheduling gap >120 ms");
  for (int w = 0; w < 4; w++) {
    fresh[w] = r[w].valid && r[w].reply_count != h->replies[w];
    if (fresh[w]) { h->reply_at[w] = now; h->replies[w] = r[w].reply_count; }
    if (t[w].valid && t[w].status_count != h->statuses[w]) {
      h->status_at[w] = now; h->statuses[w] = t[w].status_count;
    }
    const char *why = NULL;
    if (!r[w].valid || !t[w].valid) why = "unreadable telemetry";
    else if (t[w].faults || r[w].lifecycle == MOTOR_LC_FAULTED) why = "ESC fault";
    else if (t[w].vbus_v < a->min_vbus_v || !isfinite(t[w].vbus_v)) why = "Vbus below minimum";
    else if (fabs((double)r[w].rpm) > a->max_rpm || fabs((double)t[w].rpm) > a->max_rpm) why = "max speed exceeded";
    else if (now - h->reply_at[w] > 0.15 || now - h->status_at[w] > 0.5) why = "stale telemetry";
    else if (fresh[w] && status->armed == 15 && r[w].outcome == MOTOR_OUT_REJECTED) why = "rejected drive command";
    if (why) {
      snprintf(l->err, sizeof l->err, "%s: %s", IK_WHEEL_POS[w], why);
      return -1;
    }
  }
  return 0;
}

static int finish(ik_link_t *l, int rc)
{
  char error[sizeof l->err]; snprintf(error, sizeof error, "%s", l->err);
  if (friction_stop(l)) {
    fprintf(stderr, "%s\n", l->err);
    if (!rc) return -1;
  }
  if (rc) snprintf(l->err, sizeof l->err, "%s", error);
  return rc;
}

/* ARM at zero so ramp timing and the transient start at a known all-wheel boundary. */
static int start(ik_link_t *l, health_t *h, const friction_limits_t *a, unsigned mode)
{
  if (interrupted(l, a) || ik_set_mode(l, mode) || health_begin(l, h)) return -1;
  double deadline = pulse_now_s() + 2;
  for (;;) {
    double command_at = pulse_now_s();
    if (command_at - h->last_command > 0.12) return failure(l, "heartbeat scheduling gap >120 ms");
    int32_t zero[4] = {0};
    if (ik_drive_current(l, zero, true)) return -1;
    h->last_command = command_at;
    ik_status_t status; ik_link_row_t rows[4]; bool fresh[4];
    if (health_read(l, h, a, &status, rows, fresh)) return -1;
    if (status.armed == 15) return 0;
    if (pulse_now_s() >= deadline) return failure(l, "all-wheel ARM timed out");
    pace(command_at);
  }
}

int friction_speed_move(ik_link_t *l, const int32_t twist[3], double duration,
                        const friction_limits_t *a, friction_point_t out[4])
{
  memset(out, 0, 4 * sizeof *out);
  for (int w = 0; w < 4; w++) out[w].verdict = FRICTION_TOO_FEW;
  if (!twist || !limits_ok(a) || !isfinite(duration) || duration <= a->transient_s || duration > 5)
    return failure(l, "invalid speed move limits/duration");
  health_t health = {0};
  int rc = start(l, &health, a, IK_MODE_SPEED);
  friction_sample_t samples[4][MAX_SAMPLES]; size_t n[4] = {0};
  int32_t last_ref[4] = {0}; double plateau_at[4] = {0};
  double t0 = pulse_now_s();
  while (!rc && pulse_now_s() - t0 < duration) {
    double command_at = pulse_now_s();
    if (command_at - health.last_command > 0.12) { rc = failure(l, "heartbeat scheduling gap >120 ms"); break; }
    if (interrupted(l, a) || ik_drive(l, twist[0], twist[1], twist[2], true)) { rc = -1; break; }
    health.last_command = command_at;
    ik_status_t status; ik_link_row_t rows[4]; bool fresh[4];
    if (health_read(l, &health, a, &status, rows, fresh)) { rc = -1; break; }
    double elapsed = pulse_now_s() - t0;
    for (int w = 0; w < 4; w++) {
      int32_t ref = status.rpm[w];
      if (ref != last_ref[w] || ref != status.target_rpm[w] || !ref) {
        plateau_at[w] = elapsed; last_ref[w] = ref; n[w] = 0;
      }
      if (fresh[w] && ref && ref == status.target_rpm[w] && elapsed >= a->transient_s &&
          elapsed - plateau_at[w] >= 0.2 && n[w] < MAX_SAMPLES)
        samples[w][n[w]++] = (friction_sample_t){rows[w].rpm, rows[w].iq_ma};
    }
    pace(command_at);
  }
  for (int w = 0; w < 4; w++) {
    out[w] = friction_segment(samples[w], n[w]);
    if (out[w].verdict == FRICTION_STEADY && out[w].rpm * last_ref[w] <= 0)
      out[w].verdict = FRICTION_UNSTEADY;
  }
  return finish(l, rc);
}

int friction_breakaway_run(ik_link_t *l, unsigned selected, int sign, double rate,
                           double cap, double threshold, const friction_limits_t *a,
                           friction_breakaway_t out[4])
{
  memset(out, 0, 4 * sizeof *out);
  if (!limits_ok(a) || !selected || (selected & ~15U) || (sign != -1 && sign != 1) ||
      !isfinite(rate) || !isfinite(cap) || rate <= 0 || cap <= 0 || cap > 6000 || cap / rate > 5 ||
      !isfinite(threshold) || threshold <= 0 || threshold >= a->max_rpm)
    return failure(l, "invalid breakaway ramp/limits");
  for (int w = 0; w < 4; w++) out[w].attempted = (selected & (1U << w)) != 0;
  health_t health = {0};
  int rc = start(l, &health, a, IK_MODE_CURRENT);
  friction_motion_t motion[4];
  for (int w = 0; w < 4; w++) motion[w] = (friction_motion_t){health.replies[w], 0};
  unsigned pending = selected;
  int32_t current[4] = {0};
  double t0 = pulse_now_s();
  while (!rc && pending && pulse_now_s() - t0 < cap / rate + 0.1) {
    double command_at = pulse_now_s(), elapsed = command_at - t0;
    if (command_at - health.last_command > 0.12) { rc = failure(l, "heartbeat scheduling gap >120 ms"); break; }
    for (int w = 0; w < 4; w++) current[w] = pending & (1U << w) ? (int32_t)nearbyint(sign * fmin(cap, elapsed * rate)) : 0;
    if (interrupted(l, a) || ik_drive_current(l, current, true)) { rc = -1; break; }
    health.last_command = command_at;
    ik_status_t status; ik_link_row_t rows[4]; bool fresh[4];
    if (health_read(l, &health, a, &status, rows, fresh)) { rc = -1; break; }
    for (int w = 0; w < 4; w++) if ((pending & (1U << w)) && fresh[w] &&
        friction_motion(&motion[w], rows[w].reply_count, rows[w].rpm, sign, threshold)) {
      out[w] = (friction_breakaway_t){.moved = true, .command_ma = current[w],
                                   .measured_ma = rows[w].iq_ma, .attempted = true};
      pending &= ~(1U << w); current[w] = 0;
      if (ik_drive_current(l, current, true)) { rc = -1; break; }
      health.last_command = pulse_now_s();
    }
    pace(command_at);
  }
  if (!rc) for (int w = 0; w < 4; w++) if (pending & (1U << w)) out[w].capped = true;
  return finish(l, rc);
}
