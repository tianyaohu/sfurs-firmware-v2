#include "esc_ops.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RUNAWAY_RPM 200.0 /* floor; beyond max(this, 2x the recent target) a wheel is unloaded or limit-cycling */
#define RUNAWAY_FACTOR 2.0
#define RUNAWAY_HOLD_S 1.0 /* the target counts for 1 s after it drops (coasting to a stop) */
#define STALL_AMPS 4.5
#define STALL_S 0.6

static const char *const WHEEL_NAMES[ESC_OPS_WHEELS] = {"FR", "BR", "BL", "FL"};

static int cmp_double(const void *a, const void *b)
{
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

/* Median of n values (n >= 1); sorts in place. */
static double median(double *v, size_t n)
{
  qsort(v, n, sizeof *v, cmp_double);
  return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

size_t esc_breakaways(const esc_log_entry_t *log, size_t n, int min_target, double window_s, esc_start_t *starts,
                      size_t cap)
{
  size_t count = 0;
  bool idle = true;
  for (size_t k = 0; k < n; k++) {
    const esc_log_entry_t *e = &log[k];
    bool moving = false;
    for (int w = 0; w < ESC_OPS_WHEELS; w++) moving |= abs(e->target_rpm[w]) >= 1;
    if (idle && moving && count < cap) {
      esc_start_t *s = &starts[count++];
      memset(s, 0, sizeof *s);
      for (int w = 0; w < ESC_OPS_WHEELS; w++) {
        int peak = 0;
        for (size_t j = k; j < n && log[j].t - e->t <= window_s; j++)
          if (abs(log[j].target_rpm[w]) > peak) peak = abs(log[j].target_rpm[w]);
        if (peak < min_target) continue;
        s->present[w] = true;
        for (size_t j = k; j < n && log[j].t - e->t <= window_s; j++) {
          const esc_log_entry_t *x = &log[j];
          if (x->rpm_ok[w] && abs(x->rpm[w]) >= ESC_BREAKAWAY_RPM && (int64_t)x->rpm[w] * x->target_rpm[w] > 0) {
            s->moved[w] = true;
            s->seconds[w] = x->t - e->t;
            break;
          }
        }
      }
    }
    idle = !moving;
  }
  return count;
}

void esc_breakaway_summary(const esc_start_t *starts, size_t count, esc_breakaway_summary_t *out)
{
  memset(out, 0, sizeof *out);
  double *times = malloc((count + 1) * sizeof *times);
  double *spreads = malloc((count + 1) * sizeof *spreads);
  size_t n_spreads = 0;
  for (int w = 0; w < ESC_OPS_WHEELS; w++) {
    size_t moved = 0;
    for (size_t i = 0; i < count; i++) {
      if (!starts[i].present[w]) continue;
      out->wheel[w].starts++;
      if (starts[i].moved[w]) times[moved++] = starts[i].seconds[w];
    }
    out->wheel[w].moved = (int)moved;
    if (moved) {
      qsort(times, moved, sizeof *times, cmp_double);
      out->wheel[w].has_times = true;
      out->wheel[w].worst_s = times[moved - 1];
      out->wheel[w].median_s = median(times, moved);
    }
  }
  for (size_t i = 0; i < count; i++) {
    double lo = INFINITY, hi = -INFINITY;
    int have = 0;
    for (int w = 0; w < ESC_OPS_WHEELS; w++) {
      if (!starts[i].present[w] || !starts[i].moved[w]) continue;
      have++;
      lo = fmin(lo, starts[i].seconds[w]);
      hi = fmax(hi, starts[i].seconds[w]);
    }
    if (have >= 2) spreads[n_spreads++] = hi - lo;
  }
  if (n_spreads) {
    out->has_spread = true;
    double worst = spreads[0];
    for (size_t i = 1; i < n_spreads; i++) worst = fmax(worst, spreads[i]);
    out->spread_worst_s = worst;
    out->spread_median_s = median(spreads, n_spreads);
  }
  free(times);
  free(spreads);
}

void esc_guard_init(esc_guard_t *g)
{
  memset(g, 0, sizeof *g);
  for (int w = 0; w < ESC_OPS_WHEELS; w++) g->stall_since[w] = NAN;
}

static double runaway_limit(esc_guard_t *g, const esc_log_entry_t *e)
{
  double target = 0;
  for (int w = 0; w < ESC_OPS_WHEELS; w++) target = fmax(target, abs(e->target_rpm[w]));
  size_t keep = 0;
  for (size_t i = 0; i < g->n_targets; i++)
    if (e->t - g->targets[i].t <= RUNAWAY_HOLD_S) g->targets[keep++] = g->targets[i];
  if (keep == sizeof g->targets / sizeof g->targets[0]) keep--; /* drop the oldest rather than overflow */
  g->targets[keep].t = e->t;
  g->targets[keep].target = target;
  g->n_targets = keep + 1;
  double peak = 0;
  for (size_t i = 0; i < g->n_targets; i++) peak = fmax(peak, g->targets[i].target);
  return fmax(RUNAWAY_RPM, RUNAWAY_FACTOR * peak);
}

bool esc_guard_check(esc_guard_t *g, const esc_log_entry_t *e, char *reason, size_t reason_len)
{
  double limit = runaway_limit(g, e);
  for (int w = 0; w < ESC_OPS_WHEELS; w++) {
    if (e->faults[w][0]) { /* the other wheels would drive on without it */
      snprintf(reason, reason_len, "%s fault %s", WHEEL_NAMES[w], e->faults[w]);
      return true;
    }
    if (!e->rpm_ok[w] || !e->iq_ok[w]) continue;
    if (abs(e->rpm[w]) > limit) {
      snprintf(reason, reason_len, "%s at %d rpm (limit %.0f)", WHEEL_NAMES[w], (int)e->rpm[w], limit);
      return true;
    }
    if (e->rpm[w] == 0 && fabs(e->iq_ref_a[w]) >= STALL_AMPS) {
      if (isnan(g->stall_since[w])) g->stall_since[w] = e->t;
      if (e->t - g->stall_since[w] >= STALL_S) {
        snprintf(reason, reason_len, "%s stalled at %.1f A", WHEEL_NAMES[w], e->iq_ref_a[w]);
        return true;
      }
    } else {
      g->stall_since[w] = NAN;
    }
  }
  return false;
}

double esc_pulse_plan(const int32_t (*twists)[3], size_t n, double seconds, double pause, double lead,
                      esc_move_t *plan)
{
  double t = lead;
  for (size_t i = 0; i < n; i++) {
    plan[i].start = t;
    plan[i].end = t + seconds;
    memcpy(plan[i].twist, twists[i], sizeof plan[i].twist);
    t += seconds + pause;
  }
  return t;
}

void esc_plan_twist(const esc_move_t *plan, size_t n, double now, int32_t twist[3])
{
  memset(twist, 0, 3 * sizeof *twist);
  for (size_t i = 0; i < n; i++)
    if (plan[i].start <= now && now < plan[i].end) {
      memcpy(twist, plan[i].twist, 3 * sizeof *twist);
      return;
    }
}
