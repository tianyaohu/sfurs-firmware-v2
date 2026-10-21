#include "startup_metrics.h"
#include <math.h>

static int direction(const ik_record_unpacked_t *r, size_t n)
{
  for (size_t i = 0; i < n; ++i)
    if (r[i].ref_rpm) return r[i].ref_rpm > 0 ? 1 : -1;
  return 0;
}

startup_calibration_t startup_nominal(double spread_deg)
{
  return (startup_calibration_t){{7.5, 7.5, 7.5, 7.5, 7.5, 7.5}, spread_deg, -1};
}

double startup_t_ref(const ik_record_unpacked_t *r, size_t n, double angle_deg)
{
  if (!r || !n || !isfinite(angle_deg)) return INFINITY;
  if (angle_deg <= 0) return 0;
  int sign = direction(r, n);
  double travelled = 0;
  for (size_t i = 1; i < n; ++i) {
    /* Reversing references are not a single start in the commanded direction. */
    double a = (double)sign * r[i - 1].ref_rpm * 0.006;
    double b = (double)sign * r[i].ref_rpm * 0.006;
    if (a < 0 || b < 0) return INFINITY;
    double area = (a + b) / 2;
    if (area > 0 && travelled + area >= angle_deg) {
      double remaining = angle_deg - travelled;
      /* Stable quadratic inversion of a*x + (b-a)*x*x/2. */
      double x = 2 * remaining / (a + sqrt(fmax(0, a*a + 2*(b-a)*remaining)));
      return (double)(i - 1) + x;
    }
    travelled += area;
  }
  return INFINITY;
}

const char *startup_verdict_name(startup_verdict_t v)
{
  static const char *const names[] = {"PASS", "FAIL", "NOT_MEASURABLE", "INACTIVE"};
  return v >= STARTUP_PASS && v <= STARTUP_INACTIVE ? names[v] : "NOT_MEASURABLE";
}
const char *startup_rule_name(startup_rule_t r)
{
  static const char *const names[] = {"none", "deadline", "wrong_direction", "overshoot",
    "invalid_input", "missing_baseline", "unknown_sector", "reference_reversal",
    "short_record", "too_few_measurable", "overshoot_window", "no_plateau"};
  return r >= STARTUP_RULE_NONE && r < STARTUP_RULE_COUNT ? names[r] : "invalid_input";
}

typedef struct { double time_ms, width_deg; int sign; } edge_t;

startup_result_t startup_evaluate(const ik_record_unpacked_t *r, size_t n,
                                 const startup_calibration_t *cal)
{
  startup_result_t out = {.verdict = STARTUP_NOT_MEASURABLE,
    .tracking = STARTUP_NOT_MEASURABLE, .overshoot = STARTUP_NOT_MEASURABLE,
    .rule = STARTUP_RULE_INPUT, .first_transition_ms = NAN,
    .max_speed_rpm = NAN, .overshoot_ratio = NAN, .steady_rpm = NAN};
  for (int k = 0; k < STARTUP_TRANSITIONS; ++k) {
    out.transitions[k].time_ms = NAN;
    out.transitions[k].deadline_ms = INFINITY;
    out.transitions[k].tolerance_ms = INFINITY;
  }
  if (!r || !n || n > STARTUP_MAX_SAMPLES || !cal) return out;
  bool active = false;
  for (size_t i = 0; i < n; ++i) {
    if (r[i].edges_mod16 < 0 || r[i].edges_mod16 > 15) return out;
    if (fabs((double)r[i].ref_rpm) >= 2) active = true;
  }
  if (!active) {
    out.verdict = out.tracking = out.overshoot = STARTUP_INACTIVE;
    out.rule = STARTUP_RULE_NONE;
    return out;
  }
  out.sign = direction(r, n);
  double target_rpm = fabs((double)r[n - 1].ref_rpm);
  bool plateau = n >= 50 && target_rpm >= 2;
  if (plateau) for (size_t i = n - 50; i < n; ++i)
    if (fabs(fabs((double)r[i].ref_rpm) - target_rpm) > 1) plateau = false;
  if (plateau) out.steady_rpm = target_rpm;
  double sum = 0;
  bool uniform = true;
  for (int s = 0; s < 6; ++s) {
    if (!isfinite(cal->widths_deg[s]) || cal->widths_deg[s] <= 0) return out;
    sum += cal->widths_deg[s];
    if (fabs(cal->widths_deg[s] - cal->widths_deg[0]) > 1e-9) uniform = false;
  }
  /* SPEC-QUESTION: six mechanical sectors total 45 deg (48 per revolution),
   * not the task's 60 deg. Reject inconsistent maps rather than rescaling them. */
  if (fabs(sum - 45) > 1e-6 || !isfinite(cal->spread_deg) || cal->spread_deg < 0 ||
      cal->starting_sector < -1 || cal->starting_sector > 5) return out;
  /* SPEC-QUESTION: an unknown nonuniform sector or a reversing reference cannot
   * establish the single-start geometry; neither is eligible for PASS. */
  if (cal->starting_sector == -1 && !uniform) { out.rule = STARTUP_RULE_SECTOR; return out; }
  /* SPEC-QUESTION: trigger is 2 RPM, not apply. Require a zero baseline and
   * integrate from row 0; a clipped/delayed record cannot establish deadlines. */
  if (r[0].ref_rpm != 0) { out.rule = STARTUP_RULE_BASELINE; return out; }
  for (size_t i = 0; i < n; ++i)
    if ((double)out.sign * r[i].ref_rpm < 0) { out.rule = STARTUP_RULE_REFERENCE; return out; }

  edge_t edges[STARTUP_MAX_SAMPLES * 8];
  size_t count = 0;
  int sector = cal->starting_sector < 0 ? 0 : cal->starting_sector;
  int initial_sector = sector;
  size_t begin = 1;
  while (begin < n && r[begin].ref_rpm == 0) ++begin;
  for (size_t i = begin; i < n; ++i) {
    int delta = (r[i].edges_mod16 - r[i - 1].edges_mod16 + 24) % 16 - 8;
    int sign = delta < 0 ? -1 : 1;
    for (int e = 0; e < (delta < 0 ? -delta : delta); ++e) {
      edges[count++] = (edge_t){(double)i, cal->widths_deg[sector], sign};
      sector = (sector + sign + 6) % 6;
    }
  }
  if (count) out.first_transition_ms = edges[0].time_ms;
  double distance = 0;
  sector = initial_sector;
  out.tracking = STARTUP_PASS;
  out.rule = STARTUP_RULE_NONE;
  for (int k = 0; k < STARTUP_TRANSITIONS; ++k) {
    startup_transition_t *t = &out.transitions[k];
    distance += cal->widths_deg[sector];
    sector = (sector + out.sign + 6) % 6;
    t->distance_deg = distance;
    double latest = startup_t_ref(r, n, distance);
    double earliest = startup_t_ref(r, n, distance - cal->widths_deg[initial_sector] - cal->spread_deg);
    t->deadline_ms = latest + 20;
    t->tolerance_ms = isfinite(latest) ? latest - earliest : INFINITY;
    t->measurable = t->tolerance_ms <= 20;
    out.measurable_count += t->measurable;
    if ((size_t)k < count) {
      t->time_ms = edges[k].time_ms;
      t->direction = edges[k].sign;
    }
    t->met = isfinite(t->deadline_ms) && t->direction == out.sign && t->time_ms <= t->deadline_ms;
    out.deadlines_met += t->met;
    /* SPEC-QUESTION: any reversed transition among k=1..8 fails, even after
     * the first correct one, as required by "every transition ... in direction". */
    if (out.tracking != STARTUP_FAIL && t->direction && t->direction != out.sign &&
        t->time_ms <= t->deadline_ms) {
      out.tracking = STARTUP_FAIL; out.rule = STARTUP_RULE_DIRECTION; out.first_failed_k = k + 1;
    } else if (out.tracking != STARTUP_FAIL && !t->met && t->deadline_ms <= (double)(n - 1)) {
      out.tracking = STARTUP_FAIL; out.rule = STARTUP_RULE_DEADLINE; out.first_failed_k = k + 1;
    } else if (out.tracking == STARTUP_PASS && !t->met) {
      out.tracking = STARTUP_NOT_MEASURABLE; out.rule = STARTUP_RULE_RECORD;
    }
  }
  if (out.tracking == STARTUP_PASS && out.measurable_count < 3) {
    out.tracking = STARTUP_NOT_MEASURABLE; out.rule = STARTUP_RULE_MEASURABLE;
  }

  double max_speed = NAN;
  for (size_t j = 1; j < count; ++j) {
    double angle = 0;
    for (size_t i = j; i > 0; --i) {
      if (edges[i].sign != edges[i - 1].sign) break;
      angle += edges[i].width_deg;
      double dt = edges[j].time_ms - edges[i - 1].time_ms;
      if (dt >= 25) {
        double speed = angle / dt / 0.006;
        if (!isfinite(max_speed) || speed > max_speed) max_speed = speed;
        break;
      }
    }
  }
  out.max_speed_rpm = max_speed;
  /* A long window without a complete edge interval or a reference plateau
   * is missing evidence, never zero overshoot. */
  if (count && (double)(n - 1) - edges[0].time_ms >= 200 &&
      isfinite(max_speed) && plateau) {
    out.overshoot_ratio = max_speed / out.steady_rpm;
    out.overshoot = out.overshoot_ratio <= 1.3 ? STARTUP_PASS : STARTUP_FAIL;
  }
  out.verdict = out.tracking;
  if (out.tracking != STARTUP_FAIL && out.overshoot == STARTUP_FAIL) {
    out.verdict = STARTUP_FAIL; out.rule = STARTUP_RULE_OVERSHOOT;
  } else if (out.tracking == STARTUP_PASS && out.overshoot != STARTUP_PASS) {
    /* SPEC-QUESTION: the TSV start verdict includes overshoot; missing overshoot
     * evidence must not turn a tracking-only PASS into a gate PASS. */
    out.verdict = STARTUP_NOT_MEASURABLE;
    out.rule = plateau ? STARTUP_RULE_WINDOW : STARTUP_RULE_NO_PLATEAU;
  }
  return out;
}

startup_sync_t startup_synchronization(const double *apply_s, size_t n)
{
  startup_sync_t out = {STARTUP_NOT_MEASURABLE, NAN};
  if (!apply_s || !n || n > 4) return out;
  double lo = INFINITY, hi = -INFINITY;
  for (size_t i = 0; i < n; ++i) {
    if (!isfinite(apply_s[i]) || apply_s[i] < 0) return out;
    if (apply_s[i] < lo) lo = apply_s[i];
    if (apply_s[i] > hi) hi = apply_s[i];
  }
  out.skew_ms = (hi - lo) * 1000;
  out.verdict = hi <= lo + 0.002 ? STARTUP_PASS : STARTUP_FAIL;
  return out;
}
