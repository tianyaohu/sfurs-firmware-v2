/** @file startup_metrics.h
 * @brief Pure startup acceptance measurements; times are ms from recorder row 0.
 */
#ifndef STARTUP_METRICS_H
#define STARTUP_METRICS_H
#include "ik_link.h"

#define STARTUP_TRANSITIONS 8
#define STARTUP_MAX_SAMPLES 384

typedef enum {
  STARTUP_PASS, STARTUP_FAIL, STARTUP_NOT_MEASURABLE, STARTUP_INACTIVE
} startup_verdict_t;
typedef enum {
  STARTUP_RULE_NONE, STARTUP_RULE_DEADLINE, STARTUP_RULE_DIRECTION,
  STARTUP_RULE_OVERSHOOT, STARTUP_RULE_INPUT, STARTUP_RULE_BASELINE,
  STARTUP_RULE_SECTOR, STARTUP_RULE_REFERENCE, STARTUP_RULE_RECORD,
  STARTUP_RULE_MEASURABLE, STARTUP_RULE_WINDOW, STARTUP_RULE_NO_PLATEAU,
  STARTUP_RULE_COUNT
} startup_rule_t;
typedef struct {
  double widths_deg[6]; /**< Mechanical sector widths, total 45 degrees. */
  double spread_deg;
  int starting_sector; /**< 0..5, or -1 when unknown. Positive rotation increments sectors. */
} startup_calibration_t;
typedef struct {
  double time_ms, distance_deg, deadline_ms, tolerance_ms;
  int direction; /**< -1/+1; zero means no observed transition. */
  bool measurable, met;
} startup_transition_t;
typedef struct {
  startup_verdict_t verdict, tracking, overshoot;
  startup_rule_t rule;
  int sign, first_failed_k; /**< k is 1..8 for deadline/direction, otherwise zero. */
  unsigned deadlines_met, measurable_count;
  startup_transition_t transitions[STARTUP_TRANSITIONS];
  double first_transition_ms, max_speed_rpm, overshoot_ratio;
  double steady_rpm; /**< Final reference magnitude, or NAN without a valid plateau. */
} startup_result_t;
typedef struct {
  startup_verdict_t verdict;
  double skew_ms;
} startup_sync_t;

/** @brief Nominal 7.5 degree sectors, unknown starting sector, given spread in degrees. */
startup_calibration_t startup_nominal(double spread_deg);
/** @brief Integrate linearly interpolated reference in its first nonzero direction.
 * @param rows Consecutive 1 ms samples, including the zero baseline.
 * @param n Sample count.
 * @param angle_deg Mechanical travel; nonpositive distances map to row 0.
 * @return Time in ms from row 0, or INFINITY when never reached/invalid.
 */
double startup_t_ref(const ik_record_unpacked_t *rows, size_t n, double angle_deg);
/** @brief Evaluate one record (at most 384 rows). The steady target is the last
 * reference magnitude, >= 2 RPM, with all last 50 magnitudes within +/-1 RPM.
 * Missing inputs produce NOT_MEASURABLE. Tracking and overshoot remain separate.
 */
startup_result_t startup_evaluate(const ik_record_unpacked_t *rows, size_t n,
                                 const startup_calibration_t *cal);
/** @brief Skew of 1..4 active-wheel apply timestamps in seconds; missing timestamps
 * must be supplied as NAN (never omitted). Zero active wheels is not measurable.
 */
startup_sync_t startup_synchronization(const double *apply_s, size_t n);
/** @brief Stable verdict and failed-rule names for human and machine reports. */
const char *startup_verdict_name(startup_verdict_t verdict);
const char *startup_rule_name(startup_rule_t rule);
#endif
