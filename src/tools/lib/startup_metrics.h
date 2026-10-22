/** @file startup_metrics.h
 * @brief Pure startup acceptance measurements; times are ms from recorder row 0.
 */
#ifndef STARTUP_METRICS_H
#define STARTUP_METRICS_H
#include "ik_link.h"

#define STARTUP_TRANSITIONS 8
#define STARTUP_MAX_SAMPLES 384

/** @brief Outcome of a startup check. */
typedef enum {
  STARTUP_PASS, STARTUP_FAIL, STARTUP_NOT_MEASURABLE, STARTUP_INACTIVE
} startup_verdict_t;
/** @brief Rule that decided a non-PASS verdict. */
typedef enum {
  STARTUP_RULE_NONE, STARTUP_RULE_DEADLINE, STARTUP_RULE_DIRECTION,
  STARTUP_RULE_OVERSHOOT, STARTUP_RULE_INPUT, STARTUP_RULE_BASELINE,
  STARTUP_RULE_SECTOR, STARTUP_RULE_REFERENCE, STARTUP_RULE_RECORD,
  STARTUP_RULE_MEASURABLE, STARTUP_RULE_WINDOW, STARTUP_RULE_NO_PLATEAU,
  STARTUP_RULE_HALL_SEQUENCE, STARTUP_RULE_COUNT
} startup_rule_t;
/** Sector k is the k-th Hall state of the MCSDK decoder's positive electrical order
 * 5-1-3-2-6-4 (sector 0 = state 5, the one decoded at PhaseShift). */
typedef struct {
  double widths_deg[6]; /**< Mechanical sector widths, total 45 degrees, indexed by sector. */
  double spread_deg;
  /** 0..5, or -1 when unknown. Positive rotation increments sectors. A valid recorded
   * Hall state on the last zero-reference row overrides -1 and must agree otherwise. */
  int starting_sector;
} startup_calibration_t;
/** @brief One Hall boundary crossing: time and deadline (ms from row 0), travel (mechanical degrees). */
typedef struct {
  double time_ms, distance_deg, deadline_ms, tolerance_ms;
  int direction; /**< -1/+1; zero means no observed transition. */
  bool measurable, met;
} startup_transition_t;
/** @brief Startup evaluation of one wheel record. */
typedef struct {
  startup_verdict_t verdict, tracking, overshoot;
  startup_rule_t rule;
  int sign, first_failed_k; /**< k is 1..8 for deadline/direction, otherwise zero. */
  unsigned deadlines_met, measurable_count;
  startup_transition_t transitions[STARTUP_TRANSITIONS];
  double first_transition_ms, max_speed_rpm, overshoot_ratio;
  double steady_rpm; /**< Final reference magnitude, or NAN without a valid plateau. */
} startup_result_t;
/** @brief Wheel synchronisation: verdict and apply-time skew, ms. */
typedef struct {
  startup_verdict_t verdict;
  double skew_ms;
} startup_sync_t;

/** @brief Sector 0..5 of a raw Hall state (1..6) in the positive electrical order, or -1
 * for an invalid state (0, 7, or out of range).
 * @param hall_state raw Hall state (HALL_M1.HallState)
 */
int startup_hall_sector(int hall_state);
/** @brief Nominal 7.5 degree sectors, unknown starting sector, given spread in degrees.
 * @param spread_deg sector boundary uncertainty, mechanical degrees
 */
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
 * When the last zero-reference row has a valid Hall state, the starting sector is
 * taken from it and every later row's Hall state must equal the sector reached by
 * the recorded net edges; any mismatch is NOT_MEASURABLE (hall_sequence).
 * @param rows recorder samples, 1 ms apart
 * @param n    sample count
 * @param cal  sector widths (mechanical degrees) and starting sector
 */
startup_result_t startup_evaluate(const ik_record_unpacked_t *rows, size_t n,
                                 const startup_calibration_t *cal);
/** @brief Skew of 1..4 active-wheel apply timestamps in seconds; missing timestamps
 * must be supplied as NAN (never omitted). Zero active wheels is not measurable.
 * @param apply_s apply time per active wheel, s
 * @param n       number of active wheels
 */
startup_sync_t startup_synchronization(const double *apply_s, size_t n);
/** @brief Synchronize recorded active wheels using first-start reply times in wrapping uint32 microseconds.
 * A wheel whose row is torn or lacks first_start_valid makes the result NOT_MEASURABLE; INACTIVE wheels are excluded.
 * @param results Startup verdicts for the four recorded wheels.
 * @param links Seqlocked mailbox rows read after the run.
 * @return Skew in milliseconds and the <=2 ms verdict.
 */
startup_sync_t startup_link_synchronization(const startup_result_t results[IK_WHEELS],
                                           const ik_link_row_t links[IK_WHEELS]);
/** @brief Stable verdict name for human and machine reports.
 * @param verdict verdict
 */
const char *startup_verdict_name(startup_verdict_t verdict);
/** @brief Stable failed-rule name for human and machine reports.
 * @param rule rule
 */
const char *startup_rule_name(startup_rule_t rule);
#endif
