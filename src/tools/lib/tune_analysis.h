/** @file tune_analysis.h
 * @brief Pure telemetry analysis of wheel starts: friction currents, breakaway time and overshoot.
 */
#ifndef TUNE_ANALYSIS_H
#define TUNE_ANALYSIS_H

#include <stdbool.h>
#include <stddef.h>

#define TUNE_WHEELS 4
#define TUNE_MIN_TARGET_RPM 40.0    /**< smallest |target| that counts as a start, RPM */
#define TUNE_TRACK_TOL 0.3          /**< kinetic samples need |rpm - target| <= this x target */
#define TUNE_OVERSHOOT_WINDOW_S 0.6 /**< overshoot is the peak within this long after a start, s */

/* Wheel order throughout is FR, BR, BL, FL, matching ik_link.WHEELS. */
typedef struct {
  double t; /**< s. */
  double target_rpm[TUNE_WHEELS], rpm[TUNE_WHEELS], iq_ref_a[TUNE_WHEELS];
  bool rpm_valid[TUNE_WHEELS], iq_ref_valid[TUNE_WHEELS];
} tune_log_entry;

typedef struct { size_t first, end; } tune_segment; /**< Half-open sample indices. */
typedef struct {
  double *data;
  size_t capacity, count;
} tune_values;
typedef struct { tune_values breakaway, kinetic; } tune_friction;

typedef struct {
  size_t moved, starts;
  double breakaway_s, breakaway_worst_s;
  double overshoot, overshoot_worst; /**< Peak RPM / target RPM. */
} tune_wheel_response;
typedef struct {
  tune_wheel_response wheels[TUNE_WHEELS];
  double spread_s, spread_worst_s;
} tune_response_result;

enum { TUNE_OK = 0, TUNE_INVALID = -1, TUNE_CAPACITY = -2, TUNE_MEMORY = -3 };

/** @brief Find nonzero target runs reaching min_target [rpm].
 * @param log n entries [s, rpm, A], in time order.
 * @param out Caller-owned array of capacity segments; NULL is valid at capacity 0.
 * @param count Required segment count, also set on TUNE_CAPACITY.
 * @return TUNE_OK, TUNE_INVALID, or TUNE_CAPACITY.
 */
int tune_segments(const tune_log_entry *log, size_t n, double min_target,
                  tune_segment *out, size_t capacity, size_t *count);

/** @brief Per-wheel breakaway/kinetic currents [A], one per qualifying start.
 * Caller supplies all eight buffers; counts reset on entry. min_target is rpm;
 * track_tol is relative tracking error. Returns TUNE_* status (capacity may
 * leave partial results). Missing RPM or Iq entries are ignored.
 */
int tune_friction_samples(const tune_log_entry *log, size_t n, double min_target,
                          double track_tol, tune_friction out[TUNE_WHEELS]);

/** @brief Dimensionless median absolute deviation / median; infinity for fewer
 * than two values or nonpositive median, NaN on allocation/argument failure.
 */
double tune_spread(const double *values, size_t n);

/** @brief Per-wheel peak signed RPM / target RPM within window [s] of starts.
 * min_target is rpm. Caller supplies four buffers; counts reset on entry.
 * Returns TUNE_* status; capacity failure may leave partial results.
 */
int tune_overshoot(const tune_log_entry *log, size_t n, double min_target,
                   double window, tune_values out[TUNE_WHEELS]);

/** @brief Start metrics [s, RPM ratios] using 40 rpm targets, 20 rpm breakaway,
 * a 1 s breakaway window and 0.6 s overshoot window. Missing metrics are NaN.
 * Returns TUNE_* status; an empty log produces zero counts and NaN metrics.
 */
int tune_response(const tune_log_entry *log, size_t n, tune_response_result *out);

#endif
