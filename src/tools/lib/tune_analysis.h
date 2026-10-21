/** @file tune_analysis.h
 * @brief Pure telemetry and camera-move analysis for ground feed-forward tuning.
 */
#ifndef TUNE_ANALYSIS_H
#define TUNE_ANALYSIS_H

#include <stdbool.h>
#include <stddef.h>

#define TUNE_WHEELS 4
#define TUNE_MIN_TARGET_RPM 40.0
#define TUNE_TRACK_TOL 0.3
#define TUNE_OVERSHOOT_WINDOW_S 0.6

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

typedef struct {
  double breakaway_s, spread_s, overshoot;
  double direction_deg, heading_deg, ratio_spread;
} tune_limits;
typedef struct { bool slow, over; } tune_judgement;

typedef struct {
  double twist[3]; /**< Commanded vx, vy [mm/s], yaw [deg/s]. */
  bool seen;
  double distance_mm, direction_deg, max_sideways_mm, heading_change_deg;
  double commanded_mm; /**< 0 or NaN uses hypot(vx, vy), as in Python. */
} tune_move;
typedef struct {
  double twist[3];
  double distance_mm, ratio, direction_error_deg, sideways_mm, heading_change_deg;
} tune_move_row;
typedef struct {
  bool ok;
  size_t missing, count;
  double mount_yaw_deg;
  struct {
    double direction_error_deg, heading_change_deg, sideways_per_m, ratio_spread;
  } worst;
  char reason[128]; /**< Empty for a complete set of at least two moves. */
} tune_accept_result;

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

/** @brief Judge start speed [s] and overshoot [RPM ratio] against limits and
 * baseline overshoot [RPM ratio]; NaN metrics represent Python None.
 */
tune_judgement tune_judge(const tune_response_result *resp, const tune_limits *limits,
                          double baseline_overshoot);

/** @brief Next dimensionless feed-forward fraction, or NaN when done/bounded.
 * Defaults: step_down=0.15, step_up=0.1, lo=0.2, hi=1.0; rounded to 0.001.
 */
double tune_next_fraction(double frac, bool slow, bool over, double step_down,
                          double step_up, double lo, double hi);

/** @brief Wrap angle [deg] into [-180, 180), including negative input angles. */
double tune_wrap_deg(double a);

/** @brief Camera acceptance [mm, deg, dimensionless ratios] after removing
 * circular mean mounting yaw. Caller supplies n rows; returns TUNE_* status.
 * Unseen/short (<=20 mm) moves or fewer than two moves produce ok=false and
 * zero rows, with reason/missing set. Sideways drift [mm/m] is reported only,
 * matching Python: it is not an acceptance limit.
 */
int tune_accept_moves(const tune_move *moves, size_t n, const tune_limits *limits,
                      tune_move_row *rows, size_t capacity, tune_accept_result *out);

#endif
