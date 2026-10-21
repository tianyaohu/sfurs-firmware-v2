/**
 * @file esc_ops.h
 * @brief Pure logic of esc_tool: breakaway timing from pulse logs, the pulse
 *        abort guard and the pulse plan. No hardware access.
 */
#ifndef ESC_OPS_H
#define ESC_OPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESC_OPS_WHEELS 4
#define ESC_BREAKAWAY_RPM 20 /**< RPM a wheel must reach (right sign) to count as moved */

/** One pulse-log entry. A *_ok flag false means "no reply from that wheel" (rpm / iq_ref_a invalid). */
typedef struct {
  double t;                          /**< s since the run started */
  int32_t target_rpm[ESC_OPS_WHEELS]; /**< Nucleo's slewed wheel targets, RPM */
  int32_t rpm[ESC_OPS_WHEELS];
  bool rpm_ok[ESC_OPS_WHEELS];
  double iq_ref_a[ESC_OPS_WHEELS];
  bool iq_ok[ESC_OPS_WHEELS];
  char faults[ESC_OPS_WHEELS][96]; /**< space-separated fault names; empty = none */
} esc_log_entry_t;

/** One start (all targets 0 -> any non-zero): time to BREAKAWAY_RPM per wheel. */
typedef struct {
  bool present[ESC_OPS_WHEELS]; /**< wheel's target reached min_target in the window */
  bool moved[ESC_OPS_WHEELS];   /**< reached BREAKAWAY_RPM the right way within the window */
  double seconds[ESC_OPS_WHEELS];
} esc_start_t;

typedef struct {
  int starts, moved;
  bool has_times;
  double median_s, worst_s;
} esc_wheel_breakaway_t;

typedef struct {
  esc_wheel_breakaway_t wheel[ESC_OPS_WHEELS];
  bool has_spread;
  double spread_median_s, spread_worst_s;
} esc_breakaway_summary_t;

/**
 * @brief Find starts in a log and time each wheel's breakaway.
 * @param log        entries in time order.
 * @param n          number of entries.
 * @param min_target only wheels whose target reaches this many RPM are timed.
 * @param window_s   search window after the start, s.
 * @param starts     output, capacity cap.
 * @return number of starts found (<= cap).
 */
size_t esc_breakaways(const esc_log_entry_t *log, size_t n, int min_target, double window_s, esc_start_t *starts,
                      size_t cap);

/** @brief Per-wheel median/worst time and the spread between wheels of each start. */
void esc_breakaway_summary(const esc_start_t *starts, size_t count, esc_breakaway_summary_t *out);

/** @brief Abort guard: wheel fault, a wheel far above any target, or one held at high current without turning. */
typedef struct {
  double stall_since[ESC_OPS_WHEELS]; /**< NaN = not stalled */
  struct { double t, target; } targets[512]; /**< recent peak targets, for the runaway limit */
  size_t n_targets;
} esc_guard_t;

void esc_guard_init(esc_guard_t *g);
/**
 * @brief Check one log entry.
 * @param reason receives the abort reason if the result is true.
 * @return true to abort the run.
 */
bool esc_guard_check(esc_guard_t *g, const esc_log_entry_t *e, char *reason, size_t reason_len);

/** One move of a pulse plan: twist (mm/s, mm/s, mrad/s) from start to end seconds. */
typedef struct { double start, end; int32_t twist[3]; } esc_move_t;

/**
 * @brief Moves of `seconds` each with `pause` s still between them, the first after `lead` s.
 * @return total duration, s. Writes n moves into plan.
 */
double esc_pulse_plan(const int32_t (*twists)[3], size_t n, double seconds, double pause, double lead,
                      esc_move_t *plan);

/** @brief Twist of the plan at time `now`, or zeros if none is active. */
void esc_plan_twist(const esc_move_t *plan, size_t n, double now, int32_t twist[3]);

#endif
