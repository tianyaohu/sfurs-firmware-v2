/**
 * @file pulse_run.h
 * @brief Drive a plan of body twists through the Nucleo with per-wheel RAM settings,
 *        guard it, log it, and restore the firmware defaults afterwards.
 */
#ifndef PULSE_RUN_H
#define PULSE_RUN_H

#include <stddef.h>
#include <stdio.h>

#include "esc_ops.h"
#include "ik_link.h"

#define PULSE_MAX_KEYS 8

/** Per-wheel RAM settings sent before the run (CONFIG keys; defaults restored after). */
typedef struct {
  size_t n_keys;
  const char *keys[PULSE_MAX_KEYS];
  int value[IK_WHEELS][PULSE_MAX_KEYS]; /**< value[wheel][key] */
} pulse_settings_t;

/** One log entry with the display extras. */
typedef struct {
  esc_log_entry_t e;
  int32_t twist[3]; /**< commanded body twist, mm/s, mm/s, mrad/s */
  const char *mc_state[IK_WHEELS];
  double vbus_v[IK_WHEELS];
} pulse_entry_t;

typedef struct {
  pulse_entry_t *log;
  size_t n;
  char aborted[128]; /**< guard reason, empty if the run completed */
  double t0_monotonic_s; /**< CLOCK_MONOTONIC time of log t = 0 */
} pulse_result_t;

/** @brief CLOCK_MONOTONIC seconds. */
double pulse_now_s(void);
/** @brief Sleep for s seconds. */
void pulse_sleep_s(double s);

/**
 * @brief Wait until the given wheels report IDLE: CONFIG is refused in any other
 *        state, and STOP -> IDLE takes ~0.7 s after a parked or aborted run.
 */
int pulse_wait_idle(ik_link_t *l, const int *wheel_indices, int n, double timeout_s);

/**
 * @brief Run the plan. Refuses (returns -1, nothing sent) if a wheel has no live reply.
 *        Always stops driving, waits for STOPPED/IDLE and restores the defaults of
 *        every key in settings, even after an error or a guard abort.
 * @param plan       moves from esc_pulse_plan.
 * @param duration   total run time, s.
 * @param settings   keys and per-wheel values to set first (every tunable, not only the
 *                   non-default ones: the ESCs may still hold values from an earlier run).
 * @param accel_rpmps wheel acceleration limit (0 = Nucleo default 900).
 * @param out        filled on success and on a guard abort; free with pulse_result_free.
 * @return 0 ok (also when the guard aborted: see out->aborted), -1 on error with l->err set.
 */
int pulse_run(ik_link_t *l, const esc_move_t *plan, size_t n_moves, double duration,
              const pulse_settings_t *settings, int accel_rpmps, pulse_result_t *out);

void pulse_result_free(pulse_result_t *r);
/** @brief Write the log as JSON lines (the format of the Python tools). */
int pulse_log_write(const char *path, const pulse_result_t *r);

/** @brief JSON array of the space-separated names, e.g. `["a","b"]`. */
void pulse_json_names(FILE *f, const char *names);

#endif
