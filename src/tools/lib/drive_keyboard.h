/** @file drive_keyboard.h
 * @brief Keyboard state and telemetry formatting shared with host tests.
 */
#ifndef DRIVE_KEYBOARD_H
#define DRIVE_KEYBOARD_H
#include "drive_ops.h"
typedef struct {
  drive_keys_t keys;
  double speed_mps, max_speed_mps, yaw_radps, quit_deadline_s;
  bool quitting;
} drive_keyboard_t;
/** @brief Apply a key; X/ESC begins the three-second STOP wait, +/- adjusts speed. */
void drive_keyboard_press(drive_keyboard_t *state, int key, double now_s);
/** @brief Generate the next command; quitting always produces zero and disable. */
bool drive_keyboard_vector(drive_keyboard_t *state, double now_s, int32_t vector[3]);
/** @brief JSONL keyboard telemetry with absolute CLOCK_MONOTONIC time and origin. */
int drive_keyboard_log(FILE *f, double now_s, double t0_s, const int32_t vector[3],
                        const ik_status_t *status, const ik_row_t rows[4]);
/** @brief Print the four telemetry rows, including stale replies and alignment flags. */
void drive_keyboard_show(const ik_status_t *status, const ik_row_t rows[4]);
#endif
