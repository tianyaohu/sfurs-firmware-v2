/**
 * @file hall_angle.h
 * @brief Rotor electrical angle from the Hall edges, once per PWM period (pure C, no MCSDK types).
 *
 * Replaces MCSDK's extrapolation, which runs the angle up to 60 deg electrical past the last edge at the last edge's
 * speed and only falls back to the sector centre after 150 ms without an edge. A wheel stuck just past an edge then
 * gets its current up to 60 deg off (half the torque per measured amp); on the mat stuck wheels moved again 150 ms
 * after their last edge (hardware-test-results/2026-10-22-mat-boost-8a.md). Here a slow or late rotor is put at the
 * centre of the sector the Hall sensors report (at most 30 deg off, >= 87 % of the torque).
 * Angles are s16 electrical (65536 = 360 deg), speeds s16 per PWM period, as in MCSDK.
 */
#ifndef HALL_ANGLE_H
#define HALL_ANGLE_H

#include <stdint.h>

#define HALL_ANGLE_S60 10923 /**< 60 deg electrical, s16 */
/** Below this |speed| (one edge per 20 ms at 16 kHz, ~62 RPM with 8 pole pairs) the angle is the sector centre,
 * s16 per PWM period. */
#define HALL_ANGLE_SLOW_DPP 34
/** MCSDK's "speed unknown" marker in hElSpeedDpp. */
#define HALL_ANGLE_PSEUDO_SPEED INT16_MAX

/** @brief Per-wheel state; fields private to hall_angle.c. */
struct hall_angle {
  uint16_t edges_pos, edges_neg; /**< Hall edge counters at the last step */
  int16_t base;                  /**< angle of the last edge (the sector boundary it entered by), s16 */
  int8_t dir;                    /**< direction of the last edge: +1, -1, 0 before any */
  uint16_t since;                /**< PWM periods since that edge (saturating) */
};

/**
 * @brief Start with no edge seen.
 * @param a         state
 * @param edges_pos Hall edges counted in + direction now (wrapping)
 * @param edges_neg Hall edges counted in - direction now (wrapping)
 */
void hall_angle_init(struct hall_angle *a, uint16_t edges_pos, uint16_t edges_neg);

/**
 * @brief Electrical angle for this PWM period.
 * @param a          state
 * @param edges_pos  Hall edges counted in + direction (wrapping)
 * @param edges_neg  Hall edges counted in - direction (wrapping)
 * @param edge_angle angle of the boundary at the latest edge (MCSDK MeasuredElAngle as its edge interrupt sets it), s16
 * @param speed_dpp  electrical speed, s16 per PWM period (MCSDK hElSpeedDpp; ::HALL_ANGLE_PSEUDO_SPEED = unknown)
 * @param fallback   angle to use before the first edge (MCSDK's sector-centre start angle), s16
 * @return electrical angle, s16: the sector centre when slow, late or against the last edge; else extrapolated from
 *         the edge, held at the next boundary for at most a quarter of a sector period more
 */
int16_t hall_angle_step(struct hall_angle *a, uint16_t edges_pos, uint16_t edges_neg, int16_t edge_angle,
                        int16_t speed_dpp, int16_t fallback);

#endif
