#ifndef ROVER_KINEMATICS_H
#define ROVER_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/** @file rover_kinematics.h
 *  @brief Omni-wheel inverse kinematics: body twist to raw motor RPM. */

/** @brief Wheel index, also the telemetry and CAN-select order (FR=1 .. FL=8). */
enum { ROVER_FR, ROVER_BR, ROVER_BL, ROVER_FL, ROVER_WHEEL_COUNT };
/** @brief Default per-wheel motor RPM limit used by ::rover_ik_provisional. */
#define ROVER_IK_MAX_MOTOR_RPM 900.0f

/**
 * @brief Geometry and drive sign of one wheel.
 *
 * Coordinates: x right, y forward, positive yaw CCW from above.
 */
struct rover_wheel_geometry {
  float x_m;        /**< wheel contact x relative to the body centre, m */
  float y_m;        /**< wheel contact y relative to the body centre, m */
  float dx;         /**< rolling direction x for positive wheel travel, BEFORE motor_sign (unit vector) */
  float dy;         /**< rolling direction y, as dx */
  float radius_m;   /**< wheel radius, m */
  float gear_ratio; /**< motor revolutions per wheel revolution (direct drive = 1) */
  float motor_sign; /**< +1 or -1: positive wheel travel to raw CAN motor RPM */
  float max_motor_rpm; /**< positive magnitude limit, not a minimum running speed */
};

/**
 * @brief The geometry table the drive uses.
 *
 * Working assumptions ONLY: contact positions come from hand measurements and
 * the angles, gear ratio and raw motor signs are not independently verified
 * (see DRIVE.md, Kinematics).
 */
extern const struct rover_wheel_geometry rover_ik_provisional[ROVER_WHEEL_COUNT];

/**
 * @brief Inverse kinematics for a body twist.
 *
 * Outputs raw signed motor RPM, ready for a transport that does NOT mirror
 * signs again. All wheels are scaled by a common factor <= 1 when any limit is
 * exceeded, preserving the requested twist direction. Zero stays zero (no
 * speed floor). On failure rpm[] is zeroed.
 *
 * @param geometry two-or-more-wheel table, ::ROVER_WHEEL_COUNT entries, non-NULL
 * @param vx, vy   body velocity, m/s
 * @param omega    body yaw rate, rad/s
 * @param rpm      out: motor RPM in FR/BR/BL/FL order; must hold four floats
 * @param applied_scale optional out: common scale actually applied (0 on failure)
 * @return 1 on success, 0 for invalid/nonfinite input or overflow
 */
int rover_ik(const struct rover_wheel_geometry geometry[ROVER_WHEEL_COUNT],
             float vx, float vy, float omega,
             float rpm[ROVER_WHEEL_COUNT], float *applied_scale);

#ifdef __cplusplus
}
#endif
#endif
