#ifndef ROVER_KINEMATICS_H
#define ROVER_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

enum { ROVER_FR, ROVER_BR, ROVER_BL, ROVER_FL, ROVER_WHEEL_COUNT };

/* Coordinates: x right, y forward, positive yaw CCW from above.
 * dx/dy: unit rolling direction for positive wheel travel BEFORE motor_sign.
 * gear_ratio: motor revolutions per wheel revolution (direct drive = 1).
 * motor_sign: +1 or -1 converts positive wheel travel to raw CAN motor RPM.
 * max_motor_rpm: positive magnitude limit, not a minimum running speed. */
struct rover_wheel_geometry {
  float x_m, y_m;
  float dx, dy;
  float radius_m;
  float gear_ratio;
  float motor_sign;
  float max_motor_rpm;
};

/* Working assumptions ONLY; contact positions, angles, gear ratio and raw
 * motor signs are not independently verified. See docs/OMNI_KINEMATICS.md.
 * The optional, exclusive IK image uses this table; the normal bench does not. */
extern const struct rover_wheel_geometry rover_ik_provisional[ROVER_WHEEL_COUNT];

/* vx/vy in m/s, omega in rad/s. Outputs raw signed motor RPM in FR/BR/BL/FL
 * order, ready for a transport that does NOT mirror signs again.
 * All wheels are scaled by a common factor <=1 when any limit is exceeded,
 * preserving the requested twist direction. Zero stays zero (no speed floor).
 * Returns 1 on success, 0 for invalid/nonfinite input or overflow. On failure
 * rpm[] is zeroed; applied_scale is optional and set to zero on failure.
 * geometry and rpm must be non-NULL; rpm must hold four floats. */
int rover_ik(const struct rover_wheel_geometry geometry[ROVER_WHEEL_COUNT],
             float vx, float vy, float omega,
             float rpm[ROVER_WHEEL_COUNT], float *applied_scale);

#ifdef __cplusplus
}
#endif
#endif
