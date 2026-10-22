#ifndef FRICTION_FF_H
#define FRICTION_FF_H

#include "rover_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file friction_ff.h
 *  @brief Body-frame friction model: commanded twist to per-wheel feed-forward current.
 *
 *  The robot resists motion with a Coulomb plus viscous wrench, separately for
 *  translation (along the commanded direction) and yaw. The wrench that
 *  cancels it is spread over the four wheels with the minimum-norm solution
 *  (pseudo-inverse of the wheel force map), then turned into motor current.
 *  Identified on the mat by roll-out tests, as TIGERs Mannheim do; zero friction
 *  coefficients give zero current with valid conversion and scale factors. */

/** @brief Friction model parameters. */
struct friction_ff_params {
  float coulomb_xy_n;        /**< translation friction force at any nonzero speed, N */
  float viscous_xy_n_per_mps; /**< added translation force per commanded speed, N/(m/s) */
  float coulomb_w_nm;        /**< yaw friction torque at any nonzero yaw rate, N*m */
  float viscous_w_nm_per_radps; /**< added yaw torque per yaw rate, N*m/(rad/s) */
  float motor_a_per_nm;      /**< positive motor current per motor torque (1 / Kt), A/(N*m) */
  float wheel_scale[ROVER_WHEEL_COUNT]; /**< positive per-wheel current factor, dimensionless; 1 = nominal */
};

/** @brief Translation speeds below this threshold give no translation friction, m/s. */
#define FRICTION_FF_MIN_SPEED_MPS 0.01f
/** @brief Yaw rates below this magnitude give no yaw friction, rad/s. */
#define FRICTION_FF_MIN_OMEGA_RADPS 0.05f

/**
 * @brief Per-wheel feed-forward current for a commanded body twist.
 *
 * Positive current is the sign of positive Iq, the same sign convention as the
 * raw motor RPM of ::rover_ik, so a wheel whose command is positive and which
 * must push forward gets a positive value.
 *
 * @param geometry wheel table, ::ROVER_WHEEL_COUNT entries, as for ::rover_ik
 * @param params   friction model, non-NULL
 * @param vx       commanded body velocity along x (right), m/s
 * @param vy       commanded body velocity along y (forward), m/s
 * @param omega    commanded yaw rate, rad/s (CCW positive)
 * @param amps     out: current in A in FR/BR/BL/FL order; must hold four floats
 * @return 1 on success, 0 for invalid input or a singular geometry (amps zeroed
 *         when non-NULL)
 */
int friction_ff_wheel_amps(const struct rover_wheel_geometry geometry[ROVER_WHEEL_COUNT],
                           const struct friction_ff_params *params,
                           float vx, float vy, float omega,
                           float amps[ROVER_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif
#endif
