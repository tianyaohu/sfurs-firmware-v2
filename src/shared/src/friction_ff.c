#include "friction_ff.h"
#include <math.h>
#include <stddef.h>

/* Wheel force f_i acts along the rolling direction d_i at the contact point p_i,
 * so the body wrench (Fx, Fy, Tz) = G f with G column i = (dx, dy, x*dy - y*dx).
 * The same yaw arm as rover_ik, so wheel speed and force stay duals. */
static int geometry_ok(const struct rover_wheel_geometry *w)
{
  return isfinite(w->x_m) && isfinite(w->y_m) && isfinite(w->dx) && isfinite(w->dy) &&
         isfinite(w->radius_m) && w->radius_m > 0.0f && isfinite(w->gear_ratio) &&
         w->gear_ratio > 0.0f && (w->motor_sign == 1.0f || w->motor_sign == -1.0f);
}

/* Solve (G G^T) y = w for the 3x3 symmetric system by Cramer's rule. */
static int solve3(const float m[3][3], const float b[3], float y[3])
{
  const float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (!isfinite(det) || fabsf(det) < 1e-12f) return 0;
  y[0] = (b[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
          m[0][1] * (b[1] * m[2][2] - m[1][2] * b[2]) +
          m[0][2] * (b[1] * m[2][1] - m[1][1] * b[2])) / det;
  y[1] = (m[0][0] * (b[1] * m[2][2] - m[1][2] * b[2]) -
          b[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
          m[0][2] * (m[1][0] * b[2] - b[1] * m[2][0])) / det;
  y[2] = (m[0][0] * (m[1][1] * b[2] - b[1] * m[2][1]) -
          m[0][1] * (m[1][0] * b[2] - b[1] * m[2][0]) +
          b[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det;
  return 1;
}

int friction_ff_wheel_amps(const struct rover_wheel_geometry geometry[ROVER_WHEEL_COUNT],
                           const struct friction_ff_params *params,
                           float vx, float vy, float omega,
                           float amps[ROVER_WHEEL_COUNT])
{
  if (amps == NULL) return 0;
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) amps[i] = 0.0f;
  if (geometry == NULL || params == NULL || !isfinite(vx) || !isfinite(vy) || !isfinite(omega) ||
      !isfinite(params->coulomb_xy_n) || !isfinite(params->viscous_xy_n_per_mps) ||
      !isfinite(params->coulomb_w_nm) || !isfinite(params->viscous_w_nm_per_radps) ||
      !isfinite(params->motor_a_per_nm) || params->motor_a_per_nm <= 0.0f)
    return 0;

  float g[3][ROVER_WHEEL_COUNT];
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    const struct rover_wheel_geometry *w = &geometry[i];
    if (!geometry_ok(w) || !isfinite(params->wheel_scale[i]) || params->wheel_scale[i] <= 0.0f)
      return 0;
    g[0][i] = w->dx;
    g[1][i] = w->dy;
    g[2][i] = w->x_m * w->dy - w->y_m * w->dx;
  }

  /* The wrench that cancels friction points along the commanded motion. */
  float wrench[3] = {0.0f, 0.0f, 0.0f};
  const float speed = hypotf(vx, vy);
  if (speed >= FRICTION_FF_MIN_SPEED_MPS) {
    const float force = params->coulomb_xy_n + params->viscous_xy_n_per_mps * speed;
    wrench[0] = force * vx / speed;
    wrench[1] = force * vy / speed;
  }
  if (fabsf(omega) >= FRICTION_FF_MIN_OMEGA_RADPS)
    wrench[2] = copysignf(params->coulomb_w_nm + params->viscous_w_nm_per_radps * fabsf(omega), omega);

  float ggt[3][3];
  for (unsigned r = 0; r < 3U; ++r)
    for (unsigned c = 0; c < 3U; ++c) {
      float sum = 0.0f;
      for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) sum += g[r][i] * g[c][i];
      ggt[r][c] = sum;
    }
  float y[3];
  if (!solve3(ggt, wrench, y)) return 0;

  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    const float force = g[0][i] * y[0] + g[1][i] * y[1] + g[2][i] * y[2]; /* N along d_i */
    const float torque = force * geometry[i].radius_m / geometry[i].gear_ratio; /* N*m at the motor */
    amps[i] = geometry[i].motor_sign * torque * params->motor_a_per_nm * params->wheel_scale[i];
  }
  return 1;
}
