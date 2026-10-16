#include "rover_kinematics.h"
#include <math.h>
#include <stddef.h>

const struct rover_wheel_geometry rover_ik_provisional[ROVER_WHEEL_COUNT] = {
  /* Contact x/y; forward rolling dx/dy; radius; ratio; motor sign; RPM cap. */
  { 0.0720f,  0.0475f, -0.5000000f, 0.8660254f, 0.03275f, 1.0f,  1.0f, ROVER_IK_MAX_MOTOR_RPM},
  { 0.0705f, -0.0475f,  0.5735764f, 0.8191520f, 0.03275f, 1.0f,  1.0f, ROVER_IK_MAX_MOTOR_RPM},
  {-0.0705f, -0.0475f, -0.5735764f, 0.8191520f, 0.03275f, 1.0f, -1.0f, ROVER_IK_MAX_MOTOR_RPM},
  {-0.0720f,  0.0475f,  0.5000000f, 0.8660254f, 0.03275f, 1.0f, -1.0f, ROVER_IK_MAX_MOTOR_RPM}
};

static int geometry_valid(const struct rover_wheel_geometry *w)
{
  return isfinite(w->x_m) && isfinite(w->y_m) &&
         isfinite(w->dx) && isfinite(w->dy) &&
         fabsf(w->dx * w->dx + w->dy * w->dy - 1.0f) < 0.001f &&
         isfinite(w->radius_m) && w->radius_m > 0.0f &&
         isfinite(w->gear_ratio) && w->gear_ratio > 0.0f &&
         (w->motor_sign == 1.0f || w->motor_sign == -1.0f) &&
         isfinite(w->max_motor_rpm) && w->max_motor_rpm > 0.0f;
}

int rover_ik(const struct rover_wheel_geometry geometry[ROVER_WHEEL_COUNT],
             float vx, float vy, float omega,
             float rpm[ROVER_WHEEL_COUNT], float *applied_scale)
{
  float target[ROVER_WHEEL_COUNT];
  float scale = 1.0f;
  if (applied_scale != NULL) *applied_scale = 0.0f;
  if (rpm == NULL) return 0;
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) rpm[i] = 0.0f;
  if (geometry == NULL || !isfinite(vx) || !isfinite(vy) || !isfinite(omega)) return 0;

  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    const struct rover_wheel_geometry *w = &geometry[i];
    if (!geometry_valid(w)) return 0;
    const float circumference = 6.28318530718f * w->radius_m;
    if (!isfinite(circumference) || circumference <= 0.0f) return 0;
    const float yaw_arm = w->x_m * w->dy - w->y_m * w->dx;
    const float linear = w->dx * vx + w->dy * vy + yaw_arm * omega;
    target[i] = linear * (60.0f / circumference) *
                w->gear_ratio * w->motor_sign;
    if (!isfinite(target[i])) return 0;
    const float magnitude = fabsf(target[i]);
    if (magnitude > w->max_motor_rpm) {
      const float factor = w->max_motor_rpm / magnitude;
      if (factor < scale) scale = factor;
    }
  }
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) rpm[i] = target[i] * scale;
  if (applied_scale != NULL) *applied_scale = scale;
  return 1;
}
