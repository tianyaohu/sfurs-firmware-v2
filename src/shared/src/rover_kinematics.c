#include "rover_kinematics.h"

#include <math.h>
#include <stddef.h>

/* Hand-measured contacts (track ~141–144 mm, wheelbase ~95 mm).
 * Roller angles are nominal, not fit to a motion test: front ±30° from +y,
 * rear ±35°. dir_* is positive wheel travel before motor_sign. Left motors
 * are mirrored, so +RPM on the left is opposite the right. Direct drive:
 * gear ratio is 1 and is not stored. */
const struct rover_wheel rover_wheels[ROVER_WHEEL_COUNT] = {
    { 0.0720f,  0.0475f, -0.5000000f, 0.8660254f,  1.0f}, /* FR */
    { 0.0705f, -0.0475f,  0.5735764f, 0.8191520f,  1.0f}, /* BR */
    {-0.0705f, -0.0475f, -0.5735764f, 0.8191520f, -1.0f}, /* BL */
    {-0.0720f,  0.0475f,  0.5000000f, 0.8660254f, -1.0f}, /* FL */
};

int rover_twist_to_rpm(float vx_m_s, float vy_m_s, float omega_rad_s,
                       float rpm[ROVER_WHEEL_COUNT], float *applied_scale)
{
    float raw[ROVER_WHEEL_COUNT];
    float scale = 1.0f;
    const float rpm_per_m_s =
        60.0f / (2.0f * 3.14159265f * ROVER_WHEEL_RADIUS_M);

    if (applied_scale != NULL) {
        *applied_scale = 0.0f;
    }
    if (rpm == NULL) {
        return -1;
    }
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        rpm[i] = 0.0f;
    }
    if (!isfinite(vx_m_s) || !isfinite(vy_m_s) || !isfinite(omega_rad_s)) {
        return -1;
    }

    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        const struct rover_wheel *w = &rover_wheels[i];
        /* Yaw contribution along the roller: r × ω, then dot dir. */
        float arm_m = w->x_m * w->dir_y - w->y_m * w->dir_x;
        float along_m_s = w->dir_x * vx_m_s + w->dir_y * vy_m_s
                        + arm_m * omega_rad_s;
        float magnitude;

        raw[i] = along_m_s * rpm_per_m_s * w->motor_sign;
        if (!isfinite(raw[i])) {
            return -1;
        }
        magnitude = fabsf(raw[i]);
        if (magnitude > ROVER_MOTOR_RPM_LIMIT) {
            float fit = ROVER_MOTOR_RPM_LIMIT / magnitude;
            if (fit < scale) {
                scale = fit;
            }
        }
    }

    /* One scale for every wheel, so the body direction does not skew. */
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        rpm[i] = raw[i] * scale;
    }
    if (applied_scale != NULL) {
        *applied_scale = scale;
    }
    return 0;
}
