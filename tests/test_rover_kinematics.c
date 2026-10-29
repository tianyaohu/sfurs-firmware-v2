#include "rover_kinematics.h"

#include <math.h>
#include <stdio.h>

/* Golden RPM for a unit twist. Computed from the wheel map and
 *   along = dir_x*vx + dir_y*vy + (x*dir_y - y*dir_x)*omega
 *   rpm   = along * 60 / (2*pi*radius) * motor_sign
 * then rounded to 4 decimals. Not produced by the function under test.
 *
 * Radius 32.75 mm, direct drive. Front rollers 30 deg off +y, rear 35 deg.
 * motor_sign is +1 on FR/BR and -1 on BL/FL.
 */

static const char *k_wheel[ROVER_WHEEL_COUNT] = {"FR", "BR", "BL", "FL"};

/* +1 m/s forward. Front dir_y = cos(30 deg) -> 252.5 RPM; rear cos(35 deg)
 * -> 238.8 RPM. Right side positive, left side negative. */
static const float k_forward_1mps[ROVER_WHEEL_COUNT] = {
    252.5170f, 238.8496f, -238.8496f, -252.5170f
};

/* +1 rad/s CCW. Moment arm is ~86 mm, so surface speed is ~0.086 m/s and
 * motor RPM is ~25. After motor_sign every wheel is positive. */
static const float k_rotate_ccw_1radps[ROVER_WHEEL_COUNT] = {
    25.1063f, 24.7830f, 24.7830f, 25.1063f
};

/* +1 m/s to the right. Rear rollers (35 deg) are faster than the front
 * (30 deg). FR and FL go negative; BR and BL go positive. */
static const float k_strafe_right_1mps[ROVER_WHEEL_COUNT] = {
    -145.7908f, 167.2443f, 167.2443f, -145.7908f
};

static int g_failures;

static void fail(const char *name, const char *msg)
{
    fprintf(stderr, "FAIL %s: %s\n", name, msg);
    g_failures++;
}

static void expect_ok(const char *name, float vx_m_s, float vy_m_s,
                      float omega_rad_s, const float want[ROVER_WHEEL_COUNT],
                      float want_scale)
{
    float got[ROVER_WHEEL_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    float scale = -1.0f;
    int rc = rover_twist_to_rpm(vx_m_s, vy_m_s, omega_rad_s, got, &scale);
    if (rc != 0) {
        fprintf(stderr, "FAIL %s: rc %d\n", name, rc);
        g_failures++;
        return;
    }
    if (fabsf(scale - want_scale) > 0.0001f) {
        fprintf(stderr, "FAIL %s: scale %.6f want %.6f\n", name, scale, want_scale);
        g_failures++;
    }
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        if (fabsf(got[i] - want[i]) > 0.02f) {
            fprintf(stderr, "FAIL %s %s: got %.4f want %.4f\n",
                    name, k_wheel[i], got[i], want[i]);
            g_failures++;
        }
    }
}

static void test_pure_forward(void)
{
    float reverse[ROVER_WHEEL_COUNT];
    expect_ok("forward", 0.0f, 1.0f, 0.0f, k_forward_1mps, 1.0f);
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        reverse[i] = -k_forward_1mps[i];
    }
    expect_ok("reverse", 0.0f, -1.0f, 0.0f, reverse, 1.0f);
}

static void test_pure_rotate(void)
{
    float cw[ROVER_WHEEL_COUNT];
    expect_ok("rotate-ccw", 0.0f, 0.0f, 1.0f, k_rotate_ccw_1radps, 1.0f);
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        cw[i] = -k_rotate_ccw_1radps[i];
    }
    expect_ok("rotate-cw", 0.0f, 0.0f, -1.0f, cw, 1.0f);
}

static void test_pure_strafe(void)
{
    expect_ok("strafe-right", 1.0f, 0.0f, 0.0f, k_strafe_right_1mps, 1.0f);
}

static void test_zero_has_no_floor(void)
{
    float rpm[ROVER_WHEEL_COUNT] = {4.0f, 4.0f, 4.0f, 4.0f};
    float scale = -1.0f;
    int rc = rover_twist_to_rpm(0.0f, 0.0f, 0.0f, rpm, &scale);
    if (rc != 0) {
        fail("zero", "rc");
        return;
    }
    if (scale != 1.0f) {
        fail("zero", "scale");
    }
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        if (rpm[i] != 0.0f) {
            fprintf(stderr, "FAIL zero %s: got %.4f\n", k_wheel[i], rpm[i]);
            g_failures++;
        }
    }
}

static void test_axes_add(void)
{
    float want[ROVER_WHEEL_COUNT];
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        want[i] = 0.2f * k_forward_1mps[i] + 0.1f * k_rotate_ccw_1radps[i];
    }
    expect_ok("forward+yaw", 0.0f, 0.2f, 0.1f, want, 1.0f);
}

static void test_saturation_keeps_direction(void)
{
    float slow[ROVER_WHEEL_COUNT];
    float fast[ROVER_WHEEL_COUNT];
    float scale = -1.0f;
    float peak = 0.0f;
    float limited = 0.0f;
    const float speed_m_s = 4.0f;

    if (rover_twist_to_rpm(0.0f, 1.0f, 0.0f, slow, NULL) != 0) {
        fail("saturate", "unit forward");
        return;
    }
    if (rover_twist_to_rpm(0.0f, speed_m_s, 0.0f, fast, &scale) != 0) {
        fail("saturate", "rc");
        return;
    }
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        float unscaled = fabsf(slow[i] * speed_m_s);
        if (unscaled > peak) {
            peak = unscaled;
        }
    }
    if (peak <= ROVER_MOTOR_RPM_LIMIT) {
        fail("saturate", "4 m/s did not reach the cap");
        return;
    }
    if (fabsf(scale - ROVER_MOTOR_RPM_LIMIT / peak) > 0.0001f) {
        fprintf(stderr, "FAIL saturate: scale %.6f\n", scale);
        g_failures++;
    }
    for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
        float want = slow[i] * speed_m_s * scale;
        float mag = fabsf(fast[i]);
        if (fabsf(fast[i] - want) > 0.05f) {
            fprintf(stderr, "FAIL saturate %s: got %.4f want %.4f\n",
                    k_wheel[i], fast[i], want);
            g_failures++;
        }
        if (mag > limited) {
            limited = mag;
        }
        if (mag > ROVER_MOTOR_RPM_LIMIT + 0.05f) {
            fprintf(stderr, "FAIL saturate %s: %.4f over the cap\n",
                    k_wheel[i], fast[i]);
            g_failures++;
        }
    }
    if (fabsf(limited - ROVER_MOTOR_RPM_LIMIT) > 0.05f) {
        fprintf(stderr, "FAIL saturate: peak %.4f\n", limited);
        g_failures++;
    }
}

static void test_rejects_bad_input(void)
{
    float rpm[ROVER_WHEEL_COUNT];
    float scale;
    const float bad[3] = {NAN, INFINITY, -INFINITY};

    if (rover_twist_to_rpm(0.0f, 0.0f, 0.0f, NULL, NULL) != -1) {
        fail("null", "rpm pointer");
    }
    rpm[0] = rpm[1] = rpm[2] = rpm[3] = 5.0f;
    scale = 5.0f;
    if (rover_twist_to_rpm(1.0e38f, 0.0f, 0.0f, rpm, &scale) != -1 ||
        scale != 0.0f || rpm[0] != 0.0f) {
        fail("overflow", "huge vx");
    }
    for (int axis = 0; axis < 3; ++axis) {
        for (int b = 0; b < 3; ++b) {
            float v[3] = {0.1f, 0.2f, 0.3f};
            v[axis] = bad[b];
            rpm[0] = rpm[1] = rpm[2] = rpm[3] = 5.0f;
            scale = 5.0f;
            if (rover_twist_to_rpm(v[0], v[1], v[2], rpm, &scale) != -1 ||
                scale != 0.0f) {
                fprintf(stderr, "FAIL nonfinite axis %d value %d\n", axis, b);
                g_failures++;
                continue;
            }
            for (int i = 0; i < ROVER_WHEEL_COUNT; ++i) {
                if (rpm[i] != 0.0f) {
                    fprintf(stderr, "FAIL nonfinite leftover %s\n", k_wheel[i]);
                    g_failures++;
                }
            }
        }
    }
}

int main(void)
{
    test_pure_forward();
    test_pure_rotate();
    test_pure_strafe();
    test_zero_has_no_floor();
    test_axes_add();
    test_saturation_keeps_direction();
    test_rejects_bad_input();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("rover kinematics: all wheels ok\n");
    return 0;
}
