#include "rover_kinematics.h"
#include "check.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct result {
  int ok;
  float rpm[4], scale;
};

static struct result run_ik(const float velocity[3], const struct rover_wheel_geometry *geometry) {
  struct result result = {.rpm = {999, 999, 999, 999}, .scale = -1};
  result.ok = rover_ik(geometry, velocity[0], velocity[1], velocity[2], result.rpm, &result.scale);
  return result;
}

static void assert_close(const float actual[4], const float expected[4]) {
  for (unsigned i = 0; i < 4; ++i) CHECK_NEAR(actual[i], expected[i], 0.0002);
}

static void check_failure(struct result result) {
  CHECK_EQ(result.ok, 0);
  CHECK(result.scale == 0);
  for (unsigned i = 0; i < 4; ++i) CHECK(result.rpm[i] == 0);
}

static void test_axes_against_independent_coefficients(void) {
  /* Independently reviewed coefficients catch geometry and sign regressions. */
  const float expected[4][3] = {
    {-145.791f, 252.517f, 25.106f},
    {167.244f, 238.850f, 24.783f},
    {167.244f, -238.850f, 24.783f},
    {-145.791f, -252.517f, 25.106f}
  };
  for (unsigned axis = 0; axis < 3; ++axis) {
    float velocity[3] = {0};
    velocity[axis] = 0.1f;
    struct result result = run_ik(velocity, rover_ik_provisional);
    CHECK_EQ(result.ok, 1);
    CHECK(result.scale == 1);
    for (unsigned i = 0; i < 4; ++i)
      CHECK_NEAR(result.rpm[i], expected[i][axis] * 0.1f, 0.0002);
  }
}

static void test_zero_and_small_commands_have_no_floor(void) {
  struct result zero = run_ik((float[3]){0, 0, 0}, rover_ik_provisional);
  CHECK_EQ(zero.ok, 1);
  CHECK(zero.scale == 1);
  for (unsigned i = 0; i < 4; ++i) CHECK(zero.rpm[i] == 0);
  struct result small = run_ik((float[3]){0, 0.001f, 0}, rover_ik_provisional);
  CHECK_EQ(small.ok, 1);
  for (unsigned i = 0; i < 4; ++i) CHECK(0 < fabsf(small.rpm[i]) && fabsf(small.rpm[i]) < 1);
}

static void test_mixed_linearity_and_reversal(void) {
  const float a[3] = {0.1f, -0.12f, 0.2f}, b[3] = {-0.03f, 0.04f, -0.1f};
  float sum[3], reversed[3], expected[4];
  for (unsigned axis = 0; axis < 3; ++axis) {
    sum[axis] = a[axis] + b[axis];
    reversed[axis] = -a[axis];
  }
  struct result ra = run_ik(a, rover_ik_provisional), rb = run_ik(b, rover_ik_provisional);
  struct result combined = run_ik(sum, rover_ik_provisional);
  CHECK_EQ(ra.ok, 1);
  CHECK_EQ(rb.ok, 1);
  CHECK_EQ(combined.ok, 1);
  for (unsigned i = 0; i < 4; ++i) expected[i] = ra.rpm[i] + rb.rpm[i];
  assert_close(combined.rpm, expected);
  struct result reverse = run_ik(reversed, rover_ik_provisional);
  CHECK_EQ(reverse.ok, 1);
  for (unsigned i = 0; i < 4; ++i) expected[i] = -ra.rpm[i];
  assert_close(reverse.rpm, expected);
}

static void test_uniform_saturation_preserves_twist(void) {
  struct result small = run_ik((float[3]){0.1f, 0.2f, 0.3f}, rover_ik_provisional);
  struct result result = run_ik((float[3]){10, 20, 30}, rover_ik_provisional);
  CHECK_EQ(result.ok, 1);
  CHECK(0 < result.scale && result.scale < 1);
  float expected[4];
  int at_cap = 0;
  for (unsigned i = 0; i < 4; ++i) {
    expected[i] = small.rpm[i] * 100 * result.scale;
    CHECK(fabsf(result.rpm[i]) <= rover_ik_provisional[i].max_motor_rpm + 0.0001);
    at_cap |= fabs(fabs(result.rpm[i]) - rover_ik_provisional[i].max_motor_rpm) < 0.0001;
  }
  assert_close(result.rpm, expected);
  CHECK(at_cap);
}

static void test_all_four_900_caps_and_tournament_speed_scaling(void) {
  for (unsigned i = 0; i < 4; ++i) CHECK(rover_ik_provisional[i].max_motor_rpm == 900);
  struct result unit = run_ik((float[3]){0, 1, 0}, rover_ik_provisional);
  for (unsigned speed = 1; speed <= 3; ++speed) {
    struct result result = run_ik((float[3]){0, (float)speed, 0}, rover_ik_provisional);
    CHECK_EQ(result.ok, 1);
    CHECK(result.scale == 1);
    float expected[4];
    for (unsigned i = 0; i < 4; ++i) expected[i] = unit.rpm[i] * speed;
    assert_close(result.rpm, expected);
  }
  const float velocities[4][3] = {{0, 5, 0}, {0, -5, 0}, {5, 5, 1}, {-5, -5, -1}};
  for (unsigned v = 0; v < 4; ++v) {
    struct result result = run_ik(velocities[v], rover_ik_provisional);
    CHECK_EQ(result.ok, 1);
    CHECK(0 < result.scale && result.scale < 1);
    float largest = 0;
    for (unsigned i = 0; i < 4; ++i) {
      CHECK(fabsf(result.rpm[i]) <= 900.0001);
      if (fabsf(result.rpm[i]) > largest) largest = fabsf(result.rpm[i]);
    }
    CHECK_NEAR(largest, 900, 0.0001);
  }
}

static void test_motor_sign_and_gearing_applied_once(void) {
  const float velocity[3] = {0.05f, 0.1f, 0.2f};
  struct result base = run_ik(velocity, rover_ik_provisional);
  for (unsigned wheel = 0; wheel < 4; ++wheel) {
    struct rover_wheel_geometry geometry[4];
    memcpy(geometry, rover_ik_provisional, sizeof(geometry));
    geometry[wheel].motor_sign = -geometry[wheel].motor_sign;
    geometry[wheel].gear_ratio = 2;
    struct result result = run_ik(velocity, geometry);
    CHECK_EQ(result.ok, 1);
    float expected[4];
    for (unsigned i = 0; i < 4; ++i) expected[i] = base.rpm[i] * (i == wheel ? -2 : 1);
    assert_close(result.rpm, expected);
  }
}

static void solve3(const double matrix[3][3], const double rhs[3], double out[3]) {
  double rows[3][4];
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 3; ++j) rows[i][j] = matrix[i][j];
    rows[i][3] = rhs[i];
  }
  for (unsigned col = 0; col < 3; ++col) {
    unsigned pivot = col;
    for (unsigned i = col + 1; i < 3; ++i)
      if (fabs(rows[i][col]) > fabs(rows[pivot][col])) pivot = i;
    for (unsigned j = 0; j < 4; ++j) {
      double tmp = rows[col][j];
      rows[col][j] = rows[pivot][j];
      rows[pivot][j] = tmp;
    }
    double divisor = rows[col][col];
    CHECK(fabs(divisor) > 1e-12);
    for (unsigned j = 0; j < 4; ++j) rows[col][j] /= divisor;
    for (unsigned i = 0; i < 3; ++i) {
      if (i == col) continue;
      double factor = rows[i][col];
      for (unsigned j = 0; j < 4; ++j) rows[i][j] -= factor * rows[col][j];
    }
  }
  for (unsigned i = 0; i < 3; ++i) out[i] = rows[i][3];
}

static float random_velocity(uint32_t *state) {
  /* A fixed generator makes the 100 reconstruction trials reproducible. */
  *state ^= *state << 13;
  *state ^= *state >> 17;
  *state ^= *state << 5;
  return (float)(4.0 * *state / UINT32_MAX - 2.0);
}

static void test_forward_reconstruction(void) {
  double matrix[4][3], normal[3][3] = {{0}};
  for (unsigned wheel = 0; wheel < 4; ++wheel) {
    const struct rover_wheel_geometry *w = &rover_ik_provisional[wheel];
    matrix[wheel][0] = w->dx;
    matrix[wheel][1] = w->dy;
    matrix[wheel][2] = -(double)w->y_m * w->dx + (double)w->x_m * w->dy;
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = 0; j < 3; ++j) normal[i][j] += matrix[wheel][i] * matrix[wheel][j];
  }
  uint32_t state = 2026;
  for (unsigned trial = 0; trial < 100; ++trial) {
    float velocity[3];
    for (unsigned i = 0; i < 3; ++i) velocity[i] = random_velocity(&state);
    struct result result = run_ik(velocity, rover_ik_provisional);
    CHECK_EQ(result.ok, 1);
    double rhs[3] = {0}, reconstructed[3];
    for (unsigned wheel = 0; wheel < 4; ++wheel) {
      const struct rover_wheel_geometry *w = &rover_ik_provisional[wheel];
      double linear = result.rpm[wheel] * (6.283185307179586 * w->radius_m) /
                      (60.0 * w->gear_ratio * w->motor_sign);
      for (unsigned i = 0; i < 3; ++i) rhs[i] += matrix[wheel][i] * linear;
    }
    solve3(normal, rhs, reconstructed);
    for (unsigned i = 0; i < 3; ++i) CHECK_NEAR(reconstructed[i], velocity[i] * result.scale, 0.0002);
  }
}

static void test_invalid_geometry_and_nonfinite_inputs(void) {
  const struct {
    size_t offset;
    unsigned count;
    float values[4];
  } bad_fields[] = {
    {offsetof(struct rover_wheel_geometry, radius_m), 4, {0, -1, NAN, 1e38f}},
    {offsetof(struct rover_wheel_geometry, gear_ratio), 2, {0, INFINITY}},
    {offsetof(struct rover_wheel_geometry, motor_sign), 2, {0, 2}},
    {offsetof(struct rover_wheel_geometry, max_motor_rpm), 2, {0, INFINITY}},
    {offsetof(struct rover_wheel_geometry, dx), 2, {0, NAN}},
    {offsetof(struct rover_wheel_geometry, x_m), 1, {INFINITY}}
  };
  const float velocity[3] = {0.1f, 0.2f, 0.3f};
  for (unsigned wheel = 0; wheel < 4; ++wheel) {
    for (unsigned field = 0; field < sizeof(bad_fields) / sizeof(bad_fields[0]); ++field) {
      for (unsigned value = 0; value < bad_fields[field].count; ++value) {
        struct rover_wheel_geometry geometry[4];
        memcpy(geometry, rover_ik_provisional, sizeof(geometry));
        float *member = (float *)((unsigned char *)&geometry[wheel] + bad_fields[field].offset);
        *member = bad_fields[field].values[value];
        check_failure(run_ik(velocity, geometry));
      }
    }
  }
  const float bad_values[4] = {NAN, INFINITY, -INFINITY, 3.4e38f};
  for (unsigned axis = 0; axis < 3; ++axis) {
    for (unsigned bad = 0; bad < 4; ++bad) {
      float v[3] = {0.1f, 0.2f, 0.3f};
      v[axis] = bad_values[bad];
      check_failure(run_ik(v, rover_ik_provisional));
    }
  }
  check_failure(run_ik((float[3]){0, 0, 0}, NULL));
  CHECK_EQ(rover_ik(rover_ik_provisional, 0, 0, 0, NULL, NULL), 0);
}

int main(void) {
  test_axes_against_independent_coefficients();
  test_zero_and_small_commands_have_no_floor();
  test_mixed_linearity_and_reversal();
  test_uniform_saturation_preserves_twist();
  test_all_four_900_caps_and_tournament_speed_scaling();
  test_motor_sign_and_gearing_applied_once();
  test_forward_reconstruction();
  test_invalid_geometry_and_nonfinite_inputs();
  return 0;
}
