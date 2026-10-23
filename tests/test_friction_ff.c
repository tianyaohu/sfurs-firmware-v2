#include "friction_ff.h"
#include "check.h"
#include <stddef.h>
#include <string.h>

struct result {
  int ok;
  float amps[ROVER_WHEEL_COUNT];
};

static struct friction_ff_params params(void) {
  return (struct friction_ff_params){
    .coulomb_x_n = 2.0f, .coulomb_y_n = 2.0f, .coulomb_w_nm = 0.3f, .motor_a_per_nm = 30.0f,
    .wheel_scale = {1, 1, 1, 1}
  };
}

static struct result run_ff_breakaway(const struct rover_wheel_geometry *geometry,
                                      const struct friction_ff_params *p, const float twist[3], int breakaway) {
  struct result result = {.amps = {9, 9, 9, 9}};
  result.ok = friction_ff_wheel_amps(geometry, p, twist[0], twist[1], twist[2], breakaway, result.amps);
  return result;
}

static struct result run_ff(const struct rover_wheel_geometry *geometry,
                            const struct friction_ff_params *p, const float twist[3]) {
  return run_ff_breakaway(geometry, p, twist, 0);
}

/* Body wrench the wheel currents produce, reversed independently in double precision, so this oracle does not
 * repeat the model's matrix solver. */
static void wrench_of(const struct friction_ff_params *p, const float amps[ROVER_WHEEL_COUNT], double wrench[3]) {
  wrench[0] = wrench[1] = wrench[2] = 0;
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    const struct rover_wheel_geometry *w = &rover_ik_provisional[i];
    double torque = (double)amps[i] * w->motor_sign / ((double)p->motor_a_per_nm * p->wheel_scale[i]);
    double force = torque * w->gear_ratio / w->radius_m;
    wrench[0] += force * w->dx;
    wrench[1] += force * w->dy;
    wrench[2] += force * ((double)w->x_m * w->dy - (double)w->y_m * w->dx);
  }
}

static void check_zero(struct result result, int ok) {
  CHECK_EQ(result.ok, ok);
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) CHECK(result.amps[i] == 0.0f);
}

static void test_defaults_are_off(void) {
  struct friction_ff_params p = params();
  p.coulomb_x_n = 0;
  p.coulomb_y_n = 0;
  p.coulomb_w_nm = 0;
  check_zero(run_ff(rover_ik_provisional, &p, (float[3]){0.3f, 0.2f, 1}), 1);
}

static void test_wheels_rebuild_the_cancelling_wrench(void) {
  struct friction_ff_params p = params();
  p.coulomb_x_n = 3.5f;
  p.coulomb_y_n = 2.5f;
  p.viscous_xy_n_per_mps = 1.5f;
  p.coulomb_w_nm = 0.4f;
  p.viscous_w_nm_per_radps = 0.2f;
  p.motor_a_per_nm = 40;
  const float twists[][3] = {
    {0.5f, 0, 0}, {-0.5f, 0, 0}, {0, 0.5f, 0}, {0, -0.5f, 0},
    {0, 0, 2}, {0, 0, -2}, {0.3f, 0.4f, 0}, {0.3f, -0.2f, 1}
  };
  for (unsigned t = 0; t < sizeof(twists) / sizeof(twists[0]); ++t) {
    const float *v = twists[t];
    struct result result = run_ff(rover_ik_provisional, &p, v);
    CHECK_EQ(result.ok, 1);
    double wrench[3];
    wrench_of(&p, result.amps, wrench);
    double speed = hypot(v[0], v[1]);
    double expected[3] = {
      speed ? (p.coulomb_x_n + p.viscous_xy_n_per_mps * speed) * v[0] / speed : 0,
      speed ? (p.coulomb_y_n + p.viscous_xy_n_per_mps * speed) * v[1] / speed : 0,
      v[2] ? copysign(p.coulomb_w_nm + p.viscous_w_nm_per_radps * fabs(v[2]), v[2]) : 0
    };
    for (unsigned axis = 0; axis < 3; ++axis)
      CHECK_NEAR(wrench[axis], expected[axis], 1e-3 * (1 + fabs(expected[axis])));
  }
}

static void test_odd_in_the_twist(void) {
  struct friction_ff_params p = params();
  struct result plus = run_ff(rover_ik_provisional, &p, (float[3]){0.2f, 0.3f, 0.5f});
  struct result minus = run_ff(rover_ik_provisional, &p, (float[3]){-0.2f, -0.3f, -0.5f});
  CHECK_EQ(plus.ok, 1);
  CHECK_EQ(minus.ok, 1);
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i)
    CHECK_NEAR(plus.amps[i], -minus.amps[i], 1e-5);
}

static void test_every_wheel_gets_current_in_the_motion_direction(void) {
  struct friction_ff_params p = params();
  const float twists[][3] = {{0.4f, 0, 0}, {0, 0.4f, 0}, {0, 0, 1.5f}};
  for (unsigned t = 0; t < sizeof(twists) / sizeof(twists[0]); ++t) {
    const float *v = twists[t];
    float rpm[ROVER_WHEEL_COUNT];
    CHECK_EQ(rover_ik(rover_ik_provisional, v[0], v[1], v[2], rpm, NULL), 1);
    struct result result = run_ff(rover_ik_provisional, &p, v);
    CHECK_EQ(result.ok, 1);
    for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
      CHECK(fabsf(result.amps[i]) > 0);
      CHECK(result.amps[i] * rpm[i] > 0);
    }
  }
}

static void test_per_wheel_scale_touches_only_that_wheel(void) {
  struct friction_ff_params p = params();
  const float twist[3] = {0.3f, 0.2f, 0.5f};
  struct result base = run_ff(rover_ik_provisional, &p, twist);
  CHECK_EQ(base.ok, 1);
  for (unsigned wheel = 0; wheel < ROVER_WHEEL_COUNT; ++wheel) {
    struct friction_ff_params scaled = p;
    scaled.wheel_scale[wheel] = 0.5f;
    struct result result = run_ff(rover_ik_provisional, &scaled, twist);
    CHECK_EQ(result.ok, 1);
    CHECK(fabsf(base.amps[wheel]) > 0);
    for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i)
      CHECK_NEAR(result.amps[i], base.amps[i] * (i == wheel ? 0.5f : 1.0f), 1e-5);
  }
}

static void test_coulomb_viscous_and_standstill(void) {
  struct friction_ff_params p = params();
  p.coulomb_x_n = 3;
  p.coulomb_y_n = 3;
  struct result slow = run_ff(rover_ik_provisional, &p, (float[3]){0, 0.05f, 0});
  struct result fast = run_ff(rover_ik_provisional, &p, (float[3]){0, 0.50f, 0});
  CHECK_EQ(slow.ok, 1);
  CHECK_EQ(fast.ok, 1);
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i)
    CHECK_NEAR(slow.amps[i], fast.amps[i], 1e-5);
  p.viscous_xy_n_per_mps = 2;
  struct result slowv = run_ff(rover_ik_provisional, &p, (float[3]){0, 0.05f, 0});
  struct result fastv = run_ff(rover_ik_provisional, &p, (float[3]){0, 0.50f, 0});
  CHECK_EQ(slowv.ok, 1);
  CHECK_EQ(fastv.ok, 1);
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    CHECK_NEAR(fastv.amps[i] / slow.amps[i], (3.0 + 2.0 * 0.5) / 3.0, 1e-4);
    CHECK_NEAR(slowv.amps[i] / slow.amps[i], (3.0 + 2.0 * 0.05) / 3.0, 1e-4);
  }
  check_zero(run_ff(rover_ik_provisional, &p, (float[3]){0, 0, 0}), 1);
  check_zero(run_ff(rover_ik_provisional, &p, (float[3]){0.005f, 0.005f, 0.01f}), 1);
  for (unsigned axis = 0; axis < 3; ++axis) {
    float threshold = axis == 2 ? FRICTION_FF_MIN_OMEGA_RADPS : FRICTION_FF_MIN_SPEED_MPS;
    for (int sign = -1; sign <= 1; sign += 2) {
      float twist[3] = {0};
      twist[axis] = sign * nextafterf(threshold, 0);
      check_zero(run_ff(rover_ik_provisional, &p, twist), 1);
      twist[axis] = sign * threshold;
      struct result at_threshold = run_ff(rover_ik_provisional, &p, twist);
      CHECK_EQ(at_threshold.ok, 1);
      for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i)
        CHECK(fabsf(at_threshold.amps[i]) > 0);
    }
  }
}

static void test_invalid_input_returns_zero_currents(void) {
  const struct friction_ff_params p = params();
  const float twist[3] = {0.1f, 0.2f, 0.3f};
  const float nonfinite[] = {NAN, INFINITY, -INFINITY};
  for (unsigned axis = 0; axis < 3; ++axis) {
    for (unsigned bad = 0; bad < sizeof(nonfinite) / sizeof(nonfinite[0]); ++bad) {
      float v[3] = {0.1f, 0.2f, 0.3f};
      v[axis] = nonfinite[bad];
      check_zero(run_ff(rover_ik_provisional, &p, v), 0);
    }
  }
  const float invalid[] = {0, -1, NAN, INFINITY, -INFINITY};
  for (unsigned bad = 0; bad < sizeof(invalid) / sizeof(invalid[0]); ++bad) {
    struct friction_ff_params broken = p;
    broken.motor_a_per_nm = invalid[bad];
    check_zero(run_ff(rover_ik_provisional, &broken, twist), 0);
    for (unsigned wheel = 0; wheel < ROVER_WHEEL_COUNT; ++wheel) {
      broken = p;
      broken.wheel_scale[wheel] = invalid[bad];
      check_zero(run_ff(rover_ik_provisional, &broken, twist), 0);
    }
  }
  for (unsigned wheel = 0; wheel < ROVER_WHEEL_COUNT; ++wheel) {
    struct rover_wheel_geometry broken[ROVER_WHEEL_COUNT];
    memcpy(broken, rover_ik_provisional, sizeof(broken));
    broken[wheel].radius_m = 0;
    check_zero(run_ff(broken, &p, twist), 0);
  }
  struct rover_wheel_geometry parallel[ROVER_WHEEL_COUNT];
  memcpy(parallel, rover_ik_provisional, sizeof(parallel));
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    parallel[i].dx = 0;
    parallel[i].dy = 1;
    parallel[i].x_m = 0;
    parallel[i].y_m = 0;
  }
  check_zero(run_ff(parallel, &p, twist), 0);
  check_zero(run_ff(NULL, &p, twist), 0);
  check_zero(run_ff(rover_ik_provisional, NULL, twist), 0);
  CHECK_EQ(friction_ff_wheel_amps(rover_ik_provisional, &p, 0.1f, 0, 0, 0, NULL), 0);
  for (unsigned field = 0; field < 3; ++field) { /* non-finite breakaway levels are refused too */
    struct friction_ff_params broken = p;
    float *f = field == 0 ? &broken.breakaway_x_n : field == 1 ? &broken.breakaway_y_n : &broken.breakaway_w_nm;
    *f = NAN;
    check_zero(run_ff(rover_ik_provisional, &broken, twist), 0);
  }
}

/* Pure x uses only Cx, pure y only Cy, a diagonal gets (Cx*ux, Cy*uy); every wheel still pushes along its motion. */
static void test_coulomb_differs_by_axis(void) {
  struct friction_ff_params p = params();
  p.coulomb_x_n = 6;
  p.coulomb_y_n = 2;
  const float twists[][3] = {{0.5f, 0, 0}, {-0.5f, 0, 0}, {0, 0.5f, 0}, {0, -0.5f, 0}, {0.3f, 0.4f, 0}};
  for (unsigned t = 0; t < sizeof(twists) / sizeof(twists[0]); ++t) {
    const float *v = twists[t];
    struct result result = run_ff(rover_ik_provisional, &p, v);
    CHECK_EQ(result.ok, 1);
    double wrench[3];
    wrench_of(&p, result.amps, wrench);
    double speed = hypot(v[0], v[1]);
    CHECK_NEAR(wrench[0], 6.0 * v[0] / speed, 1e-3);
    CHECK_NEAR(wrench[1], 2.0 * v[1] / speed, 1e-3);
    CHECK_NEAR(wrench[2], 0, 1e-4);
  }
  struct friction_ff_params px = p, py = p; /* changing one axis level leaves pure motion along the other alone */
  px.coulomb_x_n = 9;
  py.coulomb_y_n = 9;
  struct result x0 = run_ff(rover_ik_provisional, &p, (float[3]){0.5f, 0, 0});
  struct result x1 = run_ff(rover_ik_provisional, &py, (float[3]){0.5f, 0, 0});
  struct result y0 = run_ff(rover_ik_provisional, &p, (float[3]){0, 0.5f, 0});
  struct result y1 = run_ff(rover_ik_provisional, &px, (float[3]){0, 0.5f, 0});
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) {
    CHECK_NEAR(x1.amps[i], x0.amps[i], 1e-5);
    CHECK_NEAR(y1.amps[i], y0.amps[i], 1e-5);
  }
}

/* Breakaway raises each Coulomb level to its breakaway level only while asked, and only where it is higher. */
static void test_breakaway_levels(void) {
  struct friction_ff_params p = params();
  p.coulomb_x_n = 3;
  p.coulomb_y_n = 2;
  p.coulomb_w_nm = 0.3f;
  p.viscous_xy_n_per_mps = 1;
  p.breakaway_x_n = 9;
  p.breakaway_y_n = 1; /* below coulomb_y: no effect */
  p.breakaway_w_nm = 0.9f;
  const float twist[3] = {0.3f, 0.4f, 1.0f};
  struct result rolling = run_ff_breakaway(rover_ik_provisional, &p, twist, 0);
  struct result stuck = run_ff_breakaway(rover_ik_provisional, &p, twist, 1);
  CHECK_EQ(rolling.ok, 1);
  CHECK_EQ(stuck.ok, 1);
  double wr[3], ws[3];
  wrench_of(&p, rolling.amps, wr);
  wrench_of(&p, stuck.amps, ws);
  CHECK_NEAR(wr[0], (3 + 0.5) * 0.6, 1e-3);
  CHECK_NEAR(ws[0], (9 + 0.5) * 0.6, 1e-3);
  CHECK_NEAR(ws[1], wr[1], 1e-3);
  CHECK_NEAR(wr[1], (2 + 0.5) * 0.8, 1e-3);
  CHECK_NEAR(wr[2], 0.3, 1e-4);
  CHECK_NEAR(ws[2], 0.9, 1e-4);
  p.breakaway_x_n = p.breakaway_y_n = p.breakaway_w_nm = 0; /* all zero: breakaway changes nothing */
  struct result off = run_ff_breakaway(rover_ik_provisional, &p, twist, 1);
  for (unsigned i = 0; i < ROVER_WHEEL_COUNT; ++i) CHECK_NEAR(off.amps[i], rolling.amps[i], 1e-5);
  check_zero(run_ff_breakaway(rover_ik_provisional, &p, (float[3]){0, 0, 0}, 1), 1); /* no motion, no current */
}

int main(void) {
  test_defaults_are_off();
  test_wheels_rebuild_the_cancelling_wrench();
  test_odd_in_the_twist();
  test_every_wheel_gets_current_in_the_motion_direction();
  test_per_wheel_scale_touches_only_that_wheel();
  test_coulomb_viscous_and_standstill();
  test_invalid_input_returns_zero_currents();
  test_coulomb_differs_by_axis();
  test_breakaway_levels();
  return 0;
}
