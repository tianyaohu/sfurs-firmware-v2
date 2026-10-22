#include "check.h"
#include "friction_fit.h"
#include <limits.h>

static friction_point_t point(double rpm, double ma)
{
  friction_sample_t s[10];
  for (int i = 0; i < 10; i++) s[i] = (friction_sample_t){rpm, ma};
  return friction_segment(s, 10);
}

static void exact_and_signs(void)
{
  friction_point_t p[8];
  for (int wheel = 0; wheel < 4; wheel++) {
    double ic = 100 + 50 * wheel, b = 2 + wheel;
    for (int i = 0; i < 8; i++) {
      double v = (i < 4 ? -1 : 1) * (20 + 30 * (i % 4));
      p[i] = point(v, ic * (v > 0 ? 1 : -1) + b * v);
    }
    for (int direction = -1; direction <= 1; direction++) {
      friction_fit_t fit = friction_fit(p, 8, direction);
      CHECK(fit.valid); CHECK_EQ(fit.points, direction ? 4 : 8);
      CHECK_NEAR(fit.ic_ma, ic, 1e-9); CHECK_NEAR(fit.b_ma_per_rpm, b, 1e-9);
      CHECK_NEAR(fit.residual_rms_ma, 0, 1e-9);
    }
    /* Signed current is never converted to its absolute magnitude. */
    for (int i = 0; i < 8; i++) p[i].iq_ma *= -1;
    friction_fit_t fit = friction_fit(p, 8, 0);
    CHECK(fit.valid); CHECK_NEAR(fit.ic_ma, -ic, 1e-9); CHECK_NEAR(fit.b_ma_per_rpm, -b, 1e-9);
  }
}

static void residual_and_singular(void)
{
  friction_point_t p[] = {point(10, 121), point(20, 138), point(30, 161)};
  friction_fit_t fit = friction_fit(p, 3, 0);
  CHECK(fit.valid); CHECK_NEAR(fit.ic_ma, 100, 1e-9); CHECK_NEAR(fit.b_ma_per_rpm, 2, 1e-9);
  CHECK_NEAR(fit.residual_rms_ma, sqrt(2), 1e-9);
  p[1].verdict = FRICTION_UNSTEADY;
  CHECK_EQ(friction_fit(p, 3, 0).points, 2);
  CHECK(!friction_fit(p, 1, 0).valid);
  p[1] = point(-10, -120);
  CHECK(!friction_fit(p, 2, 0).valid); /* opposite signs at one magnitude cannot identify slope */
  CHECK(!friction_fit(NULL, 0, 0).valid);
  CHECK(!friction_fit(p, 2, 2).valid);
}

static void segment_rejection(void)
{
  friction_sample_t samples[5] = {{90, 200}, {110, 210}, {90, 220}, {110, 230}, {100, 240}};
  friction_point_t p = friction_segment(samples, 5);
  CHECK_EQ(p.verdict, FRICTION_STEADY); CHECK_EQ(p.samples, 5);
  CHECK_NEAR(p.rpm, 100, 1e-9); CHECK_NEAR(p.iq_ma, 220, 1e-9);
  CHECK_NEAR(p.speed_sd, sqrt(80), 1e-9);
  samples[0].rpm = 60;
  CHECK_EQ(friction_segment(samples, 5).verdict, FRICTION_UNSTEADY);
  CHECK_EQ(friction_segment(samples, 4).verdict, FRICTION_TOO_FEW);
  CHECK_EQ(friction_segment(NULL, 0).verdict, FRICTION_TOO_FEW);
  for (int i = 0; i < 5; i++) samples[i].rpm = 0;
  CHECK_EQ(friction_segment(samples, 5).verdict, FRICTION_NO_MOTION);
  samples[0].iq_ma = NAN;
  CHECK_EQ(friction_segment(samples, 5).verdict, FRICTION_BAD_SAMPLE);
  samples[0].iq_ma = 0; samples[0].rpm = INFINITY;
  CHECK_EQ(friction_segment(samples, 5).verdict, FRICTION_BAD_SAMPLE);
  for (int i = 0; i < 5; i++) samples[i] = (friction_sample_t){-100, -400};
  CHECK_EQ(friction_segment(samples, 5).verdict, FRICTION_STEADY);
  CHECK_EQ(friction_segment(samples, 5).rpm, -100);
  samples[0].rpm = 100;
  CHECK_EQ(friction_segment(samples, 5).verdict, FRICTION_UNSTEADY);
}

static void fresh_consecutive_motion(void)
{
  for (int sign = -1; sign <= 1; sign += 2) {
    friction_motion_t s = {UINT_MAX - 1, 0};
    CHECK(!friction_motion(&s, UINT_MAX, sign * 20, sign, 20));
    CHECK(!friction_motion(&s, 0, sign * 20, sign, 20));
    CHECK(!friction_motion(&s, 0, sign * 20, sign, 20)); /* duplicate */
    CHECK(friction_motion(&s, 1, sign * 20, sign, 20)); /* wrap */
    CHECK(!friction_motion(&s, 2, -sign * 20, sign, 20));
    CHECK_EQ(s.streak, 0);
    CHECK(!friction_motion(&s, 3, sign * 20, sign, 20));
    CHECK(!friction_motion(&s, 5, sign * 20, sign, 20)); /* gap resets */
    CHECK(!friction_motion(&s, 6, sign * 20, sign, 20));
    CHECK(friction_motion(&s, 7, sign * 20, sign, 20));
    CHECK(!friction_motion(&s, 8, NAN, sign, 20));
  }
}

int main(void)
{
  exact_and_signs(); residual_and_singular(); segment_rejection(); fresh_consecutive_motion();
  puts("friction_fit: 4 test cases passed");
  return 0;
}
