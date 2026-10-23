/* Rotor angle (hall_angle.c). Inputs are the Hall edge counters, the edge angle MCSDK's edge interrupt stores and
 * MCSDK's speed, as the HALL_CalcElAngle override passes them; no motor model. Each case asserts one decision of
 * hall_angle.h with exact s16 values. */
#include "hall_angle.h"
#include "check.h"

#include <string.h>

#define HALF (HALL_ANGLE_S60 / 2)

static struct hall_angle a;
static uint16_t pos, neg;

static int16_t step(int16_t edge_angle, int16_t speed) { return hall_angle_step(&a, pos, neg, edge_angle, speed, 777); }
static void setup(void) { pos = 100; neg = 200; hall_angle_init(&a, pos, neg); }

/* No edge since init: MCSDK's start angle (its sector centre) is kept, whatever the speed says. */
static void before_edge(void)
{
  setup();
  for (int n = 0; n < 1000; ++n) CHECK_EQ(step(1234, 500), 777);
}

/* Slower than one edge per 20 ms: the centre of the sector the last edge entered, both directions. */
static void slow_is_centre(void)
{
  setup();
  ++pos;
  for (int n = 0; n < 5000; ++n) CHECK_EQ(step(1000, HALL_ANGLE_SLOW_DPP - 1), 1000 + HALF);
  ++neg;
  for (int n = 0; n < 5000; ++n) CHECK_EQ(step(-3000, -(HALL_ANGLE_SLOW_DPP - 1)), -3000 - HALF);
  ++pos;
  CHECK_EQ(step(4000, 0), 4000 + HALF); /* standing still */
}

/* At speed: extrapolate from the edge, hold at the next boundary for a quarter period more, then the centre. */
static void extrapolate_hold_centre(void)
{
  for (int dir = -1; dir <= 1; dir += 2) {
    setup();
    if (dir > 0) ++pos; else ++neg;
    const int16_t v = (int16_t)(200 * dir);
    const int period = HALL_ANGLE_S60 / 200; /* 54 PWM periods per sector */
    for (int k = 0; k <= period; ++k) CHECK_EQ(step(0, v), 200 * dir * k);
    for (int k = period + 1; k <= period + period / 4; ++k) CHECK_EQ(step(0, v), dir * HALL_ANGLE_S60);
    for (int k = 0; k < 3000; ++k) CHECK_EQ(step(0, v), dir * HALF); /* late: stuck or slowing */
  }
}

/* A wheel that creeps then sticks: MCSDK held the angle up to 60 deg ahead for 150 ms (2400 periods); here the
 * centre comes after 1.25 sector periods at the creep speed. */
static void stuck_after_creep(void)
{
  setup();
  ++pos;
  const int16_t v = HALL_ANGLE_SLOW_DPP + 6; /* ~73 RPM */
  const int period = HALL_ANGLE_S60 / v;
  int n = 0;
  while (step(500, v) != 500 + HALF) ++n;
  CHECK_EQ(n, period + period / 4 + 1);
  CHECK(n < 2400 / 4);
}

/* Speed against the last edge (rocking back) or unknown: the centre. */
static void against_or_unknown(void)
{
  setup();
  ++pos;
  CHECK_EQ(step(2000, -300), 2000 + HALF);
  CHECK_EQ(step(2000, HALL_ANGLE_PSEUDO_SPEED), 2000 + HALF);
}

/* s16 angles wrap like MCSDK's. */
static void angle_wrap(void)
{
  setup();
  ++pos;
  for (int k = 0; k < 20; ++k) (void)step(30000, 200);
  CHECK_EQ(step(30000, 200), (int16_t)(uint16_t)(30000 + 200 * 20));
  setup();
  ++neg;
  CHECK_EQ(step(-32000, 0), (int16_t)(uint16_t)(-32000 - HALF + 65536));
}

/* Counter wrap counts as one edge; a new edge rebases; a forward and a backward edge in one period cancel. */
static void edges(void)
{
  pos = 65535; neg = 7; hall_angle_init(&a, pos, neg);
  ++pos; /* wraps to 0 */
  CHECK_EQ(step(100, 0), 100 + HALF);
  for (int k = 0; k < 500; ++k) (void)step(100, 300);
  ++pos;
  CHECK_EQ(step(9000, 300), 9000);
  CHECK_EQ(step(9000, 300), 9300);
  ++pos; ++neg;
  CHECK_EQ(step(-5000, 300), 9600); /* base and direction unchanged */
}

/* Four wheels, four independent states. */
static void four_instances(void)
{
  struct hall_angle w[4];
  for (int i = 0; i < 4; ++i) hall_angle_init(&w[i], 0, 0);
  for (int i = 0; i < 4; ++i)
    CHECK_EQ(hall_angle_step(&w[i], (uint16_t)(i + 1), 0, (int16_t)(1000 * i), 0, 0), 1000 * i + HALF);
  for (int i = 0; i < 4; ++i)
    CHECK_EQ(hall_angle_step(&w[i], (uint16_t)(i + 1), 0, 0, 0, 0), 1000 * i + HALF); /* base kept per wheel */
}

struct test_case {
  const char *name;
  void (*run)(void);
};

int main(int argc, char **argv)
{
  const struct test_case cases[] = {
    { "before_edge", before_edge }, { "slow_is_centre", slow_is_centre },
    { "extrapolate_hold_centre", extrapolate_hold_centre }, { "stuck_after_creep", stuck_after_creep },
    { "against_or_unknown", against_or_unknown }, { "angle_wrap", angle_wrap }, { "edges", edges },
    { "four_instances", four_instances },
  };
  unsigned ran = 0;
  for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
    if (argc > 1 && strcmp(argv[1], cases[i].name) != 0) continue;
    cases[i].run();
    printf("%s: PASS\n", cases[i].name);
    ++ran;
  }
  if (!ran) { fprintf(stderr, "unknown case\n"); return 2; }
  return 0;
}
