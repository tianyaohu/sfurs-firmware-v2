#include "hall_angle.h"

static int16_t wrap(int32_t angle)
{
  return (int16_t)(uint16_t)(uint32_t)angle;
}

void hall_angle_init(struct hall_angle *a, uint16_t edges_pos, uint16_t edges_neg)
{
  *a = (struct hall_angle){ .edges_pos = edges_pos, .edges_neg = edges_neg };
}

int16_t hall_angle_step(struct hall_angle *a, uint16_t edges_pos, uint16_t edges_neg, int16_t edge_angle,
                        int16_t speed_dpp, int16_t fallback)
{
  const int32_t moved = (int32_t)(uint16_t)(edges_pos - a->edges_pos) - (int32_t)(uint16_t)(edges_neg - a->edges_neg);
  a->edges_pos = edges_pos;
  a->edges_neg = edges_neg;
  if (moved != 0) {
    a->base = edge_angle;
    a->dir = moved > 0 ? 1 : -1;
    a->since = 0U;
  } else if (a->since < UINT16_MAX) {
    ++a->since; /* also when a forward and a backward edge cancel within one period */
  }
  if (a->dir == 0) return fallback;

  const int32_t centre = a->base + a->dir * (HALL_ANGLE_S60 / 2);
  const int32_t v = speed_dpp == HALL_ANGLE_PSEUDO_SPEED ? 0 : speed_dpp;
  /* Slow, or moving against the last edge (rocking): the speed says nothing about where in the sector it is. */
  if (v * a->dir < HALL_ANGLE_SLOW_DPP) return wrap(centre);
  const int32_t lead = v * (int32_t)a->since;
  if (lead * a->dir <= HALL_ANGLE_S60) return wrap(a->base + lead);
  /* The next edge is due: hold at that boundary while it may still come on time (sector widths vary by about
   * 6 %), then the rotor is late, i.e. slowing or stuck, and somewhere inside the sector. */
  const int32_t period = HALL_ANGLE_S60 / (v * a->dir);
  if ((int32_t)a->since <= period + period / 4) return wrap(a->base + a->dir * HALL_ANGLE_S60);
  return wrap(centre);
}
