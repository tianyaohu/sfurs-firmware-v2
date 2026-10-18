#include "drive_speed_ctrl.h"

static int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
  return v < lo ? lo : v > hi ? hi : v;
}

static uint16_t sat_inc(uint16_t v)
{
  return v < UINT16_MAX ? (uint16_t)(v + 1U) : v;
}

static uint16_t edges_along(const dsc_in_t *in, int8_t dir)
{
  return dir > 0 ? in->edges_pos : in->edges_neg;
}

static void enter(dsc_state_t *s, uint8_t state, uint8_t reason, const dsc_in_t *in)
{
  s->state = state;
  s->reason = reason;
  s->t = 0U;
  s->at_cap = 0U;
  s->edges0 = edges_along(in, s->dir);
  s->edges_last = s->edges0;
  s->since_edge = 0U;
  s->caps0 = in->captures;
}

static int16_t fail(dsc_state_t *s, PID_Handle_t *pid, uint8_t reason, const dsc_in_t *in)
{
  enter(s, DSC_FAILED, reason, in);
  pid->wIntegralTerm = 0;
  s->p = s->i = s->ff = s->out = 0;
  return 0;
}

void dsc_reset(dsc_state_t *s)
{
  *s = (dsc_state_t){0};
}

/* P + I + ff against the active limit. Conditional integration: the integral
 * only moves when the combined command is inside the limit or the error pulls
 * it back in, so it never winds against saturation caused by the ff. */
static int16_t pi_ff(dsc_state_t *s, PID_Handle_t *pid, int32_t err, int32_t ff,
                     int32_t limit, int integrate, int32_t pcap)
{
  int32_t p = (pid->hKpGain * err) >> pid->hKpDivisorPOW2;
  if (pcap >= 0 && s->dir != 0 && s->dir * p > pcap) p = s->dir * pcap;
  const int32_t i = pid->wIntegralTerm >> pid->hKiDivisorPOW2;
  const int32_t raw = p + i + ff;
  const int32_t out = clamp(raw, -limit, limit);
  if (integrate && pid->hKiGain != 0) {
    const int32_t inc = pid->hKiGain * err;
    if (raw == out || (raw > out && inc < 0) || (raw < out && inc > 0))
      pid->wIntegralTerm = clamp(pid->wIntegralTerm + inc, pid->wLowerIntegralLimit,
                                 pid->wUpperIntegralLimit);
  }
  s->p = (int16_t)clamp(p, INT16_MIN, INT16_MAX);
  s->i = (int16_t)i;
  s->ff = (int16_t)ff;
  s->out = (int16_t)out;
  return (int16_t)out;
}

int16_t dsc_step(const dsc_cfg_t *cfg, dsc_state_t *s, const dsc_in_t *in, PID_Handle_t *pid)
{
  const int32_t limit = in->limit > 0 ? in->limit : 0;
  const int32_t ref = in->ref_rpm, err = ref - in->meas_rpm;
  const int8_t cmd = !in->active ? 0 : ref >= DSC_STOP_RPM ? 1 : ref <= -DSC_STOP_RPM ? -1 : 0;
  s->t = sat_inc(s->t);
  if (in->captures != s->caps_seen) { /* any fresh Hall capture is feedback */
    s->caps_seen = in->captures;
    s->resticks = 0U;
  }

  if (cmd == 0) {
    s->resticks = 0U;
    const uint8_t reason = in->active ? DSC_R_STOP : DSC_R_INACTIVE;
    if (s->state != DSC_STOPPED || s->dir != 0 || s->reason != reason) {
      s->dir = 0;
      enter(s, DSC_STOPPED, reason, in);
    }
    return pi_ff(s, pid, err, 0, limit, 1, -1); /* hold / park: plain PI */
  }
  if (s->state == DSC_FAILED) { /* latched until the command returns to stop */
    pid->wIntegralTerm = 0;
    s->p = s->i = s->ff = s->out = 0;
    return 0;
  }
  if (cmd != s->dir) { /* new start or reversal */
    const uint8_t reason = s->dir == 0 ? DSC_R_ARMED : DSC_R_REVERSE;
    s->dir = cmd;
    enter(s, DSC_STOPPED, reason, in);
  }

  const int32_t along = s->dir * (int32_t)in->meas_rpm;
  if (s->state == DSC_STOPPED) {
    if (along <= -DSC_REVERSE_RPM) /* still moving the old way: brake on plain PI */
      return pi_ff(s, pid, err, 0, limit, 1, -1);
    if (along >= DSC_REVERSE_RPM) { /* already moving along the command */
      pid->wIntegralTerm = 0;
      enter(s, DSC_RUNNING, DSC_R_MOVING, in);
      s->capped = 0U;
    } else {
      enter(s, DSC_STARTING, s->reason, in);
      s->ramp = (int16_t)clamp(cfg->start_from, 0, INT16_MAX);
    }
  }

  const int32_t aref = ref < 0 ? -ref : ref;
  const uint32_t edge_ms = (uint32_t)DSC_EDGE_PERIOD_MS_RPM / (uint32_t)aref;
  if (s->state == DSC_RUNNING) {
    const uint16_t e = edges_along(in, s->dir);
    if (e != s->edges_last) {
      s->edges_last = e;
      s->since_edge = 0U;
    } else {
      s->since_edge = sat_inc(s->since_edge);
    }
    if (s->since_edge > (2U * edge_ms > DSC_RESTICK_MIN_MS ? 2U * edge_ms : DSC_RESTICK_MIN_MS)) {
      if (++s->resticks >= DSC_MAX_RESTICKS && aref >= DSC_CAP_MIN_RPM)
        return fail(s, pid, DSC_R_NO_CAPTURE, in);
      enter(s, DSC_STARTING, DSC_R_RESTICK, in);
      s->ramp = (int16_t)clamp(cfg->start_from, 0, INT16_MAX);
      s->capped = 0U;
    }
  }

  const int32_t ff = s->dir * ((int32_t)cfg->ff_run + (int32_t)cfg->ff_per_krpm * aref / 1000);
  if (s->state == DSC_STARTING) {
    if ((uint16_t)(edges_along(in, s->dir) - s->edges0) == 0U) {
      const int32_t cap = clamp(cfg->start_cap, 0, limit);
      if (s->ramp >= cap) {
        s->ramp = (int16_t)cap;
        s->at_cap = sat_inc(s->at_cap);
        if (s->at_cap > cfg->start_hold_ms) return fail(s, pid, DSC_R_NO_EDGE, in);
      }
      const int16_t out = (int16_t)(s->dir * s->ramp);
      s->ramp = (int16_t)clamp((int32_t)s->ramp + cfg->start_slope, 0, cap);
      s->p = 0;
      s->i = (int16_t)(pid->wIntegralTerm >> pid->hKiDivisorPOW2);
      s->ff = out;
      s->out = out;
      return out;
    }
    /* Hand-off: integral to its steady-state value, ff_run carries friction.
     * No integration this tick; P capped until fresh Hall speed arrives. */
    s->start_iq = s->out;
    pid->wIntegralTerm = 0;
    enter(s, DSC_RUNNING, DSC_R_EDGE, in);
    s->capped = 1U; /* until fresh Hall speed: P capped, integral frozen */
    return pi_ff(s, pid, err, ff, limit, 0, cfg->pcap);
  }

  /* RUNNING. The cap fallback is at least long enough for pcap_updates + 1
   * edges at the commanded speed (the SDK discards the first capture), and
   * only counts while |ref| is fast enough for captures to exist. */
  if (s->capped) {
    const uint16_t fresh = (uint16_t)(in->captures - s->caps0);
    const uint32_t slow = 2U * ((uint32_t)cfg->pcap_updates + 1U) * edge_ms;
    if (aref >= DSC_CAP_MIN_RPM) s->at_cap = sat_inc(s->at_cap);
    if (s->t >= cfg->pcap_ms && fresh >= cfg->pcap_updates) {
      s->capped = 0U;
      s->resticks = 0U;
    } else if (s->at_cap >= cfg->pcap_fallback_ms && s->at_cap >= slow) {
      return fail(s, pid, DSC_R_NO_CAPTURE, in);
    }
  }
  return pi_ff(s, pid, err, ff, limit, !s->capped, s->capped ? cfg->pcap : -1);
}
