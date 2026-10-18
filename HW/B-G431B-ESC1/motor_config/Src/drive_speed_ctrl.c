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

/* The hand-off takes the first raw edge along the command: coasting the other
 * way cannot fake it and a rock into it is handed off capped with a frozen
 * integral. Progress (restick) and captures use the qualified counters. */
static uint16_t raw_along(const dsc_in_t *in, int8_t dir)
{
  return dir > 0 ? in->raw_pos : in->raw_neg;
}

static void enter(dsc_state_t *s, uint8_t state, uint8_t reason, const dsc_in_t *in)
{
  s->state = state;
  s->reason = reason;
  s->t = 0U;
  s->at_cap = 0U;
  s->edges0 = raw_along(in, s->dir);
  s->edges_last = edges_along(in, s->dir);
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

void dsc_rev_rebase(dsc_rev_t *f, uint16_t edges_pos, uint16_t edges_neg, uint16_t captures)
{
  f->pos = edges_pos;
  f->neg = edges_neg;
  f->caps = captures;
  f->period = 0U; /* edge timing restarts too */
  f->since = 0U;
  f->net = 0;
  f->hi = 0;
  f->lo = 0;
}

int16_t dsc_reversal_filter(dsc_rev_t *f, uint16_t edges_pos, uint16_t edges_neg,
                            uint16_t captures, int16_t rpm)
{
  const uint16_t dp = (uint16_t)(edges_pos - f->pos), dn = (uint16_t)(edges_neg - f->neg);
  const uint16_t dc = (uint16_t)(captures - f->caps);
  f->pos = edges_pos;
  f->neg = edges_neg;
  f->caps = captures;
  f->net += (int32_t)dp - (int32_t)dn;
  if (f->net > (1L << 30) || f->net < -(1L << 30)) { /* keep the band, move the origin */
    f->hi -= f->net;
    f->lo -= f->net;
    f->net = 0;
  }
  if (f->net > f->hi) {
    f->qpos = (uint16_t)(f->qpos + (uint32_t)(f->net - f->hi));
    f->hi = f->net;
    f->lo = f->net - 1;
  } else if (f->net < f->lo) {
    f->qneg = (uint16_t)(f->qneg + (uint32_t)(f->lo - f->net));
    f->lo = f->net;
    f->hi = f->net + 1;
  }
  if (dp != 0U || dn != 0U) { /* edge timing for dsc_edge_rpm */
    const uint32_t n = (uint32_t)dp + dn;
    f->period = (uint16_t)(f->since >= n ? (f->since + 1U) / n : 1U);
    f->since = 0U;
  } else if (f->since < UINT16_MAX) {
    ++f->since;
  }
  if (dp != 0U && dn != 0U) { /* back and forth within one tick */
    f->dir = 0;
    f->run = 0U;
  } else if (dp != 0U || dn != 0U) {
    const int8_t d = dp != 0U ? 1 : -1;
    const uint32_t n = dp != 0U ? dp : dn;
    const uint32_t run = (d == f->dir ? f->run : 0U) + n;
    f->dir = d;
    f->run = (uint8_t)(run < 255U ? run : 255U);
  }
  if (f->run < 2U || (rpm > 0 && f->dir < 0) || (rpm < 0 && f->dir > 0)) return 0;
  f->qcaps = (uint16_t)(f->qcaps + dc);
  return rpm;
}

int16_t dsc_edge_rpm(const dsc_rev_t *f)
{
  if (f->run < 2U || f->period == 0U) return 0;
  const uint32_t ms = f->since + 1U > f->period ? f->since + 1U : f->period;
  const int32_t rpm = (int32_t)((uint32_t)DSC_EDGE_PERIOD_MS_RPM / ms);
  return (int16_t)(f->dir * rpm);
}

void dsc_reset(dsc_state_t *s)
{
  const int16_t pos = s->learned[0], neg = s->learned[1];
  *s = (dsc_state_t){0};
  s->learned[0] = pos;
  s->learned[1] = neg;
}

/* STARTING entry: the ramp begins at start_from, or at preload_pct of the
 * breakaway learned in this direction when that is higher. */
static void enter_starting(const dsc_cfg_t *cfg, dsc_state_t *s, uint8_t reason, const dsc_in_t *in,
                           int32_t from)
{
  enter(s, DSC_STARTING, reason, in);
  const int32_t pre = (int32_t)s->learned[s->dir > 0 ? 0 : 1] * cfg->preload_pct / 100;
  const int32_t lo = from > cfg->start_from ? from : cfg->start_from;
  s->ramp = (int16_t)clamp(pre > lo ? pre : lo, 0, cfg->start_cap > 0 ? cfg->start_cap : 0);
  s->peak = s->along;
  s->watch_t = 0U;
  /* a preload is a current step: let the wind-up it causes settle first */
  s->watching = cfg->watch_ms != 0U && s->ramp > lo ? DSC_W_SETTLE : DSC_W_NONE;
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
    s->nocap = 0U;
  }

  if (cmd == 0) {
    s->resticks = 0U;
    const uint8_t reason = in->active ? DSC_R_STOP : DSC_R_INACTIVE;
    if (s->state != DSC_STOPPED || s->dir != 0 || s->reason != reason) {
      if (s->dir != 0) pid->wIntegralTerm = 0; /* no held current on a zero command */
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
    s->rp = in->raw_pos;
    s->rn = in->raw_neg;
    s->along = 0;
    s->best = 0;
    s->anchored = 0U;
    s->progressed = 0U;
    s->resticks = 0U;
    s->nocap = 0U;
  }
  {
    const int32_t d = (int32_t)(uint16_t)(in->raw_pos - s->rp) - (int32_t)(uint16_t)(in->raw_neg - s->rn);
    s->rp = in->raw_pos;
    s->rn = in->raw_neg;
    s->along += s->dir * d;
    if (s->anchored && s->along > s->best) {
      s->best = s->along;
      s->progressed = 1U;
    }
    if (s->along > (1L << 30) || s->along < -(1L << 30)) { /* move the origin, keep the gap */
      const int32_t gap = s->anchored ? s->best - s->along : 0;
      s->best = gap < -(1L << 30) ? -(1L << 30) : gap; /* bounded: never overflows */
      s->along = 0;
    }
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
      enter_starting(cfg, s, s->reason, in, 0);
    }
  }

  const int32_t aref = ref < 0 ? -ref : ref;
  const uint32_t edge_ms = (uint32_t)DSC_EDGE_PERIOD_MS_RPM / (uint32_t)aref;
  if (s->state == DSC_RUNNING) {
    const uint16_t e = edges_along(in, s->dir);
    if (e != s->edges_last) { /* qualified progress */
      s->edges_last = e;
      s->since_edge = 0U;
      s->resticks = 0U;
    } else {
      s->since_edge = sat_inc(s->since_edge);
    }
    if (s->since_edge > (2U * edge_ms > DSC_RESTICK_MIN_MS ? 2U * edge_ms : DSC_RESTICK_MIN_MS)) {
      if (s->progressed) s->resticks = 0U; /* stick-slip that gained ground */
      s->progressed = 0U;
      if (++s->resticks >= DSC_MAX_RESTICKS) /* rocking or stuck at any speed */
        return fail(s, pid, DSC_R_NO_CAPTURE, in);
      /* resume the ramp at the last hand-off current: below it the wheel just stuck */
      enter_starting(cfg, s, DSC_R_RESTICK, in, s->start_iq < 0 ? -(int32_t)s->start_iq : s->start_iq);
      s->capped = 0U;
    }
  }

  const int32_t ff = s->dir * ((int32_t)cfg->ff_run + (int32_t)cfg->ff_per_krpm * aref / 1000);
  if (s->state == DSC_STARTING) {
    int moved = (uint16_t)(raw_along(in, s->dir) - s->edges0) != 0U;
    if (cfg->watch_ms != 0U) { /* moved = a second new furthest position under a frozen ramp */
      const int further = s->along > s->peak;
      if (further) s->peak = s->along;
      moved = further && s->watching == DSC_W_CONFIRM && s->watch_t >= DSC_WATCH_MIN_MS;
      if (further && s->watching == DSC_W_NONE) {
        s->watching = DSC_W_CONFIRM;
        s->watch_t = 0U;
      } else if (s->watching != DSC_W_NONE && !moved) {
        s->watch_t = sat_inc(s->watch_t);
        if (s->watch_t >= (s->watching == DSC_W_SETTLE ? DSC_SETTLE_MS : cfg->watch_ms))
          s->watching = DSC_W_NONE; /* wind-up: it stopped, keep ramping */
      }
    }
    if (!moved) {
      const int32_t cap = clamp(cfg->start_cap, 0, limit);
      if (s->ramp >= cap) {
        s->ramp = (int16_t)cap;
        s->at_cap = sat_inc(s->at_cap);
        if (s->at_cap > cfg->start_hold_ms) return fail(s, pid, DSC_R_NO_EDGE, in);
      }
      const int16_t out = (int16_t)(s->dir * s->ramp);
      if (!s->watching) s->ramp = (int16_t)clamp((int32_t)s->ramp + cfg->start_slope, 0, cap);
      s->p = 0;
      s->i = (int16_t)(pid->wIntegralTerm >> pid->hKiDivisorPOW2);
      s->ff = out;
      s->out = out;
      return out;
    }
    /* Hand-off: the integral holds the breakaway current above ff_run (never
     * below 0, so no negative settle), decaying by hold_decay per tick while
     * capped. The first edge on the ground is often tyre and gear wind-up, not
     * breakaway: dropping to ff_run there re-sticks the wheel (mat 2026-10-17).
     * No integration this tick; P capped until fresh Hall speed arrives. */
    s->start_iq = s->out;
    if (cfg->watch_ms != 0U) /* confirmed: the next start of this direction preloads from it */
      s->learned[s->dir > 0 ? 0 : 1] = (int16_t)(s->out < 0 ? -s->out : s->out);
    {
      const int32_t hold = s->dir * ((int32_t)s->out - ff) > 0 ? (int32_t)s->out - ff : 0;
      pid->wIntegralTerm = clamp(hold * (1L << pid->hKiDivisorPOW2), pid->wLowerIntegralLimit,
                                 pid->wUpperIntegralLimit);
    }
    if (!s->anchored) { /* progress is measured from the first hand-off of the command */
      s->anchored = 1U;
      s->best = s->along;
    }
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
    if (aref >= DSC_CAP_MIN_RPM) s->nocap = sat_inc(s->nocap);
    if (s->t >= cfg->pcap_ms && fresh >= cfg->pcap_updates) {
      s->capped = 0U;
      s->resticks = 0U;
    } else if (s->nocap >= cfg->pcap_fallback_ms && s->nocap >= slow) {
      return fail(s, pid, DSC_R_NO_CAPTURE, in);
    } else { /* hand-off hold decays toward 0 */
      const int32_t d = (int32_t)cfg->hold_decay * (1L << pid->hKiDivisorPOW2);
      const int32_t w = pid->wIntegralTerm;
      pid->wIntegralTerm = w > d ? w - d : w < -d ? w + d : 0;
    }
  }
  return pi_ff(s, pid, err, ff, limit, !s->capped, s->capped ? cfg->pcap : -1);
}
