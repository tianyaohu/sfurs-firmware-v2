/** @file motor_speed.c
 *  @brief Stage 4a speed loop; see motor_speed.h and technical-development-plans/esc-command-v2/E-speed-loop.md. */
#include "motor_speed.h"

#include <string.h>

#define Q14 16384
#define SLIP_UNIT MOTOR_SPEED_EDGE_RPM_MS /* slip in 1/1250 edge: |r| RPM travels |r|/1250 edges per ms */
#define SLIP_MAX (MOTOR_SPEED_SLIP_MAX_EDGES * SLIP_UNIT)
#define BACK_MAX (1L << 20) /* far beyond anything rocking produces; only keeps the sum bounded */

_Static_assert(MOTOR_SPEED_BLEED_RING > MOTOR_SPEED_BLEED_SPAN_MS,
               "records on distinct ticks: span >= 8 ms needs at most 9 of them");

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
  return v < lo ? lo : v > hi ? hi : v;
}

static int8_t sign32(int32_t v)
{
  return (int8_t)((v > 0) - (v < 0));
}

static int32_t abs32(int32_t v)
{
  return v < 0 ? -v : v;
}

static uint16_t sat_inc16(uint16_t v)
{
  return v < UINT16_MAX ? (uint16_t)(v + 1U) : v;
}

/* 10 A/s -> 89129: rounded once here so the 1 kHz path stays a single add. */
static int32_t stall_step_q14(uint16_t ma_per_s)
{
  return (int32_t)(((int64_t)ma_per_s * MOTOR_SPEED_COUNTS_PER_A * Q14 + 500000) / 1000000);
}

/* Round half away from zero, so +x and -x mA give mirrored counts. */
static int32_t ma_to_counts(int32_t ma)
{
  return (ma * MOTOR_SPEED_COUNTS_PER_A + (ma >= 0 ? 500 : -500)) / 1000;
}

void motor_speed_init(struct motor_speed *s, const struct motor_speed_config *cfg, uint16_t edges_pos,
                      uint16_t edges_neg)
{
  memset(s, 0, sizeof *s);
  s->cfg = *cfg;
  s->stall_step_q14 = stall_step_q14(cfg->stall_ramp_ma_per_s);
  s->mode = MOTOR_SPEED_OFF;
  s->prev_pos = edges_pos;
  s->prev_neg = edges_neg;
}

void motor_speed_configure(struct motor_speed *s, const struct motor_speed_config *cfg)
{
  if (cfg->kp != s->cfg.kp || cfg->ki != s->cfg.ki) s->i_q14 = 0;
  s->cfg = *cfg;
  s->stall_step_q14 = stall_step_q14(cfg->stall_ramp_ma_per_s);
}

void motor_speed_set(struct motor_speed *s, int16_t setpoint_rpm, int16_t ff_ma)
{
  s->ref_from_rpm = s->ref_rpm; /* a setpoint mid-ramp restarts from where r is, no jump */
  s->ref_to_rpm = setpoint_rpm;
  s->ref_k = 0U;
  s->ff_target_ma = ff_ma;
}

static void run_reset(struct motor_speed *s)
{
  s->run_edges = 0U;
  s->edge_dir = 0;
  s->run_span_ms = 0U;
}

/* Everything that belongs to one push along one direction of r. */
static void loop_clear(struct motor_speed *s)
{
  s->i_q14 = 0;
  s->ff_ma = 0;
  s->slip = 0;
  s->back_edges = 0;
  s->ring_len = 0U;
  s->stall_count_ms = 0U;
}

static uint16_t stale_window_ms(const struct motor_speed *s)
{
  return (uint16_t)clamp32(2 * (int32_t)s->period_ms, MOTOR_SPEED_STALE_MIN_MS, MOTOR_SPEED_STALE_MAX_MS);
}

/* The only speed estimate. Before the MCSDK buffer holds edges of this run its average mixes in the
 * previous direction or the stop, so the first edges are timed here at 1 ms resolution. */
static void estimate(struct motor_speed *s, const struct motor_speed_inputs *in, uint16_t dp, uint16_t dn,
                     struct motor_speed_outputs *out)
{
  if (dp != 0U || dn != 0U) {
    const uint32_t n = (uint32_t)dp + dn;
    const uint32_t dt = (uint32_t)s->since_edge_ms + 1U;
    if (s->seen_edge) s->period_ms = (uint16_t)(dt >= n ? dt / n : 1U);
    s->seen_edge = 1U;
    if (dp != 0U && dn != 0U) { /* back and forth inside one tick: no direction */
      run_reset(s);
    } else {
      const int8_t dir = dp != 0U ? 1 : -1;
      if (s->run_edges != 0U && dir == s->edge_dir) {
        const uint32_t span = s->run_span_ms + dt;
        s->run_span_ms = (uint16_t)(span < UINT16_MAX ? span : UINT16_MAX);
        const uint32_t edges = s->run_edges + n;
        s->run_edges = (uint8_t)(edges < UINT8_MAX ? edges : UINT8_MAX);
      } else {
        s->edge_dir = dir;
        s->run_edges = (uint8_t)(n < UINT8_MAX ? n : UINT8_MAX);
        s->run_span_ms = 0U;
      }
    }
    s->since_edge_ms = 0U;
  } else {
    s->since_edge_ms = sat_inc16(s->since_edge_ms);
  }

  const uint16_t window = stale_window_ms(s);
  s->hall_bad_ms = in->hall_code_invalid ? sat_inc16(s->hall_bad_ms) : 0U;
  out->hall_lost = s->hall_bad_ms > window;
  if (s->since_edge_ms > window) run_reset(s); /* the old period says nothing about the next edge */

  int32_t w = 0;
  int valid = s->run_edges >= MOTOR_SPEED_VALID_EDGES;
  if (valid && sign32(in->mcsdk_avg_rpm) == -s->edge_dir) valid = 0; /* MCSDK still averages the other way */
  if (valid) {
    if (s->run_edges >= MOTOR_SPEED_MCSDK_EDGES && in->mcsdk_buffer_full) {
      w = in->mcsdk_avg_rpm;
    } else {
      const int32_t span = s->run_span_ms != 0U ? s->run_span_ms : 1;
      w = s->edge_dir * (MOTOR_SPEED_EDGE_RPM_MS * (int32_t)(s->run_edges - 1U) / span);
    }
    if (s->since_edge_ms != 0U) { /* no edge for t ms: the wheel cannot be faster than one edge per t */
      const int32_t bound = MOTOR_SPEED_EDGE_RPM_MS / s->since_edge_ms;
      w = clamp32(w, -bound, bound);
    }
  }
  out->valid = (uint8_t)valid;
  out->w_hat_rpm = (int16_t)w;
  out->standstill = abs32(w) < MOTOR_SPEED_STANDSTILL_RPM;
}

/* Progress = a new furthest position along r; rocking back and forth never adds any. Returns the
 * progress edges of this tick and keeps the run of consecutive progress ticks for the bleed. */
static uint32_t progress(struct motor_speed *s, uint16_t dp, uint16_t dn)
{
  const int8_t dir = s->ref_sign;
  const uint32_t back_before = (uint32_t)s->back_edges;
  uint32_t gained = 0U;
  if (dir != 0) {
    const int32_t d = dir * ((int32_t)dp - (int32_t)dn);
    if (d > s->back_edges) {
      gained = (uint32_t)(d - s->back_edges);
      s->back_edges = 0;
    } else {
      s->back_edges = clamp32(s->back_edges - d, 0, BACK_MAX);
    }
  }
  const uint16_t backward = dir > 0 ? dn : dp;
  if ((dp != 0U || dn != 0U) && (dir == 0 || backward != 0U || back_before != 0U))
    s->ring_len = 0U; /* not a pure progress tick: the consecutive run ends here */
  if (gained != 0U) {
    if (s->ring_len == 0U) s->progress_total = 0U;
    s->progress_total += gained;
    s->ring_ms[s->ring_head] = s->now_ms;
    s->ring_edges[s->ring_head] = s->progress_total;
    s->ring_head = (uint8_t)((s->ring_head + 1U) % MOTOR_SPEED_BLEED_RING);
    if (s->ring_len < MOTOR_SPEED_BLEED_RING) ++s->ring_len;
  }
  return gained;
}

/* Rule (c) speed test: the shortest latest stretch of the run spanning >= 8 ms; +1 ms covers the edge-tick
 * quantization so +-6 % sectors at a tracked speed never read as 1.15x. */
static int bleed_overspeed(const struct motor_speed *s, int32_t ref_abs)
{
  const unsigned newest = (s->ring_head + MOTOR_SPEED_BLEED_RING - 1U) % MOTOR_SPEED_BLEED_RING;
  for (unsigned back = 1U; back < s->ring_len; ++back) {
    const unsigned j = (newest + MOTOR_SPEED_BLEED_RING - back) % MOTOR_SPEED_BLEED_RING;
    const uint32_t span = s->now_ms - s->ring_ms[j];
    if (span < MOTOR_SPEED_BLEED_SPAN_MS) continue;
    const int64_t k = (int64_t)(s->ring_edges[newest] - s->ring_edges[j]);
    /* 1250*k/(span+1) >= 1.15*|r|, compared exactly */
    return (int64_t)MOTOR_SPEED_EDGE_RPM_MS * k * 100 >= (int64_t)MOTOR_SPEED_BLEED_PCT * ref_abs * (span + 1U);
  }
  return 0;
}

static void reference_step(struct motor_speed *s)
{
  if (s->ref_k < MOTOR_PROTOCOL_PERIOD_MS) ++s->ref_k;
  const int32_t from = s->ref_from_rpm, to = s->ref_to_rpm;
  s->ref_rpm = (int16_t)(from + (to - from) * (int32_t)s->ref_k / (int32_t)MOTOR_PROTOCOL_PERIOD_MS);
}

static void ff_slew(struct motor_speed *s)
{
  /* Friction compensation only ever pushes along the command; against it the wheel brakes on P and I. */
  const int32_t target = sign32(s->ff_target_ma) == s->ref_sign ? s->ff_target_ma : 0;
  s->ff_ma = (int16_t)clamp32(target, s->ff_ma - MOTOR_SPEED_FF_SLEW_MA_PER_MS,
                              s->ff_ma + MOTOR_SPEED_FF_SLEW_MA_PER_MS);
}

void motor_speed_tick(struct motor_speed *s, const struct motor_speed_inputs *in, struct motor_speed_outputs *out)
{
  memset(out, 0, sizeof *out);
  ++s->now_ms;
  const uint16_t dp = (uint16_t)(in->edges_pos - s->prev_pos), dn = (uint16_t)(in->edges_neg - s->prev_neg);
  s->prev_pos = in->edges_pos;
  s->prev_neg = in->edges_neg;

  if (in->mode == MOTOR_SPEED_OFF) {
    /* r restarts from 0 at the next start but keeps the setpoint programmed with the start. */
    s->ref_rpm = s->ref_from_rpm = 0;
    s->ref_k = 0U;
    s->ref_sign = s->ref_dir = 0;
    loop_clear(s);
  } else {
    if (s->mode == MOTOR_SPEED_OFF) run_reset(s); /* start: validity needs edges of this run */
    reference_step(s);
    const int8_t sr = sign32(s->ref_rpm);
    if (sr != s->ref_sign) { /* start, stop or reversal of r */
      loop_clear(s);
      if (sr != 0 && s->ref_dir != 0 && sr != s->ref_dir) run_reset(s);
      if (sr != 0) s->ref_dir = sr;
      s->ref_sign = sr;
    }
  }
  s->mode = in->mode;
  out->ref_rpm = s->ref_rpm;

  estimate(s, in, dp, dn, out);
  const uint32_t gained = progress(s, dp, dn);
  if (in->mode != MOTOR_SPEED_RUNNING) {
    if (in->mode == MOTOR_SPEED_HOLD) loop_clear(s);
    return;
  }

  const int32_t L = in->limit_counts > 0 ? in->limit_counts : 0;
  const int32_t r = s->ref_rpm, r_abs = abs32(r), w = out->w_hat_rpm;
  const int8_t dir = s->ref_sign;

  s->slip = clamp32(s->slip + r_abs - (int32_t)gained * SLIP_UNIT, 0, SLIP_MAX);
  ff_slew(s);
  const int32_t ff = ma_to_counts(s->ff_ma);
  const int32_t e = clamp32(r - w, -MOTOR_SPEED_E_MAX_RPM, MOTOR_SPEED_E_MAX_RPM);
  const int32_t p_raw = (s->cfg.kp * e) >> MOTOR_SPEED_KP_SHIFT;
  const int32_t p = clamp32(p_raw, -L, L);

  if (s->slip >= SLIP_MAX && r_abs >= MOTOR_SPEED_MOVING_RPM && abs32(w) < MOTOR_SPEED_STANDSTILL_RPM) {
    /* (a) stuck: the integrator climbs at a fixed rate instead of by the error, so breakaway current
     * builds predictably and no error-driven wind-up is waiting when the wheel lets go. */
    const int32_t cap = (L - dir * (p + ff)) * Q14;
    int32_t along = dir * s->i_q14;
    if (along < cap) along = along + s->stall_step_q14 < cap ? along + s->stall_step_q14 : cap;
    s->i_q14 = dir * along;
    out->stall_ramp = 1U;
  } else if (gained != 0U && dir * s->i_q14 > 0 && bleed_overspeed(s, r_abs)) {
    s->i_q14 /= 2; /* (c) the wheel broke away on held I: drop half, C division truncates toward 0 */
    out->bleed = 1U;
  } else {
    /* (b) conditional integration on the saturated total, so ff or a lowered L cannot wind I up. */
    const int32_t u = p + (s->i_q14 >> MOTOR_SPEED_KI_SHIFT) + ff;
    const int32_t inc = s->cfg.ki * e;
    if ((u >= -L && u <= L) || (u > L && inc < 0) || (u < -L && inc > 0)) s->i_q14 += inc;
  }
  s->i_q14 = clamp32(s->i_q14, -L * Q14, L * Q14);

  const int32_t i = s->i_q14 >> MOTOR_SPEED_KI_SHIFT;
  const int32_t u = p + i + ff;
  const int32_t iq = clamp32(u, -L, L);

  if (gained != 0U || r_abs < MOTOR_SPEED_MOVING_RPM) {
    s->stall_count_ms = 0U;
  } else if (L > 0 && 5 * abs32(iq) >= 4 * L) { /* |iq_ref| >= 0.8 L; pauses below it */
    if (s->stall_count_ms < s->cfg.stall_ms) ++s->stall_count_ms;
    out->stall_timer = 1U;
  }
  out->stall_fault = s->cfg.stall_ms != 0U && s->stall_count_ms >= s->cfg.stall_ms;

  out->iq_ref_counts = (int16_t)iq;
  out->p_counts = (int16_t)p;
  out->i_counts = (int16_t)i;
  out->ff_counts = (int16_t)ff;
  out->saturated = u != iq || p != p_raw; /* a clipped P is a clipped request too */
}
