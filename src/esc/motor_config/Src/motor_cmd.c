#include "motor_cmd.h"

#include <string.h>

_Static_assert((MOTOR_CMD_INBOX & (MOTOR_CMD_INBOX - 1U)) == 0U, "index wrap needs a power of two");
_Static_assert((MOTOR_CMD_REPLIES & (MOTOR_CMD_REPLIES - 1U)) == 0U, "index wrap needs a power of two");

static int output_on(uint8_t lc)
{
  return lc == MOTOR_LC_STARTING || lc == MOTOR_LC_RUNNING || lc == MOTOR_LC_HOLD;
}

static int armed(uint8_t lc)
{
  return lc == MOTOR_LC_READY || output_on(lc);
}

static int8_t sign16(int16_t v)
{
  return (int8_t)((v > 0) - (v < 0));
}

int motor_cmd_output_on(const struct motor_cmd *m)
{
  return output_on(m->lifecycle);
}

int motor_cmd_output_off(const struct motor_cmd *m)
{
  return !output_on(m->lifecycle);
}

void motor_cmd_init(struct motor_cmd *m, const struct motor_cmd_config *cfg, uint32_t entropy)
{
  memset(m, 0, sizeof *m);
  m->cfg = *cfg;
  atomic_init(&m->inbox_head, 0U);
  atomic_init(&m->inbox_tail, 0U);
  atomic_init(&m->inbox_overflow, 0U);
  atomic_init(&m->reply_head, 0U);
  atomic_init(&m->reply_tail, 0U);
  m->lifecycle = MOTOR_LC_DISARMED;
  m->session = motor_session_next(0U, entropy);
}

void motor_cmd_rx(struct motor_cmd *m, uint32_t id, const uint8_t *data, uint32_t dlc, uint32_t now_ms)
{
  struct motor_control c;
  memset(&c, 0, sizeof c);
  const enum motor_reason reason = motor_control_decode(id, data, dlc, &c);
  if (reason == MOTOR_R_BAD_LENGTH) { /* not a well-formed CONTROL frame: no seq to answer */
    ++m->count.bad_length;
    return;
  }
  const unsigned head = atomic_load_explicit(&m->inbox_head, memory_order_relaxed);
  const unsigned tail = atomic_load_explicit(&m->inbox_tail, memory_order_acquire);
  if (head - tail >= MOTOR_CMD_INBOX) { /* only a faulty sender gets here: the tick disarms */
    atomic_store_explicit(&m->inbox_overflow, 1U, memory_order_release);
    return;
  }
  struct motor_cmd_rx_entry *e = &m->inbox[head % MOTOR_CMD_INBOX];
  e->c = c;
  e->reason = (uint8_t)reason;
  e->rx_ms = now_ms;
  atomic_store_explicit(&m->inbox_head, head + 1U, memory_order_release);
}

static void reply(struct motor_cmd *m, const struct motor_cmd_inputs *in, uint8_t seq,
                  enum motor_outcome outcome, uint8_t reason)
{
  if (outcome == MOTOR_OUT_REJECTED) {
    ++m->count.rejected;
    m->count.last_reject = reason;
  }
  const unsigned head = atomic_load_explicit(&m->reply_head, memory_order_relaxed);
  const unsigned tail = atomic_load_explicit(&m->reply_tail, memory_order_acquire);
  if (head - tail >= MOTOR_CMD_REPLIES) {
    ++m->count.replies_dropped;
    return;
  }
  /* Speed and current mean nothing with the bridge off (MCSDK keeps stale values). */
  const int on = output_on(m->lifecycle);
  const struct motor_reply r = {
    .request_seq = seq, .applied_seq = m->applied_seq, .lifecycle = m->lifecycle,
    .outcome = (uint8_t)outcome, .first_edge = m->first_edge, .reason = reason,
    .session = m->session, .rpm = on ? in->rpm : 0, .iq_ma = on ? in->iq_ma : 0 };
  motor_reply_encode(&r, m->replies[head % MOTOR_CMD_REPLIES]);
  atomic_store_explicit(&m->reply_head, head + 1U, memory_order_release);
}

int motor_cmd_reply_peek(struct motor_cmd *m, uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC])
{
  const unsigned tail = atomic_load_explicit(&m->reply_tail, memory_order_relaxed);
  const unsigned head = atomic_load_explicit(&m->reply_head, memory_order_acquire);
  if (head == tail) return 0;
  memcpy(data, m->replies[tail % MOTOR_CMD_REPLIES], MOTOR_PROTOCOL_DLC);
  *id = MOTOR_CAN_ID_CONTROL_REPLY | m->cfg.select;
  return 1;
}

void motor_cmd_reply_pop(struct motor_cmd *m)
{
  const unsigned tail = atomic_load_explicit(&m->reply_tail, memory_order_relaxed);
  const unsigned head = atomic_load_explicit(&m->reply_head, memory_order_acquire);
  if (head != tail) atomic_store_explicit(&m->reply_tail, tail + 1U, memory_order_release);
}

static void clear_target(struct motor_cmd *m)
{
  m->setpoint = 0;
  m->ff_ma = 0;
  m->holding = 0U;
  m->start_dir = 0;
  m->first_edge = 0U;
}

/* A cause is reported only with a real switch-off, so STATUS keeps the last one that happened. */
static void stop_output(struct motor_cmd *m, struct motor_cmd_actions *out, enum motor_cmd_stop cause)
{
  if (!output_on(m->lifecycle)) return;
  out->stop = 1U;
  out->stop_cause = (uint8_t)cause;
}

/* Output off and a new session: every motion frame from before is now foreign. */
static void disarm(struct motor_cmd *m, struct motor_cmd_actions *out, enum motor_cmd_stop cause)
{
  stop_output(m, out, cause);
  m->lifecycle = MOTOR_LC_DISARMED;
  m->disarm_cause = (uint8_t)cause;
  m->session = motor_session_next(m->session, m->now_ms);
  m->applied_seq = 0U;
  m->last_valid = 0U;
  clear_target(m);
}

static void fault(struct motor_cmd *m, struct motor_cmd_actions *out, enum motor_reason reason)
{
  stop_output(m, out, MOTOR_CMD_STOP_FAULT);
  m->lifecycle = MOTOR_LC_FAULTED;
  m->latched = (uint8_t)reason;
  m->last_valid = 0U;
  clear_target(m);
}

static int same_payload(const struct motor_control *a, const struct motor_control *b)
{
  return a->mode == b->mode && a->flags == b->flags && a->setpoint == b->setpoint && a->ff_ma == b->ff_ma &&
         a->session == b->session;
}

/* Why a motion frame cannot be accepted, or MOTOR_R_NONE. The order is the one in architecture/protocol.md:
 * lifecycle, session, sequence, then this ESC's limits and capabilities. A DUPLICATE is reported
 * as MOTOR_R_NONE with *duplicate set. */
static enum motor_reason motion_verdict(const struct motor_cmd *m, const struct motor_cmd_inputs *in,
                                        const struct motor_control *c, int *duplicate)
{
  *duplicate = 0;
  if (m->lifecycle == MOTOR_LC_FAULTED) return (enum motor_reason)m->latched;
  if (!armed(m->lifecycle)) return MOTOR_R_NOT_ARMED;
  if (c->session != m->session || m->session != m->tick_session) return MOTOR_R_BAD_SESSION;
  switch (motor_seq_classify(m->last_seq, c->seq)) {
    case MOTOR_SEQ_NEXT: break;
    case MOTOR_SEQ_DUPLICATE:
      if (!m->last_valid || !same_payload(c, &m->last_frame)) return MOTOR_R_CONFLICT;
      *duplicate = 1;
      return MOTOR_R_NONE;
    case MOTOR_SEQ_STALE: return MOTOR_R_STALE;
    case MOTOR_SEQ_AMBIGUOUS: return MOTOR_R_SEQ_AMBIGUOUS;
    default: return MOTOR_R_SEQ_JUMP;
  }
  const int32_t sp = c->setpoint, ff = c->ff_ma;
  const int32_t sp_max = c->mode == MOTOR_MODE_SPEED ? m->cfg.max_rpm : m->cfg.max_ma;
  if (sp > sp_max || sp < -sp_max || ff > m->cfg.max_ma || ff < -m->cfg.max_ma) return MOTOR_R_OUT_OF_RANGE;
  if (c->mode == MOTOR_MODE_CURRENT && !m->cfg.current_supported) return MOTOR_R_UNSUPPORTED;
  if (output_on(m->lifecycle) && c->mode != m->mode) return MOTOR_R_MODE_CHANGE;
  if (in->busy && c->setpoint != 0) return MOTOR_R_BUSY;
  if (m->lifecycle == MOTOR_LC_READY && c->setpoint != 0 && !in->aligned) return MOTOR_R_NOT_ALIGNED;
  return MOTOR_R_NONE;
}

/* One drain of the inbox: the motion frame that survives it, if any. */
struct candidate {
  int valid;
  struct motor_control c;
};

static void supersede(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct candidate *cand)
{
  if (!cand->valid) return;
  reply(m, in, cand->c.seq, MOTOR_OUT_SUPERSEDED, MOTOR_R_NONE);
  cand->valid = 0;
}

static void on_disable(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out,
                       struct candidate *cand, const struct motor_control *c)
{
  supersede(m, in, cand);
  if (in->busy) out->abort_busy = 1U;
  out->disabled = 1U; /* an align/capture request queued before this DISABLE must not start after it */
  if (output_on(m->lifecycle)) {
    stop_output(m, out, MOTOR_CMD_STOP_DISABLE);
    m->lifecycle = MOTOR_LC_READY;
  }
  if (m->lifecycle != MOTOR_LC_FAULTED) clear_target(m);
  reply(m, in, c->seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE); /* a FAULTED lifecycle field says the rest */
}

static void on_arm(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct candidate *cand,
                   const struct motor_cmd_rx_entry *e)
{
  /* Lifecycle before session, as for motion: the Nucleo opens every session with a session-0 probe, and a
   * FAULTED wheel must name its cause to it, or a new session never learns why the wheel is off. */
  if (m->lifecycle == MOTOR_LC_FAULTED) { reply(m, in, e->c.seq, MOTOR_OUT_REJECTED, m->latched); return; }
  if (e->c.session != m->session || m->session != m->tick_session) {
    reply(m, in, e->c.seq, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION);
    return;
  }
  supersede(m, in, cand);
  if (m->lifecycle == MOTOR_LC_DISARMED) {
    m->lifecycle = MOTOR_LC_READY;
    m->watch_ms = e->rx_ms; /* ARM starts the watchdog; only fresh motion frames refresh it */
    m->disarm_cause = MOTOR_CMD_STOP_NONE;
    clear_target(m);
  }
  m->last_seq = e->c.seq;
  m->last_valid = 0U;
  reply(m, in, e->c.seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
}

static void on_clear_fault(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out,
                           const struct motor_control *c)
{
  if (m->lifecycle != MOTOR_LC_FAULTED) { reply(m, in, c->seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE); return; }
  const int cause_gone = !in->mc_fault_present && (in->mc == MOTOR_CMD_MC_IDLE || in->mc == MOTOR_CMD_MC_FAULT);
  if (!cause_gone) { reply(m, in, c->seq, MOTOR_OUT_REJECTED, MOTOR_R_NOT_IDLE); return; }
  out->ack_fault = 1U; /* MCSDK ignores it outside FAULT_OVER */
  m->latched = MOTOR_R_NONE;
  disarm(m, out, MOTOR_CMD_STOP_NONE);
  reply(m, in, c->seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
}

static void on_motion(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct candidate *cand,
                      const struct motor_cmd_rx_entry *e)
{
  int duplicate;
  const enum motor_reason why = motion_verdict(m, in, &e->c, &duplicate);
  if (duplicate) { reply(m, in, e->c.seq, MOTOR_OUT_DUPLICATE, MOTOR_R_NONE); return; }
  if (why != MOTOR_R_NONE) { reply(m, in, e->c.seq, MOTOR_OUT_REJECTED, (uint8_t)why); return; }
  m->last_seq = e->c.seq;
  m->last_frame = e->c;
  m->last_valid = 1U;
  m->watch_ms = e->rx_ms;
  supersede(m, in, cand);
  cand->valid = 1;
  cand->c = e->c;
}

static void drain(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out,
                  struct candidate *cand)
{
  unsigned tail = atomic_load_explicit(&m->inbox_tail, memory_order_relaxed);
  const unsigned head = atomic_load_explicit(&m->inbox_head, memory_order_acquire);
  for (; tail != head; ++tail) {
    const struct motor_cmd_rx_entry e = m->inbox[tail % MOTOR_CMD_INBOX];
    atomic_store_explicit(&m->inbox_tail, tail + 1U, memory_order_release);
    /* The watchdog is judged at each frame's receive time too: a frame that arrives once the last command is
     * already 100 ms old finds the wheel disarmed, even when the tick drains it in the same millisecond. */
    if (armed(m->lifecycle) && (int32_t)(e.rx_ms - m->watch_ms) >= (int32_t)MOTOR_WATCHDOG_MS)
      disarm(m, out, MOTOR_CMD_STOP_TIMEOUT); /* signed: a frame older than the last command is not an expiry */
    if (e.reason != MOTOR_R_NONE) { reply(m, in, e.c.seq, MOTOR_OUT_REJECTED, e.reason); continue; }
    /* Motion freshness is judged by receive time, never by when the tick got to it. A late
     * DISABLE still stops (the safe direction) and a late ARM only starts the watchdog clock. */
    if (e.c.mode != MOTOR_MODE_DISABLE && m->now_ms - e.rx_ms >= MOTOR_WATCHDOG_MS) {
      reply(m, in, e.c.seq, MOTOR_OUT_REJECTED, MOTOR_R_TIMEOUT);
      continue;
    }
    if (e.c.mode != MOTOR_MODE_DISABLE) on_motion(m, in, cand, &e);
    else if (e.c.flags & MOTOR_FLAG_ARM) on_arm(m, in, cand, &e);
    else if (e.c.flags & MOTOR_FLAG_CLEAR_FAULT) on_clear_fault(m, in, out, &e.c);
    else on_disable(m, in, out, cand, &e.c);
  }
}

static void apply(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out,
                  const struct motor_control *c)
{
  const int8_t was = sign16(m->setpoint), now = sign16(c->setpoint);
  m->setpoint = c->setpoint;
  m->ff_ma = c->ff_ma;
  m->applied_seq = c->seq;
  if (!output_on(m->lifecycle)) m->mode = c->mode; /* latched for the run at READY -> STARTING */
  if (now != was) { /* a start from zero or a reversal: first_edge restarts */
    m->start_dir = now;
    m->start_pos = in->edges_pos;
    m->start_neg = in->edges_neg;
    m->first_edge = 0U;
  }
  if (output_on(m->lifecycle)) out->set_target = 1U;
  if (m->lifecycle == MOTOR_LC_HOLD && c->setpoint != 0) { /* the bridge is on: resume, no MCSDK start */
    m->lifecycle = MOTOR_LC_RUNNING;
    m->holding = 0U;
  }
}

static void rules(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out)
{
  switch (m->lifecycle) {
    case MOTOR_LC_READY:
      /* State-driven: after a stop MCSDK passes through STOP before IDLE, and the start waits for it. */
      if (m->setpoint != 0 && in->aligned && !in->busy && in->mc == MOTOR_CMD_MC_IDLE) {
        m->lifecycle = MOTOR_LC_STARTING;
        m->state_ms = m->now_ms;
        out->set_target = 1U;
        out->start = 1U;
      }
      break;
    case MOTOR_LC_STARTING:
      if (in->mc == MOTOR_CMD_MC_RUN) {
        m->lifecycle = MOTOR_LC_RUNNING;
        m->holding = 0U;
      } else if (m->now_ms - m->state_ms >= MOTOR_CMD_START_TIMEOUT_MS) {
        fault(m, out, MOTOR_R_FAULT_MCSDK);
      }
      break;
    case MOTOR_LC_RUNNING:
      if (in->hall_lost) {
        fault(m, out, MOTOR_R_FAULT_HALL_LOST);
      } else if (in->mc != MOTOR_CMD_MC_RUN) { /* MCSDK left RUN on its own */
        fault(m, out, MOTOR_R_FAULT_MCSDK);
      } else if (m->mode == MOTOR_MODE_SPEED && m->setpoint == 0 && in->standstill) {
        if (!m->holding) {
          m->holding = 1U;
          m->hold_since = m->now_ms;
        } else if (m->now_ms - m->hold_since >= MOTOR_CMD_HOLD_MS) {
          m->lifecycle = MOTOR_LC_HOLD; /* output stays on: the speed loop holds Iq at 0 */
          m->state_ms = m->now_ms;
          m->holding = 0U;
        }
      } else {
        m->holding = 0U;
      }
      break;
    case MOTOR_LC_HOLD:
      if (in->hall_lost) fault(m, out, MOTOR_R_FAULT_HALL_LOST);
      else if (in->mc != MOTOR_CMD_MC_RUN) fault(m, out, MOTOR_R_FAULT_MCSDK);
      else if (m->now_ms - m->state_ms >= MOTOR_CMD_PARK_MS) { /* a long stop: silent, next start via READY */
        stop_output(m, out, MOTOR_CMD_STOP_PARKED);
        m->lifecycle = MOTOR_LC_READY;
        clear_target(m);
      }
      break;
    default:
      break;
  }
  if (m->start_dir != 0 && !m->first_edge) {
    const uint16_t moved = m->start_dir > 0 ? (uint16_t)(in->edges_pos - m->start_pos)
                                             : (uint16_t)(in->edges_neg - m->start_neg);
    if (moved != 0U) m->first_edge = 1U;
  }
}

void motor_cmd_tick(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out)
{
  memset(out, 0, sizeof *out);
  m->now_ms = in->now_ms;
  m->tick_session = m->session;
  if (in->mc == MOTOR_CMD_MC_FAULT && m->lifecycle != MOTOR_LC_FAULTED)
    fault(m, out, in->hall_lost ? MOTOR_R_FAULT_HALL_LOST : MOTOR_R_FAULT_MCSDK);
  if (atomic_exchange_explicit(&m->inbox_overflow, 0U, memory_order_acq_rel)) {
    ++m->count.overflow;
    if (m->lifecycle != MOTOR_LC_FAULTED) {
      disarm(m, out, MOTOR_CMD_STOP_OVERFLOW);
    } else { /* stays FAULTED, but nothing from the faulty burst may outlive it */
      m->session = motor_session_next(m->session, m->now_ms);
      m->applied_seq = 0U;
      m->last_valid = 0U;
    }
  }
  struct candidate cand = { 0 };
  drain(m, in, out, &cand);
  if (cand.valid) apply(m, in, out, &cand.c);
  if (armed(m->lifecycle) && m->now_ms - m->watch_ms >= MOTOR_WATCHDOG_MS)
    disarm(m, out, MOTOR_CMD_STOP_TIMEOUT);
  rules(m, in, out);
  if (cand.valid) {
    const struct motor_control *c = &cand.c;
    if (m->lifecycle == MOTOR_LC_FAULTED)
      reply(m, in, c->seq, MOTOR_OUT_INHIBITED, m->latched);
    else if (c->setpoint != 0 && (m->lifecycle == MOTOR_LC_READY || m->lifecycle == MOTOR_LC_STARTING))
      reply(m, in, c->seq, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
    else if (in->stall_rest && m->lifecycle == MOTOR_LC_RUNNING)
      reply(m, in, c->seq, MOTOR_OUT_INHIBITED, MOTOR_R_STALLED); /* applied; the loop pushes again after the rest */
    else if ((c->mode == MOTOR_MODE_CURRENT && (c->setpoint > in->limit_ma || c->setpoint < -in->limit_ma)) ||
             (c->mode == MOTOR_MODE_SPEED && (c->ff_ma > in->limit_ma || c->ff_ma < -in->limit_ma)))
      reply(m, in, c->seq, MOTOR_OUT_INHIBITED, MOTOR_R_LIMITED); /* applied, limited to the effective limit */
    else
      reply(m, in, c->seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  }
  if (out->set_target) {
    out->mode = m->mode;
    out->setpoint = m->setpoint;
    out->ff_ma = m->ff_ma;
  }
}
