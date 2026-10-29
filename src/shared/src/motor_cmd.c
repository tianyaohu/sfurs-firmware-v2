#include "motor_cmd.h"

#include <string.h>

static void clear_target(struct motor_cmd *m)
{
    m->setpoint = 0;
    m->ff_ma = 0;
    m->mode = MOTOR_MODE_DISABLE;
    m->applied_seq = 0;
}

static int output_on(uint8_t lc)
{
    return lc == MOTOR_LC_STARTING || lc == MOTOR_LC_RUNNING;
}

static int armed(uint8_t lc)
{
    return lc == MOTOR_LC_READY || output_on(lc);
}

void motor_cmd_init(struct motor_cmd *m, const struct motor_cmd_cfg *cfg, uint32_t entropy)
{
    memset(m, 0, sizeof *m);
    m->cfg = *cfg;
    m->lifecycle = MOTOR_LC_DISARMED;
    /* Select-bit entropy is 1 + select. The helper keeps that rule in one place. */
    m->session = motor_session_next(0, entropy);
}

static int same_payload(const struct motor_control *a, const struct motor_control *b)
{
    return a->mode == b->mode && a->flags == b->flags && a->setpoint == b->setpoint &&
           a->ff_ma == b->ff_ma && a->session == b->session;
}

/* A frame older than the last command is not an expiry: unsigned wrap would say it was. */
static int rx_watch_expired(uint32_t rx_ms, uint32_t watch_ms)
{
    return (int32_t)(rx_ms - watch_ms) >= (int32_t)MOTOR_WATCHDOG_MS;
}

/* Output was on: the caller has to stop it. Disarm from READY does not. */
static void disarm(struct motor_cmd *m, struct motor_cmd_actions *act)
{
    if (act != NULL && output_on(m->lifecycle)) {
        act->stop = 1u;
    }
    m->lifecycle = MOTOR_LC_DISARMED;
    m->session = motor_session_next(m->session, m->now_ms);
    m->last_valid = 0u;
    m->disarm_reason = MOTOR_R_TIMEOUT;
    clear_target(m);
}

static void fault_start(struct motor_cmd *m, struct motor_cmd_actions *act)
{
    if (act != NULL) {
        act->stop = 1u;
        act->start = 0u;
        act->set_target = 0u;
    }
    m->lifecycle = MOTOR_LC_FAULTED;
    m->latched = MOTOR_R_FAULT_MCSDK;
    m->last_valid = 0u;
    clear_target(m);
}

/* Watchdog first: a wheel that has already gone silent must not start. */
static void after_sample(struct motor_cmd *m, const struct motor_cmd_inputs *in,
                         struct motor_cmd_actions *act)
{
    if (armed(m->lifecycle) && (m->now_ms - m->watch_ms) >= MOTOR_WATCHDOG_MS) {
        disarm(m, act);
        return;
    }
    if (m->lifecycle == MOTOR_LC_READY && m->setpoint != 0 && in->aligned != 0u &&
        in->busy == 0u && in->mc == MOTOR_CMD_MC_IDLE) {
        m->lifecycle = MOTOR_LC_STARTING;
        m->state_ms = m->now_ms;
        if (act != NULL) {
            act->start = 1u;
            act->set_target = 1u;
        }
        return;
    }
    if (m->lifecycle != MOTOR_LC_STARTING) {
        return;
    }
    if (in->mc == MOTOR_CMD_MC_RUN) {
        m->lifecycle = MOTOR_LC_RUNNING;
        return;
    }
    if ((m->now_ms - m->state_ms) >= MOTOR_CMD_START_TIMEOUT_MS) {
        fault_start(m, act);
    }
}

static enum motor_reason on_motion(struct motor_cmd *m, const struct motor_control *c,
                                   uint32_t rx_ms, struct motor_cmd_actions *act,
                                   uint8_t *outcome)
{
    int32_t sp_max;

    /* The latch wins over the session, same as ARM: a probe has to learn the cause. */
    if (m->lifecycle == MOTOR_LC_FAULTED) {
        return (enum motor_reason)m->latched;
    }
    if (!armed(m->lifecycle)) {
        return MOTOR_R_NOT_ARMED;
    }
    if (c->session != m->session) {
        return MOTOR_R_BAD_SESSION;
    }
    switch (motor_seq_classify(m->last_seq, c->seq)) {
    case MOTOR_SEQ_DUPLICATE:
        /* No stored payload yet (just after ARM) is a clash, not a replay. */
        if (m->last_valid == 0u || !same_payload(c, &m->last_frame)) {
            return MOTOR_R_CONFLICT;
        }
        *outcome = MOTOR_OUT_DUPLICATE;
        return MOTOR_R_NONE;
    case MOTOR_SEQ_NEXT:
        break;
    case MOTOR_SEQ_STALE:
        return MOTOR_R_STALE;
    case MOTOR_SEQ_AMBIGUOUS:
        return MOTOR_R_SEQ_AMBIGUOUS;
    default:
        return MOTOR_R_SEQ_JUMP;
    }
    sp_max = (c->mode == MOTOR_MODE_SPEED) ? m->cfg.max_rpm : m->cfg.max_ma;
    if ((int32_t)c->setpoint > sp_max || (int32_t)c->setpoint < -sp_max ||
        (int32_t)c->ff_ma > m->cfg.max_ma || (int32_t)c->ff_ma < -(int32_t)m->cfg.max_ma) {
        return MOTOR_R_OUT_OF_RANGE;
    }
    if (c->mode == MOTOR_MODE_CURRENT && m->cfg.current_supported == 0u) {
        return MOTOR_R_UNSUPPORTED;
    }
    m->last_seq = c->seq;
    m->last_frame = *c;
    m->last_valid = 1u;
    m->mode = c->mode;
    m->setpoint = c->setpoint;
    m->ff_ma = c->ff_ma;
    m->applied_seq = c->seq;
    m->watch_ms = rx_ms; /* duplicates and rejects returned above, so they do not refresh */
    if (act != NULL && output_on(m->lifecycle)) {
        act->set_target = 1u;
    }
    *outcome = MOTOR_OUT_APPLIED;
    return MOTOR_R_NONE;
}

static enum motor_reason on_control(struct motor_cmd *m, const struct motor_control *c,
                                    uint32_t rx_ms, uint8_t call_session,
                                    struct motor_cmd_actions *act, uint8_t *outcome)
{
    *outcome = MOTOR_OUT_REJECTED;
    if (c->mode != MOTOR_MODE_DISABLE) {
        return on_motion(m, c, rx_ms, act, outcome);
    }
    if ((c->flags & MOTOR_FLAG_ARM) != 0u) {
        /* A session-0 probe has to learn the cause, so the latch wins over the session. */
        if (m->lifecycle == MOTOR_LC_FAULTED) {
            return (enum motor_reason)m->latched;
        }
        /* call_session is the session at entry. A disarm in this call has
         * already replaced it, and motor_session_next is pure, so a frame
         * that repeats the new byte has not read it from a reply. */
        if (c->session != m->session || m->session != call_session) {
            return MOTOR_R_BAD_SESSION;
        }
        if (m->lifecycle == MOTOR_LC_DISARMED) {
            m->lifecycle = MOTOR_LC_READY;
            clear_target(m);
            m->watch_ms = rx_ms; /* ARM starts the clock; a later ARM does not refresh it */
            m->disarm_reason = 0u;
        }
        m->last_seq = c->seq;
        m->last_valid = 0u;
        *outcome = MOTOR_OUT_APPLIED;
        return MOTOR_R_NONE;
    }
    if ((c->flags & MOTOR_FLAG_CLEAR_FAULT) != 0u) {
        /* No fault word to acknowledge in this slice. */
        return MOTOR_R_UNSUPPORTED;
    }
    if (output_on(m->lifecycle)) {
        if (act != NULL) {
            act->stop = 1u;
        }
        m->lifecycle = MOTOR_LC_READY;
    }
    clear_target(m);
    *outcome = MOTOR_OUT_APPLIED;
    return MOTOR_R_NONE;
}

static void inhibit_motion(struct motor_cmd *m, const struct motor_control *frame,
                           uint8_t *outcome, enum motor_reason *reason)
{
    if (frame->mode == MOTOR_MODE_DISABLE || *outcome != MOTOR_OUT_APPLIED ||
        *reason != MOTOR_R_NONE) {
        return;
    }
    if (m->lifecycle == MOTOR_LC_FAULTED) {
        *outcome = MOTOR_OUT_INHIBITED;
        *reason = (enum motor_reason)m->latched;
        return;
    }
    /* Recorded, but the bridge is not regulating yet. RUNNING stays APPLIED. */
    if (m->setpoint != 0 &&
        (m->lifecycle == MOTOR_LC_READY || m->lifecycle == MOTOR_LC_STARTING)) {
        *outcome = MOTOR_OUT_INHIBITED;
        *reason = MOTOR_R_STARTING;
    }
}

void motor_cmd_tick(struct motor_cmd *m, const struct motor_cmd_inputs *in,
                    struct motor_cmd_actions *act)
{
    if (act != NULL) {
        memset(act, 0, sizeof *act);
    }
    if (m == NULL || in == NULL) {
        return;
    }
    m->now_ms = in->now_ms;
    after_sample(m, in, act);
}

enum motor_reason motor_cmd_control(struct motor_cmd *m, uint32_t id, const uint8_t *data,
                                    uint32_t dlc, uint32_t rx_ms,
                                    const struct motor_cmd_inputs *in,
                                    struct motor_cmd_actions *act, uint32_t *reply_id,
                                    uint8_t reply_data[MOTOR_PROTOCOL_DLC],
                                    struct motor_reply *reply)
{
    struct motor_control frame;
    struct motor_reply built;
    enum motor_reason structural;
    enum motor_reason reason;
    uint8_t outcome;
    uint8_t call_session;
    int motion_accepted;

    if (act != NULL) {
        memset(act, 0, sizeof *act);
    }
    if (m == NULL || reply_id == NULL || reply_data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    memset(&frame, 0, sizeof frame);
    structural = motor_control_decode(id, data, dlc, &frame);
    if (structural == MOTOR_R_BAD_LENGTH) {
        return MOTOR_R_BAD_LENGTH;
    }
    if (in != NULL) {
        m->now_ms = in->now_ms;
    }
    call_session = m->session;
    /* A broadcast DISABLE includes this wheel. Any other foreign select does not. */
    if (structural == MOTOR_R_NONE && (id & m->cfg.select) != m->cfg.select) {
        structural = MOTOR_R_BAD_SELECT;
    }
    /* Judged at the frame's own receive time, so a deadline frame finds the wheel disarmed.
     * call_session stays the pre-disarm value: this frame cannot arm on the new one. */
    if (in != NULL && structural == MOTOR_R_NONE && armed(m->lifecycle) &&
        rx_watch_expired(rx_ms, m->watch_ms)) {
        disarm(m, act);
    }
    if (structural != MOTOR_R_NONE) {
        reason = structural;
        outcome = MOTOR_OUT_REJECTED;
    } else if (in != NULL && frame.mode != MOTOR_MODE_DISABLE &&
               (m->now_ms - rx_ms) >= MOTOR_WATCHDOG_MS) {
        /* Age is receive time, not when this call got to the frame. A late DISABLE still stops. */
        reason = MOTOR_R_TIMEOUT;
        outcome = MOTOR_OUT_REJECTED;
    } else {
        reason = on_control(m, &frame, rx_ms, call_session, act, &outcome);
    }
    /* A fresh motion is answered after the start rules so STARTING is visible.
     * ARM, DISABLE, and rejects are answered at the verdict; the watchdog may
     * still move the state afterwards (a late ARM is applied, then disarms). */
    motion_accepted = structural == MOTOR_R_NONE && frame.mode != MOTOR_MODE_DISABLE &&
                      outcome == MOTOR_OUT_APPLIED && reason == MOTOR_R_NONE;
    if (in != NULL && motion_accepted != 0) {
        after_sample(m, in, act);
        inhibit_motion(m, &frame, &outcome, &reason);
    }
    memset(&built, 0, sizeof built);
    built.request_seq = frame.seq;
    built.applied_seq = m->applied_seq;
    built.lifecycle = m->lifecycle;
    built.outcome = outcome;
    built.reason = (uint8_t)reason;
    built.session = m->session;
    if (motor_reply_encode(&built, m->cfg.select, reply_id, reply_data) != MOTOR_R_NONE) {
        return MOTOR_R_BAD_SELECT;
    }
    if (reply != NULL) {
        *reply = built;
    }
    if (in != NULL && motion_accepted == 0) {
        after_sample(m, in, act);
    }
    return reason;
}
