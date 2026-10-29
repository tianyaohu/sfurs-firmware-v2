#include "motor_cmd.h"

#include <string.h>

static void clear_target(struct motor_cmd *m)
{
    m->setpoint = 0;
    m->ff_ma = 0;
    m->mode = MOTOR_MODE_DISABLE;
    m->applied_seq = 0;
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

/* READY only: STARTING would hide the setpoint inside MCSDK. Record it instead. */
static enum motor_reason on_motion(struct motor_cmd *m, const struct motor_control *c,
                                   uint8_t *outcome)
{
    int32_t sp_max;

    if (m->lifecycle != MOTOR_LC_READY) {
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
    *outcome = MOTOR_OUT_APPLIED;
    return MOTOR_R_NONE;
}

static enum motor_reason on_control(struct motor_cmd *m, const struct motor_control *c,
                                    uint8_t *outcome)
{
    *outcome = MOTOR_OUT_REJECTED;
    if (c->mode != MOTOR_MODE_DISABLE) {
        return on_motion(m, c, outcome);
    }
    if ((c->flags & MOTOR_FLAG_ARM) != 0u) {
        /* A session-0 probe has to learn the cause, so the latch wins over the session. */
        if (m->lifecycle == MOTOR_LC_FAULTED) {
            return (enum motor_reason)m->latched;
        }
        if (c->session != m->session) {
            return MOTOR_R_BAD_SESSION;
        }
        if (m->lifecycle == MOTOR_LC_DISARMED) {
            m->lifecycle = MOTOR_LC_READY;
            clear_target(m);
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
    /* Output-on states are absent, so DISABLE cannot be the step back to READY. */
    clear_target(m);
    *outcome = MOTOR_OUT_APPLIED;
    return MOTOR_R_NONE;
}

enum motor_reason motor_cmd_control(struct motor_cmd *m, uint32_t id, const uint8_t *data,
                                    uint32_t dlc, uint32_t *reply_id,
                                    uint8_t reply_data[MOTOR_PROTOCOL_DLC],
                                    struct motor_reply *reply)
{
    struct motor_control frame;
    struct motor_reply built;
    enum motor_reason structural;
    enum motor_reason reason;
    uint8_t outcome;

    if (m == NULL || reply_id == NULL || reply_data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    memset(&frame, 0, sizeof frame);
    structural = motor_control_decode(id, data, dlc, &frame);
    if (structural == MOTOR_R_BAD_LENGTH) {
        return MOTOR_R_BAD_LENGTH;
    }
    /* A broadcast DISABLE includes this wheel. Any other foreign select does not. */
    if (structural == MOTOR_R_NONE && (id & m->cfg.select) != m->cfg.select) {
        structural = MOTOR_R_BAD_SELECT;
    }
    if (structural != MOTOR_R_NONE) {
        reason = structural;
        outcome = MOTOR_OUT_REJECTED;
    } else {
        reason = on_control(m, &frame, &outcome);
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
    return reason;
}
