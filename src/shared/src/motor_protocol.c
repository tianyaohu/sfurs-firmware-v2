#include "motor_protocol.h"

#include <stddef.h>

/* Amp and RPM limits stay out: they are that wheel's compiled limits.
 * Session and sequence helpers below are pure. They do not store a wheel
 * and they do not apply a command. */

static int16_t get16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static int wheel_bits(uint32_t select)
{
    return select != 0u && (select & ~MOTOR_SELECT_MASK) == 0u;
}

static int one_wheel(uint32_t select)
{
    return wheel_bits(select) && (select & (select - 1u)) == 0u;
}

enum motor_reason motor_control_check(const struct motor_control *frame, uint32_t select)
{
    uint8_t life;
    int plain_disable;

    if (frame == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    /* Mode before flags: a mode this codec does not know makes the flag rules meaningless. */
    if (frame->mode > MOTOR_MODE_CURRENT) {
        return MOTOR_R_BAD_MODE;
    }
    if ((frame->flags & MOTOR_FLAG_RESERVED) != 0u) {
        return MOTOR_R_RESERVED_NONZERO;
    }
    life = (uint8_t)(frame->flags & (MOTOR_FLAG_ARM | MOTOR_FLAG_CLEAR_FAULT));
    if (life != 0u &&
        (frame->mode != MOTOR_MODE_DISABLE ||
         life == (MOTOR_FLAG_ARM | MOTOR_FLAG_CLEAR_FAULT))) {
        return MOTOR_R_BAD_FLAGS;
    }
    /* A field the mode does not use must be 0, so a sender bug cannot hide in it. */
    if (frame->mode == MOTOR_MODE_DISABLE && (frame->setpoint != 0 || frame->ff_ma != 0)) {
        return MOTOR_R_RESERVED_NONZERO;
    }
    if (frame->mode == MOTOR_MODE_CURRENT && frame->ff_ma != 0) {
        return MOTOR_R_RESERVED_NONZERO;
    }
    /* Stopping is safe for every wheel. Motion and arming name exactly one. */
    plain_disable = frame->mode == MOTOR_MODE_DISABLE && frame->flags == 0u;
    if (!wheel_bits(select) || (!plain_disable && !one_wheel(select))) {
        return MOTOR_R_BAD_SELECT;
    }
    return MOTOR_R_NONE;
}

enum motor_reason motor_control_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                       struct motor_control *out)
{
    if (data == NULL || out == NULL || dlc != MOTOR_PROTOCOL_DLC || id > 0x7FFu ||
        (id & MOTOR_CAN_ID_BASE_MASK) != MOTOR_CAN_ID_CONTROL) {
        return MOTOR_R_BAD_LENGTH;
    }
    out->mode = (uint8_t)(data[0] & 0x0Fu);
    out->flags = data[1];
    out->setpoint = get16(&data[2]);
    out->ff_ma = get16(&data[4]);
    out->seq = data[6];
    out->session = data[7];
    /* Version before the field rules: another version may not be this layout. */
    if ((data[0] >> 4) != MOTOR_PROTOCOL_VERSION) {
        return MOTOR_R_BAD_VERSION;
    }
    return motor_control_check(out, id & MOTOR_SELECT_MASK);
}

enum motor_reason motor_control_encode(const struct motor_control *frame, uint32_t select,
                                       uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC])
{
    enum motor_reason reason;

    if (id == NULL || data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    reason = motor_control_check(frame, select);
    if (reason != MOTOR_R_NONE) {
        return reason;
    }
    *id = MOTOR_CAN_ID_CONTROL | select;
    data[0] = (uint8_t)((MOTOR_PROTOCOL_VERSION << 4) | frame->mode);
    data[1] = frame->flags;
    put16(&data[2], (uint16_t)frame->setpoint);
    put16(&data[4], (uint16_t)frame->ff_ma);
    data[6] = frame->seq;
    data[7] = frame->session;
    return MOTOR_R_NONE;
}

static enum motor_reason reply_base(uint32_t id, const uint8_t *data, uint32_t dlc,
                                    uint32_t base)
{
    if (data == NULL || dlc != MOTOR_PROTOCOL_DLC || id > 0x7FFu ||
        (id & MOTOR_CAN_SERVICE_MASK) != base) {
        return MOTOR_R_BAD_LENGTH;
    }
    return MOTOR_R_NONE;
}

enum motor_reason motor_status_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                      struct motor_status *out)
{
    if (out == NULL || reply_base(id, data, dlc, MOTOR_CAN_ID_STATUS) != MOTOR_R_NONE) {
        return MOTOR_R_BAD_LENGTH;
    }
    out->rpm = get16(&data[0]);
    out->iq_counts = get16(&data[2]);
    out->iq_ref_counts = get16(&data[4]);
    out->mcsdk_state = data[6];
    out->flags = data[7];
    /* A reply comes from one ESC. A multi-wheel id has no single sender. */
    if (!one_wheel(id & 0x7Fu)) {
        return MOTOR_R_BAD_SELECT;
    }
    return MOTOR_R_NONE;
}

enum motor_reason motor_status_encode(const struct motor_status *frame, uint32_t select,
                                      uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC])
{
    if (frame == NULL || id == NULL || data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    if (!one_wheel(select)) {
        return MOTOR_R_BAD_SELECT;
    }
    *id = MOTOR_CAN_ID_STATUS | select;
    put16(&data[0], (uint16_t)frame->rpm);
    put16(&data[2], (uint16_t)frame->iq_counts);
    put16(&data[4], (uint16_t)frame->iq_ref_counts);
    data[6] = frame->mcsdk_state;
    data[7] = frame->flags;
    return MOTOR_R_NONE;
}

enum motor_reason motor_diag_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                    struct motor_diag *out)
{
    if (out == NULL || reply_base(id, data, dlc, MOTOR_CAN_ID_DIAG) != MOTOR_R_NONE) {
        return MOTOR_R_BAD_LENGTH;
    }
    out->vbus_cv = (uint16_t)get16(&data[0]);
    out->faults = (uint16_t)get16(&data[2]);
    out->hall_shift_deg = get16(&data[4]);
    out->align_result = data[6];
    out->i2t_percent = data[7];
    if (!one_wheel(id & 0x7Fu)) {
        return MOTOR_R_BAD_SELECT;
    }
    return MOTOR_R_NONE;
}

enum motor_reason motor_diag_encode(const struct motor_diag *frame, uint32_t select,
                                    uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC])
{
    if (frame == NULL || id == NULL || data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    if (!one_wheel(select)) {
        return MOTOR_R_BAD_SELECT;
    }
    *id = MOTOR_CAN_ID_DIAG | select;
    put16(&data[0], frame->vbus_cv);
    put16(&data[2], frame->faults);
    put16(&data[4], (uint16_t)frame->hall_shift_deg);
    data[6] = frame->align_result;
    data[7] = frame->i2t_percent;
    return MOTOR_R_NONE;
}

enum motor_reason motor_poll_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                    uint32_t *command, uint32_t *select)
{
    uint32_t base;
    uint32_t bits;

    if (data == NULL || command == NULL || select == NULL || dlc != MOTOR_POLL_DLC ||
        id > 0x7FFu) {
        return MOTOR_R_BAD_LENGTH;
    }
    base = id & MOTOR_CAN_SERVICE_MASK;
    if (base != MOTOR_CAN_ID_STATUS_READ && base != MOTOR_CAN_ID_DIAG_READ) {
        return MOTOR_R_BAD_LENGTH;
    }
    bits = id & 0x7Fu;
    /* Any non-empty subset: 0x10F asks every wheel, and each answers alone. */
    if (!wheel_bits(bits)) {
        return MOTOR_R_BAD_SELECT;
    }
    *command = base;
    *select = bits;
    return MOTOR_R_NONE;
}

enum motor_reason motor_poll_encode(uint32_t command, uint32_t select, uint32_t *id,
                                    uint8_t data[MOTOR_POLL_DLC])
{
    unsigned i;

    if (id == NULL || data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    if (command != MOTOR_CAN_ID_STATUS_READ && command != MOTOR_CAN_ID_DIAG_READ) {
        return MOTOR_R_BAD_LENGTH;
    }
    if (!wheel_bits(select)) {
        return MOTOR_R_BAD_SELECT;
    }
    *id = command | select;
    for (i = 0; i < MOTOR_POLL_DLC; ++i) {
        data[i] = 0u;
    }
    return MOTOR_R_NONE;
}

/* Nearest step, halves away from zero. Truncating division does that once
 * the half-step has been added with the sign of the current. */
static int8_t reply_iq_steps(int16_t iq_ma)
{
    int32_t ma = iq_ma;
    int32_t steps;
    const int32_t half = MOTOR_REPLY_IQ_STEP_MA / 2;

    ma += (ma >= 0) ? half : -half;
    steps = ma / MOTOR_REPLY_IQ_STEP_MA;
    if (steps > 127) {
        steps = 127;
    } else if (steps < -127) {
        steps = -127;
    }
    return (int8_t)steps;
}

enum motor_reason motor_reply_encode(const struct motor_reply *frame, uint32_t select,
                                     uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC])
{
    if (frame == NULL || id == NULL || data == NULL) {
        return MOTOR_R_BAD_LENGTH;
    }
    /* A value outside the enum would be truncated into a different meaning. */
    if (frame->lifecycle > MOTOR_LC_FAULTED || frame->outcome > MOTOR_OUT_DUPLICATE) {
        return MOTOR_R_RESERVED_NONZERO;
    }
    if (!one_wheel(select)) {
        return MOTOR_R_BAD_SELECT;
    }
    *id = MOTOR_CAN_ID_CONTROL_REPLY | select;
    data[0] = frame->request_seq;
    data[1] = frame->applied_seq;
    data[2] = (uint8_t)(frame->lifecycle | (uint8_t)(frame->outcome << 3) |
                        (frame->first_edge != 0u ? 0x40u : 0u));
    data[3] = frame->reason;
    data[4] = frame->session;
    put16(&data[5], (uint16_t)frame->rpm);
    data[7] = (uint8_t)reply_iq_steps(frame->iq_ma);
    return MOTOR_R_NONE;
}

enum motor_reason motor_reply_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                     struct motor_reply *out)
{
    uint8_t lifecycle;
    uint8_t outcome;

    if (data == NULL || out == NULL || dlc != MOTOR_PROTOCOL_DLC || id > 0x7FFu ||
        (id & MOTOR_CAN_ID_BASE_MASK) != MOTOR_CAN_ID_CONTROL_REPLY) {
        return MOTOR_R_BAD_LENGTH;
    }
    lifecycle = (uint8_t)(data[2] & 0x07u);
    outcome = (uint8_t)((data[2] >> 3) & 0x07u);
    if ((data[2] & 0x80u) != 0u || lifecycle > MOTOR_LC_FAULTED ||
        outcome > MOTOR_OUT_DUPLICATE) {
        return MOTOR_R_RESERVED_NONZERO;
    }
    out->request_seq = data[0];
    out->applied_seq = data[1];
    out->lifecycle = lifecycle;
    out->outcome = outcome;
    out->first_edge = (uint8_t)((data[2] & 0x40u) != 0u);
    out->reason = data[3];
    out->session = data[4];
    out->rpm = get16(&data[5]);
    out->iq_ma = (int16_t)((int)(int8_t)data[7] * MOTOR_REPLY_IQ_STEP_MA);
    if (!one_wheel(id & MOTOR_SELECT_MASK)) {
        return MOTOR_R_BAD_SELECT;
    }
    return MOTOR_R_NONE;
}

enum motor_seq_class motor_seq_classify(uint8_t last_seq, uint8_t seq)
{
    /* Unsigned distance: narrowing to int8_t is implementation-defined. */
    uint8_t ahead = (uint8_t)(seq - last_seq);

    if (ahead == 0u) {
        return MOTOR_SEQ_DUPLICATE;
    }
    if (ahead == 128u) {
        return MOTOR_SEQ_AMBIGUOUS;
    }
    if (ahead > 128u) {
        return MOTOR_SEQ_STALE;
    }
    return (ahead <= MOTOR_SEQ_WINDOW) ? MOTOR_SEQ_NEXT : MOTOR_SEQ_JUMP;
}

uint8_t motor_session_next(uint8_t prev, uint32_t entropy)
{
    /* Boot (mod 255) can land on any nonzero byte. A later step uses mod 254
     * so it skips prev; the leading 1+ is what keeps 0 out. */
    uint32_t span = (prev == 0u) ? 255u : 254u;
    uint32_t base = (prev == 0u) ? 0u : (uint32_t)prev;

    return (uint8_t)(1u + (base + (entropy % span)) % 255u);
}
