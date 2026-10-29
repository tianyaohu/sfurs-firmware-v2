#include "motor_protocol.h"

#include <stddef.h>

/* Session, sequence, and amp/RPM limits are intentionally absent. They depend
 * on ESC state and on that wheel's compiled limits; checking them here would
 * either be a second copy or a wrong one. */

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
