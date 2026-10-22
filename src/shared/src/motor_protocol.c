#include "motor_protocol.h"

#include <stddef.h>

#define REPLY_IQ_STEP_MA 50
#define REPLY_IQ_MAX_STEPS 127

static int one_bit(uint32_t v)
{
  return v != 0U && (v & (v - 1U)) == 0U;
}

static int16_t get16(const uint8_t *p)
{
  return (int16_t)(uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static void put16(uint8_t *p, int16_t v)
{
  const uint16_t u = (uint16_t)v;
  p[0] = (uint8_t)u;
  p[1] = (uint8_t)(u >> 8);
}

enum motor_reason motor_control_check(const struct motor_control *c, uint32_t select)
{
  if (c == NULL) return MOTOR_R_BAD_LENGTH;
  if (c->mode > MOTOR_MODE_CURRENT) return MOTOR_R_BAD_MODE;
  if ((c->flags & MOTOR_FLAG_RESERVED) != 0U) return MOTOR_R_RESERVED_NONZERO;
  const uint8_t lifecycle_flags = c->flags & (MOTOR_FLAG_ARM | MOTOR_FLAG_CLEAR_FAULT);
  if (lifecycle_flags != 0U &&
      (c->mode != MOTOR_MODE_DISABLE || lifecycle_flags == (MOTOR_FLAG_ARM | MOTOR_FLAG_CLEAR_FAULT)))
    return MOTOR_R_BAD_FLAGS;
  /* Fields a mode does not use must be 0, so a sender bug cannot hide in them. */
  if (c->mode == MOTOR_MODE_DISABLE && (c->setpoint != 0 || c->ff_ma != 0)) return MOTOR_R_RESERVED_NONZERO;
  if (c->mode == MOTOR_MODE_CURRENT && c->ff_ma != 0) return MOTOR_R_RESERVED_NONZERO;
  /* Only stopping may be broadcast: it is the one command that is safe for every wheel. */
  const int plain_disable = c->mode == MOTOR_MODE_DISABLE && c->flags == 0U;
  if (select == 0U || (select & ~MOTOR_CAN_SELECT_MASK) != 0U || (!plain_disable && !one_bit(select)))
    return MOTOR_R_BAD_SELECT;
  return MOTOR_R_NONE;
}

enum motor_reason motor_control_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                       struct motor_control *out)
{
  if (data == NULL || out == NULL || dlc != MOTOR_PROTOCOL_DLC ||
      (id & MOTOR_CAN_ID_BASE_MASK) != MOTOR_CAN_ID_CONTROL)
    return MOTOR_R_BAD_LENGTH;
  out->mode = data[0] & 0x0FU;
  out->flags = data[1];
  out->setpoint = get16(&data[2]);
  out->ff_ma = get16(&data[4]);
  out->seq = data[6];
  out->session = data[7];
  if ((data[0] >> 4) != MOTOR_PROTOCOL_VERSION) return MOTOR_R_BAD_VERSION;
  return motor_control_check(out, id & MOTOR_CAN_SELECT_MASK);
}

enum motor_reason motor_control_encode(const struct motor_control *c, uint32_t select,
                                       uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC])
{
  if (id == NULL || data == NULL) return MOTOR_R_BAD_LENGTH;
  const enum motor_reason r = motor_control_check(c, select);
  if (r != MOTOR_R_NONE) return r;
  *id = MOTOR_CAN_ID_CONTROL | select;
  data[0] = (uint8_t)((MOTOR_PROTOCOL_VERSION << 4) | c->mode);
  data[1] = c->flags;
  put16(&data[2], c->setpoint);
  put16(&data[4], c->ff_ma);
  data[6] = c->seq;
  data[7] = c->session;
  return MOTOR_R_NONE;
}

void motor_reply_encode(const struct motor_reply *r, uint8_t data[MOTOR_PROTOCOL_DLC])
{
  if (r == NULL || data == NULL) return;
  /* Nearest 50 mA step, halves away from zero, saturated to what int8 holds. */
  int32_t steps = ((int32_t)r->iq_ma + (r->iq_ma >= 0 ? REPLY_IQ_STEP_MA / 2 : -REPLY_IQ_STEP_MA / 2)) /
                  REPLY_IQ_STEP_MA;
  if (steps > REPLY_IQ_MAX_STEPS) steps = REPLY_IQ_MAX_STEPS;
  if (steps < -REPLY_IQ_MAX_STEPS) steps = -REPLY_IQ_MAX_STEPS;
  data[0] = r->request_seq;
  data[1] = r->applied_seq;
  data[2] = (uint8_t)((r->lifecycle & 0x07U) | ((r->outcome & 0x07U) << 3) | (r->first_edge ? 0x40U : 0U));
  data[3] = r->reason;
  data[4] = r->session;
  put16(&data[5], r->rpm);
  data[7] = (uint8_t)(int8_t)steps;
}

enum motor_reason motor_reply_decode(const uint8_t *data, uint32_t dlc, struct motor_reply *out)
{
  if (data == NULL || out == NULL || dlc != MOTOR_PROTOCOL_DLC) return MOTOR_R_BAD_LENGTH;
  const uint8_t lifecycle = data[2] & 0x07U, outcome = (data[2] >> 3) & 0x07U;
  if ((data[2] & 0x80U) != 0U || lifecycle > MOTOR_LC_FAULTED || outcome > MOTOR_OUT_DUPLICATE)
    return MOTOR_R_RESERVED_NONZERO;
  out->request_seq = data[0];
  out->applied_seq = data[1];
  out->lifecycle = lifecycle;
  out->outcome = outcome;
  out->first_edge = (data[2] & 0x40U) != 0U;
  out->reason = data[3];
  out->session = data[4];
  out->rpm = get16(&data[5]);
  out->iq_ma = (int16_t)((int8_t)data[7] * REPLY_IQ_STEP_MA);
  return MOTOR_R_NONE;
}

enum motor_seq_class motor_seq_classify(uint8_t last, uint8_t seq)
{
  /* Unsigned distance avoids implementation-defined narrowing to int8_t. */
  const uint8_t ahead = (uint8_t)(seq - last);
  if (ahead == 0U) return MOTOR_SEQ_DUPLICATE;
  if (ahead == 128U) return MOTOR_SEQ_AMBIGUOUS;
  if (ahead > 128U) return MOTOR_SEQ_STALE;
  return ahead <= MOTOR_SEQ_WINDOW ? MOTOR_SEQ_NEXT : MOTOR_SEQ_JUMP;
}

uint8_t motor_session_next(uint8_t prev, uint32_t entropy)
{
  if (prev == 0U) return (uint8_t)(1U + entropy % 255U);
  /* Step 1..254 places around the 255 nonzero values: never 0, never prev. */
  return (uint8_t)(1U + ((uint32_t)(prev - 1U) + 1U + entropy % 254U) % 255U);
}

static enum motor_reason config_check(const struct motor_config *c, uint32_t select)
{
  if (c->op != MOTOR_CONFIG_SET && c->op != MOTOR_CONFIG_GET) return MOTOR_R_BAD_MODE;
  if (c->op == MOTOR_CONFIG_GET && c->value != 0) return MOTOR_R_RESERVED_NONZERO;
  /* One wheel per frame: the reply then names the only ESC that could have applied it. */
  if ((select & ~MOTOR_CAN_SELECT_MASK) != 0U || !one_bit(select)) return MOTOR_R_BAD_SELECT;
  return MOTOR_R_NONE;
}

enum motor_reason motor_config_decode(uint32_t id, const uint8_t *data, uint32_t dlc, struct motor_config *out)
{
  if (data == NULL || out == NULL || dlc != MOTOR_PROTOCOL_DLC ||
      (id & MOTOR_CAN_ID_BASE_MASK) != MOTOR_CAN_ID_CONFIG)
    return MOTOR_R_BAD_LENGTH;
  out->op = data[0] & 0x0FU;
  out->seq = data[1];
  out->key = (uint16_t)get16(&data[2]);
  out->value = get16(&data[4]);
  if ((data[0] >> 4) != MOTOR_PROTOCOL_VERSION) return MOTOR_R_BAD_VERSION;
  if (data[6] != 0U || data[7] != 0U) return MOTOR_R_RESERVED_NONZERO;
  return config_check(out, id & MOTOR_CAN_SELECT_MASK);
}

enum motor_reason motor_config_encode(const struct motor_config *c, uint32_t select, uint32_t *id,
                                      uint8_t data[MOTOR_PROTOCOL_DLC])
{
  if (c == NULL || id == NULL || data == NULL) return MOTOR_R_BAD_LENGTH;
  const enum motor_reason r = config_check(c, select);
  if (r != MOTOR_R_NONE) return r;
  *id = MOTOR_CAN_ID_CONFIG | select;
  data[0] = (uint8_t)((MOTOR_PROTOCOL_VERSION << 4) | c->op);
  data[1] = c->seq;
  put16(&data[2], (int16_t)c->key);
  put16(&data[4], c->value);
  data[6] = 0U;
  data[7] = 0U;
  return MOTOR_R_NONE;
}

void motor_config_reply_encode(const struct motor_config_reply *r, uint8_t data[MOTOR_PROTOCOL_DLC])
{
  data[0] = r->seq;
  data[1] = r->status;
  put16(&data[2], (int16_t)r->key);
  put16(&data[4], r->stored);
  put16(&data[6], r->effective);
}

enum motor_reason motor_config_reply_decode(const uint8_t *data, uint32_t dlc, struct motor_config_reply *out)
{
  if (data == NULL || out == NULL || dlc != MOTOR_PROTOCOL_DLC) return MOTOR_R_BAD_LENGTH;
  out->seq = data[0];
  out->status = data[1];
  out->key = (uint16_t)get16(&data[2]);
  out->stored = get16(&data[4]);
  out->effective = get16(&data[6]);
  return MOTOR_R_NONE;
}
