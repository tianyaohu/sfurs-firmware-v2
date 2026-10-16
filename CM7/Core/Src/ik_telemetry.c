#include "ik_telemetry.h"
#include "can_datatypes.h"
#include <stddef.h>
#include <stdatomic.h>

_Static_assert(sizeof(struct ik_telemetry_wheel) == IK_TELEMETRY_WORDS * 4U, "telemetry row ABI");
_Static_assert(IK_CAPTURE_SAMPLES == ESC_CAPTURE_SAMPLES, "capture buffer size");

volatile struct ik_telemetry_wheel ik_drive_telem[4];
volatile uint32_t ik_drive_telem_now;
volatile struct ik_capture ik_drive_capture;

static uint32_t poll_tick;
static unsigned poll_index, ident_wheel;

uint32_t ik_telemetry_poll_due(uint32_t now)
{
  ik_drive_telem_now = now;
  if ((uint32_t)(now - poll_tick) < IK_TELEMETRY_POLL_MS) return 0;
  poll_tick = now;
  const unsigned slot = poll_index;
  poll_index = (poll_index + 1U) % IK_TELEMETRY_SLOTS;
  if (slot == 16U) {
    const uint32_t selector = 1U << ident_wheel;
    ident_wheel = (ident_wheel + 1U) % 4U;
    return IDENT_READ | selector;
  }
  const uint32_t selector = 1U << (slot % 4U);
  return (slot >= 12U ? DIAG_READ : STATUS_READ) | selector;
}

static uint32_t u16(const uint8_t *d) { return (uint32_t)d[0] | ((uint32_t)d[1] << 8); }
static int32_t s16(const uint8_t *d) { return (int32_t)(int16_t)u16(d); }
static uint32_t u32(const uint8_t *d) { return u16(d) | (u16(d + 2) << 16); }

static void capture_rx(uint32_t selector, const uint8_t data[8])
{
  const uint32_t tag = u16(data), bank = tag >> 12, index = tag & 0x3FFU;
  volatile struct ik_capture *c = &ik_drive_capture;
  if (bank == ESC_CAPTURE_INFO_BANK) {
    c->status = u16(data + 2);
    c->vbus_centivolts = u16(data + 4);
    c->samples = u16(data + 6);
  } else if (bank < 2U && index < IK_CAPTURE_SAMPLES) {
    for (unsigned k = 0; k < 3U; ++k) c->sample[index][bank * 3U + k] = (int16_t)s16(data + 2 + 2 * k);
  } else {
    return;
  }
  c->source = selector;
  ++c->frames;
}

int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now)
{
  const uint32_t command = CAN_COMMAND_FROM_ID(id);
  if (data == NULL || id > 0x7FFU ||
      (command != STATUS_REPLY && command != DIAG_REPLY && command != IDENT_REPLY &&
       command != CAPTURE_REPLY))
    return 0;
  unsigned index;
  switch (id & 0x7FU) {
    case MOTOR_NE_CAN_ID: index = 0; break;
    case MOTOR_SE_CAN_ID: index = 1; break;
    case MOTOR_SW_CAN_ID: index = 2; break;
    case MOTOR_NW_CAN_ID: index = 3; break;
    default: return 0;
  }
  if (command == CAPTURE_REPLY) {
    capture_rx(id & 0x7FU, data);
    return 1;
  }
  /* Seqlock for the debugger: odd while writing, even when stable. */
  volatile struct ik_telemetry_wheel *row = &ik_drive_telem[index];
  ++row->generation;
  atomic_thread_fence(memory_order_seq_cst);
  if (command == STATUS_REPLY) {
    row->rpm = s16(data);
    row->iq_counts = s16(data + 2);
    row->iq_ref_counts = s16(data + 4);
    row->state = data[6];
    row->flags = data[7];
    row->status_tick = now;
    ++row->status_count;
  } else if (command == DIAG_REPLY) {
    row->vbus_centivolts = u16(data);
    row->faults = u16(data + 2);
    row->hall_shift_deg = s16(data + 4);
    row->align_result = data[6];
    row->i2t_percent = data[7];
    row->diag_tick = now;
    ++row->diag_count;
  } else {
    row->app_crc = u32(data);
    row->app_size = u32(data + 4);
    row->ident_tick = now;
    ++row->ident_count;
  }
  atomic_thread_fence(memory_order_seq_cst);
  ++row->generation;
  return 1;
}
