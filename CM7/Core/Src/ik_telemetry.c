#include "ik_telemetry.h"
#include "can_datatypes.h"
#include <stddef.h>
#include <stdatomic.h>

_Static_assert(sizeof(struct ik_telemetry_wheel) == IK_TELEMETRY_WORDS * 4U, "telemetry row ABI");

volatile struct ik_telemetry_wheel ik_drive_telem[4];
volatile uint32_t ik_drive_telem_now;

static uint32_t poll_tick;
static unsigned poll_index;

uint32_t ik_telemetry_poll_due(uint32_t now)
{
  ik_drive_telem_now = now;
  if ((uint32_t)(now - poll_tick) < IK_TELEMETRY_POLL_MS) return 0;
  poll_tick = now;
  /* 16 slots: status for all four wheels three times, diag once each. */
  const unsigned slot = poll_index;
  poll_index = (poll_index + 1U) % 16U;
  const uint32_t selector = 1U << (slot % 4U);
  return (slot >= 12U ? DIAG_READ : STATUS_READ) | selector;
}

static uint32_t u16(const uint8_t *d) { return (uint32_t)d[0] | ((uint32_t)d[1] << 8); }
static int32_t s16(const uint8_t *d) { return (int32_t)(int16_t)u16(d); }

int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now)
{
  const uint32_t command = CAN_COMMAND_FROM_ID(id);
  if (data == NULL || id > 0x7FFU || (command != STATUS_REPLY && command != DIAG_REPLY))
    return 0;
  unsigned index;
  switch (id & 0x7FU) {
    case MOTOR_NE_CAN_ID: index = 0; break;
    case MOTOR_SE_CAN_ID: index = 1; break;
    case MOTOR_SW_CAN_ID: index = 2; break;
    case MOTOR_NW_CAN_ID: index = 3; break;
    default: return 0;
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
  } else {
    row->vbus_centivolts = u16(data);
    row->faults = u16(data + 2);
    row->hall_shift_deg = s16(data + 4);
    row->align_result = data[6];
    row->i2t_percent = data[7];
    row->diag_tick = now;
    ++row->diag_count;
  }
  atomic_thread_fence(memory_order_seq_cst);
  ++row->generation;
  return 1;
}
