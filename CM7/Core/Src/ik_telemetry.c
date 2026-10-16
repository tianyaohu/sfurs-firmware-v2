#include "ik_telemetry.h"
#include <stddef.h>
#include <stdatomic.h>

/* Keep the debugger's word-addressed ABI independent of compiler padding. */
_Static_assert(sizeof(struct ik_telemetry_wheel) == 52, "telemetry row ABI");
#define CHECK_FIELD(field, word) \
  _Static_assert(offsetof(struct ik_telemetry_wheel, field) == (word) * 4U, \
                 "telemetry offset: " #field); \
  _Static_assert(sizeof(((struct ik_telemetry_wheel *)0)->field) == 4, \
                 "telemetry width: " #field)
CHECK_FIELD(generation, 0);
CHECK_FIELD(rpm, 1);
CHECK_FIELD(state, 2);
CHECK_FIELD(hall, 3);
CHECK_FIELD(hall_changes, 4);
CHECK_FIELD(speed_tick, 5);
CHECK_FIELD(iq_counts, 6);
CHECK_FIELD(id_counts, 7);
CHECK_FIELD(flags, 8);
CHECK_FIELD(peak_l1_counts, 9);
CHECK_FIELD(current_tick, 10);
CHECK_FIELD(speed_count, 11);
CHECK_FIELD(current_count, 12);
#undef CHECK_FIELD

volatile struct ik_telemetry_wheel ik_drive_telem[4];
volatile uint32_t ik_drive_telem_now;

static uint32_t poll_tick;
static unsigned poll_index;

uint32_t ik_telemetry_poll_due(uint32_t now, int allowed)
{
  ik_drive_telem_now = now;
  /* poll_tick starts at boot tick zero. Rebase on each actual request, not
   * the old deadline, so a delayed caller never emits catch-up bursts. */
  if (!allowed || (uint32_t)(now - poll_tick) < 20U) return 0;
  const uint32_t selector = 1U << (poll_index / 2U);
  const uint32_t base = (poll_index & 1U) ? 0x300U : 0x100U;
  poll_tick = now;
  poll_index = (poll_index + 1U) % 8U;
  return base | selector;
}

static uint32_t read_u16(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8);
}

static int32_t read_s16(const uint8_t *data)
{
  const uint32_t bits = read_u16(data);
  return bits & 0x8000U ? (int32_t)bits - 65536 : (int32_t)bits;
}

int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now)
{
  const uint32_t base = id & ~0xFU;
  if (data == NULL || id > 0x7FFU ||
      (base != 0x280U && base != 0x380U)) return 0;

  unsigned index;
  switch (id & 0xFU) {
    case 1U: index = 0; break;
    case 2U: index = 1; break;
    case 4U: index = 2; break;
    case 8U: index = 3; break;
    default: return 0;
  }

  /* Called serially by the main-loop RX consumer. Every published field is
   * volatile; bracket its stores with odd/in-progress and even/stable words.
   * Only the matching group is refreshed, with no scaling or validity claim. */
  volatile struct ik_telemetry_wheel *row = &ik_drive_telem[index];
  ++row->generation;
  /* Publish odd before payload and payload before even on the MCU bus, not
   * merely in compiler order, for the external debugger's seqlock reads. */
  atomic_thread_fence(memory_order_seq_cst);
  if (base == 0x280U) {
    row->rpm = read_s16(data);
    row->state = data[2];
    row->hall = data[3];
    row->hall_changes = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                        ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
    row->speed_tick = now;
    ++row->speed_count;
  } else {
    row->iq_counts = read_s16(data);
    row->id_counts = read_s16(data + 2);
    row->flags = read_u16(data + 4);
    row->peak_l1_counts = read_u16(data + 6);
    row->current_tick = now;
    ++row->current_count;
  }
  atomic_thread_fence(memory_order_seq_cst);
  ++row->generation;
  return 1;
}
