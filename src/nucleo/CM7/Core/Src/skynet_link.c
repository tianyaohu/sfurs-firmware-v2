#include "skynet_link.h"
#include "ik_drive.h"
#include <stdatomic.h>
#include <string.h>

_Static_assert((SKYNET_LINK_RING & (SKYNET_LINK_RING - 1U)) == 0U, "ring size is a power of two");
_Static_assert(SKYNET_LINK_TIMEOUT_MS == IK_DRIVE_HEARTBEAT_MS, "link and drive give up together");

volatile const uint32_t skynet_link_abi = SKYNET_LINK_ABI;
volatile uint32_t skynet_link_robot_id;
volatile uint32_t skynet_link_bytes, skynet_link_records, skynet_link_commands, skynet_link_skipped;
volatile uint32_t skynet_link_overruns, skynet_link_active, skynet_link_last_ms;

static uint8_t ring[SKYNET_LINK_RING];
static volatile uint32_t ring_head, ring_tail; /* head: ISR only; tail: main loop only */

static uint8_t window[SKYNET_LINK_RECORD];
static uint32_t window_len;

uint16_t skynet_link_crc16(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)(data[i] << 8);
    for (unsigned bit = 0; bit < 8U; ++bit)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}

void skynet_link_isr_byte(uint8_t b)
{
  const uint32_t head = ring_head;
  if (head - ring_tail >= SKYNET_LINK_RING) { ++skynet_link_overruns; return; }
  ring[head & (SKYNET_LINK_RING - 1U)] = b;
  atomic_thread_fence(memory_order_release);
  ring_head = head + 1U;
}

void skynet_link_isr_overrun(void) { ++skynet_link_overruns; }

/* int8 at 127 = full scale to mm/s or mrad/s, rounded half away from zero like skynet's lround. */
static int32_t scaled_milli(int8_t v)
{
  const int32_t num = (int32_t)v * (int32_t)(SKYNET_LINK_FULL_SCALE * 1000.0f);
  return (num >= 0 ? num + 63 : num - 63) / 127;
}

static int32_t clamp(int32_t v, int32_t limit) { return v > limit ? limit : v < -limit ? -limit : v; }

static int valid_record(const uint8_t r[SKYNET_LINK_RECORD])
{
  /* The reserved bits roughly square the CRC's 1/65536 false-sync odds while searching. */
  if ((r[2] & 0xE0U) != 0U || (r[3] & 0xE0U) != 0U) return 0;
  return skynet_link_crc16(&r[2], 5U) == (uint16_t)(r[0] | (r[1] << 8));
}

static void apply(const uint8_t r[SKYNET_LINK_RECORD], uint32_t now)
{
  ++skynet_link_records;
  if ((uint32_t)(r[2] & 0x0FU) != skynet_link_robot_id) return;
  const int32_t forward = scaled_milli((int8_t)r[4]), left = scaled_milli((int8_t)r[5]);
  ik_drive_vx_mmps = clamp(-left, IK_DRIVE_MAX_LINEAR_MMPS);
  ik_drive_vy_mmps = clamp(forward, IK_DRIVE_MAX_LINEAR_MMPS);
  ik_drive_omega_mradps = clamp(scaled_milli((int8_t)r[6]), IK_DRIVE_MAX_OMEGA_MRADPS);
  ik_drive_enable = 1U;
  ++ik_drive_heartbeat;
  ++skynet_link_commands;
  skynet_link_active = 1U;
  skynet_link_last_ms = now;
}

void skynet_link_poll(uint32_t now)
{
  const uint32_t head = ring_head;
  atomic_thread_fence(memory_order_acquire);
  for (uint32_t tail = ring_tail; tail != head; ++tail) {
    window[window_len++] = ring[tail & (SKYNET_LINK_RING - 1U)];
    ++skynet_link_bytes;
    if (window_len < SKYNET_LINK_RECORD) continue;
    if (valid_record(window)) {
      apply(window, now);
      window_len = 0U;
    } else {
      memmove(window, window + 1, SKYNET_LINK_RECORD - 1U);
      window_len = SKYNET_LINK_RECORD - 1U;
      ++skynet_link_skipped;
    }
  }
  atomic_thread_fence(memory_order_release);
  ring_tail = head;
  if (skynet_link_active && (uint32_t)(now - skynet_link_last_ms) >= SKYNET_LINK_TIMEOUT_MS) {
    ik_drive_vx_mmps = 0;
    ik_drive_vy_mmps = 0;
    ik_drive_omega_mradps = 0;
    ik_drive_enable = 0U;
    skynet_link_active = 0U;
  }
}
