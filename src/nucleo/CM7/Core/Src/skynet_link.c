#include "skynet_link.h"
#include "ik_drive.h"
#include <stdatomic.h>
#include <string.h>

_Static_assert((SKYNET_LINK_RING & (SKYNET_LINK_RING - 1U)) == 0U, "ring size is a power of two");
_Static_assert(SKYNET_LINK_TIMEOUT_MS == IK_DRIVE_HEARTBEAT_MS, "link and drive give up together");
_Static_assert(SKYNET_LINK_GAP_MS * SKYNET_LINK_BAUD / 10000U > SKYNET_LINK_MAX_ROBOTS * SKYNET_LINK_RECORD,
               "a full batch fits well inside the gap");

volatile const uint32_t skynet_link_abi = SKYNET_LINK_ABI;
volatile uint32_t skynet_link_robot_id, skynet_link_ack_seq;
volatile uint32_t skynet_link_bytes, skynet_link_records, skynet_link_batches, skynet_link_commands;
volatile uint32_t skynet_link_skipped, skynet_link_broken, skynet_link_stale, skynet_link_overruns;
volatile uint32_t skynet_link_refused, skynet_link_active, skynet_link_fault, skynet_link_last_ms;

static uint8_t ring_b[SKYNET_LINK_RING];
static uint32_t ring_t[SKYNET_LINK_RING];
static volatile uint32_t ring_head, ring_tail; /* head: ISR only; tail: main loop only */

/* Parser state; main loop only. */
static uint8_t window[SKYNET_LINK_RECORD];
static uint32_t window_len, last_byte_t;
static int have_byte;
static int in_batch;                 /* a valid run that started at ID 0 is open */
static uint32_t next_id;             /* ID the run expects next */
static int have_cmd;                 /* this robot's record is in the open run */
static int32_t cmd_vx, cmd_vy, cmd_w;
static uint32_t last_ack_seq;
static int last_ack_input;

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

void skynet_link_isr_byte(uint8_t b, uint32_t now)
{
  const uint32_t head = ring_head;
  if (head - ring_tail >= SKYNET_LINK_RING) { ++skynet_link_overruns; return; }
  ring_b[head & (SKYNET_LINK_RING - 1U)] = b;
  ring_t[head & (SKYNET_LINK_RING - 1U)] = now;
  atomic_thread_fence(memory_order_release);
  ring_head = head + 1U;
}

void skynet_link_isr_overrun(void) { ++skynet_link_overruns; }

/* Elapsed ms from a to b; the ISR may stamp a tick newer than the main loop's `now`. */
static uint32_t since(uint32_t a, uint32_t b)
{
  const int32_t d = (int32_t)(b - a);
  return d > 0 ? (uint32_t)d : 0U;
}

/* int8 at 127 = full scale to mm/s or mrad/s, rounded half away from zero like skynet's lround. */
static int32_t scaled_milli(int8_t v)
{
  const int32_t num = (int32_t)v * (int32_t)(SKYNET_LINK_FULL_SCALE * 1000.0f);
  return (num >= 0 ? num + 63 : num - 63) / 127;
}

static int32_t clamp(int32_t v, int32_t limit) { return v > limit ? limit : v < -limit ? -limit : v; }

static void zero_twist(void)
{
  ik_drive_vx_mmps = 0;
  ik_drive_vy_mmps = 0;
  ik_drive_omega_mradps = 0;
}

static void end_batch(int broken)
{
  if (broken && in_batch) ++skynet_link_broken;
  in_batch = 0;
  have_cmd = 0;
}

static void reset_parser(int broken)
{
  window_len = 0U;
  end_batch(broken);
}

static void apply(uint32_t now)
{
  if (skynet_link_fault != 0U) return; /* stopped until the operator acknowledges */
  /* Take over only an idle speed-mode drive: never a current-mode diagnostic or another host's session. */
  if (ik_drive_mode != IK_MODE_SPEED || (!skynet_link_active && ik_drive_state != IK_STOPPED)) {
    ++skynet_link_refused;
    return;
  }
  ik_drive_vx_mmps = cmd_vx;
  ik_drive_vy_mmps = cmd_vy;
  ik_drive_omega_mradps = cmd_w;
  ik_drive_enable = 1U;
  ++ik_drive_heartbeat;
  ++skynet_link_commands;
  skynet_link_active = 1U;
  skynet_link_last_ms = now;
}

static void close_batch(uint32_t now)
{
  if (!in_batch) return;
  ++skynet_link_batches;
  if (have_cmd) apply(now);
  end_batch(0);
}

static int valid_record(const uint8_t r[SKYNET_LINK_RECORD])
{
  if ((r[2] & 0xE0U) != 0U || (r[3] & 0xE0U) != 0U) return 0;
  return skynet_link_crc16(&r[2], 5U) == (uint16_t)(r[0] | (r[1] << 8));
}

static void record(const uint8_t r[SKYNET_LINK_RECORD], uint32_t now)
{
  ++skynet_link_records;
  const uint32_t id = r[2] & 0x0FU;
  if (id == 0U) {
    close_batch(now); /* the next batch's first record frames the previous one */
    in_batch = 1;
    next_id = 0U;
  } else if (!in_batch || id != next_id) {
    end_batch(1);
    return;
  }
  next_id = id + 1U;
  if (id == skynet_link_robot_id) {
    cmd_vx = clamp(-scaled_milli((int8_t)r[5]), IK_DRIVE_MAX_LINEAR_MMPS);
    cmd_vy = clamp(scaled_milli((int8_t)r[4]), IK_DRIVE_MAX_LINEAR_MMPS);
    cmd_w = clamp(scaled_milli((int8_t)r[6]), IK_DRIVE_MAX_OMEGA_MRADPS);
    have_cmd = 1;
  }
}

static void byte_in(uint8_t b, uint32_t t, uint32_t now)
{
  ++skynet_link_bytes;
  if (since(t, now) >= SKYNET_LINK_TIMEOUT_MS) { /* queued too long: a command this old must not move us */
    ++skynet_link_stale;
    reset_parser(1);
    have_byte = 0;
    return;
  }
  if (have_byte && since(last_byte_t, t) >= SKYNET_LINK_GAP_MS) {
    if (window_len == 0U) close_batch(now); /* silence on a record boundary ends the batch */
    else reset_parser(1);                   /* a record cut by silence is never completed by later bytes */
  }
  have_byte = 1;
  last_byte_t = t;
  window[window_len++] = b;
  if (window_len < SKYNET_LINK_RECORD) return;
  if (valid_record(window)) {
    record(window, now);
    window_len = 0U;
  } else {
    memmove(window, window + 1, SKYNET_LINK_RECORD - 1U);
    window_len = SKYNET_LINK_RECORD - 1U;
    ++skynet_link_skipped;
    end_batch(1);
  }
}

static void supervise(uint32_t now, int ack)
{
  const uint32_t ack_seq = skynet_link_ack_seq;
  const int ack_edge = (ack && !last_ack_input) || ack_seq != last_ack_seq;
  last_ack_input = ack;
  last_ack_seq = ack_seq;
  if (skynet_link_active && ik_drive_fault != 0U) {
    /* Leave enable at 1 so ik_drive keeps the fault latched and visible; only an acknowledge clears it. */
    skynet_link_fault = ik_drive_fault;
    skynet_link_active = 0U;
    zero_twist();
  }
  if (ack_edge && skynet_link_fault != 0U) {
    skynet_link_fault = 0U;
    zero_twist();
    ik_drive_enable = 0U;  /* acknowledges ik_drive_fault */
    ++ik_drive_clear_seq;  /* and the ESCs' latches, once the drive is stopped */
  }
  if (skynet_link_active && since(skynet_link_last_ms, now) >= SKYNET_LINK_TIMEOUT_MS) {
    zero_twist();
    ik_drive_enable = 0U;
    skynet_link_active = 0U;
  }
}

void skynet_link_poll(uint32_t now, int ack)
{
  const uint32_t head = ring_head;
  atomic_thread_fence(memory_order_acquire);
  for (uint32_t tail = ring_tail; tail != head; ++tail)
    byte_in(ring_b[tail & (SKYNET_LINK_RING - 1U)], ring_t[tail & (SKYNET_LINK_RING - 1U)], now);
  atomic_thread_fence(memory_order_release);
  ring_tail = head;
  if (have_byte && window_len == 0U && since(last_byte_t, now) >= SKYNET_LINK_GAP_MS) close_batch(now);
  if (have_byte && window_len != 0U && since(last_byte_t, now) >= SKYNET_LINK_GAP_MS) reset_parser(1);
  supervise(now, ack);
}
