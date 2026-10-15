#include "ik_drive.h"
#include "rover_kinematics.h"
#include "wheel_bench.h"

volatile const uint32_t ik_drive_abi = IK_DRIVE_ABI;
volatile int32_t ik_drive_vx_mmps, ik_drive_vy_mmps, ik_drive_omega_mradps;
volatile uint32_t ik_drive_request, ik_drive_sequence, ik_drive_completed;
volatile uint32_t ik_drive_state, ik_drive_result;
volatile int32_t ik_drive_rpm[4];
volatile uint32_t ik_drive_tx_count, ik_drive_ack_count, ik_drive_fault;

static uint32_t operation, sequence, seen_sequence, pending_tick, stop_tick;
static unsigned index, stop_count;
static int pending;

static void begin_stop(uint32_t now)
{
  if (pending) wheel_bench_abort();
  pending = 0;
  operation = IK_STOP;
  stop_count = 0;
  stop_tick = now - 50U;
  ik_drive_state = IK_STOPPING;
}

static void fault(uint32_t reason, uint32_t now)
{
  if (ik_drive_fault == 0U) ik_drive_fault = reason;
  begin_stop(now);
}

static void finish(void)
{
  if (ik_drive_fault != 0U) {
    ik_drive_state = IK_FAULT;
    ik_drive_result = IK_TX_FAULT;
  } else {
    ik_drive_state = operation == IK_START ? IK_RUNNING : IK_IDLE;
    ik_drive_result = IK_SENT;
  }
  ik_drive_completed = sequence;
}

void ik_drive_tick(uint32_t now)
{
  /* The debugger publishes sequence LAST. Never clear the request mailbox:
   * a CPU read-then-clear could erase a concurrent SWD STOP write. */
  const uint32_t published = ik_drive_sequence;
  const uint32_t request = published != seen_sequence ? ik_drive_request : 0U;
  if (published != seen_sequence) seen_sequence = published;
  if (request == IK_STOP) {
    sequence = published;
    begin_stop(now);
  } else if (request != 0U) {
    const uint32_t requested_sequence = published;
    if ((request != IK_PREPARE && request != IK_START) ||
        ik_drive_state != IK_IDLE || ik_drive_fault != 0U) {
      ik_drive_result = IK_REJECTED;
      ik_drive_completed = requested_sequence;
    } else {
      float target[4];
      const int32_t vx = ik_drive_vx_mmps;
      const int32_t vy = ik_drive_vy_mmps;
      const int32_t omega = ik_drive_omega_mradps;
      if (request == IK_START &&
          (vx < -300 || vx > 300 || vy < -300 || vy > 300 ||
           omega < -1000 || omega > 1000 ||
           !rover_ik(rover_ik_provisional, vx / 1000.0f, vy / 1000.0f,
                     omega / 1000.0f, target, 0))) {
        ik_drive_result = IK_BAD_TARGET;
        ik_drive_completed = requested_sequence;
      } else {
        sequence = requested_sequence;
        operation = request;
        index = 0;
        ik_drive_result = IK_NONE;
        ik_drive_state = IK_BUSY;
        if (request == IK_START) {
          int moving = 0;
          for (unsigned i = 0; i < 4; ++i) {
            /* No second left-side mirror; model returns raw motor RPM. */
            ik_drive_rpm[i] = (int32_t)(target[i] + (target[i] < 0 ? -0.5f : 0.5f));
            moving |= ik_drive_rpm[i] != 0;
          }
          if (!moving) begin_stop(now);
        }
      }
    }
  }

  if (pending) {
    const int status = wheel_bench_poll();
    if (status == BENCH_TX_PENDING && (uint32_t)(now - pending_tick) < 100U) return;
    if (status == BENCH_TX_PENDING) wheel_bench_abort();
    pending = 0;
    if (status == BENCH_TX_OK) ++ik_drive_ack_count;
    if (operation == IK_STOP) {
      if (status != BENCH_TX_OK && ik_drive_fault == 0U)
        ik_drive_fault = status == BENCH_TX_PENDING ? 3U : 2U;
      ++stop_count;
      stop_tick = now;
    } else if (status != BENCH_TX_OK) {
      fault(status == BENCH_TX_PENDING ? 3U : 2U, now);
    } else {
      ++index;
    }
  }

  if (operation == IK_STOP && ik_drive_state == IK_STOPPING) {
    if (stop_count >= 3U) {
      /* Existing ESC STOP permanency is 400 ms. Match the bench's extra
       * 250 ms re-arm quiet after that, measured from the final STOP ACK. */
      if (ik_drive_fault == 0U && (uint32_t)(now - stop_tick) < 650U) return;
      for (unsigned i = 0; i < 4; ++i) ik_drive_rpm[i] = 0;
      finish();
      return;
    }
    if ((uint32_t)(now - stop_tick) < 50U) return;
    const uint8_t zero[4] = {0};
    if (wheel_bench_send(0x78FU, zero)) {
      ++ik_drive_tx_count;
      pending_tick = now;
      pending = 1;
    } else {
      if (ik_drive_fault == 0U) ik_drive_fault = 1U;
      ++stop_count;
      stop_tick = now;
    }
    return;
  }

  if (ik_drive_state != IK_BUSY) return;
  /* Four ARM frames, then four SPEED frames for START. PREPARE is ARM-only.
   * Skip zero speed targets: the guarded ESC rejects a zero START. */
  while (index >= 4U && index < 8U && ik_drive_rpm[index - 4U] == 0) ++index;
  if (index >= (operation == IK_PREPARE ? 4U : 8U)) {
    finish();
    return;
  }
  const uint32_t selector = 1U << (index % 4U);
  uint32_t id = 0x400U | selector;
  uint8_t data[4] = {0x32, 0x4D, 0x50, 0x52};
  if (index >= 4U) {
    const uint16_t rpm = (uint16_t)(int16_t)ik_drive_rpm[index - 4U];
    id = 0x080U | selector;
    data[0] = (uint8_t)rpm;
    data[1] = (uint8_t)(rpm >> 8);
    data[2] = 0xF4;
    data[3] = 0x01;
  }
  if (!wheel_bench_send(id, data)) {
    fault(1U, now);
    return;
  }
  ++ik_drive_tx_count;
  pending_tick = now;
  pending = 1;
}
