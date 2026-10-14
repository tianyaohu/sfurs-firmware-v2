#include "wheel_bench.h"

volatile uint32_t wheel_bench_request = 0;
volatile uint32_t wheel_bench_wheel = 0;
volatile int32_t wheel_bench_direction = 0;
volatile uint32_t wheel_bench_state = BENCH_IDLE;
volatile uint32_t wheel_bench_result = 0;
volatile uint32_t wheel_bench_run_count = 0; /* Completed RUNs, diagnostics only. */
volatile uint32_t wheel_bench_last_request = 0, wheel_bench_request_count = 0;
volatile uint32_t wheel_bench_tx_count = 0, wheel_bench_tx_ack_count = 0;
volatile uint32_t wheel_bench_first_fault = 0, wheel_bench_first_fault_tick = 0;
static uint32_t selected, started, armed_tick, pending_tick;
static uint32_t last_stop, quiet_tick, stop_count;
static int32_t direction;
static uint8_t pending, pending_stop, pending_arm, faulted, quiet;
static uint32_t arm_window = 5000U, button_changed;
static uint8_t button_permission, button_released, button_level, manual_ready;

void wheel_bench_button(uint32_t now, uint8_t pressed)
{
  pressed = pressed != 0U;
  if (pressed != button_level) {
    button_level = pressed;
    button_changed = now;
  }
  /* Fixed NE/+30 diagnostic gesture, usable after USB/power-bank transfer.
   * Boot must first see a stable release. Hold >=2 s only ARMS; it never runs.
   * Repeats need the full gesture again after the cooldown; every RUN still
   * needs a fresh ARM. */
  if ((uint32_t)(now - button_changed) >= 50U && !pressed) manual_ready = 1;
  if (pressed && manual_ready && (uint32_t)(now - button_changed) >= 2000U &&
      wheel_bench_state == BENCH_IDLE && !faulted &&
      wheel_bench_request == 0U) {
    manual_ready = 0;
    wheel_bench_wheel = 1U;
    wheel_bench_direction = 1;
    wheel_bench_request = BENCH_ARM_BUTTON;
  }
  if (wheel_bench_state != BENCH_ARMED || !button_permission) {
    button_released = 0;
    return;
  }
  if ((uint32_t)(now - button_changed) < 50U) return;
  if (!pressed) button_released = 1;
  else if (button_released && wheel_bench_request == 0U) {
    /* Require a stable release after arming, then one stable press. No
     * startup/held-button start, no wheel advancement and no repeated RUN. */
    button_permission = 0;
    button_released = 0;
    manual_ready = 0;
    wheel_bench_request = BENCH_RUN;
  }
}

static void record_fault(uint32_t reason, uint32_t now)
{
  if (wheel_bench_first_fault == 0U) {
    wheel_bench_first_fault = reason;
    wheel_bench_first_fault_tick = now;
  }
  faulted = 1;
  wheel_bench_result = 5;
}

static uint32_t selection_bit(uint32_t wheel, int32_t sign)
{
  uint32_t index;
  if (sign != 1 && sign != -1) return 0;
  switch (wheel) {
    case 1: index = 0; break;
    case 2: index = 2; break;
    case 4: index = 4; break;
    case 8: index = 6; break;
    default: return 0;
  }
  return 1U << (index + (sign < 0 ? 1U : 0U));
}

static void stop(uint32_t now)
{
  if (wheel_bench_state != BENCH_STOPPING) {
    stop_count = 0;
    last_stop = now - 50U;
  }
  wheel_bench_state = BENCH_STOPPING;
}

void wheel_bench_tick(uint32_t now)
{
  uint32_t request = wheel_bench_request;
  wheel_bench_request = 0; /* Consume each trigger once, never retry a RUN. */
  if (request != 0U) {
    wheel_bench_last_request = request;
    ++wheel_bench_request_count;
  }
  if (request == BENCH_STOP) {
    if (pending && !pending_stop) {
      wheel_bench_abort();
      pending = 0;
    }
    stop(now);
  }

  if (pending) {
    int status = wheel_bench_poll();
    if (status != BENCH_TX_PENDING || (uint32_t)(now - pending_tick) >= 100U) {
      if (status == BENCH_TX_PENDING) wheel_bench_abort();
      pending = 0;
      if (status == BENCH_TX_OK) ++wheel_bench_tx_ack_count;
      if (status != BENCH_TX_OK) {
        record_fault(status == BENCH_TX_PENDING ? 3U : 2U, now);
        pending_arm = 0;
        stop(now);
      }
      if (pending_arm) {
        pending_arm = 0;
      } else if (pending_stop) {
        ++stop_count;
        last_stop = now;
        if (stop_count >= 3U) {
          wheel_bench_state = faulted ? BENCH_FAULT : BENCH_IDLE;
          wheel_bench_result = faulted ? 5U : 3U;
          quiet_tick = now;
          quiet = 1;
        }
      }
    }
  }

  if (wheel_bench_state == BENCH_RUNNING &&
      (uint32_t)(now - started) >= ((selected == 1U && direction < 0) ? 15000U : 1000U)) stop(now);
  if (wheel_bench_state == BENCH_ARMED &&
      (uint32_t)(now - armed_tick) >= arm_window) {
    wheel_bench_state = BENCH_IDLE;
    wheel_bench_result = 4;
  }

  if (request == BENCH_ARM || request == BENCH_ARM_BUTTON) {
    uint32_t bit = selection_bit(wheel_bench_wheel, wheel_bench_direction);
    if (wheel_bench_state == BENCH_IDLE && !pending && !faulted && bit &&
        (!quiet || (uint32_t)(now - quiet_tick) >= 2000U)) {
      selected = wheel_bench_wheel;
      direction = wheel_bench_direction;
      armed_tick = now;
      arm_window = request == BENCH_ARM_BUTTON ? 60000U : 5000U;
      button_permission = request == BENCH_ARM_BUTTON;
      button_released = 0;
      /* NE reverse targets the guarded RPM2 profile: transmit CAN ARM
       * (ID 0x401, RPM2 magic) so a single-cable Nucleo can arm the ESC.
       * Other selections arm locally with no bus traffic, as before. */
      if (selected == 1U && direction < 0) {
        const uint8_t arm_data[4] = {0x32U, 0x4DU, 0x50U, 0x52U};
        if (wheel_bench_send(0x401U, arm_data)) {
          ++wheel_bench_tx_count;
          pending = 1;
          pending_arm = 1;
          pending_stop = 0;
          pending_tick = now;
          wheel_bench_state = BENCH_ARMED;
          wheel_bench_result = 1;
        } else {
          wheel_bench_result = 4;
        }
      } else {
        wheel_bench_state = BENCH_ARMED;
        wheel_bench_result = 1;
      }
    } else wheel_bench_result = 4;
  } else if (request == BENCH_RUN) {
    if (wheel_bench_state == BENCH_ARMED && !pending && !faulted &&
        wheel_bench_wheel == selected && wheel_bench_direction == direction) {
      /* Count the RUN before enqueue: even a failed write may have reached the ESC. */
      ++wheel_bench_run_count;
      /* NE reverse targets the guarded -100 RPM reverse profile; other
       * wheels retain legacy +/-30 for existing tests. */
      int16_t rpm = direction > 0 ? 30 : (selected == 1U ? -100 : -30);
      uint16_t bits = (uint16_t)rpm;
      uint8_t data[4] = {(uint8_t)bits, (uint8_t)(bits >> 8), 0xF4, 0x01};
      started = now;
      if (wheel_bench_send(0x080U | selected, data)) {
        ++wheel_bench_tx_count;
        pending = 1;
        pending_stop = 0;
        pending_tick = now;
        wheel_bench_state = BENCH_RUNNING;
        wheel_bench_result = 2;
      } else {
        record_fault(1U, now);
        stop(now);
      }
    } else {
      wheel_bench_result = 4;
      if (wheel_bench_state == BENCH_ARMED) wheel_bench_state = BENCH_IDLE;
    }
  } else if (request && request != BENCH_STOP) {
    wheel_bench_result = 4;
  }

  if (wheel_bench_state == BENCH_STOPPING && !pending &&
      (uint32_t)(now - last_stop) >= 50U) {
    /* All four verified mask filters accept bitmask 0xF. Three STOP attempts
     * are permitted; drive commands are never retried. ACK proves only bus TX. */
    const uint8_t data[4] = {0};
    if (wheel_bench_send(0x78FU, data)) {
      ++wheel_bench_tx_count;
      pending = 1;
      pending_stop = 1;
      pending_tick = now;
    } else {
      record_fault(1U, now);
      ++stop_count;
      last_stop = now;
      if (stop_count >= 3U) wheel_bench_state = BENCH_FAULT;
    }
  }
}
