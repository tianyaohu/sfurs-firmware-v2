#include "wheel_bench.h"

volatile uint32_t wheel_bench_request = 0;
volatile uint32_t wheel_bench_wheel = 0;
volatile int32_t wheel_bench_direction = 0;
volatile int32_t wheel_bench_tune_kp = -18600, wheel_bench_tune_ki = -72;
volatile uint32_t wheel_bench_state = BENCH_IDLE;
volatile uint32_t wheel_bench_result = 0;
volatile uint32_t wheel_bench_run_count = 0; /* Completed RUNs, diagnostics only. */
volatile int32_t wheel_bench_speed_rpm = 0; /* 0 = per-selection default. */
volatile uint32_t wheel_bench_last_request = 0, wheel_bench_request_count = 0;
volatile uint32_t wheel_bench_tx_count = 0, wheel_bench_tx_ack_count = 0;
volatile uint32_t wheel_bench_first_fault = 0, wheel_bench_first_fault_tick = 0;
/* Tier-1 ESC mirror. See header. 0xFFFFFFFF age = no reply stored yet. */
volatile int32_t wheel_bench_esc_rpm = 0;
volatile uint32_t wheel_bench_esc_state = 0, wheel_bench_esc_age_ms = 0xFFFFFFFFU;
static uint32_t esc_last_tick, esc_poll_tick;
static uint8_t esc_valid;
static uint32_t selected, armed_tick, pending_tick;
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

void wheel_bench_esc_rx(uint32_t id, const uint8_t data[8], uint32_t now)
{
  uint32_t wheel = id & 0x0FU;
  uint16_t bits;
  /* Only the RPM2-profile wheels answer today; store only the currently
   * selected wheel so the single slot never shows another wheel's data.
   * The TUI displays this slot only for the selected wheel. */
  if ((wheel != 1U && wheel != 2U) || wheel != wheel_bench_wheel) return;
  bits = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  wheel_bench_esc_rpm = (bits & 0x8000U) != 0U ? (int32_t)bits - 65536 : (int32_t)bits;
  wheel_bench_esc_state = data[2];
  esc_last_tick = now;
  esc_valid = 1;
  wheel_bench_esc_age_ms = 0;
}

uint32_t wheel_bench_esc_poll_due(uint32_t now)
{
  uint32_t wheel = wheel_bench_wheel;
  if (wheel != 1U && wheel != 2U) return 0;
  /* Never during STOPPING: the STOP broadcast owns the bus then. FAULT is
   * fine (no drive TX can be in flight that a poll could delay). */
  if (wheel_bench_state == BENCH_STOPPING) return 0;
  if (pending) return 0;
  if (esc_poll_tick != 0U) {
    if ((uint32_t)(now - esc_poll_tick) < 500U) return 0;
  } else if (now < 500U) return 0; /* First poll 500 ms after boot. */
  esc_poll_tick = now;
  return 0x100U | wheel;
}

void wheel_bench_tick(uint32_t now)
{
  uint32_t request = wheel_bench_request;
  /* Age the ESC mirror every pass, including STOPPING/FAULT drains. */
  if (!esc_valid) wheel_bench_esc_age_ms = 0xFFFFFFFFU;
  else {
    uint32_t age = now - esc_last_tick;
    wheel_bench_esc_age_ms = age > 0xFFFFFFFEU ? 0xFFFFFFFEU : age;
  }
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

  /* No host RUN duration cap. Operator STOP and ESC safety guards still apply. */
  if (wheel_bench_state == BENCH_ARMED &&
      (uint32_t)(now - armed_tick) >= arm_window) {
    wheel_bench_state = BENCH_IDLE;
    wheel_bench_result = 4;
  }

  if (request == BENCH_ARM || request == BENCH_ARM_BUTTON) {
    uint32_t bit = selection_bit(wheel_bench_wheel, wheel_bench_direction);
    if (wheel_bench_state == BENCH_IDLE && !pending && !faulted && bit &&
        /* 250 ms re-ARM quiet (was 500). The 3-frame STOP drain (~150 ms)
         * completes before quiet starts, so no ARM/STOP race; the ESC still
         * demands standstill/IDLE before RUN, now visible in the esc row. */
        (!quiet || (uint32_t)(now - quiet_tick) >= 250U)) {
      selected = wheel_bench_wheel;
      direction = wheel_bench_direction;
      armed_tick = now;
      arm_window = request == BENCH_ARM_BUTTON ? 60000U : 5000U;
      button_permission = request == BENCH_ARM_BUTTON;
      button_released = 0;
      /* All wheels target guarded RPM2 profiles in either direction: transmit
       * CAN ARM (ID 0x401/0x402/0x404/0x408, RPM2 magic) so a single-cable
       * Nucleo can arm the ESC. Selections are pre-gated by selection_bit. */
      {
        const uint8_t arm_data[4] = {0x32U, 0x4DU, 0x50U, 0x52U};
        if (wheel_bench_send(0x400U | selected, arm_data)) {
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
      }
    } else wheel_bench_result = 4;
  } else if (request == BENCH_RUN) {
    if (wheel_bench_state == BENCH_ARMED && !pending && !faulted &&
        wheel_bench_wheel == selected && wheel_bench_direction == direction) {
      /* Count the RUN before enqueue: even a failed write may have reached the ESC. */
      ++wheel_bench_run_count;
      /* NE follows the speed setting over the ESC +-300 RPM range
       * (0 keeps direction defaults: -100 reverse, +30 forward).
       * SE/SW/NW use the setting over 10..150 (0 keeps +30/-30). */
      int16_t rpm;
      if (selected == 1U)
      {
        int32_t mag = wheel_bench_speed_rpm;
        if (direction < 0) mag = mag == 0 ? 100 : mag;
        else mag = mag == 0 ? 30 : mag;
        if (mag < 30) mag = 30;
        if (mag > 300) mag = 300;
        rpm = direction > 0 ? (int16_t)mag : (int16_t)-mag;
      }
      else
      {
        int32_t mag = wheel_bench_speed_rpm == 0 ? 30 : wheel_bench_speed_rpm;
        if (mag < 10) mag = 10;
        if (mag > 150) mag = 150;
        rpm = direction > 0 ? (int16_t)mag : (int16_t)-mag;
      }
      /* Rover-frame convention: + means rover-forward on every wheel. The
       * left-side wheels (SW/NW, mirrored mounts) spin the same shaft
       * direction as the right side for a given command, which rolls the
       * rover the opposite way — so mirror their command sign here. ESC
       * images stay identical across wheels; direct (STLINK) pulses remain
       * shaft-frame and are NOT mirrored. */
      if (selected == 4U || selected == 8U) rpm = (int16_t)-rpm;
      uint16_t bits = (uint16_t)rpm;
      uint8_t data[4] = {(uint8_t)bits, (uint8_t)(bits >> 8), 0xF4, 0x01};
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
  } else if (request == BENCH_TUNE) {
    /* Auto-tune GAIN_SET (0x500|dev, TEST_GAIN_SET): RPM2 speed-PI numerators,
     * int16 LE kp then ki, both negative. Sent only while ARMED with the
     * armed selection unchanged; the 5 s armed window is NOT refreshed (the
     * ESC counts it from ARM). ACK proves only bus TX; ESC acceptance is read
     * back from ESC RAM (test_gain_set_result / test_speed_kp_gain). */
    if (wheel_bench_state == BENCH_ARMED && !pending && !faulted &&
        wheel_bench_wheel == selected && wheel_bench_direction == direction &&
        wheel_bench_tune_kp >= -32767 && wheel_bench_tune_kp <= -1000 &&
        wheel_bench_tune_ki >= -32767 && wheel_bench_tune_ki <= -1) {
      uint32_t packed = ((uint32_t)(uint16_t)(int16_t)wheel_bench_tune_ki << 16) |
                        (uint32_t)(uint16_t)(int16_t)wheel_bench_tune_kp;
      const uint8_t data[4] = {(uint8_t)packed, (uint8_t)(packed >> 8),
                               (uint8_t)(packed >> 16), (uint8_t)(packed >> 24)};
      if (wheel_bench_send(0x500U | selected, data)) {
        ++wheel_bench_tx_count;
        pending = 1;
        pending_arm = 0;
        pending_stop = 0;
        pending_tick = now;
        wheel_bench_result = 6;
      } else {
        record_fault(1U, now);
        stop(now);
      }
    } else {
      wheel_bench_result = 4;
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
