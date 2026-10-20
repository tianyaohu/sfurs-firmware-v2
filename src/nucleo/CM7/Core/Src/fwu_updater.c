#include "fwu_updater.h"
#include "boot_entry.h"
#include "can_tx_status.h"
#include "fwu_test_image.h"

#define ENTER_RETRY_MS 300U
#define BOOT_REQUEST_TIMEOUT_MS 100U
#define BUTTON_HOLD_MS 3000U

volatile uint32_t can_fwu_request, can_fwu_dev = 1U, can_fwu_result;
volatile uint32_t can_fwu_enter_boot, can_fwu_boot_result, can_fwu_rx_cnt;
struct can_fwu_host fwu_host;

static struct can_fwu_transport io;
static uint32_t (*clock_ms)(void);
static int armed, fresh, boot_pending, gesture_done, hold_timing;
static uint32_t last_try, boot_tick, hold_start;

static int host_idle(void)
{
  return fwu_host.state == FWU_H_IDLE || fwu_host.state == FWU_H_DONE ||
         fwu_host.state == FWU_H_FAULT;
}

static int enter_unanswered(void)
{
  return fwu_host.state == FWU_H_FAULT &&
         fwu_host.tx_id == can_fwu_id(CAN_FWU_ENTER_BASE, can_fwu_dev);
}

static void arm(void)
{
  armed = 1;
  fresh = 1;
  last_try = 0U;
}

void fwu_updater_init(const struct can_fwu_transport *transport, uint32_t (*clock)(void))
{
  io = *transport;
  clock_ms = clock;
  can_fwu_host_init(&fwu_host, transport);
}

int fwu_updater_active(void)
{
  return !host_idle();
}

static void boot_request_tick(uint32_t now)
{
  if (can_fwu_enter_boot != 0U) {
    uint8_t payload[8];
    can_fwu_enter_boot = 0U;
    if (!armed && !boot_pending && host_idle() && esc_boot_request_pack(payload, can_fwu_dev)) {
      if (io.send(ESC_BOOT_REQUEST_BASE | can_fwu_dev, payload)) {
        boot_pending = 1;
        boot_tick = now;
        can_fwu_boot_result = 1U;
      } else {
        can_fwu_boot_result = 3U;
      }
    } else {
      can_fwu_boot_result = 2U;
    }
  }
  if (boot_pending) {
    const int status = io.poll();
    if (status == CAN_TX_FAILED || (uint32_t)(now - boot_tick) >= BOOT_REQUEST_TIMEOUT_MS) {
      io.abort();
      boot_pending = 0;
      can_fwu_boot_result = 3U;
    } else if (status == CAN_TX_OK) {
      boot_pending = 0;
    }
  }
}

static void request_tick(uint32_t now, int button_held)
{
  if (can_fwu_request != 0U) {
    can_fwu_request = 0U;
    if (!boot_pending && host_idle()) arm();
    else can_fwu_result = 2U;
  }
  /* B1 held continuously from power-up for 3 s arms one update. */
  if (!gesture_done) {
    if (!button_held) {
      gesture_done = 1;
    } else if (!hold_timing) {
      hold_timing = 1;
      hold_start = now;
    } else if ((uint32_t)(now - hold_start) >= BUTTON_HOLD_MS && fwu_host.state == FWU_H_IDLE && !armed) {
      arm();
      gesture_done = 1;
    }
  }
}

void fwu_updater_tick(uint32_t now, int button_held)
{
  boot_request_tick(now);
  request_tick(now, button_held);
  /* ENTER retries are unbounded: the bootloader answers ENTER from any state.
   * Only a new request or an unanswered ENTER starts a session; a DONE session
   * must not restart (its ACK arrives in fwu_updater_rx, after this tick). */
  if (armed && (fresh || enter_unanswered()) &&
      (uint32_t)(now - last_try) >= ENTER_RETRY_MS) {
    last_try = now;
    fresh = 0;
    if (can_fwu_host_begin(&fwu_host, can_fwu_dev, fwu_test_image, FWU_TEST_IMAGE_SIZE, 1, now)) {
      can_fwu_result = 1U;
    } else {
      armed = 0;
      can_fwu_result = 2U;
    }
  }
  if (armed && fwu_host.state == FWU_H_FAULT && !enter_unanswered()) {
    armed = 0; /* progress was made, then lost: needs a new request */
    can_fwu_result = 4U;
  }
  if (!host_idle()) {
    /* begin() CRCs the image synchronously; stamp frames with a fresh clock. */
    can_fwu_host_poll(&fwu_host, clock_ms());
    can_fwu_result = can_fwu_host_done(&fwu_host) ? 3U : 1U;
    if (fwu_host.state == FWU_H_FAULT) {
      if (enter_unanswered()) {
        can_fwu_result = 1U; /* keep armed; ENTER retries above */
      } else {
        armed = 0;
        can_fwu_result = 4U;
      }
    }
  }
  if (can_fwu_host_done(&fwu_host)) {
    armed = 0;
    can_fwu_result = 3U;
  }
}

void fwu_updater_rx(uint32_t id, const uint8_t data[8], uint32_t now)
{
  if ((id & ~0xFU) != CAN_FWU_ACK_BASE) return;
  ++can_fwu_rx_cnt;
  can_fwu_host_ack(&fwu_host, id, data, now);
}
