#include "fwu_updater.h"
#include "boot_entry.h"
#include "can_fwu_boot.h"
#include "can_tx_status.h"
#include "check.h"
#include "fwu_cases.h"
#include <stdlib.h>
#include <string.h>
const uint8_t fwu_test_image[64] = {0};
const uint32_t fwu_test_image_size = 64U, fwu_test_image_crc = 0U;
static uint32_t clock_now; static unsigned sends; static uint32_t sent_id; static uint8_t sent[8];
static uint32_t fake_clock(void) { return clock_now; }
static int send(uint32_t id, const uint8_t d[8]) { sent_id = id; for (int i = 0; i < 8; ++i) sent[i] = d[i]; ++sends; return 1; }
static int poll(void) { return CAN_TX_OK; }
static void abort_tx(void) {}
static void tick(uint32_t now, int button) { clock_now = now; fwu_updater_tick(now, button); }
static uint8_t esc_flash[CAN_FWU_APP_MAX];
static int esc_erase(uint32_t page, uint32_t n, void *ctx) {
  (void)ctx; memset(&esc_flash[page * CAN_FWU_PAGE_SIZE], 0xFF, n * CAN_FWU_PAGE_SIZE); return 0;
}
static int esc_program(uint32_t off, uint64_t dw, void *ctx) {
  (void)ctx; for (int k = 0; k < 8; ++k) esc_flash[off + k] = (uint8_t)(dw >> (8 * k)); return 0;
}
static int esc_stopped(void *ctx) { (void)ctx; return 1; }
static void test_idle_without_request(void)
{
  /* nothing without a request */
  for (uint32_t t = 0; t < 10000; t += 10) tick(t, 0);
  CHECK(sends == 0 && fwu_host.state == FWU_H_IDLE && can_fwu_result == 0);
}

static void test_enter_retries_until_answered(void)
{
  const uint32_t enter = can_fwu_id(CAN_FWU_ENTER_BASE, 1);
  /* ENTER retried until answered, unbounded */
  can_fwu_request = 1; tick(1000, 0);
  CHECK(sends == 1 && sent_id == enter && can_fwu_result == 1);
  for (uint32_t t = 1001; t < 4000; ++t) tick(t, 0);
  CHECK(sends >= 9 && can_fwu_result == 1); /* ENTER times out at 100 ms, retries each 300 ms */
}

static void test_failure_latches_until_new_request(void)
{
  /* failure after ENTER latches; new request recovers */
  can_fwu_request = 1; tick(1000, 0); CHECK(sends == 1);
  fwu_host.state = FWU_H_FAULT; fwu_host.tx_id = can_fwu_id(CAN_FWU_DATA_BASE, 1);
  tick(2000, 0); CHECK(can_fwu_result == 4);
  for (uint32_t t = 2001; t < 12000; t += 10) tick(t, 0);
  CHECK(sends == 1); /* never silently retries an erased/written session */
  can_fwu_request = 1; tick(12000, 0);
  CHECK(sends == 2 && fwu_host.state == FWU_H_ENTER_SENT);
}

static void test_boot_entry_request(void)
{
  /* boot-entry request */
  can_fwu_enter_boot = 1; tick(100, 0);
  CHECK(sends == 1 && can_fwu_boot_result == 1 && esc_boot_request_valid(sent_id, sent, 1));
  CHECK(fwu_host.state == FWU_H_IDLE);
  can_fwu_dev = 3; can_fwu_enter_boot = 1; tick(200, 0);
  CHECK(sends == 1 && can_fwu_boot_result == 2);
}

static void test_fresh_clock_after_begin(void)
{
  /* frames are stamped with the clock read after begin() */
  can_fwu_request = 1; clock_now = 1191; fwu_updater_tick(1000, 0);
  CHECK(fwu_host.tx_tick == 1191U);
}

static void test_button_hold_arms(void)
{
  const uint32_t enter = can_fwu_id(CAN_FWU_ENTER_BASE, 1);
  /* B1 held 3 s from power-up arms once; early release never */
  for (uint32_t t = 0; t <= 3000; t += 10) tick(t, 1);
  CHECK(sends == 1 && sent_id == enter);
}

static void test_button_released_early_never_arms(void)
{
  for (uint32_t t = 0; t < 2000; t += 10) tick(t, 1);
  for (uint32_t t = 2000; t < 9000; t += 10) tick(t, t > 2500);
  CHECK(sends == 0);
}

static void test_completed_transfer_does_not_restart(void)
{
  /* full transfer longer than the ENTER retry period, ACKs
     after the tick as in main.c: DONE must latch */
  const struct can_fwu_flash_ops ops = { esc_erase, esc_program, esc_stopped, 0 };
  struct can_fwu_boot esc;
  can_fwu_boot_init(&esc, 1, &ops);
  can_fwu_request = 1;
  unsigned seen = 0, done_sends = 0;
  uint32_t ack_id = 0, ack_due = 0; uint8_t ack[8]; int ack_pending = 0;
  for (uint32_t t = 1000; t < 20000; ++t) {
    tick(t, 0);
    CHECK(sends - seen <= 1);
    if (sends != seen && esc.state != FWU_B_JUMP_PENDING) { /* after JUMP the app ignores FWU */
      ack_pending = can_fwu_boot_frame(&esc, sent_id, sent, t, &ack_id, ack);
      ack_due = t + 25U; /* ESC answers 25 ms later: whole session > 300 ms */
    }
    seen = sends;
    if (ack_pending && t == ack_due) { ack_pending = 0; fwu_updater_rx(ack_id, ack, t); }
    if (fwu_host.state == FWU_H_DONE && !done_sends) done_sends = sends;
  }
  CHECK(esc.state == FWU_B_JUMP_PENDING && esc.received == 64);
  CHECK(fwu_host.state == FWU_H_DONE && can_fwu_result == 3);
  CHECK(done_sends && sends == done_sends); /* no ENTER after completion */
}

int main(int argc, char **argv)
{
  const struct can_fwu_transport io = { send, poll, abort_tx };
  fwu_updater_init(&io, fake_clock);
  static const struct fwu_test_case cases[] = {
    {"test_idle_without_request", test_idle_without_request},
    {"test_enter_retries_until_answered", test_enter_retries_until_answered},
    {"test_failure_latches_until_new_request", test_failure_latches_until_new_request},
    {"test_boot_entry_request", test_boot_entry_request},
    {"test_fresh_clock_after_begin", test_fresh_clock_after_begin},
    {"test_button_hold_arms", test_button_hold_arms},
    {"test_button_released_early_never_arms", test_button_released_early_never_arms},
    {"test_completed_transfer_does_not_restart", test_completed_transfer_does_not_restart},
  };
  return fwu_run_case(argc, argv, cases, sizeof(cases) / sizeof(*cases));
}
