#!/usr/bin/env python3
"""Real Nucleo updater controller (fwu_updater.c) + real FWU host, fake CAN."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "fwu_updater.h"
#include "boot_entry.h"
#include "can_fwu_boot.h"
#include "can_tx_status.h"
#include <assert.h>
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
int main(int argc, char **argv) {
  assert(argc == 2);
  const struct can_fwu_transport io = { send, poll, abort_tx };
  fwu_updater_init(&io, fake_clock);
  const int scenario = atoi(argv[1]);
  const uint32_t enter = can_fwu_id(CAN_FWU_ENTER_BASE, 1);
  if (scenario == 0) { /* nothing without a request */
    for (uint32_t t = 0; t < 10000; t += 10) tick(t, 0);
    assert(sends == 0 && fwu_host.state == FWU_H_IDLE && can_fwu_result == 0);
  } else if (scenario == 1) { /* ENTER retried until answered, unbounded */
    can_fwu_request = 1; tick(1000, 0);
    assert(sends == 1 && sent_id == enter && can_fwu_result == 1);
    for (uint32_t t = 1001; t < 4000; ++t) tick(t, 0);
    assert(sends >= 9 && can_fwu_result == 1); /* ENTER times out at 100 ms, retries each 300 ms */
  } else if (scenario == 2) { /* failure after ENTER latches; new request recovers */
    can_fwu_request = 1; tick(1000, 0); assert(sends == 1);
    fwu_host.state = FWU_H_FAULT; fwu_host.tx_id = can_fwu_id(CAN_FWU_DATA_BASE, 1);
    tick(2000, 0); assert(can_fwu_result == 4);
    for (uint32_t t = 2001; t < 12000; t += 10) tick(t, 0);
    assert(sends == 1); /* never silently retries an erased/written session */
    can_fwu_request = 1; tick(12000, 0);
    assert(sends == 2 && fwu_host.state == FWU_H_ENTER_SENT);
  } else if (scenario == 3) { /* boot-entry request */
    can_fwu_enter_boot = 1; tick(100, 0);
    assert(sends == 1 && can_fwu_boot_result == 1 && esc_boot_request_valid(sent_id, sent, 1));
    assert(fwu_host.state == FWU_H_IDLE);
    can_fwu_dev = 3; can_fwu_enter_boot = 1; tick(200, 0);
    assert(sends == 1 && can_fwu_boot_result == 2);
  } else if (scenario == 4) { /* frames are stamped with the clock read after begin() */
    can_fwu_request = 1; clock_now = 1191; fwu_updater_tick(1000, 0);
    assert(fwu_host.tx_tick == 1191U);
  } else if (scenario == 5) { /* B1 held 3 s from power-up arms once; early release never */
    for (uint32_t t = 0; t <= 3000; t += 10) tick(t, 1);
    assert(sends == 1 && sent_id == enter);
  } else if (scenario == 6) {
    for (uint32_t t = 0; t < 2000; t += 10) tick(t, 1);
    for (uint32_t t = 2000; t < 9000; t += 10) tick(t, t > 2500);
    assert(sends == 0);
  } else if (scenario == 7) { /* full transfer longer than the ENTER retry period, ACKs
                                 after the tick as in main.c: DONE must latch */
    const struct can_fwu_flash_ops ops = { esc_erase, esc_program, esc_stopped, 0 };
    struct can_fwu_boot esc;
    can_fwu_boot_init(&esc, 1, &ops);
    can_fwu_request = 1;
    unsigned seen = 0, done_sends = 0;
    uint32_t ack_id = 0, ack_due = 0; uint8_t ack[8]; int ack_pending = 0;
    for (uint32_t t = 1000; t < 20000; ++t) {
      tick(t, 0);
      assert(sends - seen <= 1);
      if (sends != seen && esc.state != FWU_B_JUMP_PENDING) { /* after JUMP the app ignores FWU */
        ack_pending = can_fwu_boot_frame(&esc, sent_id, sent, t, &ack_id, ack);
        ack_due = t + 25U; /* ESC answers 25 ms later: whole session > 300 ms */
      }
      seen = sends;
      if (ack_pending && t == ack_due) { ack_pending = 0; fwu_updater_rx(ack_id, ack, t); }
      if (fwu_host.state == FWU_H_DONE && !done_sends) done_sends = sends;
    }
    assert(esc.state == FWU_B_JUMP_PENDING && esc.received == 64);
    assert(fwu_host.state == FWU_H_DONE && can_fwu_result == 3);
    assert(done_sends && sends == done_sends); /* no ENTER after completion */
  }
  return 0;
}
'''


class FwuUpdaterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = pathlib.Path(cls.tmp.name)
        (tmp / "t.c").write_text(HARNESS)
        cls.exe = tmp / "t"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "Common/Inc"), "-I", str(ROOT / "CM7/Core/Inc"),
                        str(tmp / "t.c"), str(ROOT / "CM7/Core/Src/fwu_updater.c"),
                        str(ROOT / "CM7/Core/Src/can_fwu_host.c"),
                        str(ROOT / "Common/Src/can_fwu_boot.c"),
                        str(ROOT / "Common/Src/can_fwu_protocol.c"), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def check(self, n):
        r = subprocess.run([str(self.exe), str(n)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_idle_without_request(self): self.check(0)
    def test_enter_retries_until_answered(self): self.check(1)
    def test_failure_latches_until_new_request(self): self.check(2)
    def test_boot_entry_request(self): self.check(3)
    def test_fresh_clock_after_begin(self): self.check(4)
    def test_button_hold_arms(self): self.check(5)
    def test_button_released_early_never_arms(self): self.check(6)
    def test_completed_transfer_does_not_restart(self): self.check(7)


if __name__ == "__main__":
    unittest.main()
