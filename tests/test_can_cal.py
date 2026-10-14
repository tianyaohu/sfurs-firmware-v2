#!/usr/bin/env python3
"""Compile and exercise the actual CAL1 sender with a fake CAN bus."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
harness = r'''
#include "can_cal_sender.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static unsigned sends, last_id, fail_send, aborts;
static int status = CAN_CAL_TX_OK;
static uint8_t payload[4];
int can_cal_send(uint32_t identifier, const uint8_t data[4]) {
  ++sends; last_id = identifier; memcpy(payload, data, 4); return !fail_send;
}
int can_cal_poll(void) { return status; }
void can_cal_abort(void) { ++aborts; }
static void req(unsigned dev, unsigned now, int idle) {
  can_cal_request = dev; can_cal_tick(now, idle);
}
int main(int argc, char **argv) {
  assert(argc == 2);
  int test = atoi(argv[1]);
  if (test == 0) {
    /* Header contract: base, magic, id/pack helpers, dev validation. */
    assert(CAL_REQUEST == 0x480U);
    assert(CAL_MAGIC == 0x43414C31U);
    assert(can_cal_id(1) == 0x481U);
    assert(can_cal_id(8) == 0x488U);
    assert(can_cal_dev_valid(1) && can_cal_dev_valid(8));
    assert(!can_cal_dev_valid(0) && !can_cal_dev_valid(3));
    assert(!can_cal_dev_valid(CAL_MAGIC));
    {
      uint8_t p[4];
      can_cal_pack(p);
      assert(p[0] == 0x43 && p[1] == 0x41 && p[2] == 0x4C && p[3] == 0x31);
      assert(can_cal_is_magic(p));
      p[3] = 0x32; assert(!can_cal_is_magic(p));
    }
    /* Dormant by default: idle ticks never transmit. */
    for (unsigned t = 0; t < 1000; t++) can_cal_tick(t, 1);
    assert(sends == 0 && can_cal_result == CAN_CAL_NONE);
    assert(!can_cal_tx_count && !can_cal_first_fault && !can_cal_request_count);
    /* Bad requests are consumed exactly once and rejected. */
    req(3, 1000, 1);
    assert(!sends && can_cal_result == CAN_CAL_REJECTED);
    assert(can_cal_request == 0 && can_cal_request_count == 1);
    can_cal_request = CAL_MAGIC; can_cal_tick(1001, 1);
    assert(!sends && can_cal_result == CAN_CAL_REJECTED);
    assert(can_cal_request_count == 2);
    can_cal_tick(1002, 1); assert(can_cal_request_count == 2);
  } else if (test == 1) {
    /* NE send: id 0x481 + "CAL1", REQUESTED then bus-ACKED. */
    req(1, 0, 1);
    assert(sends == 1 && last_id == 0x481U);
    assert(payload[0] == 0x43 && payload[1] == 0x41 &&
           payload[2] == 0x4C && payload[3] == 0x31);
    assert(can_cal_result == CAN_CAL_REQUESTED);
    assert(can_cal_used_mask == 1U && can_cal_tx_count == 1);
    can_cal_tick(1, 1);
    assert(sends == 1 && can_cal_result == CAN_CAL_ACKED);
    assert(can_cal_tx_ack_count == 1);
  } else if (test == 2) {
    /* Bench-idle gate: refused unless the caller passes idle true. */
    req(1, 0, 0);
    assert(!sends && can_cal_result == CAN_CAL_REJECTED);
    req(1, 1, 1);
    assert(sends == 1 && last_id == 0x481U);
  } else if (test == 3) {
    /* One-shot per device: no resend after ACK; others unaffected. */
    req(1, 0, 1); can_cal_tick(1, 1);
    assert(can_cal_result == CAN_CAL_ACKED);
    req(1, 2, 1);
    assert(sends == 1 && can_cal_result == CAN_CAL_REJECTED);
    req(2, 3, 1);
    assert(sends == 2 && last_id == 0x482U);
    assert(can_cal_used_mask == 3U);
  } else if (test == 4) {
    /* Enqueue failure latches a fault and blocks further sends. */
    fail_send = 1; req(1, 5, 1);
    assert(sends == 1 && can_cal_result == CAN_CAL_FAILED);
    assert(can_cal_first_fault == 1 && can_cal_first_fault_tick == 5);
    fail_send = 0; req(2, 6, 1);
    assert(sends == 1 && can_cal_result == CAN_CAL_REJECTED);
    assert(can_cal_used_mask == 1U); /* failed write may have gone out */
  } else if (test == 5) {
    /* 100 ms TX timeout aborts and latches a fault. */
    status = CAN_CAL_TX_PENDING; req(1, 0, 1);
    assert(can_cal_result == CAN_CAL_REQUESTED);
    can_cal_tick(99, 1);
    assert(!aborts && can_cal_result == CAN_CAL_REQUESTED);
    can_cal_tick(100, 1);
    assert(aborts == 1 && can_cal_result == CAN_CAL_FAILED);
    assert(can_cal_first_fault == 3 && can_cal_first_fault_tick == 100);
    status = CAN_CAL_TX_OK; req(2, 101, 1);
    assert(sends == 1 && can_cal_result == CAN_CAL_REJECTED);
  } else if (test == 6) {
    /* Canceled/failed TX latches fault 2 without an abort call. */
    status = CAN_CAL_TX_PENDING; req(4, 0, 1);
    assert(last_id == 0x484U);
    status = CAN_CAL_TX_FAILED; can_cal_tick(1, 1);
    assert(!aborts && can_cal_result == CAN_CAL_FAILED);
    assert(can_cal_first_fault == 2 && can_cal_first_fault_tick == 1);
  } else if (test == 7) {
    /* STOP aborts a pending frame (even when not idle), faults nothing. */
    status = CAN_CAL_TX_PENDING; req(8, 0, 1);
    assert(can_cal_result == CAN_CAL_REQUESTED && last_id == 0x488U);
    can_cal_request = CAN_CAL_STOP; can_cal_tick(1, 0);
    assert(aborts == 1 && can_cal_result == CAN_CAL_REJECTED);
    assert(!can_cal_first_fault);
    status = CAN_CAL_TX_OK;
    req(8, 2, 1); /* aborted frame may have gone out: still one-shot */
    assert(sends == 1 && can_cal_result == CAN_CAL_REJECTED);
    req(2, 3, 1); /* other devices unaffected, no fault latched */
    assert(sends == 2 && last_id == 0x482U);
    can_cal_tick(4, 1); assert(can_cal_result == CAN_CAL_ACKED);
  }
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='can-cal-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'CM7/Core/Inc'),
                    '-I', str(root / 'Common/Inc'),
                    str(tmp / 'test.c'),
                    str(root / 'CM7/Core/Src/can_cal_sender.c'),
                    '-o', str(tmp / 'test')], check=True)
    for case in range(8):
        subprocess.run([str(tmp / 'test'), str(case)], check=True)
print('PASS: CAL1 id/payload, idle/one-shot gates, enqueue/timeout/TX faults, STOP abort')
