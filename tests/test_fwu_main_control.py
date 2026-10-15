#!/usr/bin/env python3
"""Compile the real main update controller with the actual portable host.

Checks that an explicit NEW request can recover from a post-ENTER failure,
but merely ticking never silently retries an erased/writing session.
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'CM7/Core/Src/main.c').read_text()
start = source.index('    static uint32_t fwu_last_try;')
end = source.index('    /* Alternate speed/current telemetry', start)
block = source[start:end]
code = r'''
#include "can_fwu_host.h"
#include "wheel_bench.h"
#include "boot_entry.h"
#include <assert.h>
#include <stdlib.h>
static struct can_fwu_host fwu_host;
static int fwu_transport_ready;
static uint32_t can_fwu_request, can_fwu_result, can_fwu_dev = 1U;
static uint32_t can_fwu_enter_boot, can_fwu_boot_result;
static uint32_t can_diag_erase_reply[3], can_diag_timing[3];
volatile uint32_t wheel_bench_state;
static const uint8_t fwu_test_image[16] = {0};
#define FWU_TEST_IMAGE_SIZE 16U
#define GPIOC 0U
#define GPIO_PIN_13 13U
#define GPIO_PIN_SET 1U
static uint32_t tick;
static unsigned sends;
static uint32_t sent_id;
static uint8_t sent_data[8];
static uint32_t HAL_GetTick(void) { return tick; }
static unsigned HAL_GPIO_ReadPin(unsigned port, unsigned pin) {
  (void)port; (void)pin; return 0;
}
static int can_cal_calls;
static void can_cal_tick(uint32_t now, int bench_idle) {
  (void)now;
  can_cal_calls |= bench_idle ? 1 : 2;
}
static int fwu_can_send(uint32_t id, const uint8_t data[8]) {
  sent_id = id;
  for (unsigned i = 0; i < 8; ++i) sent_data[i] = data[i];
  ++sends; return 1;
}
static int fwu_can_poll(void) { return BENCH_TX_OK; }
static void fwu_can_abort(void) { }
static void controller(uint32_t now) {
''' + block + r'''
}
int main(int argc, char **argv) {
  assert(argc == 2);
  int scenario = atoi(argv[1]);
  tick = 1000;
  if (scenario == 2) {
    can_fwu_enter_boot = 1;
    controller(tick);
    assert(sends == 1 && can_fwu_enter_boot == 0 && can_fwu_boot_result == 1);
    assert(sent_id == 0x651 && esc_boot_request_valid(sent_id, sent_data, 1));
    assert(fwu_host.state == FWU_H_IDLE); /* Boot request does not erase. */
    can_fwu_dev = 3;
    can_fwu_enter_boot = 1;
    controller(tick);
    assert(sends == 1 && can_fwu_boot_result == 2);
    return 0;
  }
  if (scenario == 0) {
    controller(tick);
    assert(sends == 0 && fwu_host.state == FWU_H_IDLE);
    can_fwu_request = 1;
    wheel_bench_state = BENCH_RUNNING;
    controller(tick);
    assert(sends == 0 && can_fwu_result == 2);
    return 0;
  }
  can_fwu_request = 1;
  controller(tick);
  assert(sends == 1 && fwu_host.state == FWU_H_ENTER_SENT);
  fwu_host.state = FWU_H_FAULT;
  fwu_host.tx_id = can_fwu_id(CAN_FWU_DATA_BASE, 1);
  tick += 1000;
  controller(tick);
  assert(can_fwu_result == 4 && sends == 1);
  tick += 10000;
  controller(tick);
  assert(sends == 1); /* No silent restart after DATA failure. */
  can_fwu_request = 1; /* A new operator request DOES allow recovery. */
  controller(tick);
  assert(sends == 2 && fwu_host.state == FWU_H_ENTER_SENT);
  return 0;
}
'''
with tempfile.TemporaryDirectory(dir='/tmp/opencode', prefix='fwu-main-control-') as tmp:
    folder = pathlib.Path(tmp)
    (folder / 'test.c').write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'Common/Inc'),
                    '-I', str(root / 'CM7/Core/Inc'), str(folder / 'test.c'),
                    str(root / 'CM7/Core/Src/can_fwu_host.c'),
                    str(root / 'Common/Src/can_fwu_protocol.c'),
                    '-o', str(folder / 'test')], check=True)
    for case in range(3):
        subprocess.run([str(folder / 'test'), str(case)], check=True)
print('PASS: real main controller idle gate, no silent DATA retry, explicit recovery')
