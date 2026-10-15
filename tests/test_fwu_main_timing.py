#!/usr/bin/env python3
"""Compile the real main-loop pump and simulate expensive CRC initialization.

This regression exercises the integration boundary that the portable loopback
tests missed: time passes in begin() BEFORE main calls host_poll().
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'CM7/Core/Src/main.c').read_text()
start = source.index('    if (fwu_host.state != FWU_H_IDLE &&')
end = source.index('    /* Alternate speed/current telemetry', start)
pump = source[start:end]
harness = r'''
#include "can_fwu_host.h"
#include <assert.h>
#include <stdint.h>
static struct can_fwu_host fwu_host;
static uint32_t can_fwu_result, can_fwu_dev = 1;
static int fwu_armed = 1;
static uint32_t fake_tick, enqueue_tick;
static unsigned sends;
static uint32_t HAL_GetTick(void) { return fake_tick; }
void can_fwu_host_poll(struct can_fwu_host *h, uint32_t now) {
  (void)h;
  enqueue_tick = now;
  ++sends;
}
int can_fwu_host_done(const struct can_fwu_host *h) {
  return h->state == FWU_H_DONE;
}
static void main_pump(uint32_t now) {
''' + pump + r'''
}
int main(void) {
  fwu_host.state = FWU_H_ENTER_SENT;
  /* main's tick sample predates a 191ms CRC initialization. */
  fake_tick = 1191U;
  main_pump(1000U);
  assert(sends == 1 && enqueue_tick == 1191U);
  assert((uint32_t)(1192U - enqueue_tick) == 1U);
  /* Also exercise wraparound between loop entry and enqueue. */
  fake_tick = 32U;
  main_pump(0xFFFFFF80U);
  assert(sends == 2 && enqueue_tick == 32U);
  fwu_host.state = FWU_H_FAULT;
  main_pump(1000U);
  assert(sends == 2);
  return 0;
}
'''
with tempfile.TemporaryDirectory(dir='/tmp/opencode', prefix='fwu-main-clock-') as tmp:
    folder = pathlib.Path(tmp)
    (folder / 'test.c').write_text(harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'Common/Inc'),
                    '-I', str(root / 'CM7/Core/Inc'), str(folder / 'test.c'),
                    '-o', str(folder / 'test')], check=True)
    subprocess.run([str(folder / 'test')], check=True)
print('PASS: real main pump refreshes enqueue clock after CRC, including wraparound')
