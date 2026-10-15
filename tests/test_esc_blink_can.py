#!/usr/bin/env python3
"""Exercise the actual blink CAN polling driver against simulated MMIO."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
code = r'''
#include "can_boot.h"
#include "boot_entry.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
struct cell { uint32_t address, value; };
static struct cell cells[80];
static unsigned used;
volatile uint32_t *esc_can_test_reg(uint32_t address) {
  for (unsigned i = 0; i < used; ++i)
    if (cells[i].address == address) return &cells[i].value;
  assert(used < 80);
  cells[used].address = address;
  cells[used].value = 0;
  return &cells[used++].value;
}
#define REG(address) (*esc_can_test_reg(address))
int main(void) {
  assert(esc_blink_can_init());
  assert(REG(0x4000641C) == ((1U<<25)|(1U<<16)|(12U<<8)|1U));
  assert((REG(0x40006418) & 0x301U) == 0x100U);
  struct { uint32_t pre; uint8_t bytes[8]; uint32_t post; } p =
      {0x12345678, {0}, 0x12345678};
  uint32_t id;
  const uint32_t valid = (1U << 21) | (8U << 16);
  for (unsigned index = 0; index < 3; ++index) {
    uint32_t addr = 0x4000A4B0 + index * 72;
    REG(0x40006490) = 1U | (index << 8);
    REG(addr) = 0x651U << 18;
    REG(addr + 4) = valid;
    REG(addr + 8) = 0x544F4F42; /* BOOT */
    REG(addr + 12) = 0x00FE0101;
    assert(esc_blink_can_poll(&id, p.bytes));
    assert(id == 0x651 && esc_boot_request_valid(id, p.bytes, 1));
    assert(REG(0x40006494) == index);
    assert(p.pre == 0x12345678 && p.post == 0x12345678);
    const uint32_t bad[] = {0, 4U<<16, (1U<<21)|(15U<<16),
                            valid | (1U<<20)};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(*bad); ++i) {
      REG(addr + 4) = bad[i];
      assert(!esc_blink_can_poll(&id, p.bytes));
    }
    REG(addr + 4) = valid;
    REG(addr) |= 1U << 30; /* Extended ID */
    assert(!esc_blink_can_poll(&id, p.bytes));
    REG(addr) = (0x651U << 18) | (1U << 29); /* Remote frame */
    assert(!esc_blink_can_poll(&id, p.bytes));
  }
  REG(0x40006490) = 0;
  assert(!esc_blink_can_poll(&id, p.bytes));
  puts("PASS: real blink CAN init, FIFO indexes, BOOT payload, FD/DLC/header rejection");
}
'''
with tempfile.TemporaryDirectory(dir='/tmp/opencode', prefix='blink-can-mmio-') as tmp:
    folder = pathlib.Path(tmp)
    (folder / 'test.c').write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-DESC_BLINK_CAN_TEST', '-I', str(root / 'tests/esc_blink'),
                    '-I', str(root / 'Common/Inc'), str(folder / 'test.c'),
                    str(root / 'tests/esc_blink/can_boot.c'),
                    '-o', str(folder / 'test')], check=True)
    subprocess.run([str(folder / 'test')], check=True)
