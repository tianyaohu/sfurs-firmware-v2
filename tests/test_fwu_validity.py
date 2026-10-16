#!/usr/bin/env python3
"""Exercise reset-time validity gates without hardware."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = r'''
#include "can_fwu_validity.h"
#include <assert.h>
#include <stdint.h>
int main(void) {
  struct can_fwu_validity m = { FWU_VALID_MAGIC, ~FWU_VALID_MAGIC,
    128, 0x12345678, FWU_APP_BASE, CAN_FWU_PROTO_VERSION, {0,0} };
  uint32_t sp = ESC_BOOT_MAILBOX_ADDR, pc = FWU_APP_BASE + 9;
  assert(can_fwu_validity_valid(&m, sp, pc, m.crc));
  assert(!can_fwu_validity_valid(&m, sp, pc, m.crc ^ 1));
  m.magic = 0xFFFFFFFF; /* reset between metadata erase and final commit */
  assert(!can_fwu_validity_valid(&m, sp, pc, m.crc));
  m.magic = FWU_VALID_MAGIC;
  m.inverse_magic = 0xFFFFFFFF; /* torn final commit */
  assert(!can_fwu_validity_valid(&m, sp, pc, m.crc));
  assert(!can_fwu_vectors_valid(sp, pc & ~1U, 128));
  assert(!can_fwu_vectors_valid(sp, FWU_APP_BASE + 129, 128));
  assert(!can_fwu_vectors_valid(sp, 0x08000001, 128));
  assert(!can_fwu_vectors_valid(sp - 1, pc, 128));
  assert(!can_fwu_vectors_valid(0x20008000, pc, 128)); /* reserved mailbox */
  assert(!can_fwu_vectors_valid(sp, pc, CAN_FWU_APP_MAX + 1));
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='fwu-validity-') as tmp:
    folder = pathlib.Path(tmp)
    (folder / 'test.c').write_text(source)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'Common/Inc'), str(folder / 'test.c'),
                    '-o', str(folder / 'test')], check=True)
    subprocess.run([str(folder / 'test')], check=True)
print('PASS: validity magic/CRC/vector bounds and interrupted-commit rejection')
