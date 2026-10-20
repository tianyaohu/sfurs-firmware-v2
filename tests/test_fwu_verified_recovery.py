#!/usr/bin/env python3
"""Losing VERIFY's ACK must not lock out fresh sessions forever."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
code = r'''
#include "can_fwu_boot.h"
#include <assert.h>
static int erase(uint32_t page, uint32_t count, void *ctx) {
  (void)ctx; assert(page == 0 && count == 1); return 0;
}
static int program(uint32_t off, uint64_t dw, void *ctx) {
  (void)dw; (void)ctx; assert(off == 0); return 0;
}
static int stopped(void *ctx) { (void)ctx; return 1; }
int main(void) {
  struct can_fwu_boot b;
  struct can_fwu_flash_ops ops = {erase, program, stopped, 0};
  uint8_t data[8] = {1,2,3,4,5,6,7,8}, p[8], ack[8];
  uint8_t tail[6] = {7,8,0xFF,0xFF,0xFF,0xFF};
  uint32_t ack_id;
  can_fwu_boot_init(&b, 1, &ops);
  can_fwu_enter_session_pack(p, 41, sizeof(data));
  assert(can_fwu_boot_frame(&b, 0x601, p, 0, &ack_id, ack));
  can_fwu_erase_pack(p, 0, 1);
  assert(can_fwu_boot_frame(&b, 0x611, p, 1, &ack_id, ack));
  can_fwu_data_pack(p, 0, data, 6);
  assert(can_fwu_boot_frame(&b, 0x621, p, 2, &ack_id, ack));
  can_fwu_data_pack(p, 1, tail, 2);
  assert(can_fwu_boot_frame(&b, 0x621, p, 3, &ack_id, ack));
  can_fwu_verify_pack(p, can_fwu_crc32(data, sizeof(data)), sizeof(data));
  assert(can_fwu_boot_frame(&b, 0x631, p, 4, &ack_id, ack));
  assert(can_fwu_ack_status(ack) == CAN_FWU_OK && b.state == FWU_B_VERIFIED);
  /* ACK is lost; host eventually starts a fresh session instead of JUMP. */
  can_fwu_boot_poll(&b, 4 + CAN_FWU_TARGET_IDLE_TIMEOUT_MS);
  assert(b.state == FWU_B_FAULT);
  can_fwu_enter_session_pack(p, 42, sizeof(data));
  assert(can_fwu_boot_frame(&b, 0x601, p, 3000, &ack_id, ack));
  assert(can_fwu_ack_status(ack) == CAN_FWU_OK && b.state == FWU_B_ENTERED);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='verified-recovery-') as tmp:
    folder = pathlib.Path(tmp)
    (folder / 'test.c').write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'src/shared/inc'), str(folder / 'test.c'),
                    str(root / 'src/shared/src/can_fwu_boot.c'),
                    str(root / 'src/shared/src/can_fwu_protocol.c'),
                    '-o', str(folder / 'test')], check=True)
    subprocess.run([str(folder / 'test')], check=True)
print('PASS: lost VERIFY ACK times out and permits a fresh recovery session')
