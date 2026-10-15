#!/usr/bin/env python3
"""Host-test the CAN-FWU stack: protocol, bootloader target, Nucleo host."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
# Regression for the hardware failure: CRC initialization took 162-191ms;
# enqueue must receive a fresh clock, not the pre-CRC loop timestamp.
main_source = (root / 'CM7/Core/Src/main.c').read_text()
assert 'now = HAL_GetTick();\n      can_fwu_host_poll(&fwu_host, now);' in main_source
harness = r'''
#include "can_fwu_protocol.h"
#include "can_fwu_boot.h"
#include "can_fwu_host.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static uint8_t flash_img[CAN_FWU_APP_MAX];
static int motor_active;
static uint32_t ack_id;
static uint8_t ack_pay[8];

static int fake_erase(uint32_t page, uint32_t n, void *ctx) {
  (void)ctx;
  if (page + n > CAN_FWU_APP_MAX / CAN_FWU_PAGE_SIZE) return -1;
  memset(&flash_img[page * CAN_FWU_PAGE_SIZE], 0xFF, n * CAN_FWU_PAGE_SIZE);
  return 0;
}
static int fake_program(uint32_t off, uint64_t dw, void *ctx) {
  unsigned k;
  (void)ctx;
  /* Mirror HAL double-word rules: 8-aligned offset inside the app region. */
  assert((off & 7U) == 0U);
  if (off + 8U > CAN_FWU_APP_MAX) return -1;
  for (k = 0; k < 8; k++) flash_img[off + k] = (uint8_t)(dw >> (8U * k));
  return 0;
}
static int fake_motor(void *ctx) { (void)ctx; return !motor_active; }

/* Host TX capture. */
static uint32_t cap_id;
static uint8_t cap_pay[8];
static int cap_valid, send_ok = 1, poll_ret = 1;
static int host_send(uint32_t id, const uint8_t d[8]) {
  cap_id = id; memcpy(cap_pay, d, 8); cap_valid = 1; return send_ok;
}
static int host_poll(void) { return poll_ret; }
static void host_abort(void) {}

static void pump(struct can_fwu_host *h, struct can_fwu_boot *b, uint32_t *now) {
  /* One host step then immediate loopback into the target. */
  cap_valid = 0;
  can_fwu_host_poll(h, *now);
  if (cap_valid) {
    int r = can_fwu_boot_frame(b, cap_id, cap_pay, *now, &ack_id, ack_pay);
    assert(r == 1);
    can_fwu_host_ack(h, ack_id, ack_pay, *now);
  }
  can_fwu_boot_poll(b, *now);
}

static void run_session(uint32_t dev, uint8_t *img, uint32_t size) {
  struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
  struct can_fwu_transport t = { host_send, host_poll, host_abort };
  struct can_fwu_boot b;
  struct can_fwu_host h;
  uint32_t now = 1000, guard = 0;
  can_fwu_boot_init(&b, dev, &ops);
  can_fwu_host_init(&h, &t);
  assert(can_fwu_host_begin(&h, dev, img, size, 1, now));
  while (!can_fwu_host_done(&h) && h.state != FWU_H_FAULT && guard++ < 200000) {
    now++;
    pump(&h, &b, &now);
  }
  assert(can_fwu_host_done(&h));
  assert(b.state == FWU_B_VERIFIED || b.state == FWU_B_JUMP_PENDING);
  assert(b.received == size);
  assert(b.stream_crc == can_fwu_crc32(img, size));
  assert(memcmp(flash_img, img, size) == 0); /* staged bytes land verbatim */
  /* JUMP handshake. */
  if (b.state == FWU_B_VERIFIED) {
    uint8_t p[8];
    can_fwu_jump_pack(p);
    assert(can_fwu_boot_frame(&b, can_fwu_id(CAN_FWU_JUMP_BASE, dev), p,
                              now, &ack_id, ack_pay) == 1);
    assert(can_fwu_get_u32(ack_pay, 0) == CAN_FWU_OK);
    assert(b.state == FWU_B_JUMP_PENDING);
  }
}

int main(int argc, char **argv) {
  assert(argc == 2);
  int test = atoi(argv[1]);
  if (test == 0) {
    /* P2: CRC vector + codec round-trip. */
    const uint8_t v[] = "123456789";
    uint8_t p[8];
    assert(can_fwu_crc32(v, 9) == 0xCBF43926U);
    can_fwu_enter_pack(p, 0x1234U);
    assert(can_fwu_get_u32(p, 0) == CAN_FWU_MAGIC);
    assert(can_fwu_get_u32(p, 1) == 0x1234U);
    assert(can_fwu_data_count(0) == 0U && can_fwu_data_count(6) == 1U);
    assert(can_fwu_data_count(7) == 2U && can_fwu_data_count(12) == 2U);
    assert(can_fwu_pages_for_size(1) == 1U);
    assert(can_fwu_pages_for_size(2048) == 1U);
    assert(can_fwu_pages_for_size(2049) == 2U);
    assert(can_fwu_id(CAN_FWU_ENTER_BASE, 1) == 0x601U);
    assert(can_fwu_id(CAN_FWU_ACK_BASE, 4) == 0x684U);
  } else if (test == 1) {
    /* P3+P4: full 10 KiB session on NE. */
    static uint8_t img[10240];
    unsigned i;
    for (i = 0; i < sizeof(img); i++) img[i] = (uint8_t)(i * 7U + 3U);
    memset(flash_img, 0xFF, sizeof(flash_img));
    motor_active = 0;
    run_session(1, img, sizeof(img));
  } else if (test == 2) {
    /* P5: corrupted byte in flight -> VERIFY mismatch, host faults. */
    struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
    struct can_fwu_transport t = { host_send, host_poll, host_abort };
    struct can_fwu_boot b;
    struct can_fwu_host h;
    static uint8_t img[600];
    unsigned i;
    uint32_t now = 5000, guard = 0;
    for (i = 0; i < sizeof(img); i++) img[i] = (uint8_t)i;
    memset(flash_img, 0xFF, sizeof(flash_img));
    motor_active = 0;
    can_fwu_boot_init(&b, 1, &ops);
    can_fwu_host_init(&h, &t);
    assert(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
    while (h.state != FWU_H_VERIFY_SENT && h.state != FWU_H_FAULT &&
           guard++ < 50000) {
      now++;
      cap_valid = 0;
      can_fwu_host_poll(&h, now);
      if (cap_valid) {
        if (cap_id == can_fwu_id(CAN_FWU_DATA_BASE, 1) &&
            can_fwu_data_seq(cap_pay) == 3) {
          cap_pay[2] ^= 0xFF; /* corrupt one chunk byte */
        }
        assert(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id,
                                  ack_pay) == 1);
        can_fwu_host_ack(&h, ack_id, ack_pay, now);
      }
    }
    assert(h.state == FWU_H_VERIFY_SENT);
    guard = 0;
    while (h.state != FWU_H_FAULT && !can_fwu_host_done(&h) &&
           guard++ < 5000) {
      now++;
      pump(&h, &b, &now);
    }
    assert(h.state == FWU_H_FAULT && h.fault == CAN_FWU_CRC_MISMATCH);
  } else if (test == 3) {
    /* P5: motor active rejects ENTER; oversize/zero/idle refused. */
    struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
    struct can_fwu_transport t = { host_send, host_poll, host_abort };
    struct can_fwu_boot b;
    struct can_fwu_host h;
    static uint8_t img[64];
    uint8_t p[8];
    memset(img, 0xA5, sizeof(img));
    motor_active = 1;
    can_fwu_boot_init(&b, 1, &ops);
    can_fwu_enter_pack(p, sizeof(img));
    assert(can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay) == 1);
    assert(can_fwu_get_u32(ack_pay, 0) == CAN_FWU_REJECT);
    assert(b.state == FWU_B_IDLE);
    motor_active = 0;
    can_fwu_host_init(&h, &t);
    assert(!can_fwu_host_begin(&h, 1, img, CAN_FWU_APP_MAX + 1U, 1, 0));
    assert(!can_fwu_host_begin(&h, 1, img, 0U, 1, 0));
    assert(!can_fwu_host_begin(&h, 1, img, sizeof(img), 0, 0));
    assert(h.state == FWU_H_IDLE);
  } else if (test == 4) {    /* P5: wrong seq rejected; target + host idle timeouts fault. */
    struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
    struct can_fwu_transport t = { host_send, host_poll, host_abort };
    struct can_fwu_boot b;
    struct can_fwu_host h;
    static uint8_t img[32];
    uint8_t p[8], q[8];
    memset(img, 0x5A, sizeof(img));
    motor_active = 0;
    can_fwu_boot_init(&b, 1, &ops);
    can_fwu_enter_pack(p, sizeof(img));
    assert(can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay) == 1);
    can_fwu_erase_pack(p, 0U, 1U);
    assert(can_fwu_boot_frame(&b, 0x611U, p, 10, &ack_id, ack_pay) == 1);
    assert(can_fwu_get_u32(ack_pay, 0) == CAN_FWU_OK);
    memset(q, 0, 8);
    q[0] = 5; /* skip ahead */
    assert(can_fwu_boot_frame(&b, 0x621U, q, 20, &ack_id, ack_pay) == 1);
    assert(can_fwu_get_u32(ack_pay, 0) == CAN_FWU_BAD_SEQ);
    can_fwu_boot_poll(&b, 20 + CAN_FWU_TARGET_IDLE_TIMEOUT_MS + 1U);
    assert(b.state == FWU_B_FAULT);
    /* Host: no ACK within 100 ms faults. */
    can_fwu_host_init(&h, &t);
    assert(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, 0));
    poll_ret = 0; /* transport stuck pending */
    can_fwu_host_poll(&h, 0);   /* enqueue */
    can_fwu_host_poll(&h, CAN_FWU_HOST_ACK_TIMEOUT_MS + 1U);
    assert(h.state == FWU_H_FAULT);
  } else if (test == 5) {
    /* P3: odd sizes exercise the padded final double-word (100 = 12x8 + 4). */
    static uint8_t img[100];
    unsigned i;
    for (i = 0; i < sizeof(img); i++) img[i] = (uint8_t)(i * 13U + 7U);
    memset(flash_img, 0xFF, sizeof(flash_img));
    motor_active = 0;
    run_session(4, img, sizeof(img));
    assert(flash_img[100] == 0xFF && flash_img[101] == 0xFF);
    /* 10 bytes: final partial word starts mid-chunk (byte 8), the case
     * that caught the unaligned program-offset bug. */
    {
      static uint8_t tiny[10];
      for (i = 0; i < sizeof(tiny); i++) tiny[i] = (uint8_t)(i + 41U);
      memset(flash_img, 0xFF, sizeof(flash_img));
      motor_active = 0;
      run_session(2, tiny, sizeof(tiny));
      assert(flash_img[10] == 0xFF);
    }
  }
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='can-fwu-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'Common/Inc'),
                    '-I', str(root / 'CM7/Core/Inc'),
                    str(tmp / 'test.c'),
                    str(root / 'Common/Src/can_fwu_protocol.c'),
                    str(root / 'Common/Src/can_fwu_boot.c'),
                    str(root / 'CM7/Core/Src/can_fwu_host.c'),
                    '-o', str(tmp / 'test')], check=True)
    for case in range(6):
        subprocess.run([str(tmp / 'test'), str(case)], check=True)
print('PASS: FWU protocol CRC/codec, 10KiB session + jump, corruption/motor/timeout gates')
