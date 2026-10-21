#include "can_fwu_protocol.h"
#include "can_fwu_boot.h"
#include "can_fwu_host.h"
#include "check.h"
#include "fwu_cases.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static uint8_t flash_img[CAN_FWU_APP_MAX];
static int motor_active;
static uint32_t ack_id;
static uint8_t ack_pay[8];
static unsigned program_calls, erase_calls;

static int fake_erase(uint32_t page, uint32_t n, void *ctx) {
  (void)ctx;
  erase_calls++;
  if (page + n > CAN_FWU_APP_MAX / CAN_FWU_PAGE_SIZE) return -1;
  memset(&flash_img[page * CAN_FWU_PAGE_SIZE], 0xFF, n * CAN_FWU_PAGE_SIZE);
  return 0;
}
static int fake_program(uint32_t off, uint64_t dw, void *ctx) {
  unsigned k;
  (void)ctx;
  program_calls++;
  /* Mirror HAL double-word rules: 8-aligned offset inside the app region. */
  CHECK((off & 7U) == 0U);
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
    CHECK(r == 1);
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
  CHECK(can_fwu_host_begin(&h, dev, img, size, 1, now));
  while (!can_fwu_host_done(&h) && h.state != FWU_H_FAULT && guard++ < 200000) {
    now++;
    pump(&h, &b, &now);
  }
  CHECK(can_fwu_host_done(&h));
  CHECK(b.state == FWU_B_VERIFIED || b.state == FWU_B_JUMP_PENDING);
  CHECK(b.received == size);
  CHECK(b.stream_crc == can_fwu_crc32(img, size));
  CHECK(memcmp(flash_img, img, size) == 0); /* staged bytes land verbatim */
  /* JUMP handshake. */
  if (b.state == FWU_B_VERIFIED) {
    uint8_t p[8];
    can_fwu_jump_pack(p);
    CHECK(can_fwu_boot_frame(&b, can_fwu_id(CAN_FWU_JUMP_BASE, dev), p,
                              now, &ack_id, ack_pay) == 1);
    CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK);
    CHECK(b.state == FWU_B_JUMP_PENDING);
  }
}

static void test_crc_codec_round_trip(void)
{
  /* P2: CRC vector + codec round-trip. */
  const uint8_t v[] = "123456789";
  uint8_t p[8];
  CHECK(can_fwu_crc32(v, 9) == 0xCBF43926U);
  can_fwu_enter_pack(p, 0x1234U);
  CHECK(CAN_FWU_PROTO_VERSION == 2U);
  CHECK(can_fwu_enter_magic(p) == CAN_FWU_MAGIC);
  CHECK(can_fwu_enter_session(p) == 1U);
  CHECK(can_fwu_get_u32(p, 1) == 0x1234U);
  CHECK(can_fwu_data_count(0) == 0U && can_fwu_data_count(6) == 1U);
  CHECK(can_fwu_data_count(7) == 2U && can_fwu_data_count(12) == 2U);
  CHECK(can_fwu_pages_for_size(1) == 1U);
  CHECK(can_fwu_pages_for_size(2048) == 1U);
  CHECK(can_fwu_pages_for_size(2049) == 2U);
  CHECK(can_fwu_id(CAN_FWU_ENTER_BASE, 1) == 0x601U);
  CHECK(can_fwu_id(CAN_FWU_ACK_BASE, 4) == 0x684U);
  can_fwu_enter_session_pack(p, 0xABCDU, 10240U);
  CHECK(p[0] == 0x57U && p[1] == 0x32U && p[2] == 0xCDU && p[3] == 0xABU);
  can_fwu_ack_reply_pack(p, CAN_FWU_OK, CAN_FWU_VERIFY_BASE, 0xABCDU, 0x12345678U);
  CHECK(p[0] == 0 && p[1] == 0x63U && p[2] == 0xCDU && p[3] == 0xABU);
  can_fwu_ack_set_status(p, CAN_FWU_FLASH_ERR);
  CHECK(can_fwu_ack_status(p) == CAN_FWU_FLASH_ERR);
  CHECK(can_fwu_ack_command(p) == (CAN_FWU_VERIFY_BASE >> 4));
  CHECK(can_fwu_ack_session(p) == 0xABCDU);
  CHECK(can_fwu_ack_detail(p) == 0x12345678U);
  can_fwu_ack_set_status_detail(p, CAN_FWU_REJECT, 42U);
  CHECK(can_fwu_ack_status(p) == CAN_FWU_REJECT && can_fwu_ack_detail(p) == 42U);
  CHECK(can_fwu_ack_session(p) == 0xABCDU && can_fwu_ack_command(p) == 0x63U);
}

static void test_full_10k_session(void)
{
  /* P3+P4: full 10 KiB session on NE. */
  static uint8_t img[10240];
  unsigned i;
  for (i = 0; i < sizeof(img); i++) img[i] = (uint8_t)(i * 7U + 3U);
  memset(flash_img, 0xFF, sizeof(flash_img));
  motor_active = 0;
  run_session(1, img, sizeof(img));
}

static void test_corrupted_transfer_faults(void)
{
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
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
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
      CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id,
                                ack_pay) == 1);
      can_fwu_host_ack(&h, ack_id, ack_pay, now);
    }
  }
  CHECK(h.state == FWU_H_VERIFY_SENT);
  guard = 0;
  while (h.state != FWU_H_FAULT && !can_fwu_host_done(&h) &&
         guard++ < 5000) {
    now++;
    pump(&h, &b, &now);
  }
  CHECK(h.state == FWU_H_FAULT && h.fault == CAN_FWU_CRC_MISMATCH);
}

static void test_motor_size_idle_rejected(void)
{
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
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay) == 1);
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT);
  CHECK(b.state == FWU_B_IDLE);
  motor_active = 0;
  can_fwu_host_init(&h, &t);
  CHECK(!can_fwu_host_begin(&h, 1, img, CAN_FWU_APP_MAX + 1U, 1, 0));
  CHECK(!can_fwu_host_begin(&h, 1, img, 0U, 1, 0));
  CHECK(!can_fwu_host_begin(&h, 1, img, sizeof(img), 0, 0));
  CHECK(h.state == FWU_H_IDLE);
}

static void test_sequence_and_idle_timeouts(void)
{
  /* P5: wrong seq rejected; target + host idle timeouts fault. */
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
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay) == 1);
  can_fwu_erase_pack(p, 0U, 1U);
  CHECK(can_fwu_boot_frame(&b, 0x611U, p, 10, &ack_id, ack_pay) == 1);
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK);
  memset(q, 0, 8);
  q[0] = 5; /* skip ahead */
  CHECK(can_fwu_boot_frame(&b, 0x621U, q, 20, &ack_id, ack_pay) == 1);
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SEQ);
  CHECK(can_fwu_ack_detail(ack_pay) == 5U); /* identifies rejected request */
  can_fwu_boot_poll(&b, 20 + CAN_FWU_TARGET_IDLE_TIMEOUT_MS + 1U);
  CHECK(b.state == FWU_B_FAULT);
  /* Host: no ACK within 100 ms faults. */
  can_fwu_host_init(&h, &t);
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, 0));
  poll_ret = 0; /* transport stuck pending */
  can_fwu_host_poll(&h, 0);   /* enqueue */
  can_fwu_host_poll(&h, CAN_FWU_HOST_ACK_TIMEOUT_MS + 1U);
  CHECK(h.state == FWU_H_FAULT);
}

static void test_padded_and_tiny_sessions(void)
{
  /* P3: odd sizes exercise the padded final double-word (100 = 12x8 + 4). */
  static uint8_t img[100];
  unsigned i;
  for (i = 0; i < sizeof(img); i++) img[i] = (uint8_t)(i * 13U + 7U);
  memset(flash_img, 0xFF, sizeof(flash_img));
  motor_active = 0;
  run_session(4, img, sizeof(img));
  CHECK(flash_img[100] == 0xFF && flash_img[101] == 0xFF);
  /* 10 bytes: final partial word starts mid-chunk (byte 8), the case
   * that caught the unaligned program-offset bug. */
  {
    static uint8_t tiny[10];
    for (i = 0; i < sizeof(tiny); i++) tiny[i] = (uint8_t)(i + 41U);
    memset(flash_img, 0xFF, sizeof(flash_img));
    motor_active = 0;
    run_session(2, tiny, sizeof(tiny));
    CHECK(flash_img[10] == 0xFF);
    for (i = CAN_FWU_APP_MIN; i <= sizeof(tiny); i++) run_session(8, tiny, i);
  }
}

static void test_host_ack_replay_correlation(void)
{
  /* Real transport/target ACK replay at every dangerous host boundary. */
  struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
  struct can_fwu_transport t = { host_send, host_poll, host_abort };
  struct can_fwu_boot b;
  struct can_fwu_host h;
  static uint8_t img[20];
  uint8_t enter_ack[8], data_ack[8], saved[8], forged[8];
  uint32_t now = 100;
  uint16_t old_session;
  can_fwu_boot_init(&b, 1, &ops);
  can_fwu_host_init(&h, &t);
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
  old_session = h.session;
  /* An ACK before enqueue cannot progress the host, even if correlated. */
  can_fwu_ack_reply_pack(forged, CAN_FWU_OK, CAN_FWU_ENTER_BASE, h.session,
                         CAN_FWU_BOOT_VERSION);
  can_fwu_host_ack(&h, 0x681U, forged, now);
  CHECK(h.state == FWU_H_ENTER_SENT && !h.tx_sent);
  CHECK(!can_fwu_host_begin(&h, 2, img, sizeof(img), 1, now));
  CHECK(h.session == old_session && h.dev == 1U);
  can_fwu_host_poll(&h, ++now);
  CHECK(cap_id == 0x601U && h.tx_sent);
  CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id, enter_ack));
  /* Wrong device, stale session, wrong detail, wrong command, stale error. */
  can_fwu_host_ack(&h, 0x682U, enter_ack, now);
  CHECK(h.state == FWU_H_ENTER_SENT);
  memcpy(forged, enter_ack, 8); forged[2] ^= 0x80U;
  can_fwu_host_ack(&h, ack_id, forged, now);
  CHECK(h.state == FWU_H_ENTER_SENT);
  forged[0] = CAN_FWU_FLASH_ERR;
  can_fwu_host_ack(&h, ack_id, forged, now);
  CHECK(h.state == FWU_H_ENTER_SENT);
  memcpy(forged, enter_ack, 8); forged[4]++;
  can_fwu_host_ack(&h, ack_id, forged, now);
  CHECK(h.state == FWU_H_ENTER_SENT);
  memcpy(forged, enter_ack, 8); forged[1]++;
  can_fwu_host_ack(&h, ack_id, forged, now);
  CHECK(h.state == FWU_H_ENTER_SENT);
  can_fwu_host_ack(&h, ack_id, enter_ack, now);
  CHECK(h.state == FWU_H_ERASE_SENT && !h.tx_sent && h.tx_id == 0x611U);
  memcpy(saved, h.tx_payload, 8);
  /* Duplicate ENTER ACK before ERASE enqueue used to replace it with DATA. */
  can_fwu_host_ack(&h, ack_id, enter_ack, now);
  CHECK(h.state == FWU_H_ERASE_SENT && !h.tx_sent && h.tx_id == 0x611U);
  CHECK(memcmp(saved, h.tx_payload, 8) == 0);
  can_fwu_host_poll(&h, ++now);
  can_fwu_host_ack(&h, ack_id, enter_ack, now);
  CHECK(h.state == FWU_H_ERASE_SENT && h.tx_sent && h.tx_id == 0x611U);
  CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id, ack_pay));
  can_fwu_host_ack(&h, ack_id, ack_pay, now);
  CHECK(h.state == FWU_H_SENDING && !h.tx_sent);
  can_fwu_host_poll(&h, ++now);
  CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id, data_ack));
  can_fwu_host_ack(&h, ack_id, data_ack, now);
  CHECK(h.seq_next == 1U && !h.tx_sent);
  can_fwu_host_ack(&h, ack_id, data_ack, now);
  CHECK(h.seq_next == 1U && !h.tx_sent);
  can_fwu_host_poll(&h, ++now);
  can_fwu_host_ack(&h, ack_id, data_ack, now);
  CHECK(h.seq_next == 1U && h.tx_sent && h.state == FWU_H_SENDING);
  memcpy(forged, data_ack, 8); forged[0] = CAN_FWU_FLASH_ERR;
  can_fwu_host_ack(&h, ack_id, forged, now);
  CHECK(h.state == FWU_H_SENDING);
  CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id, ack_pay));
  can_fwu_host_ack(&h, ack_id, ack_pay, now);
  while (!can_fwu_host_done(&h) && h.state != FWU_H_FAULT) {
    CHECK(++now < 200); pump(&h, &b, &now);
  }
  CHECK(can_fwu_host_done(&h));
  /* New begin allocates a different session; delayed ACKs cannot restart it. */
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, ++now));
  CHECK(h.session != old_session && h.session != 0U);
  can_fwu_host_poll(&h, ++now);
  can_fwu_host_ack(&h, ack_id, enter_ack, now);
  CHECK(h.state == FWU_H_ENTER_SENT);
  can_fwu_host_poll(&h, now + CAN_FWU_HOST_ACK_TIMEOUT_MS);
  CHECK(h.state == FWU_H_FAULT);
  old_session = h.session;
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
  CHECK(h.session != old_session);
}

static void test_target_retries_idempotent(void)
{
  /* Target retries must neither rewind the stream nor program twice. */
  struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
  struct can_fwu_boot b;
  uint8_t p[8], enter[8], last[8];
  uint8_t bytes[6] = {1, 2, 3, 4, 5, 6};
  unsigned calls;
  uint32_t crc;
  can_fwu_boot_init(&b, 1, &ops);
  can_fwu_enter_session_pack(enter, 42U, 10U);
  CHECK(can_fwu_boot_frame(&b, 0x602U, enter, 0, &ack_id, ack_pay) == 0);
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 0, &ack_id, ack_pay));
  CHECK(b.session == 42U && b.state == FWU_B_ENTERED);
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 0, &ack_id, ack_pay));
  can_fwu_erase_pack(p, 0U, 1U);
  CHECK(can_fwu_boot_frame(&b, 0x611U, p, 1, &ack_id, ack_pay));
  calls = erase_calls;
  CHECK(can_fwu_boot_frame(&b, 0x611U, p, 1, &ack_id, ack_pay));
  CHECK(erase_calls == calls && can_fwu_ack_status(ack_pay) == CAN_FWU_OK);
  can_fwu_data_pack(p, 0U, bytes, 6);
  CHECK(can_fwu_boot_frame(&b, 0x621U, p, 2, &ack_id, ack_pay));
  calls = program_calls; crc = b.stream_crc;
  CHECK(can_fwu_boot_frame(&b, 0x621U, p, 2, &ack_id, ack_pay));
  CHECK(program_calls == calls && b.received == 6U && b.stream_crc == crc);
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK);
  p[2] ^= 1U;
  CHECK(can_fwu_boot_frame(&b, 0x621U, p, 2, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SEQ);
  CHECK(program_calls == calls && b.received == 6U && b.stream_crc == crc);
  /* Same-session ENTER after DATA must preserve every bit of progress. */
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 3, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK && b.received == 6U);
  CHECK(b.expect_seq == 1U && b.dw_fill == 6U && b.stream_crc == crc);
  can_fwu_enter_session_pack(p, 43U, 10U);
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 3, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT && b.session == 42U);
  CHECK(can_fwu_ack_session(ack_pay) == 43U); /* reject is correlated to caller */
  can_fwu_enter_session_pack(p, 42U, 11U);
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 3, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT && b.img_size == 10U);
  can_fwu_data_pack(p, 3U, bytes, 6);
  CHECK(can_fwu_boot_frame(&b, 0x621U, p, 3, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SEQ && b.expect_seq == 1U);
  can_fwu_data_pack(last, 1U, bytes, 4);
  CHECK(can_fwu_boot_frame(&b, 0x621U, last, 4, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK && b.received == 10U);
  calls = program_calls; crc = b.stream_crc;
  CHECK(can_fwu_boot_frame(&b, 0x621U, last, 4, &ack_id, ack_pay));
  CHECK(program_calls == calls && b.stream_crc == crc && b.received == 10U);
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK);
  last[7] ^= 1U; /* padding is part of exact payload matching */
  CHECK(can_fwu_boot_frame(&b, 0x621U, last, 4, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SEQ && program_calls == calls);
  last[7] ^= 1U;
  can_fwu_verify_pack(p, crc, 10U);
  CHECK(can_fwu_boot_frame(&b, 0x631U, p, 5, &ack_id, ack_pay));
  CHECK(b.state == FWU_B_VERIFIED);
  CHECK(can_fwu_boot_frame(&b, 0x621U, last, 5, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK && program_calls == calls);
  /* Fault recovery accepts a fresh session, not a stale same-session ENTER. */
  can_fwu_boot_init(&b, 1, &ops);
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 0, &ack_id, ack_pay));
  can_fwu_boot_poll(&b, CAN_FWU_TARGET_IDLE_TIMEOUT_MS);
  CHECK(b.state == FWU_B_FAULT);
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 2001, &ack_id, ack_pay));
  CHECK(b.state == FWU_B_FAULT && can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT);
  can_fwu_enter_session_pack(p, 43U, 10U);
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 2002, &ack_id, ack_pay));
  CHECK(b.state == FWU_B_ENTERED && b.session == 43U);
}

static void test_invalid_devices_and_send_timeout(void)
{
  struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
  struct can_fwu_transport t = { host_send, host_poll, host_abort };
  struct can_fwu_boot b;
  struct can_fwu_host h;
  static uint8_t img[10];
  uint8_t p[8];
  const uint32_t invalid[] = {0U, 3U, 7U, 15U, 17U, 0x101U, 0xFFFFFFFFU};
  unsigned i;
  can_fwu_host_init(&h, &t);
  for (i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++) {
    CHECK(!can_fwu_host_begin(&h, invalid[i], img, sizeof(img), 1, 0));
    CHECK(h.state == FWU_H_IDLE);
    can_fwu_boot_init(&b, invalid[i], &ops);
    can_fwu_enter_pack(p, sizeof(img));
    CHECK(!can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay));
    CHECK(!can_fwu_boot_frame(&b, can_fwu_id(CAN_FWU_ENTER_BASE, invalid[i]),
                               p, 0, &ack_id, ack_pay));
  }
  can_fwu_boot_init(&b, 1, &ops);
  can_fwu_enter_session_pack(p, 0U, sizeof(img));
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT);
  can_fwu_put_u32(p, 0x46555743U, sizeof(img)); /* v1 ENTER rejected */
  CHECK(can_fwu_boot_frame(&b, 0x601U, p, 0, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT);
  /* A long delay BEFORE send must not consume the ACK timeout. */
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, 0));
  can_fwu_host_poll(&h, 10000U);
  CHECK(h.tx_tick == 10000U && h.state == FWU_H_ENTER_SENT);
  can_fwu_host_poll(&h, 10099U);
  CHECK(h.state == FWU_H_ENTER_SENT);
  can_fwu_host_poll(&h, 10100U);
  CHECK(h.state == FWU_H_FAULT);
  {
    uint16_t previous = h.session;
    can_fwu_host_init(&h, &t);
    CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, 0));
    CHECK(h.session != 0U && h.session != previous);
  }
}

static void test_lost_enter_ack_recovery(void)
{
  /* Lost ENTER ACK + NEW begins every 300ms must eventually recover.
   * Repeat across uint32 tick wrap, using real sends and target replies. */
  struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
  struct can_fwu_transport t = { host_send, host_poll, host_abort };
  static uint8_t img[10];
  unsigned pass, i;
  for (pass = 0; pass < 2; pass++) {
    struct can_fwu_boot b;
    struct can_fwu_host h;
    uint32_t start = pass ? UINT32_MAX - 1000U : 0U;
    uint32_t now, guard = 0;
    uint16_t original;
    uint8_t malformed[8];
    can_fwu_boot_init(&b, 1, &ops);
    can_fwu_host_init(&h, &t);
    CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, start));
    original = h.session;
    can_fwu_host_poll(&h, start);
    CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, start, &ack_id, ack_pay));
    CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK);
    /* Drop the first ACK; the host times out, then retries new sessions. */
    can_fwu_host_poll(&h, start + CAN_FWU_HOST_ACK_TIMEOUT_MS);
    CHECK(h.state == FWU_H_FAULT);
    for (i = 1; i <= 6; i++) {
      now = start + i * 300U;
      CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
      CHECK(h.session != original);
      can_fwu_host_poll(&h, now);
      CHECK(can_fwu_boot_frame(&b, cap_id, cap_pay, now, &ack_id, ack_pay));
      CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT);
      can_fwu_host_ack(&h, ack_id, ack_pay, now);
      CHECK(h.state == FWU_H_FAULT);
      CHECK(b.session == original && b.last_tick == start);
      memcpy(malformed, cap_pay, 8); malformed[0] ^= 1U;
      CHECK(can_fwu_boot_frame(&b, cap_id, malformed, now + 1U, &ack_id, ack_pay));
      CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT);
      can_fwu_boot_poll(&b, now + 1U);
      CHECK(b.state == FWU_B_ENTERED && b.last_tick == start);
    }
    can_fwu_boot_poll(&b, start + CAN_FWU_TARGET_IDLE_TIMEOUT_MS - 1U);
    CHECK(b.state == FWU_B_ENTERED);
    can_fwu_boot_poll(&b, start + CAN_FWU_TARGET_IDLE_TIMEOUT_MS);
    CHECK(b.state == FWU_B_FAULT && b.last_tick == start);
    now = start + 2100U;
    CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
    pump(&h, &b, &now);
    CHECK(b.state == FWU_B_ENTERED && b.session == h.session && b.last_tick == now);
    CHECK(h.state == FWU_H_ERASE_SENT);
    while (!can_fwu_host_done(&h) && h.state != FWU_H_FAULT && guard++ < 20U) {
      now++; pump(&h, &b, &now);
    }
    CHECK(can_fwu_host_done(&h));
  }
}

static void test_minimum_size_and_accepted_traffic(void)
{
  /* Enforce vector-table minimum and distinguish accepted/rejected traffic. */
  struct can_fwu_flash_ops ops = { fake_erase, fake_program, fake_motor, 0 };
  struct can_fwu_transport t = { host_send, host_poll, host_abort };
  struct can_fwu_boot b;
  struct can_fwu_host h;
  static uint8_t img[10];
  uint8_t p[8], enter[8], data[8];
  unsigned size;
  can_fwu_boot_init(&b, 1, &ops);
  can_fwu_host_init(&h, &t);
  for (size = 0; size < CAN_FWU_APP_MIN; size++) {
    CHECK(!can_fwu_host_begin(&h, 1, img, size, 1, 100U));
    can_fwu_enter_session_pack(p, 1U, size);
    CHECK(can_fwu_boot_frame(&b, 0x601U, p, 100U, &ack_id, ack_pay));
    CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SIZE);
    CHECK(b.state == FWU_B_IDLE && b.last_tick == 0U);
  }
  can_fwu_enter_session_pack(enter, 1U, sizeof(img));
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 100U, &ack_id, ack_pay));
  CHECK(b.last_tick == 100U);
  can_fwu_erase_pack(p, 1U, 1U); /* invalid page range */
  CHECK(can_fwu_boot_frame(&b, 0x611U, p, 150U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT && b.last_tick == 100U);
  motor_active = 1;
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 160U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT && b.last_tick == 100U);
  motor_active = 0;
  CHECK(can_fwu_boot_frame(&b, 0x601U, enter, 170U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK && b.last_tick == 170U);
  can_fwu_erase_pack(p, 0U, 1U);
  CHECK(can_fwu_boot_frame(&b, 0x611U, p, 200U, &ack_id, ack_pay));
  CHECK(b.last_tick == 200U);
  CHECK(can_fwu_boot_frame(&b, 0x611U, p, 250U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK && b.last_tick == 250U);
  can_fwu_data_pack(data, 0U, img, 6U);
  CHECK(can_fwu_boot_frame(&b, 0x621U, data, 300U, &ack_id, ack_pay));
  CHECK(b.last_tick == 300U);
  CHECK(can_fwu_boot_frame(&b, 0x621U, data, 350U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_OK && b.last_tick == 350U);
  data[2] ^= 1U; /* changed duplicate */
  CHECK(can_fwu_boot_frame(&b, 0x621U, data, 400U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SEQ && b.last_tick == 350U);
  can_fwu_data_pack(p, 3U, img, 6U);
  CHECK(can_fwu_boot_frame(&b, 0x621U, p, 450U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SEQ && b.last_tick == 350U);
  can_fwu_verify_pack(p, 0U, sizeof(img));
  CHECK(can_fwu_boot_frame(&b, 0x631U, p, 500U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_BAD_SIZE && b.last_tick == 350U);
  can_fwu_jump_pack(p);
  CHECK(can_fwu_boot_frame(&b, 0x641U, p, 550U, &ack_id, ack_pay));
  CHECK(can_fwu_ack_status(ack_pay) == CAN_FWU_REJECT && b.last_tick == 350U);
  can_fwu_boot_poll(&b, 350U + CAN_FWU_TARGET_IDLE_TIMEOUT_MS - 1U);
  CHECK(b.state == FWU_B_RECEIVING);
  can_fwu_boot_poll(&b, 350U + CAN_FWU_TARGET_IDLE_TIMEOUT_MS);
  CHECK(b.state == FWU_B_FAULT);
}

static void test_tick_and_session_wrap(void)
{
  /* Host timeout arithmetic across tick wrap and actual session allocation
   * across 65535 -> 1, without manually forcing host states. */
  struct can_fwu_transport t = { host_send, host_poll, host_abort };
  struct can_fwu_host h;
  static uint8_t img[8];
  uint32_t now = UINT32_MAX - 50U;
  uint16_t previous;
  unsigned i, wraps = 0;
  can_fwu_host_init(&h, &t);
  CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
  can_fwu_host_poll(&h, now);
  can_fwu_host_poll(&h, now + CAN_FWU_HOST_ACK_TIMEOUT_MS - 1U);
  CHECK(h.state == FWU_H_ENTER_SENT);
  can_fwu_host_poll(&h, now + CAN_FWU_HOST_ACK_TIMEOUT_MS);
  CHECK(h.state == FWU_H_FAULT);
  previous = h.session;
  for (i = 0; i < 65535U; i++) {
    uint16_t expected = previous == UINT16_MAX ? 1U : (uint16_t)(previous + 1U);
    now += 300U;
    CHECK(can_fwu_host_begin(&h, 1, img, sizeof(img), 1, now));
    CHECK(h.session == expected && h.session != 0U);
    if (previous == UINT16_MAX) wraps++;
    previous = h.session;
    can_fwu_host_poll(&h, now);
    can_fwu_host_poll(&h, now + CAN_FWU_HOST_ACK_TIMEOUT_MS);
    CHECK(h.state == FWU_H_FAULT);
  }
  CHECK(wraps == 1U);
}

int main(int argc, char **argv)
{
  static const struct fwu_test_case cases[] = {
    {"test_crc_codec_round_trip", test_crc_codec_round_trip},
    {"test_full_10k_session", test_full_10k_session},
    {"test_corrupted_transfer_faults", test_corrupted_transfer_faults},
    {"test_motor_size_idle_rejected", test_motor_size_idle_rejected},
    {"test_sequence_and_idle_timeouts", test_sequence_and_idle_timeouts},
    {"test_padded_and_tiny_sessions", test_padded_and_tiny_sessions},
    {"test_host_ack_replay_correlation", test_host_ack_replay_correlation},
    {"test_target_retries_idempotent", test_target_retries_idempotent},
    {"test_invalid_devices_and_send_timeout", test_invalid_devices_and_send_timeout},
    {"test_lost_enter_ack_recovery", test_lost_enter_ack_recovery},
    {"test_minimum_size_and_accepted_traffic", test_minimum_size_and_accepted_traffic},
    {"test_tick_and_session_wrap", test_tick_and_session_wrap},
  };
  return fwu_run_case(argc, argv, cases, sizeof(cases) / sizeof(*cases));
}
