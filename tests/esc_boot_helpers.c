#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "boot_entry.h"
#include "boot_metadata.h"
#include "boot_rx.h"

static struct can_fwu_validity flash;
static unsigned erases, programs, fail_program, fail_erase;
static unsigned written;
static int erase(void *ctx)
{
  (void)ctx;
  ++erases;
  if (fail_erase) return -1;
  memset(&flash, 0xFF, sizeof(flash));
  written = 0;
  return 0;
}
static int program(unsigned word, uint64_t value, void *ctx)
{
  (void)ctx;
  ++programs;
  if (programs == fail_program) return -1;
  assert((word & 1U) == 0 && word < 8U);
  assert(!(written & (1U << word))); /* G4 doubleword cannot be reprogrammed. */
  if (word == 0) assert(written == ((1U << 2) | (1U << 4) | (1U << 6)));
  written |= 1U << word;
  memcpy((uint8_t *)&flash + word * 4U, &value, 8);
  return 0;
}
static const struct esc_metadata_ops ops = {erase, program, NULL};
static int commit(uint32_t size, uint32_t crc, uint32_t actual)
{
  return esc_metadata_commit(&flash, size, crc, ESC_BOOT_MAILBOX_ADDR,
      FWU_APP_BASE + 9U, actual, &ops);
}
static void reset_flash(void)
{
  memset(&flash, 0xFF, sizeof(flash));
  erases = programs = written = fail_program = fail_erase = 0;
}
static void test_metadata(void)
{
  reset_flash();
  assert(commit(128, 42, 42));
  assert(erases == 1 && programs == 4);
  for (unsigned i = 0; i < 10; ++i) assert(commit(128, 42, 42));
  assert(erases == 1 && programs == 4); /* repeated VERIFY is flash-free */
  struct can_fwu_validity saved = flash;
  assert(!commit(128, 43, 43)); /* changed image, even with matching live CRC */
  assert(!commit(256, 42, 42));
  assert(!commit(128, 42, 43)); /* changed live flash */
  assert(!esc_metadata_commit(&flash, 128, 42, 0, FWU_APP_BASE + 9, 42, &ops));
  assert(erases == 1 && programs == 4 && !memcmp(&saved, &flash, sizeof(flash)));
  /* Noncanonical committed metadata may not be erased by a replay. */
  flash.reserved[0] = 1;
  assert(!commit(128, 42, 42));
  assert(erases == 1 && programs == 4);
  /* Each interrupted program is recoverable by erasing invalid metadata. */
  for (unsigned failure = 1; failure <= 4; ++failure) {
    reset_flash();
    fail_program = failure;
    assert(!commit(128, 42, 42));
    assert(flash.magic != FWU_VALID_MAGIC);
    unsigned before = programs;
    fail_program = 0;
    assert(commit(128, 42, 42));
    assert(erases == 2 && programs == before + 4);
  }
  reset_flash();
  fail_erase = 1;
  assert(!commit(128, 42, 42) && programs == 0 && erases == 1);
  reset_flash();
  flash.magic = 0; /* arbitrary invalid/stale metadata */
  assert(commit(128, 42, 42) && erases == 1 && programs == 4);
  /* A real new update invalidates metadata BEFORE erasing application pages. */
  assert(erase(NULL) == 0);
  assert(commit(256, 43, 43));
  assert(erases == 3 && programs == 8);
  assert(flash.size == 256 && flash.crc == 43);
}
static void test_mailbox(void)
{
  struct esc_boot_mailbox m;
  for (uint32_t dev = 1; dev <= 8; dev <<= 1) {
    esc_boot_mailbox_write(&m, dev);
    assert(esc_boot_mailbox_consume(&m, dev, 1));
    assert(!esc_boot_mailbox_consume(&m, dev, 1));
    esc_boot_mailbox_write(&m, dev);
    assert(!esc_boot_mailbox_consume(&m, dev, 0));
    assert(!esc_boot_mailbox_consume(&m, dev, 1));
    esc_boot_mailbox_write(&m, dev);
    assert(!esc_boot_mailbox_consume(&m, dev == 1 ? 2 : 1, 1));
    for (unsigned word = 0; word < 4; ++word) {
      esc_boot_mailbox_write(&m, dev);
      uint32_t words[4];
      memcpy(words, &m, sizeof(m));
      words[word] ^= 1U;
      memcpy(&m, words, sizeof(m));
      assert(!esc_boot_mailbox_consume(&m, dev, 1));
      assert(m.magic == 0 && m.inverse_magic == 0 && m.device == 0 && m.inverse_device == 0);
    }
    uint8_t p[8] = {'B', 'O', 'O', 'T', 1, dev, dev ^ 255U, 0};
    assert(esc_boot_request_valid(0x650 | dev, p, dev));
    assert(!esc_boot_request_valid(0x650 | 15, p, dev));
    for (unsigned i = 0; i < 8; ++i) {
      p[i] ^= 1;
      assert(!esc_boot_request_valid(0x650 | dev, p, dev));
      p[i] ^= 1;
    }
  }
  esc_boot_mailbox_write(&m, 3);
  assert(!esc_boot_mailbox_consume(&m, 3, 1));
}
static int app_erase(uint32_t page, uint32_t n, void *ctx)
{ (void)page; (void)n; (void)ctx; ++erases; return 0; }
static int app_program(uint32_t off, uint64_t dw, void *ctx)
{ (void)off; (void)dw; (void)ctx; ++programs; return 0; }
static int stopped(void *ctx) { (void)ctx; return 1; }
static void test_receive(void)
{
  const unsigned lengths[] = {0, 1, 4, 8, 12, 16, 20, 24, 32, 48, 64};
  const uint32_t ids[] = {CAN_FWU_ENTER_BASE, CAN_FWU_ERASE_BASE,
      CAN_FWU_DATA_BASE, CAN_FWU_VERIFY_BASE, CAN_FWU_JUMP_BASE};
  struct can_fwu_flash_ops app_ops = {app_erase, app_program, stopped, NULL};
  struct can_fwu_boot b, before;
  uint32_t ack_id;
  uint8_t ack[8];
  struct { uint32_t pre; uint8_t data[ESC_BOOT_RX_CAPACITY]; uint32_t post; } rx;
  rx.pre = rx.post = 0x12345678U;
  erases = programs = 0;
  can_fwu_boot_init(&b, 1, &app_ops);
  for (unsigned state = FWU_B_IDLE; state <= FWU_B_FAULT; ++state) {
    b.state = state;
    for (unsigned l = 0; l < sizeof(lengths)/sizeof(*lengths); ++l) {
      if (lengths[l] == 8) continue;
      memset(rx.data, 0xFF, lengths[l]); /* model HAL copying before DLC check */
      can_fwu_enter_pack(rx.data, 128);
      for (unsigned id = 0; id < sizeof(ids)/sizeof(*ids); ++id) {
        before = b;
        int valid = esc_boot_rx_header_valid(1, 1, 1, 1, lengths[l] == 8);
        assert(!esc_boot_rx_frame(&b, valid, ids[id] | 1, rx.data, 1, &ack_id, ack));
        assert(!memcmp(&b, &before, sizeof(b)));
      }
      assert(rx.pre == 0x12345678U && rx.post == 0x12345678U);
    }
  }
  assert(erases == 0 && programs == 0);
  for (unsigned flag = 0; flag < 4; ++flag) {
    int flags[] = {1, 1, 1, 1};
    flags[flag] = 0;
    assert(!esc_boot_rx_header_valid(flags[0], flags[1], flags[2], flags[3], 1));
  }
  can_fwu_boot_init(&b, 1, &app_ops);
  can_fwu_enter_pack(rx.data, 128);
  assert(esc_boot_rx_frame(&b, esc_boot_rx_header_valid(1,1,1,1,1),
      CAN_FWU_ENTER_BASE | 1, rx.data, 1, &ack_id, ack));
  assert(b.state == FWU_B_ENTERED);
}
int main(void)
{
  test_metadata();
  test_mailbox();
  test_receive();
  puts("PASS: metadata idempotence/failure recovery, mailbox ABI/consume, RX bounds/session guard");
}
