#include "can_fwu_validity.h"
#include "check.h"
#include <stdint.h>
static void test_reset_time_validity(void)
{
  struct can_fwu_validity m = { FWU_VALID_MAGIC, ~FWU_VALID_MAGIC,
    128, 0x12345678, FWU_APP_BASE, CAN_FWU_PROTO_VERSION, {0,0} };
  uint32_t sp = ESC_BOOT_MAILBOX_ADDR, pc = FWU_APP_BASE + 9;
  CHECK(can_fwu_validity_valid(&m, sp, pc, m.crc));
  CHECK(!can_fwu_validity_valid(&m, sp, pc, m.crc ^ 1));
  m.magic = 0xFFFFFFFF; /* reset between metadata erase and final commit */
  CHECK(!can_fwu_validity_valid(&m, sp, pc, m.crc));
  m.magic = FWU_VALID_MAGIC;
  m.inverse_magic = 0xFFFFFFFF; /* torn final commit */
  CHECK(!can_fwu_validity_valid(&m, sp, pc, m.crc));
  CHECK(!can_fwu_vectors_valid(sp, pc & ~1U, 128));
  CHECK(!can_fwu_vectors_valid(sp, FWU_APP_BASE + 129, 128));
  CHECK(!can_fwu_vectors_valid(sp, 0x08000001, 128));
  CHECK(!can_fwu_vectors_valid(sp - 1, pc, 128));
  CHECK(!can_fwu_vectors_valid(0x20008000, pc, 128)); /* reserved mailbox */
  CHECK(!can_fwu_vectors_valid(sp, pc, CAN_FWU_APP_MAX + 1));
}

int main(void)
{
  test_reset_time_validity();
  return 0;
}
