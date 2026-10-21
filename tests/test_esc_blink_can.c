#include "can_boot.h"
#include "boot_entry.h"
#include "check.h"
#include <stdint.h>
#include <stdio.h>
struct cell { uint32_t address, value; };
static struct cell cells[80];
static unsigned used;
volatile uint32_t *esc_can_test_reg(uint32_t address) {
  for (unsigned i = 0; i < used; ++i)
    if (cells[i].address == address) return &cells[i].value;
  CHECK(used < 80);
  cells[used].address = address;
  cells[used].value = 0;
  return &cells[used++].value;
}
#define REG(address) (*esc_can_test_reg(address))
static void test_blink_can(uint32_t wheel) {
  used = 0;
  CHECK(esc_blink_can_init());
  CHECK(REG(0x4000641C) == ((1U<<25)|(1U<<16)|(12U<<8)|1U));
  CHECK((REG(0x40006418) & 0x301U) == 0x100U);
  struct { uint32_t pre; uint8_t bytes[8]; uint32_t post; } p =
      {0x12345678, {0}, 0x12345678};
  uint32_t id;
  const uint32_t valid = (1U << 21) | (8U << 16);
  for (unsigned index = 0; index < 3; ++index) {
    uint32_t addr = 0x4000A4B0 + index * 72;
    REG(0x40006490) = 1U | (index << 8);
    REG(addr) = (0x650U | wheel) << 18;
    REG(addr + 4) = valid;
    REG(addr + 8) = 0x544F4F42; /* BOOT */
    REG(addr + 12) = ((wheel ^ 0xFFU) << 16) | (wheel << 8) | 1U;
    CHECK(esc_blink_can_poll(&id, p.bytes));
    CHECK(id == (0x650U | wheel) && esc_boot_request_valid(id, p.bytes, wheel));
    CHECK(REG(0x40006494) == index);
    CHECK(p.pre == 0x12345678 && p.post == 0x12345678);
    const uint32_t bad[] = {0, 4U<<16, (1U<<21)|(15U<<16),
                            valid | (1U<<20)};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(*bad); ++i) {
      REG(addr + 4) = bad[i];
      CHECK(!esc_blink_can_poll(&id, p.bytes));
    }
    REG(addr + 4) = valid;
    REG(addr) |= 1U << 30; /* Extended ID */
    CHECK(!esc_blink_can_poll(&id, p.bytes));
    REG(addr) = ((0x650U | wheel) << 18) | (1U << 29); /* Remote frame */
    CHECK(!esc_blink_can_poll(&id, p.bytes));
  }
  REG(0x40006490) = 0;
  CHECK(!esc_blink_can_poll(&id, p.bytes));
  puts("PASS: real blink CAN init, FIFO indexes, BOOT payload, FD/DLC/header rejection");
}

int main(void)
{
  for (uint32_t wheel = 1; wheel <= 8; wheel <<= 1)
    test_blink_can(wheel);
  return 0;
}
