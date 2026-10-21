/* elfsym.c looks up this test binary's own symbols; crc32.c against the
 * standard CRC-32 check value. */
#include <stdint.h>
#include <string.h>

#include "check.h"
#include "crc32.h"
#include "elfsym.h"

uint32_t elfsym_probe_word = 0x1234;
uint8_t elfsym_probe_buf[40];

int main(int argc, char **argv)
{
  (void)argc;
  uint32_t addr = 0, size = 0;
  CHECK_EQ(elfsym_lookup(argv[0], "elfsym_probe_word", &addr, &size), 0);
  CHECK_EQ(addr, (uint32_t)(uintptr_t)&elfsym_probe_word); /* linked -no-pie, so addresses are absolute */
  CHECK_EQ(size, 4);
  CHECK_EQ(elfsym_lookup(argv[0], "elfsym_probe_buf", &addr, &size), 0);
  CHECK_EQ(size, 40);
  CHECK(elfsym_lookup(argv[0], "no_such_symbol", &addr, NULL) != 0);
  CHECK(elfsym_lookup("/nonexistent.elf", "x", &addr, NULL) != 0);
  CHECK(elfsym_lookup("/etc/hostname", "x", &addr, NULL) != 0); /* not an ELF */

  CHECK_EQ(crc32_ieee("123456789", 9), 0xCBF43926U);
  CHECK_EQ(crc32_ieee("", 0), 0);
  printf("ok\n");
  return 0;
}
