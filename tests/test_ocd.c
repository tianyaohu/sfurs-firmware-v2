/* ocd.c against tests/fake_openocd.c (OpenOCD's Tcl RPC boundary). */
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "ocd.h"

#define ARGS_FILE "/tmp/test_ocd_args"

static char args_text[2048];

static void load_args(void)
{
  FILE *f = fopen(ARGS_FILE, "r");
  CHECK(f);
  args_text[fread(args_text, 1, sizeof args_text - 1, f)] = '\0';
  fclose(f);
}

static ocd_t *open_fake(const char *serial, const char *chip)
{
  char err[256];
  ocd_t *o = ocd_open(FAKE_OCD, serial, chip, err, sizeof err);
  if (!o) printf("open failed: %s\n", err);
  return o;
}

static void test_read_write_blocks(void)
{
  setenv("FAKE_OCD_ARGS", ARGS_FILE, 1);
  ocd_t *o = open_fake("SN123", "stm32h755");
  CHECK(o);
  uint32_t v;
  CHECK_EQ(ocd_read32(o, 0x20000000, &v), 0);
  CHECK_EQ(v, 0x20000000);
  CHECK_EQ(ocd_write32(o, 0x20000004, 0xFFFFFFFFU), 0);
  CHECK_EQ(ocd_read32(o, 0x20000004, &v), 0);
  CHECK_EQ(v, 0xFFFFFFFFU);
  uint32_t words[600]; /* spans three Tcl blocks */
  CHECK_EQ(ocd_read(o, 0x24000000, 32, 600, words), 0);
  for (unsigned i = 0; i < 600; i++) CHECK_EQ(words[i], 0x24000000U + 4 * i);
  uint8_t bytes[3];
  CHECK_EQ(ocd_read_bytes(o, 0x08000000, bytes, 3), 0);
  CHECK(bytes[0] == 0 && bytes[1] == 1 && bytes[2] == 2);
  ocd_close(o);
}

static void test_probe_serial_chip_and_version(void)
{
  setenv("FAKE_OCD_ARGS", ARGS_FILE, 1);
  setenv("FAKE_OCD_VERSION", "0.12.0", 1);
  ocd_t *o = open_fake("SN123", "stm32g431");
  CHECK(o);
  ocd_close(o);
  load_args();
  CHECK(strstr(args_text, "adapter serial SN123"));
  CHECK(strstr(args_text, "stm32g4x.cfg"));
  CHECK(strstr(args_text, "adapter speed 1000")); /* ESC runs at 1 MHz */
  CHECK(strstr(args_text, "gdb_port disabled"));

  setenv("FAKE_OCD_VERSION", "0.11.0", 1);
  o = open_fake("SN123", "stm32h755");
  CHECK(o);
  ocd_close(o);
  load_args();
  CHECK(strstr(args_text, "hla_serial SN123"));
  CHECK(strstr(args_text, "stm32h7x.cfg"));
  CHECK(!strstr(args_text, "adapter speed"));
  unsetenv("FAKE_OCD_VERSION");
}

static void test_serial_command_by_version(void)
{
  char out[64];
  ocd_serial_command("Open On-Chip Debugger 0.11.0 (2022)", "A", out, sizeof out);
  CHECK(!strcmp(out, "hla_serial A"));
  ocd_serial_command("Open On-Chip Debugger 0.12.0", "A", out, sizeof out);
  CHECK(!strcmp(out, "adapter serial A"));
  ocd_serial_command("Open On-Chip Debugger 1.0.0", "A", out, sizeof out);
  CHECK(!strcmp(out, "adapter serial A"));
}

static void test_reset_writes_aircr(void)
{
  ocd_t *o = open_fake("SN123", "stm32g431");
  CHECK(o);
  ocd_reset_sysreq(o);
  uint32_t v;
  CHECK_EQ(ocd_read32(o, 0xE000ED0C, &v), 0);
  CHECK_EQ(v, 0x05FA0004U);
  ocd_close(o);
}

static void test_failures_are_reported(void)
{
  char err[256];
  CHECK(!ocd_open(FAKE_OCD, "FAIL", "stm32h755", err, sizeof err));
  CHECK(strstr(err, "open failed"));
  CHECK(!ocd_open("/nonexistent/openocd", "SN", "stm32h755", err, sizeof err));
  CHECK(strstr(err, "not found"));
  CHECK(!ocd_open(FAKE_OCD, "SN", "stm32f407", err, sizeof err));
  CHECK(strstr(err, "chip"));
}

int main(void)
{
  test_read_write_blocks();
  test_probe_serial_chip_and_version();
  test_serial_command_by_version();
  test_reset_writes_aircr();
  test_failures_are_reported();
  printf("ok\n");
  return 0;
}
