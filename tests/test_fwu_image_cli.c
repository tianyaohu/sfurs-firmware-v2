#define _POSIX_C_SOURCE 200809L
#include "can_fwu_protocol.h"
#include "can_fwu_validity.h"
#include "check.h"
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

static char tmp[] = "/tmp/test-fwu-image-XXXXXX";
static char binary[256], generated[256];
static uint8_t data[CAN_FWU_APP_MAX + 1];

static void put_word(uint8_t *p, uint32_t word)
{
  for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(word >> (8 * i));
}

static uint32_t word(const uint8_t *p)
{
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
         (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void make_image(size_t size, uint32_t sp, uint32_t reset)
{
  CHECK(size <= sizeof(data));
  put_word(data, sp);
  put_word(data + 4, reset);
  for (size_t i = 8; i < size; ++i) data[i] = (uint8_t)((i - 8) * 13 + 7);
  FILE *f = fopen(binary, "wb");
  CHECK(f != NULL);
  CHECK(fwrite(data, 1, size, f) == size);
  CHECK(fclose(f) == 0);
}

static int run_gen(const char *mode, const char *input)
{
  pid_t pid = fork();
  CHECK(pid >= 0);
  if (pid == 0) {
    execl(FWU_IMAGE, FWU_IMAGE, mode, input, "--output", generated, (char *)NULL);
    _exit(127);
  }
  int status;
  while (waitpid(pid, &status, 0) < 0) CHECK(errno == EINTR);
  CHECK(WIFEXITED(status));
  return WEXITSTATUS(status);
}

static void check_embedding(const char *path, size_t size, int legacy_banner,
                            const char *source)
{
  FILE *f = fopen(path, "r");
  CHECK(f != NULL);
  char line[512], expected[512];
  CHECK(fgets(line, sizeof(line), f) != NULL);
  if (legacy_banner) {
    const char *name = strrchr(source, '/');
    snprintf(expected, sizeof(expected),
             "/* Generated from %s by src/tools/fwu_image.py. Do not edit. */\n",
             name ? name + 1 : source);
    CHECK(strcmp(line, expected) == 0);
  }
  CHECK(fgets(line, sizeof(line), f) != NULL);
  CHECK(strcmp(line, "#include \"fwu_test_image.h\"\n") == 0);
  CHECK(fgets(line, sizeof(line), f) != NULL);
  snprintf(expected, sizeof(expected), "const uint32_t fwu_test_image_size = %zuU;\n", size);
  CHECK(strcmp(line, expected) == 0);
  CHECK(fgets(line, sizeof(line), f) != NULL);
  snprintf(expected, sizeof(expected), "const uint32_t fwu_test_image_crc = 0x%08" PRIX32 "U;\n",
           can_fwu_crc32(data, size));
  CHECK(strcmp(line, expected) == 0);
  CHECK(fgets(line, sizeof(line), f) != NULL);
  CHECK(strcmp(line, "const uint8_t fwu_test_image[] = {\n") == 0);
  for (size_t i = 0; i < size; ++i) {
    if (i % 12 == 0) { CHECK(fgetc(f) == ' '); CHECK(fgetc(f) == ' '); }
    char byte[7];
    snprintf(byte, sizeof(byte), "0x%02X,", data[i]);
    for (const char *p = byte; *p; ++p) CHECK(fgetc(f) == *p);
    CHECK(fgetc(f) == (i % 12 == 11 || i + 1 == size ? '\n' : ' '));
  }
  CHECK(fgets(line, sizeof(line), f) != NULL);
  CHECK(strcmp(line, "};\n") == 0);
  CHECK(fgetc(f) == EOF && !ferror(f));
  CHECK(fclose(f) == 0);
}

static void test_image_sizes(void)
{
  make_image(CAN_FWU_APP_MAX + 1, ESC_BOOT_MAILBOX_ADDR, FWU_APP_BASE + 9);
  CHECK(run_gen("--bin", binary) != 0);
  make_image(CAN_FWU_APP_MAX, ESC_BOOT_MAILBOX_ADDR, FWU_APP_BASE + 9);
  CHECK(run_gen("--bin", binary) == 0);
  make_image(7, 0, 0);
  CHECK(run_gen("--bin", binary) != 0);
  make_image(8, ESC_BOOT_MAILBOX_ADDR, FWU_APP_BASE + 1);
  CHECK(run_gen("--bin", binary) == 0);
}

static void test_entry_vectors(void)
{
  const uint32_t bad_sp[] = {0x20000000, 0x20008000, ESC_BOOT_MAILBOX_ADDR - 1,
                            ESC_BOOT_MAILBOX_ADDR + 8, 0x08004000, 0};
  for (size_t i = 0; i < sizeof(bad_sp) / sizeof(*bad_sp); ++i) {
    make_image(64, bad_sp[i], FWU_APP_BASE + 9);
    CHECK(run_gen("--bin", binary) != 0);
  }
  const uint32_t good_sp[] = {ESC_BOOT_MAILBOX_ADDR, 0x20004000};
  for (size_t i = 0; i < sizeof(good_sp) / sizeof(*good_sp); ++i) {
    make_image(64, good_sp[i], FWU_APP_BASE + 9);
    CHECK(run_gen("--bin", binary) == 0);
  }
  const uint32_t bad_reset[] = {FWU_APP_BASE + 8, FWU_APP_BASE - 1,
                               FWU_APP_BASE + 65, 0x08000001};
  for (size_t i = 0; i < sizeof(bad_reset) / sizeof(*bad_reset); ++i) {
    make_image(64, ESC_BOOT_MAILBOX_ADDR, bad_reset[i]);
    CHECK(run_gen("--bin", binary) != 0);
  }
}

static void test_generated_c_round_trip(void)
{
  make_image(4096, ESC_BOOT_MAILBOX_ADDR, FWU_APP_BASE + 9);
  CHECK(run_gen("--bin", binary) == 0);
  check_embedding(generated, 4096, 1, binary);
}

static size_t read_image(const char *path)
{
  FILE *f = fopen(path, "rb");
  CHECK(f != NULL);
  size_t size = fread(data, 1, sizeof(data), f);
  CHECK(!ferror(f));
  CHECK(fclose(f) == 0);
  return size;
}

static void test_elf_round_trip(const char *elf, const char *bin)
{
  size_t size = read_image(bin);
  CHECK(run_gen("--elf", elf) == 0);
  check_embedding(generated, size, 1, elf);
}

static void test_blink_artifact(const char *bin, const char *payload)
{
  size_t size = read_image(bin);
  CHECK(size >= 8 && size <= CAN_FWU_APP_MAX);
  uint32_t sp = word(data), reset = word(data + 4);
  CHECK(sp > 0x20000000 && sp <= 0x20008000 && sp % 8 == 0);
  CHECK((reset & 1) && (reset & ~1U) >= FWU_APP_BASE &&
        (reset & ~1U) < FWU_APP_BASE + size);
  check_embedding(payload, size, 0, bin);
  CHECK(run_gen("--bin", bin) == 0);
  check_embedding(generated, size, 1, bin);
}

int main(int argc, char **argv)
{
  CHECK(mkdtemp(tmp) != NULL);
  snprintf(binary, sizeof(binary), "%s/motor.bin", tmp);
  snprintf(generated, sizeof(generated), "%s/fwu_test_image.c", tmp);
  if (argc == 1) {
    test_image_sizes();
    test_entry_vectors();
    test_generated_c_round_trip();
  } else if (argc == 4 && strcmp(argv[1], "test_elf_round_trip") == 0) {
    test_elf_round_trip(argv[2], argv[3]);
  } else if (argc == 4 && strcmp(argv[1], "test_blink_artifact") == 0) {
    test_blink_artifact(argv[2], argv[3]);
  } else {
    CHECK(argc == 2);
    if (strcmp(argv[1], "test_image_sizes") == 0) test_image_sizes();
    else if (strcmp(argv[1], "test_entry_vectors") == 0) test_entry_vectors();
    else { CHECK(strcmp(argv[1], "test_generated_c_round_trip") == 0); test_generated_c_round_trip(); }
  }
  unlink(binary);
  unlink(generated);
  CHECK(rmdir(tmp) == 0);
  return 0;
}
