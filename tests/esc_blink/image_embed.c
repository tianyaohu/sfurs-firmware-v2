#include "check.h"
#include "crc32.h"
#include <inttypes.h>

static uint32_t word(const uint8_t *bytes)
{
  return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 |
      (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

int main(int argc, char **argv)
{
  CHECK(argc == 3);
  FILE *input = fopen(argv[1], "rb");
  CHECK(input != NULL);
  uint8_t data[110 * 1024 + 1];
  size_t size = fread(data, 1, sizeof(data), input);
  CHECK(!ferror(input));
  CHECK(fclose(input) == 0);
  CHECK(size >= 8 && size <= 110 * 1024);
  uint32_t sp = word(data), reset = word(data + 4);
  CHECK(sp == 0x20007FF0U && (reset & 1U) &&
        (reset & ~1U) >= 0x08004000U && (reset & ~1U) < 0x08004000U + size);
  uint32_t crc = crc32_ieee(data, size);
  FILE *output = fopen(argv[2], "w");
  CHECK(output != NULL);
  fprintf(output, "/* Generated motor-disabled ESC blink payload. */\n"
          "#include \"fwu_test_image.h\"\n"
          "const uint32_t fwu_test_image_size = %zuU;\n"
          "const uint32_t fwu_test_image_crc = 0x%08" PRIX32 "U;\n"
          "const uint8_t fwu_test_image[] = {\n", size, crc);
  for (size_t i = 0; i < size; ++i) {
    if (i % 12 == 0) fputs("  ", output);
    fprintf(output, "0x%02X,", data[i]);
    fputc(i % 12 == 11 || i + 1 == size ? '\n' : ' ', output);
  }
  fputs("};\n", output);
  CHECK(!ferror(output));
  CHECK(fclose(output) == 0);
  printf("Blink payload: %zu bytes, CRC32=%08" PRIX32 "\n", size, crc);
  return 0;
}
