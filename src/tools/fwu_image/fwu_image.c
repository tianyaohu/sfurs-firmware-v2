#define _XOPEN_SOURCE 700
#include "can_fwu_validity.h"
#include "crc32.h"
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

static int run(char *const args[], const char *capture)
{
  pid_t pid = fork();
  if (pid < 0) return -1;
  if (pid == 0) {
    if (capture && !freopen(capture, "w", stdout)) _exit(127);
    execvp(args[0], args);
    perror(args[0]);
    _exit(127);
  }
  int status;
  while (waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) return -1;
  }
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static int parents(const char *path)
{
  char *copy = strdup(path);
  if (!copy) return -1;
  for (char *p = copy + 1; *p; ++p) {
    if (*p != '/') continue;
    *p = '\0';
    if (mkdir(copy, 0777) && errno != EEXIST) { free(copy); return -1; }
    *p = '/';
  }
  free(copy);
  return 0;
}

static uint32_t word(const uint8_t *p)
{
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
         (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

int main(int argc, char **argv)
{
  const char *input = NULL, *output = FWU_REPO "/src/nucleo/CM7/Core/Src/fwu_test_image.c";
  char *objcopy = "arm-none-eabi-objcopy", *objdump = "arm-none-eabi-objdump";
  int elf = 0, opt;
  const struct option options[] = {
    {"elf", required_argument, NULL, 'e'}, {"bin", required_argument, NULL, 'b'},
    {"output", required_argument, NULL, 'o'}, {"objcopy", required_argument, NULL, 'c'},
    {"objdump", required_argument, NULL, 'd'}, {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
  };
  while ((opt = getopt_long(argc, argv, "", options, NULL)) != -1) {
    switch (opt) {
    case 'e': case 'b':
      if (input && elf != (opt == 'e')) {
        fputs("error: choose one of --elf or --bin\n", stderr); return 2;
      }
      input = optarg; elf = opt == 'e'; break;
    case 'o': output = optarg; break;
    case 'c': objcopy = optarg; break;
    case 'd': objdump = optarg; break;
    case 'h':
      puts("Usage: fwu_image (--elf APP.elf | --bin APP.bin) [--output FILE]\n"
           "                 [--objcopy PROGRAM] [--objdump PROGRAM]"); return 0;
    default: return 2;
    }
  }
  if (!input || optind != argc) { fputs("error: required: --elf FILE or --bin FILE\n", stderr); return 2; }
  char tmp[] = "/tmp/fwu-image-XXXXXX", binary[128], sections[128];
  int result = 1;
  char *elf_path = NULL;
  if (elf) {
    if (!mkdtemp(tmp)) { perror("mkdtemp"); return 1; }
    snprintf(binary, sizeof(binary), "%s/app.bin", tmp);
    snprintf(sections, sizeof(sections), "%s/sections", tmp);
    elf_path = realpath(input, NULL);
    if (!elf_path) { perror(input); goto done; }
    char *copy_args[] = {objcopy, "-O", "binary", elf_path, binary, NULL};
    if (run(copy_args, NULL)) goto done;
    char *dump_args[] = {objdump, "-h", elf_path, NULL};
    if (run(dump_args, sections) == 0) {
      FILE *f = fopen(sections, "r");
      if (!f) goto done;
      char *line = NULL;
      size_t cap = 0;
      int any = 0, valid = 0;
      while (getline(&line, &cap, f) >= 0) {
        any = 1;
        if (strstr(line, ".isr_vector")) {
          valid = strstr(line, "08004000") != NULL;
          break;
        }
      }
      free(line);
      fclose(f);
      if (any && !valid) {
        fputs("error: ELF .isr_vector missing or not at 0x08004000\n", stderr);
        goto done;
      }
    }
  }
  FILE *f = fopen(elf ? binary : input, "rb");
  if (!f) { perror(input); goto done; }
  uint8_t data[CAN_FWU_APP_MAX + 1];
  size_t size = fread(data, 1, sizeof(data), f);
  int failed = ferror(f);
  if (fclose(f)) failed = 1;
  if (failed) { fputs("error: reading image\n", stderr); goto done; }
  if (size < CAN_FWU_APP_MIN || size > CAN_FWU_APP_MAX) {
    fprintf(stderr, "error: image size %zu outside [%u, %u]\n", size, CAN_FWU_APP_MIN, CAN_FWU_APP_MAX);
    goto done;
  }
  if (!can_fwu_vectors_valid(word(data), word(data + 4), (uint32_t)size)) {
    fputs("error: entry SP or reset vector outside valid app/SRAM range or misaligned\n", stderr);
    goto done;
  }
  uint32_t crc = crc32_ieee(data, size);
  if (parents(output)) { perror(output); goto done; }
  f = fopen(output, "w");
  if (!f) { perror(output); goto done; }
  const char *name = strrchr(input, '/');
  name = name ? name + 1 : input;
  /* Preserve the legacy banner because payloads are compared byte for byte. */
  fprintf(f, "/* Generated from %s by src/tools/fwu_image/fwu_image.c. Do not edit. */\n"
          "#include \"fwu_test_image.h\"\n"
          "const uint32_t fwu_test_image_size = %zuU;\n"
          "const uint32_t fwu_test_image_crc = 0x%08" PRIX32 "U;\n"
          "const uint8_t fwu_test_image[] = {\n", name, size, crc);
  for (size_t i = 0; i < size; ++i) {
    if (i % 12 == 0) fputs("  ", f);
    fprintf(f, "0x%02X,", data[i]);
    fputc(i % 12 == 11 || i + 1 == size ? '\n' : ' ', f);
  }
  fputs("};\n", f);
  failed = ferror(f);
  if (fclose(f)) failed = 1;
  if (failed) { fputs("error: writing payload\n", stderr); goto done; }
  char *absolute = realpath(output, NULL);
  printf("FWU image: %zu bytes, CRC32=%08" PRIX32 " -> %s\n", size, crc, absolute ? absolute : output);
  free(absolute);
  result = 0;
done:
  free(elf_path);
  if (elf) { unlink(binary); unlink(sections); rmdir(tmp); }
  return result;
}
