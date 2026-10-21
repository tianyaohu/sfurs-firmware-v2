#define _POSIX_C_SOURCE 200809L
#include "flashimg_update.h"
#include "elfsym.h"
#include <elf.h>
#include <errno.h>
#include <glob.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

const char *const FLASHIMG_ESC_SN[4] = {
  "066AFF525266555067075126", "0667FF525266555067073424",
  "066FFF525266555067073058", "066AFF515266555067023255"
};

static int error(char *err, size_t cap, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt); vsnprintf(err, cap, fmt, ap); va_end(ap);
  return -1;
}

void flashimg_usage(FILE *f)
{
  fputs("Install the built ESC application on ONE wheel over CAN (bootloader kept).\n"
        "  esc_update NE|SE|SW|NW --can [--target-in-boot]\n"
        "  esc_update NE|SE|SW|NW --battery-off [--keep-updater]\n"
        "  --backups DIR  backup/archive directory (default " FLASHIMG_DEFAULT_BACKUPS ")\n"
        "  --build DIR    build directory (default " FLASHIMG_DEFAULT_BUILD ")\n"
        "  --cli PATH     STM32_Programmer_CLI (default " FLASHIMG_DEFAULT_CLI ")\n\n"
        "--can needs only Nucleo USB. Battery ON is allowed: all four wheels must\n"
        "answer, be IDLE and report Vbus. The updater requests boot entry, transfers\n"
        "once, and the bootloader checks CRC before committing. Restoring drive must\n"
        "yield matching IDENT CRC/size and four IDLE wheels. No flash backup.\n"
        "A failed transfer leaves the ESC listening: rerun --can --target-in-boot.\n\n"
        "--battery-off confirms motor battery OFF. Attach that ESC's ST-LINK and\n"
        "Nucleo USB; no other tool may hold either probe. Back up all 128 KiB first.\n"
        "Identify the installed app by flash sections, require advancing uwTick,\n"
        "Vbus < 8 V and MC state 0/10/11; check again just before transfer.\n"
        "Program and verify the updater payload, reset ESC via SYSRESETREQ, transfer\n"
        "once (60 s deadline), then verify bootloader, app, metadata, VTOR and\n"
        "1 kHz hook. Restore drive unless --keep-updater. Save session.json and\n"
        "archive the installed ELF. Sends no motion. Raise wheels for the first run.\n", f);
}

int flashimg_parse(int argc, char **argv, flashimg_options_t *o, char *err, size_t cap)
{
  *o = (flashimg_options_t){.wheel=-1, .build=FLASHIMG_DEFAULT_BUILD,
                           .backups=FLASHIMG_DEFAULT_BACKUPS, .cli=FLASHIMG_DEFAULT_CLI};
  for (int i = 1; i < argc; i++) {
    const char *a = argv[i];
    if (!strcmp(a, "--help") || !strcmp(a, "-h")) return 1;
    if (!strcmp(a, "--can")) o->can = true;
    else if (!strcmp(a, "--battery-off")) o->battery_off = true;
    else if (!strcmp(a, "--target-in-boot")) o->target_in_boot = true;
    else if (!strcmp(a, "--keep-updater")) o->keep_updater = true;
    else if (!strcmp(a, "--build") || !strcmp(a, "--backups") || !strcmp(a, "--cli") ||
             !strncmp(a, "--build=", 8) || !strncmp(a, "--backups=", 10) || !strncmp(a, "--cli=", 6)) {
      const char *value = strchr(a, '=');
      if (value) value++;
      else if (++i < argc) value = argv[i];
      if (!value || !value[0] || !strncmp(value, "--", 2))
        return error(err, cap, "%s requires a path", a);
      if (!strncmp(a, "--build", 7)) o->build = value;
      else if (!strncmp(a, "--backups", 9)) o->backups = value;
      else o->cli = value;
    } else {
      int w = -1;
      for (int j = 0; j < 4; j++) if (!strcmp(a, IK_WHEEL_CORNER[j])) w = j;
      if (w < 0 || o->wheel >= 0) return error(err, cap, "unexpected argument: %s", a);
      o->wheel = w;
    }
  }
  if (o->wheel < 0) return error(err, cap, "choose one wheel: NE SE SW NW");
  if (o->can == o->battery_off) return error(err, cap, "exactly one of --can and --battery-off is required");
  if (o->target_in_boot && !o->can) return error(err, cap, "--target-in-boot needs --can");
  return 0;
}

int flashimg_read_file(const char *path, uint8_t **data, size_t *size)
{
  *data = NULL; *size = 0;
  FILE *f = fopen(path, "rb");
  if (!f) return -1;
  int rc = -1;
  if (fseek(f, 0, SEEK_END)) goto out;
  long n = ftell(f);
  if (n < 0 || fseek(f, 0, SEEK_SET)) goto out;
  uint8_t *b = malloc(n ? (size_t)n : 1);
  if (!b) goto out;
  if (fread(b, 1, (size_t)n, f) != (size_t)n || ferror(f)) { free(b); goto out; }
  *data = b; *size = (size_t)n; rc = 0;
out:
  fclose(f);
  return rc;
}

static bool range(uint64_t offset, uint64_t size, uint64_t len)
{
  return offset <= len && size <= len - offset;
}

/* memcpy avoids alignment assumptions in externally supplied ELF tables. */
#define MATCH(BITS) \
static bool match##BITS(const uint8_t *b, size_t len, const uint8_t *flash, size_t size) \
{ \
  Elf##BITS##_Ehdr eh; \
  if (len < sizeof eh) return false; \
  memcpy(&eh, b, sizeof eh); \
  if (eh.e_phentsize != sizeof(Elf##BITS##_Phdr) || eh.e_shentsize != sizeof(Elf##BITS##_Shdr) || \
      !range(eh.e_phoff, (uint64_t)eh.e_phnum * eh.e_phentsize, len) || \
      !range(eh.e_shoff, (uint64_t)eh.e_shnum * eh.e_shentsize, len)) return false; \
  unsigned checked = 0; \
  for (unsigned i = 0; i < eh.e_shnum; i++) { \
    Elf##BITS##_Shdr sh; \
    memcpy(&sh, b + eh.e_shoff + (size_t)i * sizeof sh, sizeof sh); \
    if (!(sh.sh_flags & SHF_ALLOC) || sh.sh_type == SHT_NOBITS || !sh.sh_size) continue; \
    if (!range(sh.sh_offset, sh.sh_size, len)) return false; \
    for (unsigned j = 0; j < eh.e_phnum; j++) { \
      Elf##BITS##_Phdr ph; \
      memcpy(&ph, b + eh.e_phoff + (size_t)j * sizeof ph, sizeof ph); \
      if (ph.p_type != PT_LOAD || !ph.p_filesz || sh.sh_offset < ph.p_offset || \
          !range(sh.sh_offset - ph.p_offset, sh.sh_size, ph.p_filesz)) continue; \
      uint64_t delta = sh.sh_offset - ph.p_offset; \
      if (delta > UINT64_MAX - ph.p_paddr) return false; \
      uint64_t lma = ph.p_paddr + delta; \
      if (lma < 0x08000000U || !range(lma - 0x08000000U, sh.sh_size, size) || \
          memcmp(flash + lma - 0x08000000U, b + sh.sh_offset, (size_t)sh.sh_size)) return false; \
      checked++; \
      break; \
    } \
  } \
  return checked > 0; \
}
MATCH(32)
MATCH(64)

bool flashimg_matches(const char *elf, const uint8_t *flash, size_t size)
{
  uint8_t *b; size_t len;
  if (flashimg_read_file(elf, &b, &len)) return false;
  bool ok = false;
  if (len >= EI_NIDENT && !memcmp(b, ELFMAG, SELFMAG) && b[EI_DATA] == ELFDATA2LSB) {
    if (b[EI_CLASS] == ELFCLASS32) ok = match32(b, len, flash, size);
    if (b[EI_CLASS] == ELFCLASS64) ok = match64(b, len, flash, size);
  }
  free(b);
  return ok;
}

int flashimg_installed(const char *build, const char *backups, const uint8_t *flash, char *path, size_t cap)
{
  const char *dirs[] = {build, backups};
  const char *suffix[] = {"/esc/*/motor_*_drive_offset.elf", "/known-apps/*.elf"};
  for (int i = 0; i < 2; i++) {
    /* Escape directory glob metacharacters; only the suffix is a pattern. */
    size_t n = strlen(dirs[i]);
    if (n > (SIZE_MAX - 64) / 2) return -1;
    char *pattern = malloc(2*n + 64);
    if (!pattern) return -1;
    size_t used = 0;
    for (size_t j = 0; j < n; j++) {
      if (strchr("\\*?[]", dirs[i][j])) pattern[used++] = '\\';
      pattern[used++] = dirs[i][j];
    }
    strcpy(pattern + used, suffix[i]);
    glob_t g = {0};
    int rc = glob(pattern, 0, NULL, &g);
    free(pattern);
    if (rc && rc != GLOB_NOMATCH) { globfree(&g); return -1; }
    for (size_t j = 0; j < g.gl_pathc; j++) {
      if (flashimg_matches(g.gl_pathv[j], flash, FLASHIMG_FLASH_SIZE)) {
        if (strlen(g.gl_pathv[j]) >= cap) { globfree(&g); return -1; }
        strcpy(path, g.gl_pathv[j]); globfree(&g); return 1;
      }
    }
    globfree(&g);
  }
  return 0;
}

int flashimg_battery_symbols(const char *elf, flashimg_battery_symbols_t *s, char *err, size_t cap)
{
  uint32_t size;
  if (elfsym_lookup(elf, "BusVoltageSensor_M1", &s->sensor, &size) || size < 8 || s->sensor > UINT32_MAX-8)
    return error(err, cap, "%s: missing/short BusVoltageSensor_M1", elf);
  if (elfsym_lookup(elf, "Mci", &s->mci, &size) || size < 2 || s->mci == UINT32_MAX)
    return error(err, cap, "%s: missing/short Mci", elf);
  if (elfsym_lookup(elf, "uwTick", &s->tick, &size) || size < 4)
    return error(err, cap, "%s: missing/short uwTick", elf);
  return 0;
}

int flashimg_battery_read(ocd_t *o, const flashimg_battery_symbols_t *s, flashimg_battery_t *b,
                         char *err, size_t cap)
{
  uint8_t sensor[8];
  if (ocd_read32(o, s->tick, &b->tick) || ocd_read_bytes(o, s->sensor, sensor, sizeof sensor) ||
      ocd_read_bytes(o, s->mci + 1, &b->mc_state, 1))
    return error(err, cap, "%s", ocd_error(o));
  uint32_t raw = sensor[6] | (uint32_t)sensor[7] << 8;
  uint32_t scale = sensor[2] | (uint32_t)sensor[3] << 8;
  b->vbus_v = raw * scale / 65536.0;
  return 0;
}

int flashimg_battery_validate(uint32_t before_tick, const flashimg_battery_t *b, char *err, size_t cap)
{
  if (b->tick == before_tick) return error(err, cap, "ESC app is not running, so its Vbus reading is stale");
  if (b->vbus_v >= 8) return error(err, cap, "Vbus %.1f V: battery is ON; switch it off", b->vbus_v);
  if (b->mc_state != 0 && b->mc_state != 10 && b->mc_state != 11)
    return error(err, cap, "motor state %u, not idle", b->mc_state);
  return 0;
}

int flashimg_check_battery_off(ocd_t *o, const char *elf, flashimg_battery_t *b, char *err, size_t cap)
{
  flashimg_battery_symbols_t s;
  uint32_t before;
  if (flashimg_battery_symbols(elf, &s, err, cap)) return -1;
  if (ocd_read32(o, s.tick, &before)) return error(err, cap, "%s", ocd_error(o));
  struct timespec delay = {0, 50000000};
  while (nanosleep(&delay, &delay) && errno == EINTR) {}
  if (flashimg_battery_read(o, &s, b, err, cap)) return -1;
  return flashimg_battery_validate(before, b, err, cap);
}

void flashimg_live_rows(const ik_row_t before[4], ik_row_t after[4])
{
  for (int i = 0; i < 4; i++)
    if (!after[i].status_count || (before[i].valid && after[i].status_count == before[i].status_count))
      after[i].valid = false;
}

int flashimg_check_bus(const ik_row_t rows[4], int skip, double vbus_v[4], char *err, size_t cap)
{
  char problems[512] = "";
  for (int i = 0; i < 4; i++) {
    vbus_v[i] = NAN;
    if (i == skip) continue;
    const ik_row_t *r = &rows[i];
    const char *why = NULL;
    if (!r->valid) why = "no reply";
    else if (r->state || r->active || r->aligning) why = r->mc_state ? r->mc_state : "not IDLE";
    else if (!r->diag_count || !isfinite(r->vbus_v)) why = "no Vbus measurement";
    if (why) {
      size_t n = strlen(problems);
      snprintf(problems+n, sizeof problems-n, "%s%s %s%s", n ? ", " : "", IK_WHEEL_POS[i], why,
               r->active ? " active" : "");
    } else vbus_v[i] = round(r->vbus_v * 100) / 100;
  }
  if (*problems) return error(err, cap, "bus not ready: %s (esc_tool clear clears a fault whose cause is gone)", problems);
  return 0;
}

static bool verified_text(const uint8_t *b, size_t n)
{
  static const char needle[] = "Download verified successfully";
  size_t matched = 0;
  for (size_t i = 0; i < n; i++) {
    if (b[i] == 27 && i+1 < n && b[i+1] == '[') {
      size_t j = i+2;
      while (j < n && ((b[j] >= '0' && b[j] <= '9') || b[j] == ';')) j++;
      if (j < n && ((b[j] >= 'A' && b[j] <= 'Z') || (b[j] >= 'a' && b[j] <= 'z'))) { i=j; continue; }
    }
    if (b[i] == (uint8_t)needle[matched]) matched++;
    else matched = b[i] == (uint8_t)needle[0] ? 1 : 0;
    if (matched == sizeof needle-1) return true;
  }
  return false;
}

int flashimg_program_nucleo(const char *cli, const char *elf, const char *log, char *err, size_t cap)
{
  FILE *out = tmpfile(), *errors = tmpfile();
  if (!out || !errors) {
    if (out) fclose(out);
    if (errors) fclose(errors);
    return error(err, cap, "cannot capture programmer output");
  }
  pid_t pid = fork();
  if (pid == 0) {
    if (dup2(fileno(out), STDOUT_FILENO) < 0 || dup2(fileno(errors), STDERR_FILENO) < 0) _exit(127);
    execlp(cli, cli, "-c", "port=SWD", "sn=" FLASHIMG_NUCLEO_SN, "-w", elf, "-v", "-rst", (char *)NULL);
    _exit(127);
  }
  int status = 0;
  pid_t waited = -1;
  if (pid > 0) do { waited = waitpid(pid, &status, 0); } while (waited < 0 && errno == EINTR);
  FILE *f = fopen(log, "wb");
  bool ok = f != NULL;
  FILE *streams[] = {out, errors};
  for (int i = 0; i < 2; i++) {
    rewind(streams[i]);
    uint8_t buf[4096]; size_t n;
    while ((n = fread(buf, 1, sizeof buf, streams[i])))
      if (f && fwrite(buf, 1, n, f) != n) ok = false;
    if (ferror(streams[i])) ok = false;
    fclose(streams[i]);
  }
  if (f && fclose(f)) ok = false;
  uint8_t *text = NULL; size_t n = 0;
  if (ok && flashimg_read_file(log, &text, &n)) ok = false;
  ok = ok && pid > 0 && waited == pid && WIFEXITED(status) && WEXITSTATUS(status) == 0 && verified_text(text, n);
  free(text);
  return ok ? 0 : error(err, cap, "Nucleo programming failed, see %s", log);
}

void flashimg_verify_flash(const uint8_t before[FLASHIMG_FLASH_SIZE], const uint8_t after[FLASHIMG_FLASH_SIZE],
                          const uint8_t *app, size_t size, uint32_t crc, bool checks[3])
{
  checks[0] = !memcmp(before, after, FLASHIMG_APP_OFFSET);
  checks[1] = size <= FLASHIMG_META_OFFSET-FLASHIMG_APP_OFFSET && !memcmp(after+FLASHIMG_APP_OFFSET, app, size);
  bool has_size = false, has_crc = false;
  for (int i = 0; i < 8; i++) {
    const uint8_t *p = after+FLASHIMG_META_OFFSET+4*i;
    uint32_t v = p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
    has_size |= v == size; has_crc |= v == crc;
  }
  checks[2] = has_size && has_crc;
}
