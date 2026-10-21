#define _POSIX_C_SOURCE 200809L
#include "flashimg_update.h"
#include "crc32.h"
#include "elfsym.h"
#include "sha256.h"
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define PATH_CAP 4096

typedef enum {
  IMAGE_KIND, HOST, REQUEST, DEVICE, RESULT, PAYLOAD, PAYLOAD_SIZE, ENTER_BOOT, BOOT_RESULT, NU_COUNT
} nu_symbol_t;
static const char *const nu_names[NU_COUNT] = {
  "nucleo_image_kind", "fwu_host", "can_fwu_request", "can_fwu_dev", "can_fwu_result",
  "fwu_test_image", "fwu_test_image_size", "can_fwu_enter_boot", "can_fwu_boot_result"
};
static const char *const swd_checks[] = {
  "bootloader unchanged", "app bytes", "metadata size+crc", "VTOR at app", "1 kHz hook running"
};
static const char *const can_checks[] = {
  "IDENT reply", "CRC32 matches build", "size matches build", "all four IDLE"
};
typedef struct {
  const flashimg_options_t *o;
  char app_elf[PATH_CAP], updater[PATH_CAP], drive[PATH_CAP], out[PATH_CAP], known[PATH_CAP];
  char app_sha[65], before_sha[65], after_sha[65], drive_sha[65];
  size_t app_size;
  uint32_t crc;
  bool complete, has_before_state, has_before_bus, has_after_bus, has_before_ident, has_after_ident;
  bool before_ident_valid, after_ident_valid;
  uint32_t before_crc, before_size, after_crc, after_size;
  double before_vbus_v[4], after_vbus_v[4];
  flashimg_battery_t before_state;
  bool checks[5];
  unsigned nchecks;
} session_t;

static volatile sig_atomic_t interrupted;
static void on_signal(int sig) { interrupted = sig; }

static int fail(char *err, size_t cap, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt); vsnprintf(err, cap, fmt, ap); va_end(ap);
  return -1;
}

static int pathf(char path[PATH_CAP], const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt); int n = vsnprintf(path, PATH_CAP, fmt, ap); va_end(ap);
  return n < 0 || n >= PATH_CAP ? -1 : 0;
}

static double now_s(void)
{
  struct timespec t;
  if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
  return (double)t.tv_sec + (double)t.tv_nsec / 1e9;
}

static void sleep_s(double seconds)
{
  struct timespec t = {(time_t)seconds, (long)((seconds-(time_t)seconds)*1e9)};
  while (!interrupted && nanosleep(&t, &t) && errno == EINTR) {}
}

static int mkdirs(const char *path)
{
  char buf[PATH_CAP];
  if (pathf(buf, "%s", path) || !buf[0]) return -1;
  for (char *p = buf+1;; p++) {
    if (*p && *p != '/') continue;
    char c = *p; *p = 0;
    if (mkdir(buf, 0777) && errno != EEXIST) return -1;
    struct stat st;
    if (stat(buf, &st) || !S_ISDIR(st.st_mode)) return -1;
    *p = c;
    if (!c) return 0;
  }
}

static int write_file(const char *path, const uint8_t *data, size_t n)
{
  FILE *f = fopen(path, "wb");
  if (!f) return -1;
  bool ok = fwrite(data, 1, n, f) == n;
  if (fflush(f) || fsync(fileno(f))) ok = false;
  if (fclose(f)) ok = false;
  return ok ? 0 : -1;
}

static void json_string(FILE *f, const char *s)
{
  fputc('"', f);
  for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
    if (*p == '"' || *p == '\\') fprintf(f, "\\%c", *p);
    else if (*p < 32) fprintf(f, "\\u%04x", *p);
    else fputc(*p, f);
  }
  fputc('"', f);
}

static void json_string_field(FILE *f, const char *key, const char *value)
{
  fputs(",\n  ", f); json_string(f, key); fputs(": ", f); json_string(f, value);
}

static void json_vbus(FILE *f, const char *key, const double vbus_v[4])
{
  fputs(",\n  ", f); json_string(f, key); fputs(": {", f);
  bool first = true;
  for (int i = 0; i < 4; i++) if (isfinite(vbus_v[i])) {
    fprintf(f, "%s\"%s\": %.2f", first ? "" : ", ", IK_WHEEL_POS[i], vbus_v[i]); first = false;
  }
  fputc('}', f);
}

static void json_ident(FILE *f, uint32_t crc, uint32_t size)
{
  fprintf(f, "{\"crc\": \"%08X\", \"size\": %u}", crc, size);
}

static int save(const session_t *s, char *err, size_t cap)
{
  char path[PATH_CAP], tmp[PATH_CAP];
  if (pathf(path, "%s/session.json", s->out) || pathf(tmp, "%s/session.json.tmp", s->out))
    return fail(err, cap, "session path too long");
  FILE *f = fopen(tmp, "w");
  if (!f) return fail(err, cap, "cannot write %s: %s", tmp, strerror(errno));
  const flashimg_options_t *o = s->o;
  fprintf(f, "{\n  \"wheel\": \"%s\",\n  \"selector\": %u", IK_WHEEL_CORNER[o->wheel], IK_WHEEL_SELECT[o->wheel]);
  if (o->can) json_string_field(f, "mode", "can");
  else json_string_field(f, "esc_probe", FLASHIMG_ESC_SN[o->wheel]);
  json_string_field(f, "app_elf", s->app_elf);
  json_string_field(f, "app_sha256", s->app_sha);
  fprintf(f, ",\n  \"app_crc32\": \"%08X\",\n  \"app_bytes\": %zu", s->crc, s->app_size);
  if (!o->can) fputs(",\n  \"battery_off_confirmed\": true", f);
  fprintf(f, ",\n  \"complete\": %s", s->complete ? "true" : "false");
  if (*s->before_sha) json_string_field(f, "before_sha256", s->before_sha);
  if (*s->after_sha) json_string_field(f, "after_sha256", s->after_sha);
  if (s->has_before_state) {
    fputs(",\n  \"before_state\": ", f);
    if (!*s->known) json_string(f, "unidentified app; operator confirmed battery OFF");
    else {
      fprintf(f, "{\"vbus_v\": %.2f, \"mc_state\": %u, \"app_elf\": ", s->before_state.vbus_v, s->before_state.mc_state);
      const char *base = strrchr(s->known, '/');
      json_string(f, base ? base+1 : s->known); fputc('}', f);
    }
  }
  if (s->has_before_bus) json_vbus(f, "before_vbus", s->before_vbus_v);
  if (s->has_after_bus) json_vbus(f, "after_vbus", s->after_vbus_v);
  if (s->has_before_ident) {
    fputs(",\n  \"before_ident\": ", f);
    if (o->target_in_boot) json_string(f, "target in bootloader");
    else if (s->before_ident_valid) json_ident(f, s->before_crc, s->before_size);
    else json_string(f, "no IDENT reply (app predates it)");
  }
  if (s->has_after_ident) {
    fputs(",\n  \"after_ident\": ", f);
    if (s->after_ident_valid) json_ident(f, s->after_crc, s->after_size);
    else fputs("null", f);
  }
  if (s->nchecks) {
    fputs(",\n  \"checks\": {", f);
    for (unsigned i = 0; i < s->nchecks; i++) {
      if (i) fputs(", ", f);
      json_string(f, o->can ? can_checks[i] : swd_checks[i]);
      fprintf(f, ": %s", s->checks[i] ? "true" : "false");
    }
    fputc('}', f);
  }
  if (*s->drive_sha) {
    fputs(",\n  \"nucleo_after\": {\"elf\": ", f); json_string(f, s->drive);
    fprintf(f, ", \"sha256\": \"%s\"}", s->drive_sha);
  }
  fputs("\n}\n", f);
  bool ok = !ferror(f);
  if (fflush(f) || fsync(fileno(f))) ok = false;
  if (fclose(f)) ok = false;
  if (ok && rename(tmp, path)) ok = false;
  return ok ? 0 : fail(err, cap, "cannot save %s", path);
}

static int resolve(const char *elf, const char *name, uint32_t *addr, char *err, size_t cap)
{
  return elfsym_lookup(elf, name, addr, NULL) ? fail(err, cap, "%s: missing symbol %s", elf, name) : 0;
}

static int read_flash(ocd_t *o, uint8_t flash[FLASHIMG_FLASH_SIZE], char *err, size_t cap)
{
  for (uint32_t off = 0; off < FLASHIMG_FLASH_SIZE; off += 1024) {
    if (interrupted) return fail(err, cap, "interrupted");
    if (ocd_read_bytes(o, 0x08000000U+off, flash+off, 1024)) return fail(err, cap, "%s", ocd_error(o));
  }
  return 0;
}

static int program(session_t *s, const char *elf, const char *logname, char *err, size_t cap)
{
  char log[PATH_CAP];
  if (interrupted) return fail(err, cap, "interrupted");
  if (pathf(log, "%s/%s", s->out, logname)) return fail(err, cap, "log path too long");
  return flashimg_program_nucleo(s->o->cli, elf, log, err, cap);
}

static int restore_drive(session_t *s, char *err, size_t cap)
{
  if (program(s, s->drive, "nucleo-drive.log", err, cap)) return -1;
  uint8_t *data; size_t n;
  if (flashimg_read_file(s->drive, &data, &n)) return fail(err, cap, "cannot read %s", s->drive);
  sha256_hex(data, n, s->drive_sha); free(data);
  return 0;
}

static int archive_app(session_t *s, char *err, size_t cap)
{
  char dir[PATH_CAP], path[PATH_CAP];
  if (pathf(dir, "%s/known-apps", s->o->backups) || mkdirs(dir) ||
      pathf(path, "%s/motor_%s_drive_%.12s.elf", dir, IK_WHEEL_CORNER[s->o->wheel], s->app_sha))
    return fail(err, cap, "cannot create known-apps archive");
  uint8_t *data; size_t n;
  if (flashimg_read_file(s->app_elf, &data, &n)) return fail(err, cap, "cannot read %s", s->app_elf);
  int rc = write_file(path, data, n); free(data);
  return rc ? fail(err, cap, "cannot write %s", path) : 0;
}

static int bus_snapshot(ik_link_t *link, ik_row_t rows[4], char *err, size_t cap)
{
  ik_row_t before[4];
  if (ik_rows(link, before)) return fail(err, cap, "%s", link->err);
  sleep_s(0.4);
  if (interrupted) return fail(err, cap, "interrupted");
  if (ik_rows(link, rows)) return fail(err, cap, "%s", link->err);
  flashimg_live_rows(before, rows);
  return 0;
}

static int wait_ident(ik_link_t *link, int index, uint32_t *crc, uint32_t *size, char *err, size_t cap)
{
  ik_row_t rows[4];
  if (ik_rows(link, rows)) return fail(err, cap, "%s", link->err);
  uint32_t count = rows[index].valid ? rows[index].ident_count : 0;
  double deadline_s = now_s() + 2;
  while (now_s() < deadline_s) {
    if (interrupted) return fail(err, cap, "interrupted");
    if (ik_rows(link, rows)) return fail(err, cap, "%s", link->err);
    if (rows[index].valid && rows[index].ident_count != count) {
      *crc = rows[index].app_crc; *size = rows[index].app_size; return 1;
    }
    sleep_s(0.05);
  }
  return 0;
}

static int can_before(session_t *s, char *err, size_t cap)
{
  ik_link_t link;
  if (ik_link_open(&link, FLASHIMG_NUCLEO_SN, s->drive)) {
    if (program(s, s->drive, "nucleo-drive-before.log", err, cap)) return -1;
    sleep_s(1);
    if (ik_link_open(&link, FLASHIMG_NUCLEO_SN, s->drive)) return fail(err, cap, "%s", link.err);
  }
  int rc = -1;
  ik_row_t rows[4];
  int index = s->o->wheel;
  if (bus_snapshot(&link, rows, err, cap) ||
      flashimg_check_bus(rows, s->o->target_in_boot ? index : -1, s->before_vbus_v, err, cap)) goto out;
  s->has_before_bus = true;
  if (s->o->target_in_boot) {
    if (rows[index].valid) { fail(err, cap, "%s answers as an app; drop --target-in-boot", IK_WHEEL_CORNER[index]); goto out; }
  } else {
    int found = wait_ident(&link, index, &s->before_crc, &s->before_size, err, cap);
    if (found < 0) goto out;
    s->before_ident_valid = found;
  }
  s->has_before_ident = true;
  rc = 0;
out:
  ik_link_close(&link);
  if (rc) return rc;
  if (save(s, err, cap)) return -1;
  printf("%s: bus ready, Vbus", IK_WHEEL_CORNER[index]);
  for (int i = 0; i < 4; i++) if (isfinite(s->before_vbus_v[i])) printf(" %s=%.2f V", IK_WHEEL_POS[i], s->before_vbus_v[i]);
  puts(""); fflush(stdout);
  return 0;
}

static int swd_before(session_t *s, uint8_t before[FLASHIMG_FLASH_SIZE], char *err, size_t cap)
{
  ocd_t *esc = ocd_open(NULL, FLASHIMG_ESC_SN[s->o->wheel], "stm32g431", err, cap);
  if (!esc) return -1;
  int rc = -1;
  char path[PATH_CAP];
  if (read_flash(esc, before, err, cap)) goto out;
  if (pathf(path, "%s/before.bin", s->out) || write_file(path, before, FLASHIMG_FLASH_SIZE)) {
    fail(err, cap, "cannot save full flash backup in %s", s->out); goto out;
  }
  sha256_hex(before, FLASHIMG_FLASH_SIZE, s->before_sha);
  if (save(s, err, cap)) goto out;
  int found = flashimg_installed(s->o->build, s->o->backups, before, s->known, sizeof s->known);
  if (found < 0) { fail(err, cap, "cannot search installed app ELFs"); goto out; }
  if (found) {
    if (flashimg_check_battery_off(esc, s->known, &s->before_state, err, cap)) goto out;
  } else if (!s->o->battery_off) {
    fail(err, cap, "installed app is unidentified; confirm battery OFF with --battery-off"); goto out;
  }
  s->has_before_state = true;
  rc = 0;
out:
  ocd_close(esc);
  if (rc) return rc;
  if (save(s, err, cap)) return -1;
  printf("%s: backed up to %s (%s)\n", IK_WHEEL_CORNER[s->o->wheel], s->out,
         *s->known ? "running app, battery OFF" : "unidentified app; operator confirmed battery OFF");
  fflush(stdout);
  return 0;
}

static int transfer(session_t *s, const uint32_t addr[NU_COUNT], const uint8_t *app, char *err, size_t cap)
{
  if (program(s, s->updater, "nucleo-updater.log", err, cap)) return -1;
  ocd_t *nu = ocd_open(NULL, FLASHIMG_NUCLEO_SN, "stm32h755", err, cap), *esc = NULL;
  if (!nu) return -1;
  int rc = -1;
  uint32_t value;
  if (ocd_read32(nu, addr[IMAGE_KIND], &value)) goto io_error;
  if (value != 2) { fail(err, cap, "Nucleo is not running the updater image"); goto out; }
  if (ocd_read32(nu, addr[PAYLOAD_SIZE], &value)) goto io_error;
  if (value != s->app_size) { fail(err, cap, "updater payload does not match the built ESC app"); goto out; }
  for (size_t off = 0; off < s->app_size; off += 1024) {
    uint8_t bytes[1024]; size_t n = s->app_size-off < sizeof bytes ? s->app_size-off : sizeof bytes;
    if ((uint64_t)addr[PAYLOAD] + off + n > (uint64_t)UINT32_MAX+1) {
      fail(err, cap, "updater payload address overflow"); goto out;
    }
    if (ocd_read_bytes(nu, addr[PAYLOAD]+(uint32_t)off, bytes, n)) goto io_error;
    if (memcmp(bytes, app+off, n)) { fail(err, cap, "updater payload does not match the built ESC app"); goto out; }
  }
  if (!s->o->can) {
    esc = ocd_open(NULL, FLASHIMG_ESC_SN[s->o->wheel], "stm32g431", err, cap);
    if (!esc) goto out;
    if (*s->known) {
      flashimg_battery_t battery;
      if (flashimg_check_battery_off(esc, s->known, &battery, err, cap)) goto out;
    }
  }
  if (interrupted) { fail(err, cap, "interrupted"); goto out; }
  if (ocd_write32(nu, addr[DEVICE], IK_WHEEL_SELECT[s->o->wheel])) goto io_error;
  if (s->o->can && !s->o->target_in_boot) {
    if (ocd_write32(nu, addr[ENTER_BOOT], 1)) goto io_error;
    sleep_s(0.2);
    if (ocd_read32(nu, addr[BOOT_RESULT], &value)) goto io_error;
    if (value != 1) { fail(err, cap, "boot-entry request not sent (result %u)", value); goto out; }
    sleep_s(0.3);
  }
  if (interrupted) { fail(err, cap, "interrupted"); goto out; }
  if (ocd_write32(nu, addr[REQUEST], 1)) goto io_error;
  if (esc) ocd_reset_sysreq(esc);
  double deadline_s = now_s()+60;
  uint32_t last_state = UINT32_MAX, last_result = UINT32_MAX;
  for (;;) {
    uint32_t state, result;
    if (ocd_read32(nu, addr[HOST], &state) || ocd_read32(nu, addr[RESULT], &result)) goto io_error;
    if (state != last_state || result != last_result) {
      printf("  updater state %u result %u\n", state, result); fflush(stdout);
      last_state = state; last_result = result;
    }
    if (state == 6 && result == 3) { rc = 0; break; }
    if (state == 7 && result == 4) {
      fail(err, cap, s->o->can ? "transfer failed; ESC stays in its bootloader. Rerun with --can --target-in-boot" :
           "transfer failed; ESC keeps its bootloader, rerun to retry"); goto out;
    }
    if (interrupted || now_s() > deadline_s) {
      ocd_reset_sysreq(nu);
      fail(err, cap, "%s%s", interrupted ? "interrupted" : "no completion within 60 s",
           s->o->can ? "; if target stopped answering, rerun with --can --target-in-boot" : "");
      goto out;
    }
    sleep_s(0.1);
  }
  goto out;
io_error:
  fail(err, cap, "%s", ocd_error(nu));
out:
  ocd_close(esc); ocd_close(nu);
  return rc;
}

static int verify_checks(session_t *s, char *err, size_t cap)
{
  if (save(s, err, cap)) return -1;
  for (unsigned i = 0; i < s->nchecks; i++)
    if (!s->checks[i]) return fail(err, cap, "verification failed: %s; see %s",
                                  s->o->can ? can_checks[i] : swd_checks[i], s->out);
  printf("%s: installed and verified%s (", IK_WHEEL_CORNER[s->o->wheel], s->o->can ? " over CAN" : "");
  for (unsigned i = 0; i < s->nchecks; i++) printf("%s%s", i ? ", " : "", s->o->can ? can_checks[i] : swd_checks[i]);
  puts(")"); fflush(stdout);
  return 0;
}

static int can_after(session_t *s, char *err, size_t cap)
{
  if (restore_drive(s, err, cap)) return -1;
  sleep_s(1);
  ik_link_t link;
  if (ik_link_open(&link, FLASHIMG_NUCLEO_SN, s->drive)) return fail(err, cap, "%s", link.err);
  int rc = -1;
  int found = wait_ident(&link, s->o->wheel, &s->after_crc, &s->after_size, err, cap);
  if (found < 0) goto out;
  ik_row_t rows[4];
  if (bus_snapshot(&link, rows, err, cap) || flashimg_check_bus(rows, -1, s->after_vbus_v, err, cap)) goto out;
  s->has_after_bus = s->has_after_ident = true;
  s->after_ident_valid = found;
  s->nchecks = 4;
  s->checks[0] = found;
  s->checks[1] = found && s->after_crc == s->crc;
  s->checks[2] = found && s->after_size == s->app_size;
  s->checks[3] = true;
  rc = 0;
out:
  ik_link_close(&link);
  return rc ? rc : verify_checks(s, err, cap);
}

static int swd_after(session_t *s, const uint8_t *app, const uint8_t *before, uint32_t tick_addr, char *err, size_t cap)
{
  ocd_t *esc = ocd_open(NULL, FLASHIMG_ESC_SN[s->o->wheel], "stm32g431", err, cap);
  if (!esc) return -1;
  int rc = -1;
  uint8_t *after = malloc(FLASHIMG_FLASH_SIZE);
  if (!after) { fail(err, cap, "out of memory"); goto out; }
  sleep_s(4);
  if (read_flash(esc, after, err, cap)) goto out;
  char path[PATH_CAP];
  if (pathf(path, "%s/after.bin", s->out) || write_file(path, after, FLASHIMG_FLASH_SIZE)) {
    fail(err, cap, "cannot save after.bin in %s", s->out); goto out;
  }
  sha256_hex(after, FLASHIMG_FLASH_SIZE, s->after_sha);
  flashimg_verify_flash(before, after, app, s->app_size, s->crc, s->checks);
  uint32_t vtor, t0, t1;
  if (ocd_read32(esc, 0xE000ED08, &vtor) || ocd_read32(esc, tick_addr, &t0)) goto io_error;
  sleep_s(0.2);
  if (ocd_read32(esc, tick_addr, &t1)) goto io_error;
  s->checks[3] = vtor == 0x08004000;
  s->checks[4] = t1 > t0;
  s->nchecks = 5;
  rc = 0;
  goto out;
io_error:
  fail(err, cap, "%s", ocd_error(esc));
out:
  free(after); ocd_close(esc);
  return rc ? rc : verify_checks(s, err, cap);
}

int flashimg_install(const flashimg_options_t *o, char *err, size_t cap)
{
  if (o->wheel < 0 || o->wheel >= 4 || o->can == o->battery_off || (o->target_in_boot && !o->can))
    return fail(err, cap, "invalid installation mode or wheel");
  session_t s = {.o=o};
  const char *w = IK_WHEEL_CORNER[o->wheel];
  char bin[PATH_CAP];
  if (pathf(s.app_elf, "%s/esc/%s/motor_%s_drive_offset.elf", o->build, w, w) ||
      pathf(bin, "%s/esc/%s/motor_%s_drive_offset.bin", o->build, w, w) ||
      pathf(s.updater, "%s/updater/%s/sfurs_firmware_v2_CM7.elf", o->build, w) ||
      pathf(s.drive, "%s/drive/sfurs_firmware_v2_CM7.elf", o->build)) return fail(err, cap, "build path too long");
  uint8_t *app = NULL, *before = NULL;
  if (flashimg_read_file(bin, &app, &s.app_size)) return fail(err, cap, "cannot read %s", bin);
  int rc = -1;
  if (!s.app_size || s.app_size > FLASHIMG_META_OFFSET-FLASHIMG_APP_OFFSET) {
    fail(err, cap, "app must fit the 110 KiB application region"); goto done;
  }
  s.crc = crc32_ieee(app, s.app_size); sha256_hex(app, s.app_size, s.app_sha);
  uint32_t addr[NU_COUNT] = {0}, tick_addr = 0;
  if (!o->can) {
    for (int i = 0; i < ENTER_BOOT; i++) if (resolve(s.updater, nu_names[i], &addr[i], err, cap)) goto done;
    if (resolve(s.app_elf, "drive_app", &tick_addr, err, cap)) goto done;
    before = malloc(FLASHIMG_FLASH_SIZE);
    if (!before) { fail(err, cap, "out of memory"); goto done; }
  }
  time_t t = time(NULL); struct tm utc; char stamp[32];
  if (!gmtime_r(&t, &utc) || !strftime(stamp, sizeof stamp, "%Y%m%dT%H%M%SZ", &utc) ||
      pathf(s.out, "%s/%s-%s-%s", o->backups, o->can ? "can-update" : "update", w, stamp) ||
      mkdirs(o->backups) || mkdir(s.out, 0777)) {
    fail(err, cap, "cannot create new session directory (existing sessions are never overwritten)"); goto done;
  }
  if (save(&s, err, cap)) goto done;
  if (o->can)
    for (int i = 0; i < NU_COUNT; i++) if (resolve(s.updater, nu_names[i], &addr[i], err, cap)) goto done;
  interrupted = 0;
  struct sigaction handler = {0}, old_int, old_term;
  handler.sa_handler = on_signal; sigemptyset(&handler.sa_mask);
  if (sigaction(SIGINT, &handler, &old_int)) { fail(err, cap, "cannot install signal handler"); goto done; }
  if (sigaction(SIGTERM, &handler, &old_term)) {
    sigaction(SIGINT, &old_int, NULL); fail(err, cap, "cannot install signal handler"); goto done;
  }
  if (o->can ? can_before(&s, err, cap) : swd_before(&s, before, err, cap)) goto cleanup;
  if (transfer(&s, addr, app, err, cap)) goto cleanup;
  if (o->can ? can_after(&s, err, cap) : swd_after(&s, app, before, tick_addr, err, cap)) goto cleanup;
  if (archive_app(&s, err, cap)) goto cleanup;
  if (!o->can && !o->keep_updater && restore_drive(&s, err, cap)) goto cleanup;
  if (interrupted) { fail(err, cap, "interrupted"); goto cleanup; }
  s.complete = true;
  if (save(&s, err, cap)) goto cleanup;
  printf("%s: done. Nucleo runs the %s image.\n", w, !o->can && o->keep_updater ? "updater" : "drive");
  rc = 0;
cleanup:
  sigaction(SIGINT, &old_int, NULL); sigaction(SIGTERM, &old_term, NULL);
done:
  free(app); free(before);
  return rc;
}
