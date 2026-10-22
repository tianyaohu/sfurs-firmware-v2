#define _POSIX_C_SOURCE 200809L
#include "drive_ops.h"
#include "esc_defaults.h"
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

const char *const DRIVE_SETTING_KEYS[DRIVE_KEYS] = {"kp", "ki", "stall_ramp_ma_per_s", "stall_ms"};

void drive_key_press(drive_keys_t *k, int key, double now_s)
{
  if (key == ' ') {
    memset(k->sign, 0, sizeof k->sign);
    return;
  }
  const char *names = "adwsqe";
  const char *p = key ? strchr(names, tolower((unsigned char)key)) : NULL;
  if (!p) return;
  int index = (int)(p - names), axis = index / 2;
  k->sign[axis] = index == 0 || index == 3 || index == 5 ? -1 : 1;
  k->expiry_s[axis] = now_s + k->hold_s;
}

void drive_key_vector(drive_keys_t *k, double now_s, double speed_mps, double yaw_radps, int32_t out[3])
{
  for (int a = 0; a < 3; a++) {
    if (k->expiry_s[a] <= now_s) k->sign[a] = 0;
    out[a] = (int32_t)nearbyint(k->sign[a] * (a == 2 ? yaw_radps : speed_mps) * 1000);
  }
}

drive_record_summary_t drive_record_summary(const ik_record_unpacked_t *rows, int n, int trigger)
{
  drive_record_summary_t s = {.samples = n, .trigger = trigger};
  for (int k = 0; k < n; k++)
    if (abs(rows[k].rpm) > s.peak_rpm) s.peak_rpm = abs(rows[k].rpm);
  return s;
}

int drive_preflight_rows(const ik_row_t before[4], const ik_row_t rows[4], double min_vbus_v,
                         char *problems, size_t cap)
{
  int count = 0;
  if (cap) problems[0] = 0;
  for (int w = 0; w < 4; w++) {
    const ik_row_t *r = &rows[w];
    char reason[256] = "";
    if (!r->valid || !before[w].valid || r->status_count == before[w].status_count)
      snprintf(reason, sizeof reason, "no live reply");
    else if (strcmp(r->mc_state, "IDLE") || r->fault_names[0])
      snprintf(reason, sizeof reason, "%s faults %s (esc_tool %s)", r->mc_state,
               r->fault_names[0] ? r->fault_names : "-", r->fault_names[0] ? "clear" : "stop");
    else if (!isfinite(r->vbus_v) || r->vbus_v < min_vbus_v)
      snprintf(reason, sizeof reason, "Vbus %.2f V < %g V", r->vbus_v, min_vbus_v);
    if (reason[0]) {
      count++;
      size_t used = cap ? strlen(problems) : 0;
      if (used < cap) snprintf(problems + used, cap - used, "%s%s: %s", used ? "\n  " : "", IK_WHEEL_POS[w], reason);
    }
  }
  return count;
}

int drive_preflight(ik_link_t *l, double min_vbus_v)
{
  ik_row_t before[4], rows[4];
  if (ik_rows(l, before)) return -1;
  pulse_sleep_s(0.3);
  if (ik_rows(l, rows)) return -1;
  uint32_t fault;
  if (ik_read(l, IK_FAULT, &fault) || ik_check_fault(l, fault)) return -1;
  char problems[1024];
  if (!drive_preflight_rows(before, rows, min_vbus_v, problems, sizeof problems)) return 0;
  fprintf(stderr, "preflight failed:\n  %s\n", problems);
  snprintf(l->err, sizeof l->err, "preflight failed");
  return -1;
}

int drive_settings_defaults(drive_settings_t *s)
{
  for (int k = 0; k < DRIVE_KEYS; k++) {
    int def;
    if (esc_default(DRIVE_SETTING_KEYS[k], &def)) return -1;
    for (int w = 0; w < 4; w++) s->value[w][k] = def;
  }
  drive_ff_off(&s->ff);
  return 0;
}

int drive_setting_set(drive_settings_t *s, const char *key, int value)
{
  uint16_t id;
  if (ik_config_key(key, &id) || value < INT16_MIN || value > INT16_MAX) return -1;
  for (int k = 0; k < DRIVE_KEYS; k++) if (!strcmp(key, DRIVE_SETTING_KEYS[k])) {
    for (int w = 0; w < 4; w++) s->value[w][k] = value;
    return 0;
  }
  return -1;
}

void drive_json_string(FILE *f, const char *s)
{
  if (!s) { fputs("null", f); return; }
  fputc('"', f);
  for (; *s; s++) {
    unsigned char ch = (unsigned char)*s;
    if (ch == '"' || ch == '\\') fprintf(f, "\\%c", ch);
    else if (ch < 32) fprintf(f, "\\u%04x", ch);
    else fputc(ch, f);
  }
  fputc('"', f);
}

void drive_json_number(FILE *f, double v)
{
  if (isfinite(v)) fprintf(f, "%.12g", v);
  else fputs("null", f);
}

int drive_path(char *out, size_t cap, const char *directory, const char *name)
{
  int n = snprintf(out, cap, "%s/%s", directory, name);
  return n < 0 || (size_t)n >= cap ? -1 : 0;
}

int drive_mkdir(const char *path)
{
  char *copy = strdup(path);
  if (!copy || !*copy) { free(copy); return -1; }
  for (char *p = copy + 1; ; p++) {
    if (*p != '/' && *p) continue;
    char saved = *p;
    *p = 0;
    if (mkdir(copy, 0777) && errno != EEXIST) { free(copy); return -1; }
    struct stat st;
    if (stat(copy, &st) || !S_ISDIR(st.st_mode)) { free(copy); return -1; }
    if (!saved) break;
    *p = saved;
  }
  free(copy);
  return 0;
}

int drive_log_write(const char *path, const pulse_result_t *r)
{
  size_t len = strlen(path) + 16;
  char *temp = malloc(len);
  if (!temp) return -1;
  snprintf(temp, len, "%s.XXXXXX", path);
  int fd = mkstemp(temp);
  if (fd < 0) { free(temp); return -1; }
  close(fd);
  int rc = pulse_log_write(temp, r);
  FILE *in = rc ? NULL : fopen(temp, "r"), *out = in ? fopen(path, "w") : NULL;
  if (!in || !out) rc = -1;
  char *line = NULL;
  size_t cap = 0, i = 0;
  while (!rc && getline(&line, &cap, in) >= 0) {
    if (i >= r->n || line[0] != '{') { rc = -1; break; }
    fprintf(out, "{\"t0_monotonic_s\":%.9f,\"t_monotonic_s\":%.9f,%s",
            r->t0_monotonic_s, r->t0_monotonic_s + r->log[i++].e.t, line + 1);
  }
  if (in) { if (ferror(in)) rc = -1; fclose(in); }
  if (out && fclose(out)) rc = -1;
  free(line);
  unlink(temp);
  free(temp);
  return rc;
}

int drive_stage_analyze(drive_stage_t *s)
{
  free(s->analysis);
  s->analysis = calloc(s->pulse.n + 1, sizeof *s->analysis);
  if (!s->analysis) return -1;
  s->faults[0] = 0;
  s->vbus_min_v = NAN;
  /* Fault names are firmware tokens; collect and sort them independently of wheel order. */
  char names[64][96];
  size_t count = 0;
  for (size_t i = 0; i < s->pulse.n; i++) {
    const pulse_entry_t *p = &s->pulse.log[i];
    tune_log_entry *e = &s->analysis[i];
    e->t = p->e.t;
    for (int w = 0; w < 4; w++) {
      e->target_rpm[w] = p->e.target_rpm[w];
      e->rpm[w] = p->e.rpm[w];
      e->iq_ref_a[w] = p->e.iq_ref_a[w];
      e->rpm_valid[w] = p->e.rpm_ok[w];
      e->iq_ref_valid[w] = p->e.iq_ok[w];
      if (p->vbus_v[w] > 0 && (isnan(s->vbus_min_v) || p->vbus_v[w] < s->vbus_min_v))
        s->vbus_min_v = p->vbus_v[w];
      char copy[96], *save;
      snprintf(copy, sizeof copy, "%s", p->e.faults[w]);
      for (char *token = strtok_r(copy, " ", &save); token; token = strtok_r(NULL, " ", &save)) {
        size_t k = 0;
        while (k < count && strcmp(names[k], token)) k++;
        if (k == count && count < 64) snprintf(names[count++], 96, "%s", token);
      }
    }
  }
  for (size_t i = 0; i < count; i++) for (size_t j = i + 1; j < count; j++)
    if (strcmp(names[i], names[j]) > 0) {
      char tmp[96]; memcpy(tmp, names[i], 96); memcpy(names[i], names[j], 96); memcpy(names[j], tmp, 96);
    }
  for (size_t i = 0; i < count; i++) {
    size_t used = strlen(s->faults);
    if (used + strlen(names[i]) + 2 >= sizeof s->faults) return -1;
    snprintf(s->faults + used, sizeof s->faults - used, "%s%s", i ? " " : "", names[i]);
  }
  return tune_response(s->analysis, s->pulse.n, &s->response);
}

_Static_assert(DRIVE_KEYS <= PULSE_MAX_KEYS, "pulse_run carries every drive setting");

int drive_stage_run(ik_link_t *l, const char *directory, const char *name,
                    const esc_move_t *plan, size_t count, double duration_s,
                    const drive_settings_t *settings, int accel_rpmps, drive_stage_t *s)
{
  memset(s, 0, sizeof *s);
  char path[4096], filename[128];
  if (snprintf(filename, sizeof filename, "%s.jsonl", name) >= (int)sizeof filename ||
      drive_path(path, sizeof path, directory, filename)) return -1;
  if (drive_preflight(l, 0)) return -1;
  /* A refused or failed write leaves the model off: drive_ff_write clears enable first. */
  if (drive_ff_write(l, &settings->ff)) return -1;
  /* pulse_run sends every key while IDLE and restores the boot defaults afterwards. */
  pulse_settings_t pulse = {.n_keys = DRIVE_KEYS};
  for (int k = 0; k < DRIVE_KEYS; k++) {
    pulse.keys[k] = DRIVE_SETTING_KEYS[k];
    for (int w = 0; w < 4; w++) pulse.value[w][k] = settings->value[w][k];
  }
  printf("-- %s: kp/ki", name);
  for (int w = 0; w < 4; w++) printf(" %s %d/%d", IK_WHEEL_POS[w], settings->value[w][0], settings->value[w][1]);
  puts(""); fflush(stdout);
  int rc = pulse_run(l, plan, count, duration_s, &pulse, accel_rpmps, &s->pulse);
  char error[sizeof l->err]; snprintf(error, sizeof error, "%s", l->err);
  if (drive_ff_clear(l) && !rc) { snprintf(error, sizeof error, "%s", l->err); rc = -1; }
  if (drive_log_write(path, &s->pulse) || drive_stage_analyze(s)) {
    if (!rc) snprintf(error, sizeof error, "cannot write/analyze stage %s", name);
    rc = -1;
  }
  if (rc) snprintf(l->err, sizeof l->err, "%s", error);
  return rc;
}

void drive_stage_free(drive_stage_t *s)
{
  free(s->analysis);
  pulse_result_free(&s->pulse);
  memset(s, 0, sizeof *s);
}

void drive_settings_json(FILE *f, const drive_settings_t *s, int wheel)
{
  if (wheel < 0) {
    fputc('[', f);
    for (int w = 0; w < 4; w++) { if (w) fputc(',', f); drive_settings_json(f, s, w); }
    fputc(']', f);
    return;
  }
  fputc('{', f);
  for (int k = 0; k < DRIVE_KEYS; k++) fprintf(f, "%s\"%s\":%d", k ? "," : "", DRIVE_SETTING_KEYS[k], s->value[wheel][k]);
  fputs(",\"ff\":", f); drive_ff_json(f, &s->ff);
  fputc('}', f);
}

void drive_response_json(FILE *f, const tune_response_result *r)
{
  fputs("{\"wheels\":{", f);
  for (int w = 0; w < 4; w++) {
    const tune_wheel_response *a = &r->wheels[w];
    fprintf(f, "%s\"%s\":{\"moved\":%zu,\"starts\":%zu,\"breakaway_s\":", w ? "," : "", IK_WHEEL_POS[w], a->moved, a->starts);
    drive_json_number(f, a->breakaway_s);
    fputs(",\"breakaway_worst_s\":", f); drive_json_number(f, a->breakaway_worst_s);
    fputs(",\"overshoot\":", f); drive_json_number(f, a->overshoot);
    fputs(",\"overshoot_worst\":", f); drive_json_number(f, a->overshoot_worst);
    fputc('}', f);
  }
  fputs("},\"spread_s\":", f); drive_json_number(f, r->spread_s);
  fputs(",\"spread_worst_s\":", f); drive_json_number(f, r->spread_worst_s);
  fputc('}', f);
}

void drive_stage_json(FILE *f, const char *name, const drive_stage_t *s,
                       const drive_settings_t *settings, const esc_move_t *plan, size_t count)
{
  fputs("\"name\":", f); drive_json_string(f, name);
  fputs(",\"settings\":", f); drive_settings_json(f, settings, !strcmp(name, "drive") ? 0 : -1);
  fputs(",\"aborted\":", f); drive_json_string(f, s->pulse.aborted[0] ? s->pulse.aborted : NULL);
  fputs(",\"faults\":", f); pulse_json_names(f, s->faults);
  fputs(",\"vbus_min\":", f); drive_json_number(f, s->vbus_min_v);
  fputs(",\"response\":", f); drive_response_json(f, &s->response);
  fprintf(f, ",\"t0_monotonic_s\":%.9f,\"moves\":[", s->pulse.t0_monotonic_s);
  for (size_t i = 0; i < count; i++) {
    fprintf(f, "%s{\"twist\":[%d,%d,%d],\"start_s\":%.9f,\"end_s\":%.9f,"
               "\"start_monotonic_s\":%.9f,\"end_monotonic_s\":%.9f}", i ? "," : "",
            plan[i].twist[0], plan[i].twist[1], plan[i].twist[2], plan[i].start, plan[i].end,
            s->pulse.t0_monotonic_s + plan[i].start, s->pulse.t0_monotonic_s + plan[i].end);
  }
  fputc(']', f);
}

void drive_response_show(const tune_response_result *r)
{
  for (int w = 0; w < 4; w++) {
    const tune_wheel_response *a = &r->wheels[w];
    printf("   %s: moved %zu/%zu breakaway %.0f ms (worst %.0f) overshoot x%.2f (worst %.2f)\n",
           IK_WHEEL_POS[w], a->moved, a->starts, a->breakaway_s * 1000, a->breakaway_worst_s * 1000,
           a->overshoot, a->overshoot_worst);
  }
  printf("   spread between wheels %.0f ms (worst %.0f)\n", r->spread_s * 1000, r->spread_worst_s * 1000);
}

static int compare(const void *a, const void *b)
{
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

double drive_median(const double *values, size_t n)
{
  if (!n) return NAN;
  double *copy = malloc(n * sizeof *copy);
  if (!copy) return NAN;
  memcpy(copy, values, n * sizeof *copy);
  qsort(copy, n, sizeof *copy, compare);
  double m = n % 2 ? copy[n / 2] : (copy[n / 2 - 1] + copy[n / 2]) / 2;
  free(copy);
  return m;
}

void drive_date_json(FILE *f)
{
  time_t now = time(NULL);
  struct tm tm;
  char date[64];
  localtime_r(&now, &tm);
  strftime(date, sizeof date, "%Y-%m-%d %H:%M:%S", &tm);
  drive_json_string(f, date);
}

int drive_parse(int argc, char **argv, drive_option *options, size_t count)
{
  for (int a = 1; a < argc; a++) {
    if (!strcmp(argv[a], "--help") || !strcmp(argv[a], "-h")) return 1;
    const char *arg = argv[a], *eq = strchr(arg, '=');
    size_t len = eq ? (size_t)(eq - arg) : strlen(arg), k = 0;
    if (strncmp(arg, "--", 2)) goto bad;
    while (k < count && (strlen(options[k].name) != len - 2 || strncmp(arg + 2, options[k].name, len - 2))) k++;
    if (k == count) goto bad;
    drive_option *o = &options[k];
    if (o->type == DRIVE_BOOL) {
      if (eq) goto bad;
      *(bool *)o->value = true; o->seen = true; continue;
    }
    const char *value = eq ? eq + 1 : (++a < argc ? argv[a] : NULL);
    if (!value || !*value) goto bad;
    char *end;
    errno = 0;
    if (o->type == DRIVE_STRING) *(const char **)o->value = value;
    else if (o->type == DRIVE_INT) {
      long v = strtol(value, &end, 10);
      if (*end || errno || v < INT_MIN || v > INT_MAX) goto bad;
      *(int *)o->value = (int)v;
    } else {
      double v = strtod(value, &end);
      if (*end || errno || !isfinite(v)) goto bad;
      *(double *)o->value = v;
    }
    o->seen = true;
    continue;
bad:
    fprintf(stderr, "invalid option or value: %s\n", argv[a < argc ? a : argc - 1]);
    return -1;
  }
  return 0;
}

void drive_args_json(FILE *f, const drive_option *options, size_t count)
{
  fputc('{', f);
  for (size_t k = 0; k < count; k++) {
    const drive_option *o = &options[k];
    if (k) fputc(',', f);
    fputc('"', f);
    for (const char *p = o->name; *p; p++) fputc(*p == '-' ? '_' : *p, f);
    fputs("\":", f);
    char value[64];
    if (o->type == DRIVE_STRING) drive_json_string(f, *(const char **)o->value ? *(const char **)o->value : "None");
    else {
      if (o->type == DRIVE_BOOL) snprintf(value, sizeof value, "%s", *(bool *)o->value ? "True" : "False");
      else if (o->type == DRIVE_INT && *(int *)o->value == INT_MIN) snprintf(value, sizeof value, "None");
      else if (o->type == DRIVE_INT) snprintf(value, sizeof value, "%d", *(int *)o->value);
      else snprintf(value, sizeof value, "%.12g", *(double *)o->value);
      drive_json_string(f, value);
    }
  }
  fputc('}', f);
}

void drive_ff_off(drive_ff_t *ff)
{
  memset(ff, 0, sizeof *ff);
  for (int w = 0; w < IK_WHEELS; w++) ff->scale_pct[w] = DRIVE_FF_SCALE_NOMINAL_PCT;
}

/* SI value -> int32 milli-unit word; rejects negative and out-of-range values. */
static int milli_word(double value, int32_t *word)
{
  if (!isfinite(value) || value < 0 || value * 1000 > INT32_MAX) return -1;
  *word = (int32_t)nearbyint(value * 1000);
  return 0;
}

int drive_ff_from_options(const drive_option *options, size_t count, drive_ff_t *ff, char *err, size_t errlen)
{
  static const char *const names[] = {"ff-coulomb-xy-n", "ff-viscous-xy-n-per-mps", "ff-coulomb-w-nm",
                                      "ff-viscous-w-nm-per-radps", "ff-a-per-nm"};
  drive_ff_off(ff);
  int32_t *words[] = {&ff->coulomb_xy_mn, &ff->viscous_xy_mn_per_mps, &ff->coulomb_w_mnm,
                      &ff->viscous_w_mnm_per_radps, &ff->a_per_nm_milli};
  bool a_given = false;
  for (size_t k = 0; k < count; k++) {
    const drive_option *o = &options[k];
    if (!o->seen || strncmp(o->name, "ff-", 3)) continue;
    ff->enable = true;
    if (!strcmp(o->name, "ff-scale")) {
      const char *p = *(const char *const *)o->value;
      for (int w = 0; w < IK_WHEELS; w++) {
        char *end;
        errno = 0;
        long v = strtol(p, &end, 10);
        if (end == p || errno || v < 1 || v > 200 || *end != (w < IK_WHEELS - 1 ? ',' : '\0')) {
          snprintf(err, errlen, "--ff-scale needs four integer trims FR,BR,BL,FL, each 1..200 %%");
          return -1;
        }
        ff->scale_pct[w] = (int32_t)v;
        p = end + 1;
      }
      continue;
    }
    for (size_t n = 0; n < sizeof names / sizeof *names; n++) if (!strcmp(o->name, names[n])) {
      if (milli_word(*(const double *)o->value, words[n])) {
        snprintf(err, errlen, "--%s must be finite, >= 0 and <= %.0f", names[n], INT32_MAX / 1000.0);
        return -1;
      }
      if (n == 4) a_given = true;
    }
  }
  /* The firmware model outputs nothing without a positive 1/Kt; refuse instead of a silent no-op. */
  if (ff->enable && (!a_given || ff->a_per_nm_milli <= 0)) {
    snprintf(err, errlen, "feed-forward needs --ff-a-per-nm > 0 (motor A per N*m, 1/Kt)");
    return -1;
  }
  return 0;
}

static int ff_write_words(ik_link_t *l, const drive_ff_t *ff)
{
  if (ik_write(l, IK_FF_ENABLE, 0) ||
      ik_write(l, IK_FF_COULOMB_XY_MN, (uint32_t)ff->coulomb_xy_mn) ||
      ik_write(l, IK_FF_VISCOUS_XY_MN_PER_MPS, (uint32_t)ff->viscous_xy_mn_per_mps) ||
      ik_write(l, IK_FF_COULOMB_W_MNM, (uint32_t)ff->coulomb_w_mnm) ||
      ik_write(l, IK_FF_VISCOUS_W_MNM_PER_RADPS, (uint32_t)ff->viscous_w_mnm_per_radps) ||
      ik_write(l, IK_FF_A_PER_NM_MILLI, (uint32_t)ff->a_per_nm_milli)) return -1;
  for (int w = 0; w < IK_WHEELS; w++)
    if (ocd_write32(l->ocd, l->addr[IK_FF_SCALE_PCT] + (uint32_t)w * 4U, (uint32_t)ff->scale_pct[w])) {
      snprintf(l->err, sizeof l->err, "%s", ocd_error(l->ocd));
      return -1;
    }
  return ff->enable ? ik_write(l, IK_FF_ENABLE, 1) : 0;
}

int drive_ff_write(ik_link_t *l, const drive_ff_t *ff)
{
  uint32_t state;
  if (ik_read(l, IK_STATE, &state)) return -1;
  if (state != 0) {
    snprintf(l->err, sizeof l->err, "drive not stopped; feed-forward change refused");
    return -1;
  }
  return ff_write_words(l, ff);
}

int drive_ff_clear(ik_link_t *l)
{
  drive_ff_t off;
  drive_ff_off(&off);
  return ff_write_words(l, &off);
}

void drive_ff_json(FILE *f, const drive_ff_t *ff)
{
  fprintf(f, "{\"enable\":%s,\"coulomb_xy_n\":%.3f,\"viscous_xy_n_per_mps\":%.3f,\"coulomb_w_nm\":%.3f,"
             "\"viscous_w_nm_per_radps\":%.3f,\"a_per_nm\":%.3f,\"scale_pct\":[%d,%d,%d,%d]}",
          ff->enable ? "true" : "false", ff->coulomb_xy_mn / 1000.0, ff->viscous_xy_mn_per_mps / 1000.0,
          ff->coulomb_w_mnm / 1000.0, ff->viscous_w_mnm_per_radps / 1000.0, ff->a_per_nm_milli / 1000.0,
          (int)ff->scale_pct[0], (int)ff->scale_pct[1], (int)ff->scale_pct[2], (int)ff->scale_pct[3]);
}
