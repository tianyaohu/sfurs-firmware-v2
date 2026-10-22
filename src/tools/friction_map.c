/** @file friction_map.c
 * @brief Floor friction measurements through the normal Nucleo drive path (MOTION).
 *
 * friction_map --out DIR --axis vx|vy|wz --speeds=-0.1,-0.2,0.1,0.2
 *   [--distance 0.2 --transient 0.3 --pause 1 --min-vbus 11 --max-rpm 300]
 * friction_map --out DIR --breakaway [--wheel FR|BR|BL|FL|NE|SE|SW|NW|all]
 *   [--rate-ma-s 2000 --cap-ma 4000 --motion-rpm 20 --reps 3]
 * Both: [--check --elf F --sn SERIAL]. --check performs preflight without motion.
 *
 * Floor/mat, clear space: each listed speed drives out then back for distance/|speed|
 * (m and m/s for vx/vy; radians and rad/s for wz), <=1 m or 1 rad and <=5 s per leg.
 * Collection uses measured 50 Hz RPM/Iq after the transient and 0.2 s of constant,
 * unslewed reference. Reject <5 samples, sign reversals or speed SD >10% of mean.
 * Fits equal-weight segment means: Iq[mA] = Ic[mA]*sign(RPM) + b[mA/RPM]*RPM.
 * Breakaway ramps selected wheel(s) in both directions, zeros each after 3 fresh
 * consecutive motion replies, and stops between trials. Reports commanded current
 * at confirmation and measured Iq; a cap without motion is censored, not a threshold.
 * Heartbeat, ESC faults, fresh telemetry, Vbus and max RPM are guarded throughout.
 * Ctrl-C/SIGTERM/SIGHUP zero currents and enable, wait for STOPPED/IDLE and release SWD.
 */
#define _POSIX_C_SOURCE 200809L
#include "drive_ops.h"
#include "friction_run.h"
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define MAX_SPEEDS 16
#define MAX_REPS 20
static volatile sig_atomic_t interrupted;
static void interrupt_handler(int sig) { interrupted = sig; }

static int speeds_parse(const char *text, double speeds[MAX_SPEEDS], size_t *n)
{
  *n = 0;
  bool positive = false, negative = false;
  while (text && *text) {
    char *end; errno = 0;
    double value = strtod(text, &end);
    if (errno || end == text || !isfinite(value) || fabs(value) < 0.001 || fabs(value) > 2 ||
        (*end && *end != ',') || *n == MAX_SPEEDS) return -1;
    speeds[(*n)++] = value;
    positive |= value > 0; negative |= value < 0;
    if (!*end) break;
    text = end + 1;
    if (!*text) return -1;
  }
  return *n && positive && negative ? 0 : -1;
}

static unsigned wheels_parse(const char *text)
{
  if (!strcasecmp(text, "all")) return 15;
  for (int w = 0; w < 4; w++)
    if (!strcasecmp(text, IK_WHEEL_POS[w]) || !strcasecmp(text, IK_WHEEL_CORNER[w])) return 1U << w;
  return 0;
}

static void fit_json(FILE *f, friction_fit_t fit)
{
  fprintf(f, "{\"valid\":%s,\"points\":%zu,\"ic_ma\":", fit.valid ? "true" : "false", fit.points);
  drive_json_number(f, fit.valid ? fit.ic_ma : NAN);
  fputs(",\"b_ma_per_rpm\":", f); drive_json_number(f, fit.valid ? fit.b_ma_per_rpm : NAN);
  fputs(",\"residual_rms_ma\":", f); drive_json_number(f, fit.valid ? fit.residual_rms_ma : NAN);
  fputc('}', f);
}

int main(int argc, char **argv)
{
  const char *out = NULL, *axis = "vy", *speeds_text = "-0.1,-0.2,0.1,0.2", *wheel = "all";
  const char *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  double distance = 0.2, pause = 1, rate = 2000, cap = 4000, motion = 20;
  int reps = 3;
  bool breakaway = false, check = false;
  friction_limits_t limits = {.min_vbus_v = 11, .max_rpm = 300, .transient_s = 0.3, .interrupted = &interrupted};
  drive_option options[] = {
    DRIVE_OPT("out", STRING, &out), DRIVE_OPT("axis", STRING, &axis), DRIVE_OPT("speeds", STRING, &speeds_text),
    DRIVE_OPT("distance", DOUBLE, &distance), DRIVE_OPT("pause", DOUBLE, &pause),
    DRIVE_OPT("transient", DOUBLE, &limits.transient_s), DRIVE_OPT("min-vbus", DOUBLE, &limits.min_vbus_v),
    DRIVE_OPT("max-rpm", DOUBLE, &limits.max_rpm), DRIVE_OPT("breakaway", BOOL, &breakaway),
    DRIVE_OPT("wheel", STRING, &wheel), DRIVE_OPT("rate-ma-s", DOUBLE, &rate), DRIVE_OPT("cap-ma", DOUBLE, &cap),
    DRIVE_OPT("motion-rpm", DOUBLE, &motion), DRIVE_OPT("reps", INT, &reps), DRIVE_OPT("check", BOOL, &check),
    DRIVE_OPT("elf", STRING, &elf), DRIVE_OPT("sn", STRING, &sn)
  };
  int parsed = drive_parse(argc, argv, options, sizeof options / sizeof *options);
  if (parsed) {
    if (parsed > 0) puts("friction_map --out DIR [--axis vx|vy|wz --speeds=-0.1,-0.2,0.1,0.2 --distance 0.2]\n"
      "  [--transient 0.3 --pause 1 --min-vbus 11 --max-rpm 300]\n"
      "  [--breakaway --wheel FR|BR|BL|FL|NE|SE|SW|NW|all --rate-ma-s 2000 --cap-ma 4000 --motion-rpm 20 --reps 3]\n"
      "  [--check --elf F --sn SERIAL] Writes DIR/result.json. MOTION on floor unless --check.\n"
      "Speed list requires both signs; each speed gets a reverse return leg. wz units: radians, rad/s.");
    return parsed < 0;
  }
  double speeds[MAX_SPEEDS]; size_t n_speeds = 0;
  unsigned selected = wheels_parse(wheel);
  int a = !strcmp(axis, "vx") ? 0 : !strcmp(axis, "vy") ? 1 : !strcmp(axis, "wz") ? 2 : -1;
  if (!out || !*out || a < 0 || !selected || distance <= 0 || distance > 1 || pause < 0 || pause > 10 ||
      limits.min_vbus_v < 8 || limits.max_rpm <= 0 || limits.max_rpm > 600 ||
      limits.transient_s < 0 || limits.transient_s > 4 || reps < 1 || reps > MAX_REPS ||
      rate <= 0 || cap <= 0 || cap > 6000 || cap / rate > 5 || motion <= 0 || motion >= limits.max_rpm ||
      (!breakaway && speeds_parse(speeds_text, speeds, &n_speeds))) {
    fprintf(stderr, "invalid friction_map arguments: require --out, axis vx/vy/wz, speeds of both signs, distance <=1, Vbus >=8, RPM <=600, cap <=6000, ramp <=5s\n");
    return 1;
  }
  for (size_t i = 0; i < n_speeds; i++) if (distance / fabs(speeds[i]) > 5 ||
      distance / fabs(speeds[i]) < limits.transient_s + 0.35) {
    fprintf(stderr, "each leg must last transient+0.35 to 5 seconds; adjust distance/speeds\n"); return 1;
  }
  if (drive_mkdir(out)) { perror(out); return 1; }
  char path[4096];
  if (drive_path(path, sizeof path, out, "result.json")) return 1;
  ik_link_t link;
  if (ik_link_open(&link, sn, elf)) { fprintf(stderr, "%s\n", link.err); return 1; }
  struct sigaction action = {.sa_handler = interrupt_handler}, old_int, old_term, old_hup;
  sigemptyset(&action.sa_mask); interrupted = 0;
  sigaction(SIGINT, &action, &old_int); sigaction(SIGTERM, &action, &old_term); sigaction(SIGHUP, &action, &old_hup);
  uint32_t state;
  int rc = ik_read(&link, IK_STATE, &state);
  if (!rc && state != 0) { snprintf(link.err, sizeof link.err, "drive must be STOPPED"); rc = -1; }
  if (!rc) rc = drive_preflight(&link, limits.min_vbus_v);
  if (!rc && check) puts("preflight passed (no motion: --check)");
  friction_point_t points[4][MAX_SPEEDS * 2] = {{{0}}}; size_t n_points = 0;
  double commands[MAX_SPEEDS * 2] = {0};
  friction_breakaway_t trials[4][2][MAX_REPS] = {{{{0}}}};
  bool acquired = !rc && !check;
  char error[sizeof link.err] = "";
  if (acquired && !breakaway) for (size_t i = 0; i < n_speeds && !rc && !interrupted; i++) {
    for (int leg = 0; leg < 2 && !rc && !interrupted; leg++) {
      if ((rc = drive_preflight(&link, limits.min_vbus_v))) break;
      double speed = speeds[i] * (leg ? -1 : 1);
      int32_t twist[3] = {0}; twist[a] = (int32_t)nearbyint(speed * 1000);
      friction_point_t measured[4];
      rc = friction_speed_move(&link, twist, distance / fabs(speed), &limits, measured);
      commands[n_points] = speed;
      for (int w = 0; w < 4; w++) {
        points[w][n_points] = measured[w];
        printf("%s speed=%g: %s n=%zu RPM=%.2f sd=%.2f Iq=%.1f mA\n", IK_WHEEL_POS[w], speed,
          friction_verdict_name(measured[w].verdict), measured[w].samples, measured[w].rpm, measured[w].speed_sd, measured[w].iq_ma);
      }
      n_points++;
      double end = pulse_now_s() + pause;
      while (!rc && !interrupted && pulse_now_s() < end) pulse_sleep_s(0.02);
    }
  }
  if (acquired && breakaway) for (int d = 0; d < 2 && !rc && !interrupted; d++)
    for (int rep = 0; rep < reps && !rc && !interrupted; rep++) {
      if ((rc = drive_preflight(&link, limits.min_vbus_v))) break;
      friction_breakaway_t measured[4];
      rc = friction_breakaway_run(&link, selected, d ? 1 : -1, rate, cap, motion, &limits, measured);
      for (int w = 0; w < 4; w++) trials[w][d][rep] = measured[w];
      double end = pulse_now_s() + pause;
      while (!rc && !interrupted && pulse_now_s() < end) pulse_sleep_s(0.02);
    }
  if (interrupted) { snprintf(link.err, sizeof link.err, "interrupted; stopping"); rc = -1; }
  if (rc) snprintf(error, sizeof error, "%s", link.err);
  if (acquired && friction_stop(&link)) { fprintf(stderr, "%s\n", link.err); if (!rc) snprintf(error, sizeof error, "%s", link.err); rc = -1; }
  if (acquired && ik_set_mode(&link, IK_MODE_SPEED)) { if (!rc) snprintf(error, sizeof error, "%s", link.err); rc = -1; }
  ik_link_close(&link);
  sigaction(SIGINT, &old_int, NULL); sigaction(SIGTERM, &old_term, NULL); sigaction(SIGHUP, &old_hup, NULL);
  if (rc) fprintf(stderr, "%s\n", error);
  if (!acquired) return rc != 0;
  FILE *f = fopen(path, "w");
  if (!f) { perror(path); return 1; }
  fprintf(f, "{\"mode\":\"%s\",\"axis\":\"%s\",\"complete\":%s,\"error\":", breakaway ? "breakaway" : "speed", axis, rc ? "false" : "true");
  drive_json_string(f, rc ? error : NULL);
  fputs(",\"args\":", f); drive_args_json(f, options, sizeof options / sizeof *options);
  fputs(",\"fit_definition\":\"Iq_mA = Ic_mA*sign(RPM) + b_mA_per_RPM*RPM; equal weight per accepted segment\",\"wheels\":{", f);
  bool insufficient = false;
  if (!breakaway) puts("wheel direction points Ic_mA b_mA_per_RPM residual_RMS_mA");
  else puts("wheel direction detected/trials command_mA_median min max");
  for (int w = 0; w < 4; w++) {
    fprintf(f, "%s\"%s\":{", w ? "," : "", IK_WHEEL_POS[w]);
    if (!breakaway) {
      const char *names[] = {"both", "negative", "positive"}; const int directions[] = {0, -1, 1};
      for (int d = 0; d < 3; d++) {
        friction_fit_t fit = friction_fit(points[w], n_points, directions[d]);
        if (!fit.valid) insufficient = true;
        fprintf(f, "%s\"%s\":", d ? "," : "", names[d]); fit_json(f, fit);
        if (fit.valid) printf("%s %s %zu %.2f %.5f %.2f\n", IK_WHEEL_POS[w], names[d], fit.points, fit.ic_ma, fit.b_ma_per_rpm, fit.residual_rms_ma);
        else printf("%s %s %zu insufficient distinct steady speeds\n", IK_WHEEL_POS[w], names[d], fit.points);
      }
      fputs(",\"points\":[", f);
      for (size_t i = 0; i < n_points; i++) {
        const friction_point_t *p = &points[w][i];
        fprintf(f, "%s{\"command\":%.9g,\"rpm\":%.9g,\"iq_ma\":%.9g,\"speed_sd\":%.9g,\"samples\":%zu,\"verdict\":\"%s\"}",
          i ? "," : "", commands[i], p->rpm, p->iq_ma, p->speed_sd, p->samples, friction_verdict_name(p->verdict));
      }
      fputs("]}", f);
    } else {
      fprintf(f, "\"selected\":%s,\"directions\":[", selected & (1U << w) ? "true" : "false");
      for (int d = 0; d < 2; d++) {
        double values[MAX_REPS], min = INFINITY, max = -INFINITY; size_t count = 0;
        for (int rep = 0; rep < reps; rep++) if (trials[w][d][rep].moved) {
          double v = trials[w][d][rep].command_ma; values[count++] = v;
          min = fmin(min, v); max = fmax(max, v);
        }
        if ((selected & (1U << w)) && count != (size_t)reps) insufficient = true;
        double median = drive_median(values, count);
        fprintf(f, "%s{\"sign\":%d,\"detected\":%zu,\"median_ma\":", d ? "," : "", d ? 1 : -1, count);
        drive_json_number(f, median); fputs(",\"min_ma\":", f); drive_json_number(f, count ? min : NAN);
        fputs(",\"max_ma\":", f); drive_json_number(f, count ? max : NAN); fputs(",\"trials\":[", f);
        for (int rep = 0; rep < reps; rep++) {
          const friction_breakaway_t *t = &trials[w][d][rep];
          fprintf(f, "%s{\"attempted\":%s,\"capped\":%s,\"moved\":%s,\"command_ma\":",
                  rep ? "," : "", t->attempted ? "true" : "false", t->capped ? "true" : "false", t->moved ? "true" : "false");
          drive_json_number(f, t->moved ? t->command_ma : NAN); fputs(",\"measured_ma\":", f);
          drive_json_number(f, t->moved ? t->measured_ma : NAN); fputc('}', f);
        }
        fputs("]}", f);
        if (selected & (1U << w)) {
          if (count) printf("%s %c %zu/%d %.0f %.0f %.0f\n", IK_WHEEL_POS[w], d ? '+' : '-', count, reps, median, min, max);
          else printf("%s %c 0/%d no confirmed motion; cap %.0f mA\n", IK_WHEEL_POS[w], d ? '+' : '-', reps, cap);
        }
      }
      fputs("]}", f);
    }
  }
  fprintf(f, "},\"sufficient_data\":%s}\n", insufficient ? "false" : "true");
  if (ferror(f)) rc = -1;
  if (fclose(f)) rc = -1;
  puts(path);
  return rc != 0 || insufficient;
}
