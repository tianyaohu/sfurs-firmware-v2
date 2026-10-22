/** @file move_test.c
 * @brief Guarded out-and-back moves with ESC telemetry and external-tracker timestamps.
 */
#include "drive_ops.h"
#include "drive_record.h"
#include "startup_metrics.h"
#include "can_datatypes.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int main(int argc, char **argv)
{
  const char *out = NULL, *axis = "vy", *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  double distance_m = 0.6, speed_mps = 0.2, pause_s = 1.5, min_vbus_v = 11;
  int reps = 1, accel_rpmps = 0, record_pre = 50, record_delay_ms = 0;
  bool first_back = false, record = false, check = false, no_camera = true;
  bool startup_metrics = false;
  double sector_spread_deg = 0;
  int kp = INT_MIN, ki = INT_MIN;
  drive_option options[] = {
    DRIVE_OPT("out", STRING, &out), DRIVE_OPT("no-camera", BOOL, &no_camera),
    DRIVE_OPT("axis", STRING, &axis), DRIVE_OPT("distance", DOUBLE, &distance_m),
    DRIVE_OPT("speed", DOUBLE, &speed_mps), DRIVE_OPT("reps", INT, &reps),
    DRIVE_OPT("accel", INT, &accel_rpmps), DRIVE_OPT("first-back", BOOL, &first_back),
    DRIVE_OPT("pause", DOUBLE, &pause_s), DRIVE_OPT("kp", INT, &kp), DRIVE_OPT("ki", INT, &ki),
    DRIVE_OPT("record", BOOL, &record),
    DRIVE_OPT("record-pre", INT, &record_pre), DRIVE_OPT("record-delay", INT, &record_delay_ms),
    DRIVE_OPT("startup-metrics", BOOL, &startup_metrics),
    DRIVE_OPT("sector-spread-deg", DOUBLE, &sector_spread_deg),
    DRIVE_OPT("min-vbus", DOUBLE, &min_vbus_v), DRIVE_OPT("check", BOOL, &check),
    DRIVE_OPT("elf", STRING, &elf), DRIVE_OPT("sn", STRING, &sn)
  };
  size_t n_options = sizeof options / sizeof *options;
  int parsed = drive_parse(argc, argv, options, n_options);
  if (parsed) {
    if (parsed > 0) puts("move_test --out DIR [--no-camera] [--axis vx|vy|wz] [--distance 0.6] [--speed 0.2]\n"
                         "  [--reps 1 --first-back --pause 1.5 --accel RPMPS --kp N --ki N]\n"
                         "  [--record --record-pre N --record-delay MS] [--min-vbus 11 --check --elf F --sn SERIAL]\n"
                         "  [--startup-metrics --record --sector-spread-deg 0]\n"
                         "Camera capture belongs to the external tracker. MOTION unless --check.");
    return parsed < 0;
  }
  if ((startup_metrics && !record) ||
      !isfinite(sector_spread_deg) || sector_spread_deg < 0) {
    fprintf(stderr, "--startup-metrics requires --record; sector spread must be finite and nonnegative\n");
    return 1;
  }
  /* wz is in rad/s and rad; the acceptance gate needs 4.8 rad/s (120 RPM peak wheel), within the Nucleo's 6. */
  const int yaw = axis && !strcmp(axis, "wz");
  const double max_speed = yaw ? 6.0 : 2.0, max_distance = yaw ? 3.0 : 1.0;
  if (!out || !(speed_mps > 0 && speed_mps <= max_speed) || !(distance_m > 0 && distance_m <= max_distance) ||
      reps < 1 || reps > 3 || distance_m / speed_mps > 5 || pause_s < 0 || min_vbus_v < 0 ||
      accel_rpmps < 0 || (strcmp(axis, "vx") && strcmp(axis, "vy") && strcmp(axis, "wz"))) {
    fprintf(stderr, "require --out, vx/vy/wz, 0 < speed <= 2 m/s (wz 6 rad/s), 0 < distance <= 1 m (wz 3 rad), "
                    "1 <= reps <= 3, distance/speed <= 5; nonnegative pause/accel/Vbus\n");
    return 1;
  }
  uint32_t record_word;
  if (ik_record_start_word(record_pre, record_delay_ms, &record_word)) {
    fprintf(stderr, "invalid recorder pre/delay\n"); return 1;
  }
  const int overrides[] = {kp, ki};
  const char *keys[] = {"kp", "ki"};
  drive_settings_t settings;
  if (drive_settings_defaults(&settings)) return 1;
  for (size_t k = 0; k < sizeof overrides / sizeof *overrides; k++) if (overrides[k] != INT_MIN) {
    if (drive_setting_set(&settings, keys[k], overrides[k])) {
      fprintf(stderr, "invalid setting %s\n", keys[k]); return 1;
    }
  }
  int32_t twists[6][3] = {{0}};
  int a = !strcmp(axis, "vx") ? 0 : !strcmp(axis, "vy") ? 1 : 2;
  for (int k = 0; k < reps * 2; k++) twists[k][a] = (int32_t)nearbyint(speed_mps * 1000) *
                                                              (k % 2 ? -1 : 1) * (first_back ? -1 : 1);
  esc_move_t plan[6];
  size_t n_moves = (size_t)reps * 2;
  double duration_s = esc_pulse_plan((const int32_t (*)[3])twists, n_moves, distance_m / speed_mps, pause_s, 0.5, plan);
  if (drive_mkdir(out)) { perror(out); return 1; }
  ik_link_t link;
  if (ik_link_open(&link, sn, elf)) { fprintf(stderr, "%s\n", link.err); return 1; }
  int rc = drive_preflight(&link, min_vbus_v);
  drive_stage_t stage = {0};
  drive_records_t records = {0};
  startup_result_t startup[4];
  startup_sync_t sync = {STARTUP_NOT_MEASURABLE, NAN};
  char startup_run_id[96] = "";
  startup_calibration_t startup_cal = startup_nominal(sector_spread_deg);
  for (int w = 0; w < 4; ++w) startup[w] = startup_evaluate(NULL, 0, &startup_cal);
  if (rc || check) {
    if (!rc) puts("preflight passed (no motion: --check)");
    ik_link_close(&link);
    return rc != 0;
  }
  if (record) for (int w = 0; w < 4 && !rc; w++)
    rc = ik_record_start(&link, IK_WHEEL_SELECT[w], record_pre, record_delay_ms);
  if (!rc) rc = drive_stage_run(&link, out, "drive", plan, n_moves, duration_s, &settings, accel_rpmps, &stage);
  if (rc) fprintf(stderr, "%s\n", link.err);
  char path[4096];
  if (record && stage.pulse.t0_monotonic_s > 0) {
    if (drive_path(path, sizeof path, out, "record.json") ||
        drive_records_read(&link, path, stage.pulse.t0_monotonic_s, &records)) rc = -1;
    /* The reader retains summaries only. A DONE recorder holds this same start
     * until rearmed, so a second read supplies rows without changing its API. */
    if (startup_metrics) for (int w = 0; w < 4; ++w) {
      ik_record_sample_t packed[ESC_RECORD_SAMPLES];
      ik_record_unpacked_t rows[ESC_RECORD_SAMPLES];
      int n = 0, trigger = 0;
      if (!records.valid[w] || ik_record_read(&link, IK_WHEEL_SELECT[w], packed, &n, &trigger)) {
        rc = -1;
        continue;
      }
      if (n <= 0 || n > STARTUP_MAX_SAMPLES || trigger < 0 || trigger >= n) { rc = -1; continue; }
      for (int i = 0; i < n; ++i) ik_unpack_record(&packed[i], &rows[i]);
      startup[w] = startup_evaluate(rows, (size_t)n, &startup_cal);
    }
  }
  if (startup_metrics) {
    ik_link_row_t links[IK_WHEELS];
    if (ik_link_rows(&link, links)) { fprintf(stderr, "%s\n", link.err); rc = -1; }
    else sync = startup_link_synchronization(startup, links);
  }
  (void)ik_write(&link, IK_ENABLE, 0);
  ik_link_close(&link);
  if (startup_metrics) {
    snprintf(startup_run_id, sizeof startup_run_id, "%lld-%.9f", (long long)time(NULL), stage.pulse.t0_monotonic_s);
    if (drive_path(path, sizeof path, out, "startup.tsv")) rc = -1;
    else {
      FILE *tsv = fopen(path, "a+");
      if (!tsv) { perror(path); rc = -1; }
      else {
        if (fseek(tsv, 0, SEEK_END)) rc = -1;
        if (ftell(tsv) == 0)
          fputs("# run_id\taxis\tsign\twheel\tverdict\tfirst_failed_k\tdeadlines_met\tmeasurable_count\tovershoot_ratio\tsteady_rpm\tfailed_rule\tapply_skew_us\n", tsv);
        for (int w = 0; w < 4; ++w) {
          const startup_result_t *s = &startup[w];
          fprintf(tsv, "%s\t%s\t%s\t%s\t%s\t%d\t%u\t%u\t",
                  startup_run_id, axis, first_back ? "-" : "+",
                  IK_WHEEL_POS[w], startup_verdict_name(s->verdict), s->first_failed_k,
                  s->deadlines_met, s->measurable_count);
          if (isfinite(s->overshoot_ratio)) fprintf(tsv, "%.17g", s->overshoot_ratio);
          else fputs("na", tsv);
          fputc('\t', tsv);
          if (isfinite(s->steady_rpm)) fprintf(tsv, "%.17g", s->steady_rpm);
          else fputs("na", tsv);
          fprintf(tsv, "\t%s\t", startup_rule_name(s->rule));
          if (isfinite(sync.skew_ms)) fprintf(tsv, "%.9g", sync.skew_ms * 1000);
          else fputs("na", tsv);
          fputc('\n', tsv);
        }
        if (ferror(tsv)) rc = -1;
        if (fclose(tsv)) rc = -1;
      }
    }
    for (int w = 0; w < 4; ++w) {
      const startup_result_t *s = &startup[w];
      printf("startup %s %s rule=%s k=%d deadlines=%u/8 measurable=%u overshoot=",
             IK_WHEEL_POS[w], startup_verdict_name(s->verdict), startup_rule_name(s->rule),
             s->first_failed_k, s->deadlines_met, s->measurable_count);
      if (isfinite(s->overshoot_ratio)) printf("%.4f", s->overshoot_ratio); else printf("na");
      putchar('\n');
    }
    printf("startup synchronization %s skew_us=", startup_verdict_name(sync.verdict));
    if (isfinite(sync.skew_ms)) printf("%.9g", sync.skew_ms * 1000); else printf("na");
    putchar('\n');
  }
  if (stage.analysis) {
    if (drive_path(path, sizeof path, out, "result.json")) rc = -1;
    else {
      FILE *f = fopen(path, "w");
      if (!f) { perror(path); rc = -1; }
      else {
        fputs("{\"date\":", f); drive_date_json(f);
        fputs(",\"args\":", f); drive_args_json(f, options, n_options);
        fputc(',', f); drive_stage_json(f, "drive", &stage, &settings, plan, n_moves);
        if (record) { fputs(",\"record\":", f); drive_records_json(f, &records); }
        if (startup_metrics) {
          fprintf(f, ",\"startup\":{\"run_id\":\"%s\",\"axis\":\"%s\",\"sign\":\"%s\","
                     "\"sector_spread_deg\":%.17g,"
                     "\"synchronization\":\"%s\",\"apply_skew_us\":",
                  startup_run_id, axis, first_back ? "-" : "+", sector_spread_deg, startup_verdict_name(sync.verdict));
          drive_json_number(f, sync.skew_ms * 1000);
          fputs(",\"wheels\":{", f);
          for (int w = 0; w < 4; ++w) {
            const startup_result_t *s = &startup[w];
            fprintf(f, "%s\"%s\":{\"verdict\":\"%s\",\"tracking\":\"%s\",\"overshoot\":\"%s\","
                       "\"failed_rule\":\"%s\",\"sign\":%d,\"first_failed_k\":%d,\"deadlines_met\":%u,"
                       "\"measurable_count\":%u,\"overshoot_ratio\":", w ? "," : "", IK_WHEEL_POS[w],
                    startup_verdict_name(s->verdict), startup_verdict_name(s->tracking),
                    startup_verdict_name(s->overshoot), startup_rule_name(s->rule), s->sign,
                    s->first_failed_k, s->deadlines_met, s->measurable_count);
            drive_json_number(f, s->overshoot_ratio);
            fputs(",\"steady_rpm\":", f); drive_json_number(f, s->steady_rpm);
            fputs(",\"transitions\":[", f);
            for (int k = 0; k < STARTUP_TRANSITIONS; ++k) {
              const startup_transition_t *t = &s->transitions[k];
              fprintf(f, "%s{\"k\":%d,\"time_ms\":", k ? "," : "", k + 1);
              drive_json_number(f, t->time_ms);
              fprintf(f, ",\"direction\":%d,\"distance_deg\":%.17g,\"deadline_ms\":", t->direction, t->distance_deg);
              drive_json_number(f, t->deadline_ms);
              fputs(",\"tolerance_ms\":", f); drive_json_number(f, t->tolerance_ms);
              fprintf(f, ",\"measurable\":%s,\"met\":%s}", t->measurable ? "true" : "false", t->met ? "true" : "false");
            }
            fputs("]}", f);
          }
          fputs("}}", f);
        }
        fputs("}\n", f);
        if (ferror(f)) rc = -1;
        if (fclose(f)) rc = -1;
        puts(path);
      }
    }
    drive_response_show(&stage.response);
    printf("aborted %s faults %s Vbus min %.2f V\n", stage.pulse.aborted, stage.faults, stage.vbus_min_v);
  }
  drive_stage_free(&stage);
  return rc != 0;
}
