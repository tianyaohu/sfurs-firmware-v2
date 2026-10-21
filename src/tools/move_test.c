/** @file move_test.c
 * @brief Guarded out-and-back moves with ESC telemetry and external-tracker timestamps.
 */
#include "drive_ops.h"
#include "drive_record.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
  const char *out = NULL, *axis = "vy", *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  double distance_m = 0.6, speed_mps = 0.2, pause_s = 1.5, min_vbus_v = 11;
  int reps = 1, accel_rpmps = 0, record_pre = 50, record_delay_ms = 0;
  bool first_back = false, record = false, check = false, dsc = false, no_camera = true;
  int kp = INT_MIN, ki = INT_MIN, ff_ma = INT_MIN, static_ma = INT_MIN, kinetic_ma = INT_MIN, edge = INT_MIN;
  int ff_run_ma = INT_MIN, from_ma = INT_MIN, cap_ma = INT_MIN, slope = INT_MIN, pcap_ma = INT_MIN;
  int decay = INT_MIN, watch_ms = INT_MIN, preload_pct = INT_MIN;
  drive_option options[] = {
    DRIVE_OPT("out", STRING, &out), DRIVE_OPT("no-camera", BOOL, &no_camera),
    DRIVE_OPT("axis", STRING, &axis), DRIVE_OPT("distance", DOUBLE, &distance_m),
    DRIVE_OPT("speed", DOUBLE, &speed_mps), DRIVE_OPT("reps", INT, &reps),
    DRIVE_OPT("accel", INT, &accel_rpmps), DRIVE_OPT("first-back", BOOL, &first_back),
    DRIVE_OPT("pause", DOUBLE, &pause_s), DRIVE_OPT("kp", INT, &kp), DRIVE_OPT("ki", INT, &ki),
    DRIVE_OPT("ff", INT, &ff_ma), DRIVE_OPT("ff-static", INT, &static_ma), DRIVE_OPT("ff-kinetic", INT, &kinetic_ma),
    DRIVE_OPT("edge-speed", INT, &edge), DRIVE_OPT("dsc", BOOL, &dsc),
    DRIVE_OPT("dsc-ff-run", INT, &ff_run_ma), DRIVE_OPT("dsc-from", INT, &from_ma),
    DRIVE_OPT("dsc-cap", INT, &cap_ma), DRIVE_OPT("dsc-slope", INT, &slope), DRIVE_OPT("dsc-pcap", INT, &pcap_ma),
    DRIVE_OPT("dsc-hold-decay", INT, &decay), DRIVE_OPT("dsc-watch", INT, &watch_ms),
    DRIVE_OPT("dsc-preload", INT, &preload_pct), DRIVE_OPT("record", BOOL, &record),
    DRIVE_OPT("record-pre", INT, &record_pre), DRIVE_OPT("record-delay", INT, &record_delay_ms),
    DRIVE_OPT("min-vbus", DOUBLE, &min_vbus_v), DRIVE_OPT("check", BOOL, &check),
    DRIVE_OPT("elf", STRING, &elf), DRIVE_OPT("sn", STRING, &sn)
  };
  size_t n_options = sizeof options / sizeof *options;
  int parsed = drive_parse(argc, argv, options, n_options);
  if (parsed) {
    if (parsed > 0) puts("move_test --out DIR [--no-camera] [--axis vx|vy|wz] [--distance 0.6] [--speed 0.2]\n"
                         "  [--reps 1 --first-back --pause 1.5 --accel RPMPS --kp N --ki N]\n"
                         "  [--ff MA | --ff-static MA --ff-kinetic MA] [--edge-speed 0|1]\n"
                         "  [--dsc --dsc-ff-run MA --dsc-from MA --dsc-cap MA --dsc-slope MA_PER_MS\n"
                         "   --dsc-pcap MA --dsc-hold-decay MA_PER_MS --dsc-watch MS --dsc-preload PCT]\n"
                         "  [--record --record-pre N --record-delay MS] [--min-vbus 11 --check --elf F --sn SERIAL]\n"
                         "Camera capture belongs to the external tracker. MOTION unless --check.");
    return parsed < 0;
  }
  if (!out || !(speed_mps > 0 && speed_mps <= 2) || !(distance_m > 0 && distance_m <= 1) ||
      reps < 1 || reps > 3 || distance_m / speed_mps > 5 || pause_s < 0 || min_vbus_v < 0 ||
      accel_rpmps < 0 || (strcmp(axis, "vx") && strcmp(axis, "vy") && strcmp(axis, "wz")) ||
      (edge != INT_MIN && edge != 0 && edge != 1)) {
    fprintf(stderr, "require --out, vx/vy/wz, 0 < speed <= 2, 0 < distance <= 1, 1 <= reps <= 3, distance/speed <= 5; nonnegative pause/accel/Vbus\n");
    return 1;
  }
  uint32_t record_word;
  if (ik_record_start_word(record_pre, record_delay_ms, &record_word)) {
    fprintf(stderr, "invalid recorder pre/delay\n"); return 1;
  }
  const int overrides[] = {kp, ki, static_ma == INT_MIN ? ff_ma : static_ma,
    kinetic_ma == INT_MIN ? ff_ma : kinetic_ma, edge, dsc ? 1 : INT_MIN,
    ff_run_ma, from_ma, cap_ma, slope, pcap_ma, decay, watch_ms, preload_pct};
  const char *keys[] = {"kp", "ki", "ff_static_ma", "ff_kinetic_ma", "speed_edge_est", "dsc_enable",
    "dsc_ff_run_ma", "dsc_start_from_ma", "dsc_start_cap_ma", "dsc_start_slope_ma", "dsc_pcap_ma",
    "dsc_hold_decay_ma", "dsc_watch_ms", "dsc_preload_pct"};
  drive_settings_t settings;
  if (drive_settings_defaults(&settings)) return 1;
  for (size_t k = 0; k < sizeof overrides / sizeof *overrides; k++) if (overrides[k] != INT_MIN) {
    if ((k >= 6 && !dsc) || drive_setting_set(&settings, keys[k], overrides[k])) {
      fprintf(stderr, "invalid setting %s (--dsc-* options need --dsc)\n", keys[k]); return 1;
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
  }
  (void)ik_write(&link, IK_ENABLE, 0);
  ik_link_close(&link);
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
        fputs("}\n", f);
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
