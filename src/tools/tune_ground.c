/** @file tune_ground.c
 * @brief Ground friction identification: per-wheel breakaway and kinetic current from guarded
 *        ESC telemetry with the boot speed-loop gains (or --kp/--ki). It sets no feed-forward:
 *        the ESC has no per-wheel friction settings (motor_speed.c); the numbers inform the
 *        stall ramp and the Nucleo feed-forward.
 */
#include "drive_ops.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static void values_json(FILE *f, const tune_values *v)
{
  fputc('[', f);
  for (size_t i = 0; i < v->count; i++) {
    if (i) fputc(',', f);
    drive_json_number(f, v->data[i]);
  }
  fputc(']', f);
}

static int save(const char *directory, const drive_option *options, size_t n_options,
                 const drive_stage_t *stage, const drive_settings_t *settings,
                 const tune_friction friction[4], const esc_move_t *plan, size_t n_moves)
{
  char path[4096];
  if (drive_path(path, sizeof path, directory, "result.json")) return -1;
  FILE *f = fopen(path, "w");
  if (!f) { perror(path); return -1; }
  fputs("{\"date\":", f); drive_date_json(f);
  fputs(",\"args\":", f); drive_args_json(f, options, n_options);
  fputs(",\"stages\":[{", f);
  drive_stage_json(f, "identify", stage, settings, plan, n_moves);
  fputs(",\"friction\":{", f);
  for (int w = 0; w < 4; w++) {
    fprintf(f, "%s\"%s\":{\"breakaway\":", w ? "," : "", IK_WHEEL_POS[w]);
    values_json(f, &friction[w].breakaway);
    fputs(",\"kinetic\":", f); values_json(f, &friction[w].kinetic);
    fputc('}', f);
  }
  fputs("}}]}\n", f);
  return fclose(f) ? -1 : 0;
}

int main(int argc, char **argv)
{
  const char *out = NULL, *axes = "both", *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  bool no_camera = true;
  double speed_mps = 0.2, x_speed_mps = 0.3, pause_s = 1.5, seconds = 0.8;
  double max_spread = 0.2, min_vbus_v = 11;
  int reps = 2, kp = INT_MIN, ki = INT_MIN;
  drive_option options[] = {
    DRIVE_OPT("out", STRING, &out), DRIVE_OPT("no-camera", BOOL, &no_camera),
    DRIVE_OPT("speed", DOUBLE, &speed_mps), DRIVE_OPT("x-speed", DOUBLE, &x_speed_mps),
    DRIVE_OPT("pause", DOUBLE, &pause_s), DRIVE_OPT("seconds", DOUBLE, &seconds),
    DRIVE_OPT("reps", INT, &reps), DRIVE_OPT("axes", STRING, &axes),
    DRIVE_OPT("kp", INT, &kp), DRIVE_OPT("ki", INT, &ki), DRIVE_OPT("max-spread", DOUBLE, &max_spread),
    DRIVE_OPT("min-vbus", DOUBLE, &min_vbus_v), DRIVE_OPT("elf", STRING, &elf), DRIVE_OPT("sn", STRING, &sn)
  };
  size_t n_options = sizeof options / sizeof *options;
  int parsed = drive_parse(argc, argv, options, n_options);
  if (parsed) {
    if (parsed > 0) puts("tune_ground --out DIR [--no-camera] [--speed 0.2 --x-speed 0.3 --seconds 0.8]\n"
                         "  [--axes both|vy|vx --reps 2 --pause 1.5 --kp N --ki N --max-spread 0.2]\n"
                         "  [--min-vbus 11 --elf F --sn SERIAL]\n"
                         "MOTION: measure per-wheel breakaway and kinetic current, restore RAM defaults after.\n"
                         "Camera capture belongs to the external tracker.");
    return parsed < 0;
  }
  if (!out || !(speed_mps > 0 && speed_mps <= 0.3) || !(x_speed_mps > 0 && x_speed_mps <= 0.3) ||
      !(seconds > 0 && seconds <= 2) || reps < 1 || reps > 5 || pause_s < 0 || min_vbus_v < 0 ||
      (strcmp(axes, "both") && strcmp(axes, "vx") && strcmp(axes, "vy"))) {
    fprintf(stderr, "require --out, axes both/vx/vy, 0 < speeds <= 0.3, 0 < seconds <= 2, 1 <= reps <= 5; nonnegative pause/Vbus\n");
    return 1;
  }
  drive_settings_t settings;
  if (drive_settings_defaults(&settings) || (kp != INT_MIN && drive_setting_set(&settings, "kp", kp)) ||
      (ki != INT_MIN && drive_setting_set(&settings, "ki", ki))) {
    fprintf(stderr, "invalid gains/defaults\n"); return 1;
  }
  int32_t twists[DRIVE_MAX_MOVES][3] = {{0}};
  size_t n_moves = 0;
  for (int r = 0; r < reps; r++) {
    if (strcmp(axes, "vx")) {
      twists[n_moves++][1] = (int32_t)nearbyint(speed_mps * 1000);
      twists[n_moves++][1] = -(int32_t)nearbyint(speed_mps * 1000);
    }
    if (strcmp(axes, "vy")) {
      twists[n_moves++][0] = (int32_t)nearbyint(x_speed_mps * 1000);
      twists[n_moves++][0] = -(int32_t)nearbyint(x_speed_mps * 1000);
    }
  }
  esc_move_t plan[DRIVE_MAX_MOVES];
  double duration_s = esc_pulse_plan((const int32_t (*)[3])twists, n_moves, seconds, pause_s, 0.5, plan);
  if (drive_mkdir(out)) { perror(out); return 1; }
  ik_link_t link;
  if (ik_link_open(&link, sn, elf)) { fprintf(stderr, "%s\n", link.err); return 1; }
  drive_stage_t stage = {0};
  double *friction_storage = NULL;
  tune_friction friction[4] = {0};
  int rc = drive_preflight(&link, min_vbus_v);
  if (rc) goto done;
  rc = drive_stage_run(&link, out, "identify", plan, n_moves, duration_s, &settings, 0, &stage);
  if (rc) goto done;
  size_t samples = stage.pulse.n + 1;
  friction_storage = calloc(8 * samples, sizeof *friction_storage);
  if (!friction_storage) { rc = -1; goto done; }
  for (int w = 0; w < 4; w++) {
    friction[w].breakaway = (tune_values){friction_storage + (2 * w) * samples, samples, 0};
    friction[w].kinetic = (tune_values){friction_storage + (2 * w + 1) * samples, samples, 0};
  }
  rc = tune_friction_samples(stage.analysis, stage.pulse.n, 40, 0.3, friction);
  if (rc || save(out, options, n_options, &stage, &settings, friction, plan, n_moves)) { rc = -1; goto done; }
  drive_response_show(&stage.response);
  if (stage.pulse.aborted[0] || stage.faults[0]) {
    fprintf(stderr, "identify: aborted %s faults %s\n", stage.pulse.aborted, stage.faults);
    rc = -1; goto done;
  }
  for (int w = 0; w < 4; w++) {
    double spread = tune_spread(friction[w].breakaway.data, friction[w].breakaway.count);
    printf("   %s: breakaway %.2f A (n %zu, spread %.0f%%) kinetic %.2f A (n %zu)\n", IK_WHEEL_POS[w],
           drive_median(friction[w].breakaway.data, friction[w].breakaway.count), friction[w].breakaway.count,
           spread * 100, drive_median(friction[w].kinetic.data, friction[w].kinetic.count), friction[w].kinetic.count);
    if (!isfinite(spread) || spread > max_spread) {
      fprintf(stderr, "identify: %s breakaway not repeatable; check wheel and surface\n", IK_WHEEL_POS[w]);
      rc = -1;
    }
  }
done:
  if (rc && link.err[0]) fprintf(stderr, "%s\n", link.err);
  (void)ik_write(&link, IK_ENABLE, 0);
  ik_link_close(&link);
  drive_stage_free(&stage);
  free(friction_storage);
  return rc != 0;
}
