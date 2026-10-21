/** @file tune_ground.c
 * @brief Ground feed-forward identification and tuning using guarded ESC telemetry.
 */
#include "drive_ops.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  char name[32];
  drive_stage_t stage;
  drive_settings_t settings;
  double fraction;
  tune_judgement judgement;
} attempt_t;

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
                 const tune_limits *limits, const attempt_t *stages, size_t n_stages,
                 const tune_friction friction[4], const esc_move_t *plan, size_t n_moves, int chosen)
{
  char path[4096];
  if (drive_path(path, sizeof path, directory, "result.json")) return -1;
  FILE *f = fopen(path, "w");
  if (!f) { perror(path); return -1; }
  fputs("{\"date\":", f); drive_date_json(f);
  fputs(",\"args\":", f); drive_args_json(f, options, n_options);
  fprintf(f, ",\"limits\":{\"breakaway_s\":%.9g,\"spread_s\":%.9g,\"overshoot\":%.9g,"
             "\"direction_deg\":3,\"heading_deg\":2,\"ratio_spread\":0.05},\"stages\":[",
          limits->breakaway_s, limits->spread_s, limits->overshoot);
  for (size_t k = 0; k < n_stages; k++) {
    const attempt_t *a = &stages[k];
    fprintf(f, "%s{", k ? "," : "");
    drive_stage_json(f, a->name, &a->stage, &a->settings, plan, n_moves);
    if (!k) {
      fputs(",\"friction\":{", f);
      for (int w = 0; w < 4; w++) {
        fprintf(f, "%s\"%s\":{\"breakaway\":", w ? "," : "", IK_WHEEL_POS[w]);
        values_json(f, &friction[w].breakaway);
        fputs(",\"kinetic\":", f); values_json(f, &friction[w].kinetic);
        fputc('}', f);
      }
      fputc('}', f);
    } else fprintf(f, ",\"frac\":%.9g,\"slow\":%s,\"overshooting\":%s", a->fraction,
                   a->judgement.slow ? "true" : "false", a->judgement.over ? "true" : "false");
    fputc('}', f);
  }
  fputs("],\"chosen\":", f);
  if (chosen < 0) fputs("null", f);
  else {
    const attempt_t *a = &stages[chosen];
    fprintf(f, "{\"frac\":%.9g,\"wheels\":{", a->fraction);
    for (int w = 0; w < 4; w++) fprintf(f, "%s\"%s\":{\"ff_static_ma\":%d,\"ff_kinetic_ma\":%d}",
        w ? "," : "", IK_WHEEL_POS[w], a->settings.value[w][2], a->settings.value[w][3]);
    fputs("}}", f);
  }
  fputs("}\n", f);
  return fclose(f) ? -1 : 0;
}

int main(int argc, char **argv)
{
  const char *out = NULL, *axes = "both", *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  bool no_camera = true;
  double speed_mps = 0.2, x_speed_mps = 0.3, pause_s = 1.5, seconds = 0.8, fraction = 0.65;
  double max_spread = 0.2, breakaway_ms = 80, spread_ms = 30, overshoot = 1.3, min_vbus_v = 11;
  int reps = 2, kp = INT_MIN, ki = INT_MIN, max_tries = 4;
  drive_option options[] = {
    DRIVE_OPT("out", STRING, &out), DRIVE_OPT("no-camera", BOOL, &no_camera),
    DRIVE_OPT("speed", DOUBLE, &speed_mps), DRIVE_OPT("x-speed", DOUBLE, &x_speed_mps),
    DRIVE_OPT("pause", DOUBLE, &pause_s), DRIVE_OPT("seconds", DOUBLE, &seconds),
    DRIVE_OPT("reps", INT, &reps), DRIVE_OPT("axes", STRING, &axes),
    DRIVE_OPT("kp", INT, &kp), DRIVE_OPT("ki", INT, &ki), DRIVE_OPT("frac", DOUBLE, &fraction),
    DRIVE_OPT("max-tries", INT, &max_tries), DRIVE_OPT("max-spread", DOUBLE, &max_spread),
    DRIVE_OPT("breakaway-ms", DOUBLE, &breakaway_ms), DRIVE_OPT("spread-ms", DOUBLE, &spread_ms),
    DRIVE_OPT("overshoot", DOUBLE, &overshoot), DRIVE_OPT("min-vbus", DOUBLE, &min_vbus_v),
    DRIVE_OPT("elf", STRING, &elf), DRIVE_OPT("sn", STRING, &sn)
  };
  size_t n_options = sizeof options / sizeof *options;
  int parsed = drive_parse(argc, argv, options, n_options);
  if (parsed) {
    if (parsed > 0) puts("tune_ground --out DIR [--no-camera] [--speed 0.2 --x-speed 0.3 --seconds 0.8]\n"
                         "  [--axes both|vy|vx --reps 2 --pause 1.5 --kp N --ki N --frac 0.65 --max-tries 4]\n"
                         "  [--max-spread 0.2 --breakaway-ms 80 --spread-ms 30 --overshoot 1.3]\n"
                         "  [--min-vbus 11 --elf F --sn SERIAL]\n"
                         "MOTION: identify without feed-forward, tune per wheel, restore RAM defaults after each stage.\n"
                         "Camera capture belongs to the external tracker.");
    return parsed < 0;
  }
  if (!out || !(speed_mps > 0 && speed_mps <= 0.3) || !(x_speed_mps > 0 && x_speed_mps <= 0.3) ||
      !(seconds > 0 && seconds <= 2) || reps < 1 || reps > 5 || pause_s < 0 || min_vbus_v < 0 ||
      (strcmp(axes, "both") && strcmp(axes, "vx") && strcmp(axes, "vy"))) {
    fprintf(stderr, "require --out, axes both/vx/vy, 0 < speeds <= 0.3, 0 < seconds <= 2, 1 <= reps <= 5; nonnegative pause/Vbus\n");
    return 1;
  }
  drive_settings_t defaults;
  if (drive_settings_defaults(&defaults) || (kp != INT_MIN && drive_setting_set(&defaults, "kp", kp)) ||
      (ki != INT_MIN && drive_setting_set(&defaults, "ki", ki))) {
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
  tune_limits limits = {breakaway_ms / 1000, spread_ms / 1000, overshoot, 3, 2, 0.05};
  size_t capacity = max_tries > 0 ? (size_t)max_tries + 1 : 1;
  attempt_t *stages = calloc(capacity, sizeof *stages);
  if (!stages) { perror("stages"); return 1; }
  if (drive_mkdir(out)) { perror(out); free(stages); return 1; }
  ik_link_t link;
  if (ik_link_open(&link, sn, elf)) { fprintf(stderr, "%s\n", link.err); free(stages); return 1; }
  int rc = drive_preflight(&link, min_vbus_v), chosen = -1;
  size_t n_stages = 0;
  double *friction_storage = NULL;
  tune_friction friction[4] = {0};
  if (rc) goto done;
  attempt_t *identify = &stages[0];
  snprintf(identify->name, sizeof identify->name, "identify");
  identify->settings = defaults;
  drive_setting_set(&identify->settings, "ff_static_ma", 0);
  drive_setting_set(&identify->settings, "ff_kinetic_ma", 0);
  rc = drive_stage_run(&link, out, identify->name, plan, n_moves, duration_s, &identify->settings, 0, &identify->stage);
  n_stages = 1;
  if (rc) goto done;
  size_t samples = identify->stage.pulse.n + 1;
  friction_storage = calloc(8 * samples, sizeof *friction_storage);
  if (!friction_storage) { rc = -1; goto done; }
  for (int w = 0; w < 4; w++) {
    friction[w].breakaway = (tune_values){friction_storage + (2 * w) * samples, samples, 0};
    friction[w].kinetic = (tune_values){friction_storage + (2 * w + 1) * samples, samples, 0};
  }
  rc = tune_friction_samples(identify->stage.analysis, identify->stage.pulse.n, 40, 0.3, friction);
  if (rc || save(out, options, n_options, &limits, stages, n_stages, friction, plan, n_moves, chosen)) { rc = -1; goto done; }
  puts("feed-forward off:"); drive_response_show(&identify->stage.response);
  if (identify->stage.pulse.aborted[0] || identify->stage.faults[0]) {
    fprintf(stderr, "identify: aborted %s faults %s; not tuning\n", identify->stage.pulse.aborted, identify->stage.faults);
    rc = -1; goto done;
  }
  double kinetic_a[4], baseline_over = 0;
  for (int w = 0; w < 4; w++) {
    double spread = tune_spread(friction[w].breakaway.data, friction[w].breakaway.count);
    kinetic_a[w] = drive_median(friction[w].kinetic.data, friction[w].kinetic.count);
    printf("   %s: breakaway %.2f A (n %zu, spread %.0f%%) kinetic %.2f A (n %zu)\n", IK_WHEEL_POS[w],
           drive_median(friction[w].breakaway.data, friction[w].breakaway.count), friction[w].breakaway.count,
           spread * 100, kinetic_a[w], friction[w].kinetic.count);
    if (!isfinite(spread) || spread > max_spread) {
      fprintf(stderr, "identify: %s breakaway not repeatable; check wheel and surface before tuning\n", IK_WHEEL_POS[w]);
      rc = -1; goto done;
    }
    if (!isfinite(kinetic_a[w])) {
      fprintf(stderr, "identify: %s never tracked target; no kinetic friction to tune from\n", IK_WHEEL_POS[w]);
      rc = -1; goto done;
    }
    if (identify->stage.response.wheels[w].overshoot > baseline_over)
      baseline_over = identify->stage.response.wheels[w].overshoot;
  }
  for (size_t attempt = 1; attempt < capacity; attempt++) {
    attempt_t *a = &stages[n_stages];
    snprintf(a->name, sizeof a->name, "tune%zu", attempt);
    a->settings = defaults;
    a->fraction = fraction;
    for (int w = 0; w < 4; w++) {
      double ff_ma = nearbyint(1000 * fraction * kinetic_a[w]);
      if (!isfinite(ff_ma) || ff_ma < INT16_MIN || ff_ma > INT16_MAX) {
        fprintf(stderr, "feed-forward outside CONFIG_SET range\n"); rc = -1; goto done;
      }
      a->settings.value[w][2] = a->settings.value[w][3] = (int)ff_ma;
    }
    rc = drive_stage_run(&link, out, a->name, plan, n_moves, duration_s, &a->settings, 0, &a->stage);
    n_stages++;
    if (rc) goto done;
    a->judgement = tune_judge(&a->stage.response, &limits, baseline_over);
    if (!a->judgement.slow && !a->judgement.over && !a->stage.pulse.aborted[0] && !a->stage.faults[0])
      chosen = (int)n_stages - 1;
    if (save(out, options, n_options, &limits, stages, n_stages, friction, plan, n_moves, chosen)) { rc = -1; goto done; }
    printf("ff x%.2f of kinetic: %s%s%s\n", fraction, a->judgement.slow ? "slow " : "",
           a->judgement.over ? "overshooting " : "", a->stage.pulse.aborted);
    drive_response_show(&a->stage.response);
    if (a->stage.faults[0]) { fprintf(stderr, "%s: faults %s; stopping\n", a->name, a->stage.faults); rc = -1; goto done; }
    if (chosen >= 0) break;
    fraction = tune_next_fraction(fraction, a->judgement.slow, a->judgement.over || a->stage.pulse.aborted[0],
                                  0.15, 0.1, 0.2, 1.0);
    if (isnan(fraction)) break;
  }
  if (save(out, options, n_options, &limits, stages, n_stages, friction, plan, n_moves, chosen)) rc = -1;
  if (chosen < 0) puts("no setting met the limits; see result.json");
  else {
    printf("chosen ff x%.2f of kinetic:", stages[chosen].fraction);
    for (int w = 0; w < 4; w++) printf(" %s %d/%d mA", IK_WHEEL_POS[w],
        stages[chosen].settings.value[w][2], stages[chosen].settings.value[w][3]);
    puts("");
  }
done:
  if (rc && link.err[0]) fprintf(stderr, "%s\n", link.err);
  (void)ik_write(&link, IK_ENABLE, 0);
  ik_link_close(&link);
  for (size_t k = 0; k < capacity; k++) drive_stage_free(&stages[k].stage);
  free(stages);
  free(friction_storage);
  return rc != 0;
}
