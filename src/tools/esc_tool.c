/*
 * ESC setup through the Nucleo streaming IK image (drive must be stopped).
 *
 *   esc_tool [--elf F] [--sn SERIAL] status
 *   esc_tool align FR|BR|BL|FL|NE|SE|SW|NW|all   measure + apply Hall phase shift (RAM)
 *   esc_tool set WHEEL KEY VALUE                 CONFIG_SET key (RAM), see ik_link.c for keys
 *   esc_tool capture WHEEL [--step-ma 3000] [--out f.json]
 *                                                current-loop step test: R, L, step response, PI gains
 *   esc_tool spin WHEEL --rpm 60 [--seconds 3] [--kp N --ki N] [--log f.jsonl]
 *                                                one wheel at a speed setpoint (motion!)
 *   esc_tool pulse [--axis vy|vx] [--speed 0.2] [--accel RPMPS] [--seconds 1.5] [--reps 3]
 *                  [--kp N --ki N] [--ff-static MA] [--ff-kinetic MA] [--log f.jsonl]
 *                                                scripted ground drive out and back (motion!)
 *   esc_tool stop                                broadcast DISABLE
 *   esc_tool clear                               clear latched ESC faults while stopped
 *
 * align turns the wheel about 1/4 revolution forward and back at 2 A: battery ON, wheels off the
 * ground. Results are lost on ESC reset; bake them into the per-wheel build (scripts/esc_build.sh
 * --hall-shift). capture holds the rotor with d-axis current (1 A, then the step): it may click into
 * position but makes no torque; battery ON. spin arms and streams SPEED to one wheel; --kp/--ki are
 * set in RAM first and the firmware defaults (drive_parameters.h) restored afterwards; DISABLE on exit,
 * and the ESC stops itself 0.1 s after the stream. pulse drives the whole robot through the Nucleo:
 * each rep goes out along --axis for --seconds and back; gains and friction feed-forward are set in
 * RAM on all four wheels and the defaults restored afterwards. Ctrl-C or switching the battery off
 * stops it: the Nucleo brakes 0.3 s after the heartbeat stops.
 */
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "can_datatypes.h"
#include "capture_analysis.h"
#include "esc_defaults.h"
#include "esc_ops.h"
#include "esc_ctl.h"
#include "motor_protocol.h"
#include "ik_link.h"
#include "pulse_run.h"

typedef struct { const char *name; int index; uint32_t sel; } wheel_t;

#define now_s pulse_now_s
#define sleep_s pulse_sleep_s

static int die(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static int die(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fputc('\n', stderr);
  return 1;
}

/* Wheels named by position (FR..) or corner (NE..), or all four. Returns count, 0 if unknown. */
static int pick_wheels(const char *arg, wheel_t out[IK_WHEELS])
{
  int n = 0;
  for (int i = 0; i < IK_WHEELS; i++) {
    bool all = !strcasecmp(arg, "all");
    if (all || !strcasecmp(arg, IK_WHEEL_POS[i]) || !strcasecmp(arg, IK_WHEEL_CORNER[i])) {
      out[n++] = (wheel_t){all ? IK_WHEEL_POS[i] : (!strcasecmp(arg, IK_WHEEL_CORNER[i]) ? IK_WHEEL_CORNER[i]
                                                                                       : IK_WHEEL_POS[i]),
                           i, IK_WHEEL_SELECT[i]};
    }
  }
  return n;
}

static int send_config(ik_link_t *l, uint32_t sel, const char *key, int value)
{
  uint32_t id, word;
  if (ik_config_frame(sel, key, value, &id, &word)) {
    snprintf(l->err, sizeof l->err, "bad config %s=%d", key, value);
    return -1;
  }
  return ik_send_raw(l, id, word, 1.0);
}

static int cmd_status(ik_link_t *l)
{
  ik_row_t before[IK_WHEELS], rows[IK_WHEELS];
  if (ik_rows(l, before)) return die("%s", l->err);
  sleep_s(0.3);
  if (ik_rows(l, rows)) return die("%s", l->err);
  ik_link_row_t links[IK_WHEELS];
  ik_status_t status;
  if (ik_link_rows(l, links) || ik_status(l, &status)) return die("%s", l->err);
  for (int i = 0; i < IK_WHEELS; i++) {
    const ik_row_t *r = &rows[i];
    const ik_link_row_t *link = &links[i];
    const bool faulted = status.fault & (1U << i), lost = status.fault & (1U << (4 + i));
    printf("%s/%s: link %s/%s/%s session %u misses %u fault %u lost %u cause %s armed %u\n",
           IK_WHEEL_POS[i], IK_WHEEL_CORNER[i], link->valid ? link->lifecycle_name : "unreadable",
           link->valid ? link->outcome_name : "?", link->valid ? link->reason_name : "?",
           link->session, link->misses, faulted, lost,
           !(faulted || lost) ? "-" : !link->valid ? "?" : link->fault_reason ? link->fault_reason_name : lost ? "no replies" : "NONE",
           !!(status.armed & (1U << i)));
    /* rows keep the last values; only a new count means the ESC is live */
    if (!r->valid || !r->status_count || (before[i].valid && before[i].status_count == r->status_count)) {
      printf("%s/%s: no reply\n", IK_WHEEL_POS[i], IK_WHEEL_CORNER[i]);
      continue;
    }
    char app[32] = "?";
    if (r->ident_count) snprintf(app, sizeof app, "%08X/%u", r->app_crc, r->app_size);
    printf("%s/%s: %-10s rpm %5d  Iq %5.2f A  Vbus %5.2f V  faults %s  hall shift %d deg (%s)  align %s  stop %s  app %s\n",
           IK_WHEEL_POS[i], IK_WHEEL_CORNER[i], r->mc_state, r->rpm, r->iq_a, r->vbus_v,
           r->fault_names[0] ? r->fault_names : "-", r->hall_shift_deg, r->aligned ? "known" : "NOT ALIGNED",
           r->align, r->stop_reason, app);
  }
  return 0;
}

static int cmd_align(ik_link_t *l, const wheel_t *w, int n)
{
  int rc = 0;
  char flags[256] = "";
  for (int k = 0; k < n; k++) {
    ik_row_t rows[IK_WHEELS];
    if (ik_rows(l, rows)) return die("%s", l->err);
    ik_row_t before = rows[w[k].index];
    uint32_t count = before.valid ? before.diag_count : 0;
    if (ik_send_raw(l, HALL_ALIGN | w[k].sel, HALL_ALIGN_MAGIC, 1.0)) return die("%s", l->err);
    printf("%s: aligning...\n", w[k].name);
    fflush(stdout);
    sleep_s(0.3);
    double deadline = now_s() + 8.0;
    ik_row_t row;
    /* wait for a fresh diag reply, then for the alignment to finish */
    for (;;) {
      if (ik_rows(l, rows)) return die("%s", l->err);
      row = rows[w[k].index];
      if (row.valid && row.diag_count != count) break;
      if (now_s() > deadline) return die("%s: no telemetry reply from this ESC (powered? CAN connected?)", w[k].name);
      sleep_s(0.02);
    }
    while (row.align_result == ESC_ALIGN_RUNNING || row.aligning) {
      if (now_s() > deadline) return die("%s: alignment did not finish", w[k].name);
      sleep_s(0.1);
      if (ik_rows(l, rows)) return die("%s", l->err);
      if (rows[w[k].index].valid) row = rows[w[k].index];
    }
    printf("%s: result %s, hall shift %d deg (was %d)\n", w[k].name, row.align, row.hall_shift_deg,
           before.valid ? before.hall_shift_deg : 0);
    if (row.align_result == ESC_ALIGN_OK)
      snprintf(flags + strlen(flags), sizeof flags - strlen(flags), "  %s:--hall-shift %d", w[k].name,
               row.hall_shift_deg);
  }
  printf("build flags:%s\n", flags);
  return rc;
}

static void json_samples(FILE *f, const ik_capture_sample_t *s, size_t n)
{
  fputs("[", f);
  for (size_t i = 0; i < n; i++)
    fprintf(f, "%s{\"id\":%d,\"iq\":%d,\"vd\":%d,\"vq\":%d,\"id_ref\":%d,\"angle\":%d}", i ? "," : "", s[i].id,
            s[i].iq, s[i].vd, s[i].vq, s[i].id_ref, s[i].angle);
  fputs("]", f);
}

static int cmd_capture(ik_link_t *l, const wheel_t *w, int n, int step_ma, const char *out)
{
  for (int k = 0; k < n; k++) {
    ik_capture_header_t info;
    ik_capture_sample_t *raw = calloc(ESC_CAPTURE_SAMPLES, sizeof *raw);
    capture_sample *cs = calloc(ESC_CAPTURE_SAMPLES, sizeof *cs);
    if (ik_capture(l, w[k].sel, step_ma, &info, raw)) return die("%s: %s", w[k].name, l->err);
    for (size_t i = 0; i < ESC_CAPTURE_SAMPLES; i++) cs[i] = (capture_sample){raw[i].id, raw[i].vd, raw[i].id_ref};
    capture_result a;
    int rc = capture_analyze(info.vbus_centivolts, cs, ESC_CAPTURE_SAMPLES, ESC_CAPTURE_PRE, CAPTURE_PWM_HZ, &a);
    if (rc == CAPTURE_STEP_TOO_SMALL) return die("%s: step too small to fit", w[k].name);
    if (rc) return die("%s: capture analysis failed", w[k].name);
    if (out) {
      FILE *f = fopen(out, "w");
      if (!f) return die("cannot write %s", out);
      fprintf(f, "{\"wheel\":\"%s\",\"step_ma\":%d,\"info\":{\"vbus_centivolts\":%u,\"samples\":%u,\"status\":%u},"
                 "\"analysis\":{\"r_ohm\":%.9g,\"l_h\":%.9g,\"tau_ms\":%.9g,\"delay_periods\":%d,\"vbus_v\":%.9g},\"samples\":",
              w[k].name, step_ma, info.vbus_centivolts, info.samples, info.status, a.r_ohm, a.l_h, a.tau_ms,
              a.delay_periods, a.vbus_v);
      json_samples(f, raw, ESC_CAPTURE_SAMPLES);
      fputs("}\n", f);
      fclose(f);
    }
    char rise[32] = "-", settle[32] = "not settled";
    if (!isnan(a.rise_us)) snprintf(rise, sizeof rise, "%.0f us", a.rise_us);
    if (!isnan(a.settle_us)) snprintf(settle, sizeof settle, "%.0f us", a.settle_us);
    printf("%s: Vbus %.2f V, Id %.2f -> %.2f A\n", w[k].name, a.vbus_v, a.hold_a, a.step_a);
    printf("  R %.0f mOhm  L %.0f uH  tau %.2f ms  PWM delay %d periods\n", a.r_ohm * 1e3, a.l_h * 1e6, a.tau_ms,
           a.delay_periods);
    printf("  step: rise 10-90%% %s, overshoot %.1f%%, settle %s (band %.0f mA), steady error %.0f mA, noise %.0f mA rms, "
           "%zu disturbed samples later\n",
           rise, a.overshoot_pct, settle, a.settle_band_a * 1e3, a.steady_error_a * 1e3, a.noise_a * 1e3,
           a.disturbance_samples);
    for (int b = 0; b < 3; b++) {
      static const int bandwidth_hz[3] = {500, 800, 1000};
      int64_t kp, ki;
      capture_pi_gains(a.r_ohm, a.l_h, a.vbus_v, bandwidth_hz[b], CAPTURE_PWM_HZ, &kp, &ki);
      printf("  for %d Hz: current_kp %lld current_ki %lld\n", bandwidth_hz[b], (long long)kp, (long long)ki);
    }
    free(raw);
    free(cs);
  }
  return 0;
}

typedef struct { double t; int cmd; int32_t rpm; double iq_a, iq_ref_a; const char *mc_state, *stop_reason; char faults[96]; } spin_row_t;

typedef struct { int rpm; double seconds; int kp, ki; bool has_kp, has_ki; const char *log; } spin_args_t;

static volatile sig_atomic_t spin_interrupted;
static void spin_interrupt(int signal_number) { spin_interrupted = signal_number; }

static int cmd_spin(ik_link_t *l, const wheel_t *w, const spin_args_t *a)
{
  if (a->rpm < -600 || a->rpm > 600 || !(a->seconds > 0 && a->seconds <= 10))
    return die("spin: |rpm| <= 600 and 0 < seconds <= 10");
  struct sigaction action = {.sa_handler = spin_interrupt}, old_int, old_term, old_hup;
  sigemptyset(&action.sa_mask);
  spin_interrupted = 0;
  sigaction(SIGINT, &action, &old_int);
  sigaction(SIGTERM, &action, &old_term);
  sigaction(SIGHUP, &action, &old_hup);
  int kp0, ki0;
  esc_default("kp", &kp0);
  esc_default("ki", &ki0);
  bool gains = a->has_kp || a->has_ki;
  int kp = a->has_kp ? a->kp : kp0, ki = a->has_ki ? a->ki : ki0;
  size_t cap = 1024, n = 0;
  spin_row_t *rows = malloc(cap * sizeof *rows);
  int err = 0;
  if (gains && (send_config(l, w->sel, "kp", kp) || send_config(l, w->sel, "ki", ki))) err = 1;
  esc_ctl_t ctl;
  esc_ctl_init(&ctl, l, 0.03);
  if (!err && esc_ctl_arm(&ctl, w->index)) err = 1;
  double t0 = now_s(), last_cmd = -1.0, now;
  uint32_t last_count = 0;
  bool have_count = false;
  bool disabled = false;
  while (!err && !spin_interrupted && (now = now_s() - t0) < a->seconds + 0.6) {
    if (now < a->seconds && now - last_cmd >= 0.02) {
      if (last_cmd >= 0 && now - last_cmd > 0.04) {
        snprintf(l->err, sizeof l->err, "spin: SPEED stream missed 40 ms deadline"); err = 1; break;
      }
      if (esc_ctl_speed(&ctl, w->index, a->rpm)) { err = 1; break; }
      last_cmd = now;
    }
    if (!disabled && now >= a->seconds) {
      if (esc_ctl_disable_all(&ctl)) { err = 1; break; }
      disabled = true;
    }
    if (!disabled) {
      ik_link_row_t links[IK_WHEELS];
      if (ik_link_rows(l, links)) { err = 1; break; }
      const ik_link_row_t *link = &links[w->index];
      if (link->valid && (link->lifecycle == MOTOR_LC_FAULTED || link->lifecycle == MOTOR_LC_DISARMED ||
                          link->outcome == MOTOR_OUT_REJECTED || link->session != ctl.wheel[w->index].session)) {
        snprintf(l->err, sizeof l->err, "%s spin stopped: %s/%s/%s", w->name,
                 link->lifecycle_name, link->outcome_name, link->reason_name); err = 1; break;
      }
    }
    ik_row_t r[IK_WHEELS];
    if (ik_rows(l, r)) { err = 1; break; }
    const ik_row_t *x = &r[w->index];
    if (x->valid && (!have_count || x->status_count != last_count)) { /* one entry per fresh reply */
      have_count = true;
      last_count = x->status_count;
      if (n == cap) rows = realloc(rows, (cap *= 2) * sizeof *rows);
      rows[n++] = (spin_row_t){round(now * 1e4) / 1e4, now < a->seconds ? a->rpm : 0, x->rpm, x->iq_a, x->iq_ref_a,
                               x->mc_state, x->stop_reason, ""};
      snprintf(rows[n - 1].faults, sizeof rows[n - 1].faults, "%s", x->fault_names);
    }
    sleep_s(0.005);
  }
  if (err) fprintf(stderr, "%s\n", l->err);
  if (esc_ctl_disable_all(&ctl)) { fprintf(stderr, "%s\n", l->err); err = 1; }
  if (gains) {
    const int idx[1] = {w->index};
    if (!pulse_wait_idle(l, idx, 1, 3.0)) printf("%s: not IDLE; speed gains may not be restored (esc_tool set)\n", w->name);
    (void)send_config(l, w->sel, "kp", kp0);
    (void)send_config(l, w->sel, "ki", ki0);
  }
  if (a->log) {
    FILE *f = fopen(a->log, "w");
    if (f) {
      for (size_t i = 0; i < n; i++) {
        fprintf(f, "{\"t\":%.4f,\"cmd\":%d,\"rpm\":%d,\"iq_a\":%.6g,\"iq_ref_a\":%.6g,\"mc_state\":\"%s\",\"fault_names\":",
                rows[i].t, rows[i].cmd, rows[i].rpm, rows[i].iq_a, rows[i].iq_ref_a, rows[i].mc_state);
        pulse_json_names(f, rows[i].faults);
        fprintf(f, ",\"stop_reason\":\"%s\"}\n", rows[i].stop_reason);
      }
      fclose(f);
    }
  }
  double sum = 0, sum2 = 0, ref_sum = 0, ref_peak = 0;
  int wrong = 0;
  size_t m = 0;
  char faults[96] = "";
  for (size_t i = 0; i < n; i++) {
    if (rows[i].faults[0] && !strstr(faults, rows[i].faults)) {
      snprintf(faults + strlen(faults), sizeof faults - strlen(faults), "%s%s", faults[0] ? "," : "", rows[i].faults);
    }
    if (!(0.52 < rows[i].t && rows[i].t < a->seconds)) continue;
    m++;
    sum += rows[i].rpm;
    sum2 += (double)rows[i].rpm * rows[i].rpm;
    ref_sum += rows[i].iq_ref_a;
    ref_peak = fmax(ref_peak, fabs(rows[i].iq_ref_a));
    wrong += (int64_t)rows[i].rpm * a->rpm < 0;
  }
  if (!m) {
    printf("%s: no telemetry during the run\n", w->name);
  } else {
    double mean = sum / (double)m, sd = sqrt(fmax(0, sum2 / (double)m - mean * mean));
    printf("%s: cmd %d rpm (kp %d ki %d): mean %.1f sd %.1f wrong-sign %d/%zu | Iq ref mean %.2f A peak %.2f A | faults %s\n",
           w->name, a->rpm, kp, ki, mean, sd, wrong, m, ref_sum / (double)m, ref_peak, faults[0] ? faults : "-");
  }
  free(rows);
  sigaction(SIGINT, &old_int, NULL);
  sigaction(SIGTERM, &old_term, NULL);
  sigaction(SIGHUP, &old_hup, NULL);
  return err;
}

typedef struct {
  const char *axis, *log;
  double speed, seconds;
  int accel, reps, kp, ki, ff_static, ff_kinetic;
  bool has_kp, has_ki, has_ffs, has_ffk;
} pulse_args_t;

static int cmd_pulse(ik_link_t *l, const pulse_args_t *a)
{
  if (!(a->speed > 0 && a->speed <= 2.0) || !(a->seconds > 0 && a->seconds <= 3) || a->reps < 1 || a->reps > 10)
    return die("pulse: 0 < speed <= 2 m/s, 0 < seconds <= 3, 1 <= reps <= 10");
  /* Send every tunable, not only the non-default ones: the ESCs may still hold values from an earlier run
   * whose restore did not land. */
  pulse_settings_t settings = {.n_keys = 4, .keys = {"kp", "ki", "ff_static_ma", "ff_kinetic_ma"}};
  const bool has[4] = {a->has_kp, a->has_ki, a->has_ffs, a->has_ffk};
  const int given[4] = {a->kp, a->ki, a->ff_static, a->ff_kinetic};
  int used[4];
  for (int k = 0; k < 4; k++) {
    int def;
    esc_default(settings.keys[k], &def);
    used[k] = has[k] ? given[k] : def;
    for (int w = 0; w < IK_WHEELS; w++) settings.value[w][k] = used[k];
  }
  int32_t v = (int32_t)lround(a->speed * 1000);
  int32_t out[3] = {0, v, 0};
  if (!strcmp(a->axis, "vx")) { out[0] = v; out[1] = 0; }
  size_t n_moves = (size_t)a->reps * 2;
  int32_t (*twists)[3] = malloc(n_moves * sizeof *twists);
  for (size_t i = 0; i < n_moves; i++)
    for (int c = 0; c < 3; c++) twists[i][c] = i % 2 ? -out[c] : out[c];
  esc_move_t *plan = malloc(n_moves * sizeof *plan);
  double duration = esc_pulse_plan((const int32_t(*)[3])twists, n_moves, a->seconds, 1.0, 0.5, plan);

  pulse_result_t res;
  int err = pulse_run(l, plan, n_moves, duration, &settings, a->accel, &res);
  if (err) fprintf(stderr, "%s\n", l->err);
  if (a->log && !err) pulse_log_write(a->log, &res);

  esc_log_entry_t *entries = malloc((res.n + 1) * sizeof *entries);
  for (size_t i = 0; i < res.n; i++) entries[i] = res.log[i].e;
  esc_start_t starts[16];
  size_t n_starts = esc_breakaways(entries, res.n, 40, 1.0, starts, 16);
  esc_breakaway_summary_t s;
  esc_breakaway_summary(starts, n_starts, &s);
  char faults[96] = "";
  double vmin = 1e9;
  for (size_t i = 0; i < res.n; i++)
    for (int w = 0; w < 4; w++) {
      const char *f = res.log[i].e.faults[w];
      if (f[0] && !strstr(faults, f))
        snprintf(faults + strlen(faults), sizeof faults - strlen(faults), "%s%s", faults[0] ? "," : "", f);
      if (res.log[i].vbus_v[w] > 0) vmin = fmin(vmin, res.log[i].vbus_v[w]);
    }
  if (res.aborted[0]) printf("ABORTED: %s\n", res.aborted);
  printf("pulse %s %g m/s x%d (kp %d ki %d ff %d/%d mA): faults %s, Vbus min %.2f V\n", a->axis, a->speed, a->reps,
         used[0], used[1], used[2], used[3], faults[0] ? faults : "-", vmin > 1e8 ? 0.0 : vmin);
  for (int i = 0; i < IK_WHEELS; i++) {
    const esc_wheel_breakaway_t *b = &s.wheel[i];
    char med[16] = "-", worst[16] = "-";
    if (b->has_times) {
      snprintf(med, sizeof med, "%.0f", b->median_s * 1000);
      snprintf(worst, sizeof worst, "%.0f", b->worst_s * 1000);
    }
    printf("  %s: moved %d/%d, to %d rpm median %s ms worst %s ms\n", IK_WHEEL_POS[i], b->moved, b->starts,
           ESC_BREAKAWAY_RPM, med, worst);
  }
  char sm[16] = "-", sw[16] = "-";
  if (s.has_spread) {
    snprintf(sm, sizeof sm, "%.0f", s.spread_median_s * 1000);
    snprintf(sw, sizeof sw, "%.0f", s.spread_worst_s * 1000);
  }
  printf("  spread between wheels: median %s ms worst %s ms\n", sm, sw);
  pulse_result_free(&res);
  free(entries);
  free(plan);
  free(twists);
  return err;
}

static int usage(void)
{
  return die("usage: esc_tool [--elf F] [--sn SERIAL] status|stop|clear|align W|set W KEY VALUE|capture W [--step-ma N] [--out F]|"
             "spin W --rpm N [...]|pulse [...]  (see the header of src/tools/esc_tool.c)");
}

int main(int argc, char **argv)
{
  const char *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  int i = 1;
  for (; i + 1 < argc && argv[i][0] == '-'; i += 2) {
    if (!strcmp(argv[i], "--elf")) elf = argv[i + 1];
    else if (!strcmp(argv[i], "--sn")) sn = argv[i + 1];
    else break;
  }
  if (i >= argc) return usage();
  const char *cmd = argv[i++];
  wheel_t w[IK_WHEELS];
  int nw = 0;
  if (strcmp(cmd, "status") && strcmp(cmd, "stop") && strcmp(cmd, "clear") && strcmp(cmd, "pulse")) {
    if (i >= argc) return usage();
    if (!(nw = pick_wheels(argv[i], w))) return die("unknown wheel %s; use FR BR BL FL (or NE SE SW NW) or all", argv[i]);
    i++;
  }
  spin_args_t sa = {.rpm = 0, .seconds = 3.0};
  pulse_args_t pa = {.axis = "vy", .speed = 0.2, .seconds = 1.5, .reps = 3};
  int step_ma = 3000, set_value = 0;
  const char *out = NULL, *key = NULL;
  bool have_rpm = false;
  if (!strcmp(cmd, "set")) {
    if (i + 1 >= argc) return usage();
    key = argv[i];
    set_value = atoi(argv[i + 1]);
    i += 2;
  }
  for (; i < argc; i++) {
    const char *o = argv[i];
    const char *v = i + 1 < argc ? argv[i + 1] : NULL;
    if (!v) return usage();
    if (!strcmp(o, "--step-ma")) step_ma = atoi(v);
    else if (!strcmp(o, "--out")) out = v;
    else if (!strcmp(o, "--rpm")) sa.rpm = atoi(v), have_rpm = true;
    else if (!strcmp(o, "--seconds")) sa.seconds = pa.seconds = atof(v);
    else if (!strcmp(o, "--kp")) sa.kp = pa.kp = atoi(v), sa.has_kp = pa.has_kp = true;
    else if (!strcmp(o, "--ki")) sa.ki = pa.ki = atoi(v), sa.has_ki = pa.has_ki = true;
    else if (!strcmp(o, "--log")) sa.log = pa.log = v;
    else if (!strcmp(o, "--axis")) pa.axis = v;
    else if (!strcmp(o, "--speed")) pa.speed = atof(v);
    else if (!strcmp(o, "--accel")) pa.accel = atoi(v);
    else if (!strcmp(o, "--reps")) pa.reps = atoi(v);
    else if (!strcmp(o, "--ff-static")) pa.ff_static = atoi(v), pa.has_ffs = true;
    else if (!strcmp(o, "--ff-kinetic")) pa.ff_kinetic = atoi(v), pa.has_ffk = true;
    else return die("unknown option %s", o);
    i++;
  }
  if (strcmp(pa.axis, "vx") && strcmp(pa.axis, "vy")) return die("--axis must be vx or vy");

  ik_link_t link;
  if (ik_link_open(&link, sn, elf)) return die("%s", link.err);
  esc_ctl_t ctl;
  esc_ctl_init(&ctl, &link, 1.0);
  int rc = 0;
  if (!strcmp(cmd, "status")) {
    rc = cmd_status(&link);
  } else if (!strcmp(cmd, "stop")) {
    rc = esc_ctl_disable_all(&ctl) ? die("%s", link.err) : 0;
  } else if (!strcmp(cmd, "clear")) {
    rc = esc_ctl_clear_faults(&ctl) ? die("%s", link.err) : 0;
  } else if (!strcmp(cmd, "align")) {
    rc = cmd_align(&link, w, nw);
  } else if (!strcmp(cmd, "capture")) {
    rc = cmd_capture(&link, w, nw, step_ma, out);
  } else if (!strcmp(cmd, "spin")) {
    if (nw != 1) rc = die("spin: one wheel at a time");
    else if (!have_rpm) rc = die("spin: --rpm is required");
    else rc = cmd_spin(&link, &w[0], &sa);
  } else if (!strcmp(cmd, "pulse")) {
    rc = cmd_pulse(&link, &pa);
  } else if (!strcmp(cmd, "set")) {
    for (int k = 0; k < nw && !rc; k++) {
      if (send_config(&link, w[k].sel, key, set_value)) rc = die("%s", link.err);
      else printf("%s: %s = %d (RAM)\n", w[k].name, key, set_value);
    }
  } else {
    rc = usage();
  }
  ik_link_close(&link);
  return rc;
}
