/*
 * ESC setup through the Nucleo streaming IK image (drive must be stopped).
 *
 *   esc_tool [--elf F] [--sn SERIAL] status
 *   esc_tool align FR|BR|BL|FL|NE|SE|SW|NW|all   measure + apply Hall phase shift (RAM)
 *   esc_tool set WHEEL KEY VALUE                 CONFIG key (RAM), acknowledged readback, see ik_link.c for keys
 *   esc_tool get WHEEL [KEY|all]                  read stored/effective values (all keys by default)
 *   esc_tool capture WHEEL [--step-ma 3000] [--out f.json]
 *                                                current-loop step test: R, L, step response, PI gains
 *   esc_tool reject WHEEL [--json F]              zero-motion protocol cases, DISABLE on exit
 *   esc_tool stream --seconds T [--hold-fault-s 2] [--json F]
 *                                                zero twist + heartbeat through the normal drive path
 *   esc_tool spin WHEEL --rpm 60 [--seconds 3] [--kp N --ki N] [--log f.jsonl]
 *                 [--gap-ms N [--gap-at S]]        omit sending once (default S=1 second)
 *                                                one wheel at a speed setpoint (motion!)
 *   esc_tool pulse [--axis vy|vx] [--speed 0.2] [--accel RPMPS] [--seconds 1.5] [--reps 3]
 *                  [--kp N --ki N] [--log f.jsonl]
 *                                                scripted ground drive out and back (motion!)
 * CONFIG keys: iq_limit_ma, hall_shift_deg, kp, ki, current_kp, current_ki, stall_ramp_ma_per_s, stall_ms.
 *
 *   esc_tool stop                                broadcast DISABLE
 *   esc_tool clear                               clear latched ESC faults while stopped
 *
 * align turns the wheel about 1/4 revolution forward and back at 2 A: battery ON, wheels off the
 * ground. Results are lost on ESC reset; bake them into the per-wheel build (scripts/esc_build.sh
 * --hall-shift). capture holds the rotor with d-axis current (1 A, then the step): it may click into
 * position but makes no torque; battery ON. spin arms and streams SPEED to one wheel; --kp/--ki are
 * set in RAM first and the firmware defaults (drive_parameters.h) restored afterwards; DISABLE on exit,
 * and the ESC stops itself 0.1 s after the stream. pulse drives the whole robot through the Nucleo:
 * each rep goes out along --axis for --seconds and back; the speed-loop gains and stall settings are
 * set in RAM on all four wheels and the defaults restored afterwards. Ctrl-C or switching the battery off
 * stops it: the Nucleo brakes 0.3 s after the heartbeat stops.
 * reject waits for watchdog disarm and re-arms for each case; only rejected range/conflict frames
 * have nonzero speed setpoints. stream preserves enable=1 for the fault hold so the fault latch
 * and all four replies can be inspected, then acknowledges it with enable=0. Both emit JSON on request.
 */
#include <errno.h>
#include <limits.h>
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
#include "elfsym.h"
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
  uint16_t id;
  if (ik_config_key(key, &id)) {
    snprintf(l->err, sizeof l->err, "unknown CONFIG key %s", key);
    return -1;
  }
  for (int w = 0; w < IK_WHEELS; w++) if (IK_WHEEL_SELECT[w] == sel) {
    struct motor_config_reply reply;
    return ik_config_exchange(l, w, MOTOR_CONFIG_SET, id, value, 1.0, &reply);
  }
  snprintf(l->err, sizeof l->err, "CONFIG requires one wheel");
  return -1;
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
    printf("%s/%s: link %s/%s/%s session %u misses %u fault %u lost %u cause %s armed %u restarts %u\n",
           IK_WHEEL_POS[i], IK_WHEEL_CORNER[i], link->valid ? link->lifecycle_name : "unreadable",
           link->valid ? link->outcome_name : "?", link->valid ? link->reason_name : "?",
           link->session, link->misses, faulted, lost,
           !(faulted || lost) ? "-" : ik_fault_cause(link, true, lost),
           !!(status.armed & (1U << i)), link->restarts);
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

typedef struct { int rpm; double seconds, gap_ms, gap_at; int kp, ki; bool has_kp, has_ki; const char *log; } spin_args_t;

static volatile sig_atomic_t spin_interrupted;
static void spin_interrupt(int signal_number) { spin_interrupted = signal_number; }

static int cmd_spin(ik_link_t *l, const wheel_t *w, const spin_args_t *a)
{
  if (a->rpm < -600 || a->rpm > 600 || !(a->seconds > 0 && a->seconds <= 10))
    return die("spin: |rpm| <= 600 and 0 < seconds <= 10");
  if (!isfinite(a->gap_ms) || a->gap_ms < 0 || (a->gap_ms > 0 &&
      !(a->gap_at >= 0 && a->gap_at + a->gap_ms / 1000.0 + 0.05 < a->seconds)))
    return die("spin: gap must leave at least 50 ms of stream after --gap-at + --gap-ms/1000");
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
  bool gap_started = false, gap_sent = false, gap_reported = false;
  double gap_start = 0;
  uint32_t gap_before_us = 0;
  uint8_t gap_seq = 0;
  while (!err && !spin_interrupted && (now = now_s() - t0) < a->seconds + 0.6) {
    if (now < a->seconds && now - last_cmd >= 0.02) {
      if (gap_sent && !gap_reported) {
        if (now - last_cmd > 0.04) {
          snprintf(l->err, sizeof l->err, "spin: first reply after gap timed out"); err = 1; break;
        }
        goto read_spin_reply;
      }
      if (a->gap_ms > 0 && !gap_started && last_cmd >= 0 && now >= a->gap_at) {
        if (now - last_cmd > 0.04) {
          snprintf(l->err, sizeof l->err, "spin: SPEED stream missed 40 ms deadline before gap"); err = 1; break;
        }
        ik_link_row_t r;
        if (ik_link_row(l, w->index, &r)) { err = 1; break; }
        if (!r.valid || r.request_seq != ctl.wheel[w->index].seq) {
          if (now - last_cmd > 0.04) { snprintf(l->err, sizeof l->err, "spin: no reply before gap"); err = 1; break; }
          sleep_s(0.002); continue;
        }
        gap_before_us = r.reply_us;
        gap_start = now;
        gap_started = true;
      }
      if (gap_started && !gap_sent && now - gap_start < a->gap_ms / 1000.0) {
        sleep_s(0.002); continue;
      }
      bool resume = gap_started && !gap_sent;
      if (!resume && last_cmd >= 0 && now - last_cmd > 0.04) {
        snprintf(l->err, sizeof l->err, "spin: SPEED stream missed 40 ms deadline"); err = 1; break;
      }
      if (esc_ctl_speed(&ctl, w->index, a->rpm)) { err = 1; break; }
      if (resume) { gap_sent = true; gap_seq = ctl.wheel[w->index].seq; }
      last_cmd = now;
    }
read_spin_reply:
    if (!disabled && now >= a->seconds) {
      if (esc_ctl_disable_all(&ctl)) { err = 1; break; }
      disabled = true;
    }
    if (!disabled) {
      ik_link_row_t reply;
      if (ik_link_row(l, w->index, &reply)) { err = 1; break; }
      const ik_link_row_t *link = &reply;
      if (gap_sent && !gap_reported && link->valid && link->request_seq == gap_seq) {
        gap_reported = true;
        bool trip = link->session != ctl.wheel[w->index].session ||
                    (link->outcome == MOTOR_OUT_REJECTED && link->reason == MOTOR_R_NOT_ARMED);
        bool no_trip = link->outcome == MOTOR_OUT_APPLIED && link->session == ctl.wheel[w->index].session;
        printf("%s: gap reply_us delta %u us (requested %.3f ms): %s; first reply %s/%s/%s session %u (before %u)\n",
               w->name, (uint32_t)(link->reply_us - gap_before_us), a->gap_ms,
               trip ? "trip" : no_trip ? "no trip" : "inconclusive", link->outcome_name,
               link->reason_name, link->lifecycle_name, link->session, ctl.wheel[w->index].session);
        if (trip) break;
        if (!no_trip) {
          snprintf(l->err, sizeof l->err, "spin: gap verdict inconclusive"); err = 1; break;
        }
      }
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
  if (a->gap_ms > 0 && !gap_reported) {
    fprintf(stderr, "spin: gap first reply not measured\n"); err = 1;
  }
  if (err) fprintf(stderr, "%s\n", l->err);
  if (esc_ctl_disable_all(&ctl)) { fprintf(stderr, "%s\n", l->err); err = 1; }
  if (gains) {
    const int idx[1] = {w->index};
    if (!pulse_wait_idle(l, idx, 1, 3.0)) printf("%s: not IDLE; speed gains may not be restored (esc_tool set)\n", w->name);
    if (send_config(l, w->sel, "kp", kp0)) { fprintf(stderr, "%s\n", l->err); err = 1; }
    if (send_config(l, w->sel, "ki", ki0)) { fprintf(stderr, "%s\n", l->err); err = 1; }
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
  int accel, reps, kp, ki;
  bool has_kp, has_ki;
} pulse_args_t;

static int cmd_pulse(ik_link_t *l, const pulse_args_t *a)
{
  if (!(a->speed > 0 && a->speed <= 2.0) || !(a->seconds > 0 && a->seconds <= 3) || a->reps < 1 || a->reps > 10)
    return die("pulse: 0 < speed <= 2 m/s, 0 < seconds <= 3, 1 <= reps <= 10");
  /* Send every tunable, not only the non-default ones: the ESCs may still hold values from an earlier run
   * whose restore did not land. */
  pulse_settings_t settings = {.n_keys = 4, .keys = {"kp", "ki", "stall_ramp_ma_per_s", "stall_ms"}};
  const bool has[4] = {a->has_kp, a->has_ki, false, false};
  const int given[4] = {a->kp, a->ki, 0, 0};
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
  printf("pulse %s %g m/s x%d (kp %d ki %d): faults %s, Vbus min %.2f V\n", a->axis, a->speed, a->reps,
         used[0], used[1], faults[0] ? faults : "-", vmin > 1e8 ? 0.0 : vmin);
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

static int cmd_reject(ik_link_t *l, const wheel_t *wheel, const char *json)
{
  size_t count;
  const esc_reject_case_t *cases = esc_ctl_reject_cases(&count);
  FILE *f = json ? fopen(json, "w") : NULL;
  if (json && !f) return die("cannot write %s", json);
  if (f) fprintf(f, "{\"wheel\":\"%s\",\"cases\":[", wheel->name);
  esc_ctl_t c;
  esc_ctl_init(&c, l, 0.03);
  struct sigaction action = {.sa_handler = spin_interrupt}, old_int, old_term, old_hup;
  sigemptyset(&action.sa_mask); spin_interrupted = 0;
  sigaction(SIGINT, &action, &old_int); sigaction(SIGTERM, &action, &old_term); sigaction(SIGHUP, &action, &old_hup);
  int failed = 0;
  for (size_t k = 0; k < count && !spin_interrupted; k++) {
    const esc_reject_case_t *test = &cases[k];
    ik_link_row_t replies[IK_WHEELS] = {{0}};
    uint32_t id = MOTOR_CAN_ID_CONTROL | wheel->sel;
    uint8_t data[8];
    int rc = esc_ctl_disable(&c, wheel->index);
    /* DISABLE keeps READY armed. Expire the old watchdog before each new ARM. */
    if (!rc) sleep_s(0.12);
    if (!rc && !test->disarmed) {
      rc = esc_ctl_arm(&c, wheel->index);
      if (!rc) {
        struct motor_control zero = {.mode = MOTOR_MODE_SPEED, .seq = ++c.wheel[wheel->index].seq,
                                     .session = c.wheel[wheel->index].session};
        rc = motor_control_encode(&zero, wheel->sel, &id, data) != MOTOR_R_NONE ||
             esc_ctl_exchange(&c, id, data, replies);
        const ik_link_row_t *r = &replies[wheel->index];
        if (!rc && (r->outcome != MOTOR_OUT_APPLIED || r->reason != MOTOR_R_NONE ||
                    r->lifecycle != MOTOR_LC_READY || r->session != zero.session)) {
          snprintf(l->err, sizeof l->err, "zero baseline not APPLIED/NONE/READY in the armed session"); rc = -1;
        }
      }
    }
    memset(replies, 0, sizeof replies);
    if (!rc) rc = test->build(wheel->index, c.wheel[wheel->index].seq, c.wheel[wheel->index].session, &id, data);
    if (!rc) rc = esc_ctl_exchange(&c, id, data, replies);
    bool pass = !rc;
    for (int w = 0; w < IK_WHEELS; w++) if (id & IK_WHEEL_SELECT[w]) {
      const ik_link_row_t *r = &replies[w];
      bool ok = !rc && r->valid && r->outcome == (uint32_t)test->outcome && r->reason == (uint32_t)test->reason;
      if (w == wheel->index && !test->disarmed)
        ok = ok && r->lifecycle == MOTOR_LC_READY && r->session == c.wheel[w].session;
      if (w == wheel->index && test->disarmed) ok = ok && r->lifecycle == MOTOR_LC_DISARMED;
      pass = pass && ok;
      printf("%s %s %s: %s/%s lifecycle %s session %u%s%s\n", ok ? "PASS" : "FAIL", test->name,
             IK_WHEEL_POS[w], r->valid ? r->outcome_name : "no reply", r->valid ? r->reason_name : "UNKNOWN",
             r->valid ? r->lifecycle_name : "UNKNOWN", r->session, rc ? ": " : "", rc ? l->err : "");
    }
    failed |= !pass;
    if (f) {
      fprintf(f, "%s{\"name\":\"%s\",\"pass\":%s,\"expected_outcome\":%u,\"expected_reason\":%u,\"replies\":[",
              k ? "," : "", test->name, pass ? "true" : "false", test->outcome, test->reason);
      bool comma = false;
      for (int w = 0; w < IK_WHEELS; w++) if (id & IK_WHEEL_SELECT[w]) {
        const ik_link_row_t *r = &replies[w];
        fprintf(f, "%s{\"wheel\":\"%s\",\"valid\":%s,\"outcome\":%u,\"reason\":%u,\"lifecycle\":%u,\"session\":%u}",
                comma ? "," : "", IK_WHEEL_POS[w], r->valid ? "true" : "false", r->outcome, r->reason, r->lifecycle, r->session);
        comma = true;
      }
      fputs("]}", f);
    }
  }
  if (spin_interrupted) failed = 1;
  if (esc_ctl_disable(&c, wheel->index)) { fprintf(stderr, "%s\n", l->err); failed = 1; }
  const char *uncovered = "MODE_CHANGE, LIMITED, UNSUPPORTED (every mode is supported), NOT_ALIGNED, FAULT_HALL_LOST, TIMEOUT-as-reject, "
                          "LIFECYCLE_OVERFLOW, BAD_LENGTH, STARTING/BUSY/STALLED/NOT_IDLE, FAULT_MCSDK";
  printf("%s: not produced by this command\n", uncovered);
  if (f) {
    fprintf(f, "],\"pass\":%s,\"not_produced\":\"%s\"}\n", failed ? "false" : "true", uncovered);
    if (fclose(f)) failed = 1;
  }
  sigaction(SIGINT, &old_int, NULL); sigaction(SIGTERM, &old_term, NULL); sigaction(SIGHUP, &old_hup, NULL);
  return failed;
}

static void print_links(const ik_link_row_t rows[IK_WHEELS])
{
  for (int w = 0; w < IK_WHEELS; w++) {
    const ik_link_row_t *r = &rows[w];
    printf("%s: %s/%s/%s session %u reply_count %u fault_reason %s\n", IK_WHEEL_POS[w],
           r->valid ? r->lifecycle_name : "unreadable", r->valid ? r->outcome_name : "?",
           r->valid ? r->reason_name : "?", r->session, r->reply_count, r->valid ? r->fault_reason_name : "?");
  }
}

static const char *latency_text(uint32_t edge, char text[32])
{
  if (edge == UINT32_MAX) return "> 3.75 ms";
  if (!edge) return "n/a";
  snprintf(text, 32, "%.2f ms", edge / 1000.0);
  return text;
}

static void report_latency(const char *name, esc_latency_t s)
{
  char p50[32], p99[32], max[32];
  printf("%s latency: count %llu p50 %s p99 %s max %s (bin upper edges)\n", name,
         (unsigned long long)s.count, latency_text(s.p50_us, p50), latency_text(s.p99_us, p99), latency_text(s.max_us, max));
}

static void json_latency(FILE *f, esc_latency_t s)
{
  /* null distinguishes an overflow edge from a finite microsecond bound. */
  fprintf(f, "{\"count\":%llu", (unsigned long long)s.count);
  const uint32_t edges[] = {s.p50_us, s.p99_us, s.max_us};
  const char *names[] = {"p50", "p99", "max"};
  for (int i = 0; i < 3; i++) {
    fprintf(f, ",\"%s_us\":", names[i]);
    if (!s.count || edges[i] == UINT32_MAX) fputs("null", f); else fprintf(f, "%u", edges[i]);
    fprintf(f, ",\"%s_overflow\":%s", names[i], edges[i] == UINT32_MAX ? "true" : "false");
  }
  fputc('}', f);
}

typedef struct { double seconds, hold_fault_s; const char *json; } stream_args_t;

static int cmd_stream(ik_link_t *l, uint32_t rx_addr, const stream_args_t *a)
{
  if (!(a->seconds > 0) || !isfinite(a->seconds) || a->hold_fault_s < 0 || !isfinite(a->hold_fault_s))
    return die("stream: finite --seconds > 0 and --hold-fault-s >= 0 required");
  uint32_t state;
  if (ik_read(l, IK_STATE, &state)) return die("%s", l->err);
  if (state != 0) return die("stream: drive must be stopped");
  if (ik_set_mode(l, IK_MODE_SPEED)) return die("%s", l->err);
  FILE *f = a->json ? fopen(a->json, "w") : NULL;
  if (a->json && !f) return die("cannot write %s", a->json);
  ik_counters_t before, after;
  if (ik_counters(l, rx_addr, &before)) { if (f) fclose(f); return die("%s", l->err); }
  struct sigaction action = {.sa_handler = spin_interrupt}, old_int, old_term, old_hup;
  sigemptyset(&action.sa_mask); spin_interrupted = 0;
  sigaction(SIGINT, &action, &old_int); sigaction(SIGTERM, &action, &old_term); sigaction(SIGHUP, &action, &old_hup);
  double start = now_s(), fault_at = -1;
  uint32_t fault = before.fault;
  uint32_t fault_state = 0;
  ik_link_row_t fault_rows[IK_WHEELS] = {{0}};
  bool printed_links = false;
  int failed = fault != 0;
  bool io_error = false;
  while (!spin_interrupted) {
    double elapsed = now_s() - start;
    if (ik_read(l, IK_FAULT, &fault) || ik_read(l, IK_STATE, &state)) { io_error = true; break; }
    if (fault) {
      failed = 1;
      if (fault_at < 0) {
        fault_at = elapsed;
        fault_state = state;
        printf("stream: fault 0x%08x ik_drive_state %u (enable remains 1 for %.3f s)\n", fault, state, a->hold_fault_s);
        for (unsigned bit = 0; bit < 32; bit++) if (fault & (1U << bit))
          printf("  bit %u: %s %s\n", bit, bit < 8 ? IK_WHEEL_POS[bit % 4] : "unknown", bit < 4 ? "FAULTED" : bit < 8 ? "LOST" : "unknown");
      }
      /* Let all three broadcast DISABLEs arrive before printing the four rows once. */
      if (!printed_links && (elapsed - fault_at >= 0.08 || elapsed - fault_at >= a->hold_fault_s)) {
        if (ik_link_rows(l, fault_rows)) { io_error = true; break; }
        print_links(fault_rows); printed_links = true;
      }
      if (elapsed - fault_at >= a->hold_fault_s) break;
    } else if (elapsed >= a->seconds) break;
    if (ik_drive(l, 0, 0, 0, true)) { io_error = true; break; }
    sleep_s(0.01);
  }
  if (io_error) { fprintf(stderr, "%s\n", l->err); failed = 1; }
  if (spin_interrupted) failed = 1;
  uint32_t observed_fault = fault;
  if (ik_drive(l, 0, 0, 0, false)) { fprintf(stderr, "%s\n", l->err); failed = 1; }
  bool stopped = false;
  double deadline = now_s() + 1.0;
  do {
    if (ik_read(l, IK_STATE, &state) || ik_read(l, IK_FAULT, &fault)) { failed = 1; break; }
    observed_fault |= fault;
    if (state == 0 && fault == 0) { stopped = true; break; }
    sleep_s(0.01);
  } while (now_s() < deadline);
  /* The last broadcast may be queued when state becomes STOPPED. */
  if (stopped) sleep_s(0.02);
  bool sampled = ik_counters(l, rx_addr, &after) == 0;
  double duration = now_s() - start;
  if (!stopped || !sampled) { fprintf(stderr, "stream: could not confirm stopped counters/fault clear: %s\n", l->err); failed = 1; }
  if (!sampled && f) fputs("{\"pass\":false,\"error\":\"counter snapshot failed\"}\n", f);
  if (sampled) {
    observed_fault |= after.fault;
    failed |= observed_fault != 0 || after.fault != 0;
    printf("stream: enable 0, fault word 0x%08x %s\n", after.fault, after.fault == 0 ? "cleared" : "NOT CLEARED");
    uint32_t tx = after.tx_count - before.tx_count, tx_fail = after.tx_fail - before.tx_fail;
    uint32_t rx = after.can_rx - before.can_rx, per_wheel = 0;
    bool tx_known = esc_ctl_stream_tx(tx, &per_wheel) == 0;
    failed |= tx_fail != 0 || !tx_known;
    uint64_t total_hist[IK_LAT_BINS] = {0}, replies = 0;
    if (f) fprintf(f, "{\"hist_bin_us\":250,\"hist_overflow_lower_us\":3750,\"requested_seconds\":%.9g,\"counter_seconds\":%.9g,\"tx_count\":%u,\"tx_fail\":%u,\"can_rx\":%u,"
                     "\"observed_fault\":%u,\"fault_state\":%u,\"final_fault\":%u,\"wheels\":[",
                   a->seconds, duration, tx, tx_fail, rx, observed_fault, fault_state, after.fault);
    for (int w = 0; w < IK_WHEELS; w++) {
      uint32_t reply = after.rows[w].reply_count - before.rows[w].reply_count;
      uint32_t rejects = after.rows[w].rejects - before.rows[w].rejects;
      bool ok = tx_known && reply == per_wheel && rejects <= 1;
      failed |= !ok;
      replies += reply;
      uint64_t hist[IK_LAT_BINS];
      for (int b = 0; b < IK_LAT_BINS; b++) {
        hist[b] = (uint32_t)(after.hist[w][b] - before.hist[w][b]);
        total_hist[b] += hist[b];
      }
      esc_latency_t latency = esc_ctl_latency(hist);
      printf("%s: %s reply_count delta %u tx_count delta %u (inferred) rejects delta %u fault word 0x%08x\n",
             IK_WHEEL_POS[w], ok ? "PASS" : "FAIL", reply, per_wheel, rejects, observed_fault);
      report_latency(IK_WHEEL_POS[w], latency);
      printf("  cumulative latency max_us %u -> %u (since boot)\n", before.max_us[w], after.max_us[w]);
      if (f) {
        const ik_link_row_t *r = printed_links ? &fault_rows[w] : &after.rows[w];
        fprintf(f, "%s{\"wheel\":\"%s\",\"reply_count\":%u,\"tx_count_inferred\":%u,\"rejects\":%u,"
                   "\"lifecycle\":%u,\"outcome\":%u,\"reason\":%u,\"session\":%u,\"fault_reason\":%u,\"latency\":",
                w ? "," : "", IK_WHEEL_POS[w], reply, per_wheel, rejects,
                r->lifecycle, r->outcome, r->reason, r->session, r->fault_reason);
        json_latency(f, latency);
        fprintf(f, ",\"hist_delta\":[");
        for (int b = 0; b < IK_LAT_BINS; b++) fprintf(f, "%s%llu", b ? "," : "", (unsigned long long)hist[b]);
        fputs("]}", f);
      }
    }
    esc_latency_t total = esc_ctl_latency(total_hist);
    report_latency("overall", total);
    /* All receive traffic is 8 bytes. Poll requests are not in tx_count; replies are in can_rx. */
    double polls = duration / 0.005;
    uint64_t frames4 = (uint64_t)llround(polls);
    double load = esc_ctl_bus_load((uint64_t)tx + rx, frames4, duration);
    printf("stream: %.3f s requested, %.3f s counter window; tx_count delta %u tx_fail delta %u can_rx_frame_count delta %u\n",
           a->seconds, duration, tx, tx_fail, rx);
    printf("bus load estimate %.2f%%: 500 kbit/s, 130 bits/8-byte frame, 100 bits/4-byte poll; "
           "one STATUS/DIAG/IDENT poll every 5 ms (~%llu requests); all replies counted in RX, "
           "including ~%llu poll replies. Counter window includes stop/fault hold.\n", load,
           (unsigned long long)frames4, (unsigned long long)(rx >= replies ? rx - replies : 0));
    printf("TX per wheel inferred from four-wheel cycles plus three broadcast DISABLEs; tx_fail is global. "
           "Expected ARM-probe rejects: at most one per wheel in this drive session.\n");
    printf("stream: %s\n", failed ? "FAIL" : "PASS");
    if (f) {
      fputs("],\"overall_latency\":", f); json_latency(f, total);
      fprintf(f, ",\"poll_requests_estimated\":%llu,\"bus_load_pct\":%.9g,\"pass\":%s,"
                 "\"assumption\":\"500 kbit/s; 130 bits/8-byte TX and RX; 100 bits/4-byte poll every 5 ms; "
                 "counter window includes stop and fault hold; per-wheel TX inferred with three broadcasts\"}\n",
              (unsigned long long)frames4, load, failed ? "false" : "true");
    }
  }
  if (f && fclose(f)) failed = 1;
  sigaction(SIGINT, &old_int, NULL); sigaction(SIGTERM, &old_term, NULL); sigaction(SIGHUP, &old_hup, NULL);
  return failed;
}

static int usage(void)
{
  return die("usage: esc_tool [--elf F] [--sn SERIAL] status|stop|clear|align W|set W KEY VALUE|get W [KEY|all]|capture W [--step-ma N] [--out F]|"
             "spin W --rpm N [--seconds T] [--gap-ms N [--gap-at S]] [...]|reject W [--json F]|"
             "stream --seconds T [--hold-fault-s 2] [--json F]|pulse [...]  (see the header of src/tools/esc_tool.c)");
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
  if (strcmp(cmd, "status") && strcmp(cmd, "stop") && strcmp(cmd, "clear") && strcmp(cmd, "pulse") && strcmp(cmd, "stream")) {
    if (i >= argc) return usage();
    if (!(nw = pick_wheels(argv[i], w))) return die("unknown wheel %s; use FR BR BL FL (or NE SE SW NW) or all", argv[i]);
    i++;
  }
  spin_args_t sa = {.rpm = 0, .seconds = 3.0, .gap_at = 1.0};
  stream_args_t stream = {.hold_fault_s = 2.0};
  pulse_args_t pa = {.axis = "vy", .speed = 0.2, .seconds = 1.5, .reps = 3};
  int step_ma = 3000, set_value = 0;
  const char *out = NULL, *key = NULL, *json = NULL;
  bool have_rpm = false;
  if (!strcmp(cmd, "set")) {
    if (i + 1 >= argc) return usage();
    key = argv[i];
    char *end;
    errno = 0;
    long parsed_value = strtol(argv[i + 1], &end, 0);
    if (errno || !*argv[i + 1] || *end || parsed_value < INT16_MIN || parsed_value > INT16_MAX)
      return die("CONFIG value must be a signed 16-bit integer");
    set_value = (int)parsed_value;
    i += 2;
  }
  if (!strcmp(cmd, "get")) {
    key = i < argc ? argv[i++] : "all";
    if (i != argc) return usage();
  }
  if (!strcmp(cmd, "set") || (!strcmp(cmd, "get") && strcmp(key, "all"))) {
    uint16_t id;
    if (ik_config_key(key, &id)) return die("unknown CONFIG key %s", key);
    if (i != argc) return usage();
  }
  for (; i < argc; i++) {
    const char *o = argv[i];
    const char *v = i + 1 < argc ? argv[i + 1] : NULL;
    if (!v) return usage();
    if (!strcmp(o, "--step-ma")) step_ma = atoi(v);
    else if (!strcmp(o, "--out")) out = v;
    else if (!strcmp(o, "--rpm")) sa.rpm = atoi(v), have_rpm = true;
    else if (!strcmp(o, "--seconds")) sa.seconds = pa.seconds = stream.seconds = atof(v);
    else if (!strcmp(o, "--gap-ms")) sa.gap_ms = atof(v);
    else if (!strcmp(o, "--gap-at")) sa.gap_at = atof(v);
    else if (!strcmp(o, "--hold-fault-s")) stream.hold_fault_s = atof(v);
    else if (!strcmp(o, "--json")) json = stream.json = v;
    else if (!strcmp(o, "--kp")) sa.kp = pa.kp = atoi(v), sa.has_kp = pa.has_kp = true;
    else if (!strcmp(o, "--ki")) sa.ki = pa.ki = atoi(v), sa.has_ki = pa.has_ki = true;
    else if (!strcmp(o, "--log")) sa.log = pa.log = v;
    else if (!strcmp(o, "--axis")) pa.axis = v;
    else if (!strcmp(o, "--speed")) pa.speed = atof(v);
    else if (!strcmp(o, "--accel")) pa.accel = atoi(v);
    else if (!strcmp(o, "--reps")) pa.reps = atoi(v);
    else return die("unknown option %s", o);
    i++;
  }
  if (strcmp(pa.axis, "vx") && strcmp(pa.axis, "vy")) return die("--axis must be vx or vy");

  uint32_t rx_addr = 0;
  if (!strcmp(cmd, "stream") && elfsym_lookup(elf, "can_rx_frame_count", &rx_addr, NULL))
    return die("missing can_rx_frame_count in %s", elf);
  if (!strcmp(cmd, "reject") && nw != 1) return die("reject: one wheel at a time");

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
  } else if (!strcmp(cmd, "reject")) {
    rc = cmd_reject(&link, &w[0], json);
  } else if (!strcmp(cmd, "stream")) {
    rc = cmd_stream(&link, rx_addr, &stream);
  } else if (!strcmp(cmd, "pulse")) {
    rc = cmd_pulse(&link, &pa);
  } else if (!strcmp(cmd, "set") || !strcmp(cmd, "get")) {
    bool get = !strcmp(cmd, "get");
    size_t keys = get && !strcmp(key, "all") ? ik_config_key_count() : 1;
    for (int k = 0; k < nw; k++) for (size_t j = 0; j < keys; j++) {
      const char *name = get && !strcmp(key, "all") ? ik_config_key_name(j, NULL) : key;
      uint16_t id;
      struct motor_config_reply reply;
      if (ik_config_key(name, &id)) { rc = die("unknown CONFIG key %s", name); continue; }
      int failed = ik_config_exchange(&link, w[k].index, get ? MOTOR_CONFIG_GET : MOTOR_CONFIG_SET,
                                     id, get ? 0 : set_value, 1.0, &reply);
      if (!failed || reply.status)
        printf("%s: %s status=%s stored=%d effective=%d (RAM)\n", w[k].name, name,
               ik_config_status_name(reply.status), reply.stored, reply.effective);
      if (failed) rc = die("%s", link.err);
    }
  } else {
    rc = usage();
  }
  ik_link_close(&link);
  return rc;
}
