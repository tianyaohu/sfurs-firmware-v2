/*
 * Plan or execute oblique out-and-back pulses through the Nucleo IK link.
 *
 *   sweep [--angles-deg A...] [--speeds S...] [--seconds 1.5] [--pause 1]
 *         [--out FILE --execute] [--elf F] [--sn SERIAL]
 *
 * Default: print the plan, no hardware access. --execute drives the robot! Battery ON, wheels
 * raised for the first run after firmware changes; otherwise a clear floor with an operator
 * ready to stop. Read system-info/hardware.md first. The log (JSON lines, one per 20 ms
 * tick, CLOCK_MONOTONIC "t") is what the rig's friction-ID analysis reads.
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ik_link.h"
#include "pulse_run.h"

#define MAX_STEPS 4096

typedef struct { double duration_s; int32_t twist[3]; } step_t;

static int build_plan(const double *angles, int na, const double *speeds, int ns, double seconds, double pause,
                      step_t *plan, int *n)
{
  if (!na || !ns || !isfinite(seconds) || !isfinite(pause)) return -1;
  for (int i = 0; i < ns; i++)
    if (!(speeds[i] > 0 && speeds[i] <= 0.3)) return -1;
  if (!(seconds > 0 && seconds <= 3) || pause < 0.5) return -1;
  int k = 0;
  plan[k++] = (step_t){pause, {0, 0, 0}};
  for (int a = 0; a < na; a++)
    for (int s = 0; s < ns; s++) {
      double rad = angles[a] * M_PI / 180.0;
      int32_t vx = (int32_t)lround(1000 * speeds[s] * cos(rad)), vy = (int32_t)lround(1000 * speeds[s] * sin(rad));
      for (int sign = 1; sign >= -1; sign -= 2) {
        if (k + 2 > MAX_STEPS) return -1;
        plan[k++] = (step_t){seconds, {sign * vx, sign * vy, 0}};
        plan[k++] = (step_t){pause, {0, 0, 0}};
      }
    }
  *n = k;
  return 0;
}

static void write_row(FILE *f, const ik_row_t *r)
{
  fprintf(f, "{\"generation\":%u,\"rpm\":%d,\"iq_counts\":%d,\"iq_ref_counts\":%d,\"state\":%u,\"flags\":%u,"
             "\"status_tick\":%u,\"status_count\":%u,\"vbus_centivolts\":%u,\"faults\":%u,\"hall_shift_deg\":%d,"
             "\"align_result\":%u,\"i2t_percent\":%u,\"diag_tick\":%u,\"diag_count\":%u,\"app_crc\":%u,"
             "\"app_size\":%u,\"ident_tick\":%u,\"ident_count\":%u,\"iq_a\":%.9g,\"iq_ref_a\":%.9g,"
             "\"vbus_v\":%.9g,\"stop_reason\":\"%s\",\"active\":%s,\"derated\":%s,\"aligning\":%s,\"aligned\":%s,"
             "\"mc_state\":\"%s\",\"fault_names\":",
          r->generation, r->rpm, r->iq_counts, r->iq_ref_counts, r->state, r->flags, r->status_tick,
          r->status_count, r->vbus_centivolts, r->faults, r->hall_shift_deg, r->align_result, r->i2t_percent,
          r->diag_tick, r->diag_count, r->app_crc, r->app_size, r->ident_tick, r->ident_count, r->iq_a,
          r->iq_ref_a, r->vbus_v, r->stop_reason, r->active ? "true" : "false", r->derated ? "true" : "false",
          r->aligning ? "true" : "false", r->aligned ? "true" : "false", r->mc_state);
  pulse_json_names(f, r->fault_names);
  fprintf(f, ",\"align\":\"%s\"}", r->align);
}

static int execute(const char *sn, const char *elf, const char *out, const step_t *plan, int n)
{
  ik_link_t l;
  if (ik_link_open(&l, sn, elf)) { fprintf(stderr, "%s\n", l.err); return 1; }
  FILE *f = fopen(out, "w");
  int rc = 0;
  uint32_t state = 1; /* assume DRIVING until read back */
  ik_row_t rows[IK_WHEELS];
  if (!f) { fprintf(stderr, "cannot write %s\n", out); rc = 1; goto stop; }
  if (ik_read(&l, IK_STATE, &state) || state != 0) { fprintf(stderr, "drive must be stopped before the sweep\n"); rc = 1; goto stop; }
  if (ik_rows(&l, rows)) { fprintf(stderr, "%s\n", l.err); rc = 1; goto stop; }
  for (int i = 0; i < IK_WHEELS; i++)
    if (!rows[i].valid || strcmp(rows[i].mc_state, "IDLE") || rows[i].faults || !rows[i].aligned) {
      fprintf(stderr, "all four wheels must report IDLE, aligned, without faults\n");
      rc = 1;
      goto stop;
    }
  uint32_t last_count[IK_WHEELS];
  double last_seen[IK_WHEELS];
  for (int i = 0; i < IK_WHEELS; i++) { last_count[i] = 0xFFFFFFFFU; last_seen[i] = pulse_now_s(); }
  for (int s = 0; s < n && !rc; s++) {
    double deadline = pulse_now_s() + plan[s].duration_s;
    while (pulse_now_s() < deadline) {
      double tick = pulse_now_s();
      ik_status_t st;
      if (ik_drive(&l, plan[s].twist[0], plan[s].twist[1], plan[s].twist[2], true) || ik_status(&l, &st) ||
          ik_rows(&l, rows)) { fprintf(stderr, "%s\n", l.err); rc = 1; break; }
      for (int i = 0; i < IK_WHEELS && !rc; i++) {
        if (!rows[i].valid) { fprintf(stderr, "incomplete telemetry; sweep stopped\n"); rc = 1; break; }
        if (rows[i].faults || rows[i].derated) { fprintf(stderr, "wheel fault/derating; sweep stopped\n"); rc = 1; break; }
        if (rows[i].status_count != last_count[i]) { last_count[i] = rows[i].status_count; last_seen[i] = pulse_now_s(); }
        if (pulse_now_s() - last_seen[i] > 0.2) { fprintf(stderr, "stale %s telemetry; sweep stopped\n", IK_WHEEL_POS[i]); rc = 1; }
      }
      if (rc) break;
      fprintf(f, "{\"t\":%.6f,\"twist\":[%d,%d,%d],\"target_rpm\":[%d,%d,%d,%d],\"rows\":[", pulse_now_s(),
              plan[s].twist[0], plan[s].twist[1], plan[s].twist[2], st.target_rpm[0], st.target_rpm[1],
              st.target_rpm[2], st.target_rpm[3]);
      for (int i = 0; i < IK_WHEELS; i++) {
        if (i) fputc(',', f);
        write_row(f, &rows[i]);
      }
      fputs("]}\n", f);
      fflush(f);
      double wait = tick + 0.02 - pulse_now_s();
      if (wait > 0) pulse_sleep_s(wait);
    }
  }
stop: /* always stop driving and wait for STOPPED */
  (void)ik_write(&l, IK_ENABLE, 0);
  double give_up = pulse_now_s() + 3;
  while (ik_read(&l, IK_STATE, &state) == 0 && state != 0 && pulse_now_s() < give_up) pulse_sleep_s(0.05);
  printf("drive state: %s\n", state == 0 ? "STOPPED" : state == 1 ? "DRIVING" : state == 2 ? "STOPPING" : "unknown");
  if (f) fclose(f);
  ik_link_close(&l);
  return rc;
}

int main(int argc, char **argv)
{
  double angles[64], speeds[16], seconds = 1.5, pause = 1;
  int na = 0, ns = 0, run = 0;
  const char *out = NULL, *elf = IK_DEFAULT_ELF, *sn = IK_DEFAULT_SN;
  for (int i = 1; i < argc; i++) {
    const char *a = argv[i];
    if (!strcmp(a, "--execute")) { run = 1; continue; }
    if (!strcmp(a, "--angles-deg") || !strcmp(a, "--speeds")) {
      int is_angle = !strcmp(a, "--angles-deg");
      int *count = is_angle ? &na : &ns;
      *count = 0;
      /* a value is a number; a leading '-' counts only before a digit or '.', never for "--option" */
      while (i + 1 < argc && (isdigit((unsigned char)argv[i + 1][0]) || argv[i + 1][0] == '.' ||
                              (argv[i + 1][0] == '-' && (isdigit((unsigned char)argv[i + 1][1]) || argv[i + 1][1] == '.')))) {
        if (*count >= (is_angle ? 64 : 16)) { fprintf(stderr, "too many values\n"); return 2; }
        (is_angle ? angles : speeds)[(*count)++] = atof(argv[++i]);
      }
      continue;
    }
    if (i + 1 >= argc) { fprintf(stderr, "bad option %s\n", a); return 2; }
    if (!strcmp(a, "--seconds")) seconds = atof(argv[++i]);
    else if (!strcmp(a, "--pause")) pause = atof(argv[++i]);
    else if (!strcmp(a, "--out")) out = argv[++i];
    else if (!strcmp(a, "--elf")) elf = argv[++i];
    else if (!strcmp(a, "--sn")) sn = argv[++i];
    else { fprintf(stderr, "unknown option %s\n", a); return 2; }
  }
  if (!na) for (; na < 8; na++) angles[na] = 11.25 + 22.5 * na;
  if (!ns) { speeds[0] = 0.1; speeds[1] = 0.2; speeds[2] = 0.3; ns = 3; }
  static step_t plan[MAX_STEPS];
  int n = 0;
  if (build_plan(angles, na, speeds, ns, seconds, pause, plan, &n)) {
    fprintf(stderr, "require speeds 0..0.3 m/s, seconds 0..3, pause >= 0.5 s, finite and nonempty\n");
    return 2;
  }
  if (!run) {
    double total = 0;
    for (int i = 0; i < n; i++) total += plan[i].duration_s;
    printf("duration_s %.1f, %d steps\n", total, n);
    for (int i = 0; i < n; i++) printf("  %.2f s  twist %d %d %d\n", plan[i].duration_s, plan[i].twist[0], plan[i].twist[1], plan[i].twist[2]);
    return 0;
  }
  if (!out) { fprintf(stderr, "--execute requires --out\n"); return 2; }
  return execute(sn, elf, out, plan, n);
}
