#include "pulse_run.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esc_defaults.h"

double pulse_now_s(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

void pulse_sleep_s(double s) { nanosleep(&(struct timespec){(time_t)s, (long)((s - (time_t)s) * 1e9)}, NULL); }

int pulse_wait_idle(ik_link_t *l, const int *idx, int n, double timeout_s)
{
  double deadline = pulse_now_s() + timeout_s;
  while (pulse_now_s() < deadline) {
    ik_row_t rows[IK_WHEELS];
    if (ik_rows(l, rows)) return 0;
    int all = 1;
    for (int k = 0; k < n; k++) all &= rows[idx[k]].valid && !strcmp(rows[idx[k]].mc_state, "IDLE");
    if (all) return 1;
    pulse_sleep_s(0.05);
  }
  return 0;
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

void pulse_json_names(FILE *f, const char *names)
{
  fputc('[', f);
  int first = 1;
  for (const char *p = names; *p;) {
    const char *e = strchr(p, ' ');
    size_t len = e ? (size_t)(e - p) : strlen(p);
    fprintf(f, "%s\"%.*s\"", first ? "" : ",", (int)len, p);
    first = 0;
    p += len + (e ? 1 : 0);
  }
  fputc(']', f);
}

void pulse_result_free(pulse_result_t *r)
{
  free(r->log);
  memset(r, 0, sizeof *r);
}

int pulse_run(ik_link_t *l, const esc_move_t *plan, size_t n_moves, double duration,
              const pulse_settings_t *settings, int accel_rpmps, pulse_result_t *out)
{
  memset(out, 0, sizeof *out);
  ik_row_t before[IK_WHEELS], rows[IK_WHEELS];
  if (ik_rows(l, before)) return -1;
  pulse_sleep_s(0.3);
  if (ik_rows(l, rows)) return -1;
  char dead[64] = "";
  for (int i = 0; i < IK_WHEELS; i++) /* rows keep the last values; only a moving reply count is live */
    if (!rows[i].valid || !before[i].valid || rows[i].status_count == before[i].status_count)
      snprintf(dead + strlen(dead), sizeof dead - strlen(dead), "%s%s", dead[0] ? ", " : "", IK_WHEEL_POS[i]);
  if (dead[0]) {
    snprintf(l->err, sizeof l->err, "pulse: no live reply from %s (battery on? CAN?)", dead);
    return -1;
  }

  size_t cap = 4096;
  out->log = malloc(cap * sizeof *out->log);
  esc_guard_t guard;
  esc_guard_init(&guard);
  uint32_t last_counts[IK_WHEELS];
  int have_last = 0, err = 0;
  int32_t twist[3] = {0, 0, 0};
  out->t0_monotonic_s = pulse_now_s();
  for (int i = 0; !err && i < IK_WHEELS; i++)
    for (size_t k = 0; !err && k < settings->n_keys; k++)
      err = send_config(l, IK_WHEEL_SELECT[i], settings->keys[k], settings->value[i][k]);
  if (!err) err = ik_write(l, IK_ACCEL_RPMPS, (uint32_t)accel_rpmps);
  if (!err) {
    out->t0_monotonic_s = pulse_now_s();
    double next_cmd = 0, now;
    while (!err && (now = pulse_now_s() - out->t0_monotonic_s) < duration) {
      if (now >= next_cmd) {
        esc_plan_twist(plan, n_moves, now, twist);
        if (ik_drive(l, twist[0], twist[1], twist[2], true)) { err = 1; break; }
        next_cmd = now + 0.02;
      }
      ik_status_t st;
      if (ik_status(l, &st) || ik_rows(l, rows)) { err = 1; break; }
      uint32_t counts[IK_WHEELS];
      int changed = !have_last;
      for (int i = 0; i < IK_WHEELS; i++) {
        counts[i] = rows[i].valid ? rows[i].status_count : 0xFFFFFFFFU;
        changed |= counts[i] != last_counts[i];
      }
      if (!changed) continue;
      memcpy(last_counts, counts, sizeof counts);
      have_last = 1;
      if (out->n == cap) out->log = realloc(out->log, (cap *= 2) * sizeof *out->log); /* an entry whenever any wheel replied */
      pulse_entry_t *p = &out->log[out->n++];
      memset(p, 0, sizeof *p);
      p->e.t = round(now * 1e4) / 1e4;
      memcpy(p->twist, twist, sizeof twist);
      for (int i = 0; i < IK_WHEELS; i++) {
        p->e.target_rpm[i] = st.target_rpm[i];
        p->e.rpm_ok[i] = p->e.iq_ok[i] = rows[i].valid;
        if (rows[i].valid) {
          p->e.rpm[i] = rows[i].rpm;
          p->e.iq_ref_a[i] = round(rows[i].iq_ref_a * 1000) / 1000;
          snprintf(p->e.faults[i], sizeof p->e.faults[i], "%s", rows[i].fault_names);
          p->mc_state[i] = rows[i].mc_state;
          p->vbus_v[i] = rows[i].vbus_v;
        }
      }
      if (esc_guard_check(&guard, &p->e, out->aborted, sizeof out->aborted)) break;
    }
  }
  char err_text[sizeof l->err];
  snprintf(err_text, sizeof err_text, "%s", l->err);
  /* Always: stop driving, wait for STOPPED and IDLE, restore the firmware defaults. */
  (void)ik_write(l, IK_ENABLE, 0);
  double deadline = pulse_now_s() + 3.0;
  uint32_t state = 1;
  while (ik_read(l, IK_STATE, &state) == 0 && state != 0 && pulse_now_s() < deadline) pulse_sleep_s(0.05);
  const int all[IK_WHEELS] = {0, 1, 2, 3};
  if (!pulse_wait_idle(l, all, IK_WHEELS, 3.0))
    fprintf(stderr, "pulse: not all wheels IDLE; defaults may not be restored (esc_tool set)\n");
  for (int i = 0; i < IK_WHEELS; i++)
    for (size_t k = 0; k < settings->n_keys; k++) {
      int def;
      if (esc_default(settings->keys[k], &def) == 0) (void)send_config(l, IK_WHEEL_SELECT[i], settings->keys[k], def);
    }
  if (err) {
    snprintf(l->err, sizeof l->err, "%s", err_text);
    return -1;
  }
  return 0;
}

int pulse_log_write(const char *path, const pulse_result_t *r)
{
  FILE *f = fopen(path, "w");
  if (!f) return -1;
  for (size_t i = 0; i < r->n; i++) {
    const pulse_entry_t *p = &r->log[i];
    const esc_log_entry_t *e = &p->e;
    fprintf(f, "{\"t\":%.4f,\"twist\":[%d,%d,%d],\"target_rpm\":[%d,%d,%d,%d],\"rpm\":[", e->t, p->twist[0],
            p->twist[1], p->twist[2], e->target_rpm[0], e->target_rpm[1], e->target_rpm[2], e->target_rpm[3]);
    for (int w = 0; w < IK_WHEELS; w++) {
      if (e->rpm_ok[w]) fprintf(f, "%s%d", w ? "," : "", e->rpm[w]);
      else fprintf(f, "%snull", w ? "," : "");
    }
    fputs("],\"iq_ref_a\":[", f);
    for (int w = 0; w < IK_WHEELS; w++) {
      if (e->iq_ok[w]) fprintf(f, "%s%.3f", w ? "," : "", e->iq_ref_a[w]);
      else fprintf(f, "%snull", w ? "," : "");
    }
    fputs("],\"mc_state\":[", f);
    for (int w = 0; w < IK_WHEELS; w++) fprintf(f, "%s\"%s\"", w ? "," : "", p->mc_state[w] ? p->mc_state[w] : "");
    fputs("],\"faults\":[", f);
    for (int w = 0; w < IK_WHEELS; w++) {
      if (w) fputc(',', f);
      pulse_json_names(f, e->faults[w]);
    }
    fprintf(f, "],\"vbus_v\":[%.2f,%.2f,%.2f,%.2f]}\n", p->vbus_v[0], p->vbus_v[1], p->vbus_v[2], p->vbus_v[3]);
  }
  fclose(f);
  return 0;
}
