#include "drive_keyboard.h"
#include <ctype.h>
#include <math.h>
#include <string.h>

void drive_keyboard_press(drive_keyboard_t *s, int key, double now_s)
{
  key = tolower((unsigned char)key);
  if (key == 'x' || key == 27) {
    s->quitting = true;
    s->quit_deadline_s = now_s + 3;
  } else if (key == '+') s->speed_mps = fmin(s->max_speed_mps, nearbyint((s->speed_mps + 0.1) * 100) / 100);
  else if (key == '-') s->speed_mps = fmax(0.1, nearbyint((s->speed_mps - 0.1) * 100) / 100);
  else if (!s->quitting) drive_key_press(&s->keys, key, now_s);
}

bool drive_keyboard_vector(drive_keyboard_t *s, double now_s, int32_t vector[3])
{
  if (s->quitting) memset(vector, 0, 3 * sizeof *vector);
  else drive_key_vector(&s->keys, now_s, s->speed_mps, s->yaw_radps, vector);
  return !s->quitting;
}

static const char *state_name(uint32_t state, char value[16])
{
  static const char *const names[] = {"STOPPED", "DRIVING", "STOPPING"};
  if (state < 3) return names[state];
  snprintf(value, 16, "%u", state);
  return value;
}

static void vector_json(FILE *f, const int32_t *v, int n)
{
  fputc('[', f);
  for (int k = 0; k < n; k++) fprintf(f, "%s%d", k ? "," : "", v[k]);
  fputc(']', f);
}

int drive_keyboard_log(FILE *f, double now_s, double t0_s, const int32_t vector[3],
                        const ik_status_t *s, const ik_row_t rows[4])
{
  char value[16];
  fprintf(f, "{\"t\":%.3f,\"t_monotonic_s\":%.9f,\"t0_monotonic_s\":%.9f,\"vector\":", now_s, now_s, t0_s);
  vector_json(f, vector, 3);
  fprintf(f, ",\"status\":{\"state\":%u,\"state_name\":\"%s\",\"tx_count\":%u,\"tx_fail\":%u,"
             "\"bad_input\":%u,\"raw_done\":%u,\"telem_now\":%u,\"fault\":%u,\"armed\":%u,\"target_rpm\":", s->state,
          state_name(s->state, value), s->tx_count, s->tx_fail, s->bad_input, s->raw_done, s->telem_now, s->fault, s->armed);
  vector_json(f, s->target_rpm, 4);
  fputs(",\"rpm\":", f); vector_json(f, s->rpm, 4);
  fputs("},\"rows\":[", f);
  for (int w = 0; w < 4; w++) {
    const ik_row_t *r = &rows[w];
    if (w) fputc(',', f);
    if (!r->valid) { fputs("null", f); continue; }
    fprintf(f, "{\"generation\":%u,\"rpm\":%d,\"iq_counts\":%d,\"iq_ref_counts\":%d,"
               "\"state\":%u,\"flags\":%u,\"status_tick\":%u,\"status_count\":%u,"
               "\"vbus_centivolts\":%u,\"faults\":%u,\"hall_shift_deg\":%d,\"align_result\":%u,"
               "\"i2t_percent\":%u,\"diag_tick\":%u,\"diag_count\":%u,\"app_crc\":%u,\"app_size\":%u,"
               "\"ident_tick\":%u,\"ident_count\":%u,\"iq_a\":%.9g,\"iq_ref_a\":%.9g,\"vbus_v\":%.9g,"
               "\"stop_reason\":\"%s\",\"mc_state\":\"%s\",\"align\":\"%s\",\"active\":%s,"
               "\"derated\":%s,\"aligning\":%s,\"aligned\":%s,\"fault_names\":",
            r->generation, r->rpm, r->iq_counts, r->iq_ref_counts, r->state, r->flags, r->status_tick,
            r->status_count, r->vbus_centivolts, r->faults, r->hall_shift_deg, r->align_result, r->i2t_percent,
            r->diag_tick, r->diag_count, r->app_crc, r->app_size, r->ident_tick, r->ident_count, r->iq_a,
            r->iq_ref_a, r->vbus_v, r->stop_reason, r->mc_state, r->align,
            r->active ? "true" : "false", r->derated ? "true" : "false", r->aligning ? "true" : "false", r->aligned ? "true" : "false");
    pulse_json_names(f, r->fault_names);
    fputc('}', f);
  }
  fputs("]}\n", f);
  return fflush(f) || ferror(f) ? -1 : 0;
}

void drive_keyboard_show(const ik_status_t *s, const ik_row_t rows[4])
{
  puts("wheel  cmd/target rpm   rpm    Iq A  Iqref A  state            stop     Vbus   I2T  faults  hall  align");
  for (int w = 0; w < 4; w++) {
    const ik_row_t *r = &rows[w];
    printf("%s/%s %5d/%-5d ", IK_WHEEL_POS[w], IK_WHEEL_CORNER[w], s->rpm[w], s->target_rpm[w]);
    if (!r->valid || !r->status_count) { puts("no reply"); continue; }
    printf("%6d %6.2f %6.2f %-10s %c%c%c%c %-7s %5.2f %3u%% %-6s %4d %s%s\n",
           r->rpm, r->iq_a, r->iq_ref_a, r->mc_state, r->active ? 'A' : '-', r->derated ? 'D' : '-',
           r->aligning ? 'L' : '-', r->aligned ? 'H' : '!', r->stop_reason, r->vbus_v,
           r->i2t_percent, r->fault_names[0] ? r->fault_names : "-", r->hall_shift_deg, r->align,
           (uint32_t)(s->telem_now - r->status_tick) > 200 ? " STALE" : "");
  }
}
