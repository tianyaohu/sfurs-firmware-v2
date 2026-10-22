#include "drive_record.h"
#include "can_datatypes.h"
#include <string.h>

#define B(x) ((x) ? "true" : "false")

void drive_records_json(FILE *f, const drive_records_t *r)
{
  fputc('{', f);
  for (int w = 0; w < 4; w++) {
    fprintf(f, "%s\"%s\":", w ? "," : "", IK_WHEEL_POS[w]);
    if (!r->valid[w]) { drive_json_string(f, r->errors[w]); continue; }
    const drive_record_summary_t *s = &r->wheels[w];
    fprintf(f, "{\"samples\":%d,\"trigger\":%d,\"peak_rpm\":%d}", s->samples, s->trigger, s->peak_rpm);
  }
  fputc('}', f);
}

int drive_records_read(ik_link_t *l, const char *path, double t0_monotonic_s, drive_records_t *out)
{
  memset(out, 0, sizeof *out);
  FILE *f = fopen(path, "w");
  if (!f) return -1;
  fprintf(f, "{\"t0_monotonic_s\":%.9f", t0_monotonic_s);
  for (int w = 0; w < 4; w++) {
    ik_record_sample_t samples[ESC_RECORD_SAMPLES];
    ik_record_unpacked_t rows[ESC_RECORD_SAMPLES];
    int n, trigger;
    if (ik_record_read(l, IK_WHEEL_SELECT[w], samples, &n, &trigger)) {
      snprintf(out->errors[w], sizeof out->errors[w], "%s", l->err);
      fprintf(stderr, "record %s: %s\n", IK_WHEEL_POS[w], l->err);
      continue;
    }
    for (int k = 0; k < n; k++) ik_unpack_record(&samples[k], &rows[k]);
    out->valid[w] = true;
    out->wheels[w] = drive_record_summary(rows, n, trigger);
    fprintf(f, ",\"%s\":{\"info\":{\"status\":\"done\",\"samples\":%d,\"trigger_index\":%d},\"samples\":[", IK_WHEEL_POS[w], n, trigger);
    for (int k = 0; k < n; k++) {
      const ik_record_unpacked_t *r = &rows[k];
      fprintf(f, "%s{\"ref_rpm\":%d,\"rpm\":%d,\"p_a\":%.6g,\"i_a\":%.6g,\"ff_a\":%.6g,"
                 "\"iq_ref_a\":%.6g,\"iq_a\":%.6g,\"mode\":\"%s\",\"valid\":%s,\"saturated\":%s,"
                 "\"bleed\":%s,\"stall_timer\":%s,\"loop\":%s,\"edges_mod16\":%d,\"hall_state\":%d}",
              k ? "," : "", r->ref_rpm, r->rpm, r->p_a, r->i_a, r->ff_a, r->iq_ref_a, r->iq_a,
              r->mode, B(r->valid), B(r->saturated), B(r->bleed), B(r->stall_timer), B(r->loop),
              r->edges_mod16, r->hall_state);
    }
    fputs("]}", f);
  }
  fputs("}\n", f);
  return fclose(f) ? -1 : 0;
}
