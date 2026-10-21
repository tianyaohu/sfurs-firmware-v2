#include "drive_record.h"
#include "can_datatypes.h"
#include <string.h>

void drive_records_json(FILE *f, const drive_records_t *r)
{
  fputc('{', f);
  for (int w = 0; w < 4; w++) {
    fprintf(f, "%s\"%s\":", w ? "," : "", IK_WHEEL_POS[w]);
    if (!r->valid[w]) { drive_json_string(f, r->errors[w]); continue; }
    const drive_record_summary_t *s = &r->wheels[w];
    fprintf(f, "{\"samples\":%d,\"trigger\":%d,\"handoff_ms\":", s->samples, s->trigger);
    if (s->handoff) fprintf(f, "%d", s->handoff_ms); else fputs("null", f);
    fputs(",\"handoff_iq_a\":", f);
    if (s->handoff) fprintf(f, "%.2f", s->handoff_iq_a); else fputs("null", f);
    fprintf(f, ",\"peak_rpm\":%d}", s->peak_rpm);
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
                 "\"iq_ref_a\":%.6g,\"iq_a\":%.6g,\"state\":\"%s\",\"reason\":\"%s\","
                 "\"capped\":%s,\"dsc\":%s,\"loop\":%s,\"edges_mod16\":%d,\"captures_mod8\":%d}",
              k ? "," : "", r->ref_rpm, r->rpm, r->p_a, r->i_a, r->ff_a, r->iq_ref_a, r->iq_a,
              r->state, r->reason, r->capped ? "true" : "false", r->dsc ? "true" : "false",
              r->loop ? "true" : "false", r->edges_mod16, r->captures_mod8);
    }
    fputs("]}", f);
  }
  fputs("}\n", f);
  return fclose(f) ? -1 : 0;
}
