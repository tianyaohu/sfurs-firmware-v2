/** @file startup_report.c
 * @brief Aggregate move_test startup.tsv files; missing evidence cannot pass.
 * TSV schema (one row per recorded wheel, not per out/back leg):
 * run_id, axis (vx/vy/wz), sign (+/- body command), wheel (FR/BR/BL/FL),
 * verdict, first_failed_k (0 if not applicable), deadlines_met, measurable_count,
 * overshoot_ratio (or na), steady_rpm (validated plateau magnitude, or na),
 * failed_rule. Fields are separated by tabs.
 * SPEC-QUESTION: failed_rule is an additional final column because k alone
 * cannot distinguish a missed deadline from a reversed edge in the histogram.
 * This gate covers tracking and overshoot. APPLIED timestamps are not in this
 * recorder/TSV, so synchronization is explicitly reported as NOT_MEASURABLE.
 */
#include "startup_metrics.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  char id[128];
  int axis, sign;
  unsigned wheels;
} report_run_t;

static int name_index(const char *name, const char *const *names, int n)
{
  for (int i = 0; i < n; ++i) if (!strcmp(name, names[i])) return i;
  return -1;
}
static int bounded_int(const char *text, int hi)
{
  char *end;
  errno = 0;
  long value = strtol(text, &end, 10);
  return errno || end == text || *end || value < 0 || value > hi ? -1 : (int)value;
}

int main(int argc, char **argv)
{
  const char *const axes[] = {"vx", "vy", "wz"};
  const char *const wheels[] = {"FR", "BR", "BL", "FL"};
  unsigned counts[3][2][4][4] = {{{{0}}}};
  unsigned histogram[STARTUP_RULE_COUNT][9] = {{0}};
  report_run_t *runs = NULL;
  size_t n_runs = 0;
  unsigned active = 0;
  bool invalid = false, failed = false, missing = false;
  if (argc < 2) { fprintf(stderr, "usage: startup_report DIR...\n"); return 1; }
  for (int arg = 1; arg < argc; ++arg) {
    char path[4096];
    int length = snprintf(path, sizeof path, "%s/startup.tsv", argv[arg]);
    if (length < 0 || (size_t)length >= sizeof path) { invalid = true; continue; }
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); invalid = true; continue; }
    char line[1024];
    unsigned line_no = 0, file_rows = 0;
    while (fgets(line, sizeof line, f)) {
      ++line_no;
      if (line[0] == '#') continue;
      char *newline = strchr(line, '\n');
      if (!newline) {
        fprintf(stderr, "%s:%u: truncated row\n", path, line_no);
        invalid = true; break;
      }
      *newline = 0;
      char *fields[11];
      size_t n_fields = 1;
      fields[0] = line;
      for (char *p = line; *p; ++p) if (*p == '\t') {
        *p = 0;
        if (n_fields == 11) { n_fields = 12; break; }
        fields[n_fields++] = p + 1;
      }
      if (n_fields != 11) goto bad_row;
      for (size_t i = 0; i < 11; ++i) if (!*fields[i]) goto bad_row;
      int a = name_index(fields[1], axes, 3);
      int s = !strcmp(fields[2], "+") ? 1 : !strcmp(fields[2], "-") ? 0 : -1;
      int w = name_index(fields[3], wheels, 4);
      int v = -1, rule = -1;
      for (int i = STARTUP_PASS; i <= STARTUP_INACTIVE; ++i)
        if (!strcmp(fields[4], startup_verdict_name(i))) v = i;
      for (int i = 0; i < STARTUP_RULE_COUNT; ++i)
        if (!strcmp(fields[10], startup_rule_name(i))) rule = i;
      int k = bounded_int(fields[5], 8), met = bounded_int(fields[6], 8);
      int measurable = bounded_int(fields[7], 8);
      double ratio = NAN;
      if (strcmp(fields[8], "na")) {
        char *end;
        errno = 0;
        ratio = strtod(fields[8], &end);
        if (errno || end == fields[8] || *end || !isfinite(ratio) || ratio < 0) goto bad_row;
      }
      double steady_rpm = NAN;
      if (strcmp(fields[9], "na")) {
        char *end;
        errno = 0;
        steady_rpm = strtod(fields[9], &end);
        if (errno || end == fields[9] || *end || !isfinite(steady_rpm) || steady_rpm < 2) goto bad_row;
      }
      if (isfinite(ratio) && !isfinite(steady_rpm)) goto bad_row;
      if (rule == STARTUP_RULE_NO_PLATEAU && (isfinite(steady_rpm) || isfinite(ratio))) goto bad_row;
      if (a < 0 || s < 0 || w < 0 || v < 0 || rule < 0 || k < 0 || met < 0 || measurable < 0 ||
          strlen(fields[0]) >= sizeof runs[0].id) goto bad_row;
      if (v == STARTUP_PASS && (k || met != 8 || measurable < 3 || !isfinite(ratio) || ratio > 1.3 || rule)) goto bad_row;
      if (v == STARTUP_FAIL && (rule < STARTUP_RULE_DEADLINE || rule > STARTUP_RULE_OVERSHOOT)) goto bad_row;
      if (v == STARTUP_FAIL && ((rule == STARTUP_RULE_OVERSHOOT && (k || !isfinite(ratio) || ratio <= 1.3)) ||
          (rule != STARTUP_RULE_OVERSHOOT && !k))) goto bad_row;
      if (v == STARTUP_NOT_MEASURABLE && (k || rule < STARTUP_RULE_INPUT)) goto bad_row;
      if (v == STARTUP_INACTIVE && (k || met || measurable || isfinite(ratio) || rule)) goto bad_row;
      size_t run;
      for (run = 0; run < n_runs; ++run) if (!strcmp(runs[run].id, fields[0])) break;
      if (run == n_runs) {
        report_run_t *more = realloc(runs, (n_runs + 1) * sizeof *runs);
        if (!more) { fclose(f); free(runs); return 1; }
        runs = more;
        runs[n_runs] = (report_run_t){.axis = a, .sign = s};
        strcpy(runs[n_runs++].id, fields[0]);
      }
      if (runs[run].axis != a || runs[run].sign != s || (runs[run].wheels & (1U << w))) goto bad_row;
      runs[run].wheels |= 1U << w;
      ++file_rows;
      ++counts[a][s][w][v];
      if (v != STARTUP_INACTIVE) ++active;
      if (v == STARTUP_FAIL) { failed = true; ++histogram[rule][k]; }
      if (v == STARTUP_NOT_MEASURABLE) missing = true;
      continue;
    bad_row:
      fprintf(stderr, "%s:%u: invalid or duplicate startup row\n", path, line_no);
      invalid = true;
    }
    if (ferror(f) || !file_rows) invalid = true;
    if (fclose(f)) invalid = true;
  }
  /* SPEC-QUESTION: empty/all-inactive input, incomplete four-wheel runs and
   * duplicate run/wheel rows cannot establish an acceptance gate. */
  for (size_t i = 0; i < n_runs; ++i) if (runs[i].wheels != 15) {
    fprintf(stderr, "run %s: missing wheel rows\n", runs[i].id);
    invalid = true;
  }
  free(runs);
  puts("axis\tsign\twheel\tPASS\tFAIL\tNOT_MEASURABLE\tINACTIVE");
  for (int a = 0; a < 3; ++a) for (int s = 0; s < 2; ++s) for (int w = 0; w < 4; ++w) {
    const unsigned *c = counts[a][s][w];
    printf("%s\t%s\t%s\t%u\t%u\t%u\t%u\n", axes[a], s ? "+" : "-", wheels[w], c[0], c[1], c[2], c[3]);
  }
  puts("failed_rule\tk\tcount");
  for (int r = 0; r < STARTUP_RULE_COUNT; ++r) for (int k = 0; k <= 8; ++k)
    if (histogram[r][k]) printf("%s\t%d\t%u\n", startup_rule_name(r), k, histogram[r][k]);
  bool pass = active && !failed && !missing && !invalid;
  puts("scope: startup tracking + overshoot; synchronization: NOT_MEASURABLE (no APPLIED timestamps)");
  printf("overall: %s%s\n", pass ? "PASS" : failed || invalid ? "FAIL" : "NOT_MEASURABLE",
         invalid ? " (invalid or missing evidence)" : "");
  return pass ? 0 : 1;
}
