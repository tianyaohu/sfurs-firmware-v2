#include "startup_metrics.h"
#include "check.h"
#include <string.h>
#include <unistd.h>

/* Compile the real report entry point so file/schema/gate failures are tested. */
#define main startup_report_main
#include "../src/tools/startup_report.c"
#undef main
#define main startup_move_test_main
#include "../src/tools/move_test.c"
#undef main

static ik_record_unpacked_t rows[STARTUP_MAX_SAMPLES];
static int missing_wheel = -1;
static unsigned reads[4], arms[4], opens;
static double fake_start_s = 100;
static const int wheel_plateaus[4] = {120, 130, 140, 150};

/* Fake documented tool I/O boundaries only; every reference and edge below is
 * supplied directly by trace(), and all output/acceptance code remains real. */
int __wrap_ik_link_open(ik_link_t *link, const char *sn, const char *elf)
{
  (void)sn; (void)elf; memset(link, 0, sizeof *link); ++opens; return 0;
}
void __wrap_ik_link_close(ik_link_t *link) { (void)link; }
int __wrap_drive_preflight(ik_link_t *link, double vbus) { (void)link; (void)vbus; return 0; }
int __wrap_ik_write(ik_link_t *link, ik_sym_t sym, uint32_t value)
{
  (void)link; CHECK_EQ(sym, IK_ENABLE); CHECK_EQ(value, 0); return 0;
}
int __wrap_ik_record_start(ik_link_t *link, uint32_t selector, int pre, int delay_ms)
{
  (void)link; CHECK_EQ(pre, 50); CHECK_EQ(delay_ms, 0);
  for (int w = 0; w < 4; ++w) if (selector == IK_WHEEL_SELECT[w]) { ++arms[w]; return 0; }
  CHECK(false); return -1;
}
int __wrap_drive_stage_run(ik_link_t *link, const char *dir, const char *name,
                          const esc_move_t *plan, size_t n, double duration_s,
                          const drive_settings_t *settings, int accel, drive_stage_t *out)
{
  (void)link; (void)dir; (void)name; (void)plan; (void)n;
  (void)duration_s; (void)settings; (void)accel;
  out->analysis = calloc(1, sizeof *out->analysis);
  CHECK(out->analysis != NULL);
  out->pulse.t0_monotonic_s = fake_start_s++;
  return 0;
}
int __wrap_drive_records_read(ik_link_t *link, const char *path, double t0, drive_records_t *out)
{
  (void)link; (void)path; (void)t0;
  for (int w = 0; w < 4; ++w) out->valid[w] = true;
  return 0;
}
int __wrap_ik_record_read(ik_link_t *link, uint32_t selector, ik_record_sample_t *packed,
                        int *n, int *trigger)
{
  (void)link;
  for (int w = 0; w < 4; ++w) if (selector == IK_WHEEL_SELECT[w]) {
    ++reads[w];
    if (w == missing_wheel) return -1;
    *n = STARTUP_MAX_SAMPLES; *trigger = 2;
    for (int i = 0; i < *n; ++i) {
      int sign = w == 1 || w == 2 ? -1 : 1;
      int edges = (sign * rows[i].edges_mod16 + 16) % 16;
      int ref = i >= STARTUP_MAX_SAMPLES - 50 ? wheel_plateaus[w] : rows[i].ref_rpm;
      packed[i] = (ik_record_sample_t){.ref_rpm = sign * ref, .packed = edges << 8};
    }
    return 0;
  }
  CHECK(false); return -1;
}

/* MCSDK positive electrical order (hall_speed_pos_fdbk.c): sector k shows state HALL_ORDER[k]. */
static const int HALL_ORDER[6] = {5, 1, 3, 2, 6, 4};
static bool trace_hall; /* also record the Hall state of every row */

/* Construct reference and boundary crossings directly, without motor dynamics. */
static void trace(int sign, double offset, int delay, const startup_calibration_t *cal)
{
  memset(rows, 0, sizeof rows);
  double angle = 0;
  int sector = cal->starting_sector < 0 ? 0 : cal->starting_sector;
  double next = cal->widths_deg[sector] - offset;
  int counter = 15;
  rows[0].edges_mod16 = counter;
  if (trace_hall) rows[0].hall_state = HALL_ORDER[sector];
  for (int t = 1; t < STARTUP_MAX_SAMPLES; ++t) {
    rows[t].ref_rpm = sign * (t < 120 ? t : 120);
    int source = t - delay;
    if (source > 0) {
      double a = source - 1 < 120 ? source - 1 : 120;
      double b = source < 120 ? source : 120;
      angle += (a + b) * 0.003;
    }
    while (angle + 1e-10 >= next) {
      counter = (counter + sign + 16) % 16;
      sector = (sector + sign + 6) % 6;
      next += cal->widths_deg[sector];
    }
    rows[t].edges_mod16 = counter;
    if (trace_hall) rows[t].hall_state = HALL_ORDER[sector];
  }
}
static startup_result_t nominal(void)
{
  startup_calibration_t c = startup_nominal(0);
  return startup_evaluate(rows, STARTUP_MAX_SAMPLES, &c);
}
static void tracking(void)
{
  startup_calibration_t c = startup_nominal(0);
  const int signs[4] = {1, -1, -1, 1};
  for (int w = 0; w < 4; ++w) {
    for (int u = 0; u < 30; ++u) {
      trace(signs[w], u * 0.25, 0, &c);
      startup_result_t r = nominal();
      CHECK_EQ(r.verdict, STARTUP_PASS);
      CHECK_EQ(r.deadlines_met, 8);
      CHECK(r.measurable_count >= 3);
      CHECK_EQ(r.sign, signs[w]);
      for (int k = 0; k < 8; ++k) CHECK_EQ(r.transitions[k].direction, signs[w]);
    }
    trace(signs[w], 0, 25, &c);
    CHECK_EQ(nominal().verdict, STARTUP_FAIL);
    CHECK_EQ(nominal().first_failed_k, 1);
    trace(signs[w], 0, 15, &c);
    CHECK_EQ(nominal().verdict, STARTUP_PASS);
    trace(signs[w], 0, 20, &c);
    /* Rounded tick timestamps put boundary 2 just beyond the exact deadline. */
    CHECK_EQ(nominal().verdict, STARTUP_FAIL);
    CHECK_EQ(nominal().first_failed_k, 2);
    trace(signs[w], 0, 21, &c);
    CHECK_EQ(nominal().rule, STARTUP_RULE_DEADLINE);
    trace(signs[w], 0, 0, &c);
    rows[60].edges_mod16 = (rows[59].edges_mod16 - signs[w] + 16) % 16;
    CHECK_EQ(nominal().rule, STARTUP_RULE_DIRECTION);
    CHECK_EQ(nominal().first_failed_k, 2);
    for (int t = 0; t < STARTUP_MAX_SAMPLES; ++t) rows[t].edges_mod16 = 0;
    CHECK_EQ(nominal().rule, STARTUP_RULE_DEADLINE);
    CHECK_EQ(nominal().first_failed_k, 1);
    CHECK_EQ(nominal().verdict, STARTUP_FAIL);
    rows[1].edges_mod16 = signs[w] > 0 ? 15 : 1;
    CHECK_EQ(nominal().rule, STARTUP_RULE_DIRECTION);
  }
}
static void interpolation_and_incomplete(void)
{
  startup_calibration_t c = startup_nominal(0);
  trace(1, 0, 0, &c);
  CHECK_NEAR(startup_t_ref(rows, 384, 0.00075), 0.5, 1e-12);
  CHECK_NEAR(startup_t_ref(rows, 384, 7.5), 50, 1e-10);
  CHECK_EQ(startup_t_ref(rows, 384, -10), 0);
  CHECK(isinf(startup_t_ref(rows, 384, 1000)));
  CHECK_EQ(startup_evaluate(rows, 40, &c).verdict, STARTUP_NOT_MEASURABLE);
  CHECK_EQ(startup_evaluate(rows, 180, &c).verdict, STARTUP_NOT_MEASURABLE);
  CHECK_EQ(startup_evaluate(NULL, 0, &c).verdict, STARTUP_NOT_MEASURABLE);
  CHECK_EQ(startup_evaluate(rows, 385, &c).verdict, STARTUP_NOT_MEASURABLE);
  rows[0].ref_rpm = 2;
  CHECK_EQ(nominal().rule, STARTUP_RULE_BASELINE);
  rows[0].ref_rpm = 0;
  rows[200].ref_rpm = -1;
  CHECK_EQ(nominal().rule, STARTUP_RULE_REFERENCE);
  memset(rows, 0, sizeof rows);
  CHECK_EQ(nominal().verdict, STARTUP_INACTIVE);
  for (int i = 0; i < 384; ++i) rows[i].ref_rpm = -1;
  CHECK_EQ(nominal().verdict, STARTUP_INACTIVE);
  c.spread_deg = NAN;
  trace(1, 0, 0, &c);
  CHECK_EQ(startup_evaluate(rows, 384, &c).verdict, STARTUP_NOT_MEASURABLE);
  c = startup_nominal(30);
  CHECK_EQ(startup_evaluate(rows, 384, &c).rule, STARTUP_RULE_MEASURABLE);
}
static void multi_edge_and_wrap(void)
{
  startup_calibration_t c = startup_nominal(0);
  for (int sign = -1; sign <= 1; sign += 2) {
    memset(rows, 0, sizeof rows);
    for (int i = 1; i < 384; ++i) {
      rows[i].ref_rpm = sign * 3000;
      rows[i].edges_mod16 = (rows[i-1].edges_mod16 + sign * 2 + 16) % 16;
    }
    startup_result_t r = startup_evaluate(rows, 384, &c);
    CHECK_EQ(r.verdict, STARTUP_PASS);
    for (int k = 0; k < 8; ++k) {
      CHECK_NEAR(r.transitions[k].time_ms, k / 2 + 1, 1e-12);
      CHECK_EQ(r.transitions[k].direction, sign);
    }
  }
  rows[1].edges_mod16 = 8; /* -8 is the documented ambiguous half-wrap. */
  CHECK_EQ(startup_evaluate(rows, 384, &c).rule, STARTUP_RULE_DIRECTION);
}
static void overshoot(void)
{
  startup_calibration_t c = startup_nominal(0);
  /* A 25 ms burst with 130 or 131 intervals gives exactly 6500 or 6550 RPM.
   * Integer reference 5000 avoids rounding a target to recorder RPM units. */
  for (int sign = -1; sign <= 1; sign += 2) {
    for (int extra = 0; extra <= 1; ++extra) {
      memset(rows, 0, sizeof rows);
      for (int i = 1; i < 384; ++i) {
        rows[i].ref_rpm = sign * 5000;
        int edges = i == 1 ? 1 : i <= 6 + extra ? 6 : i <= 26 ? 5 : 0;
        rows[i].edges_mod16 = (rows[i-1].edges_mod16 + sign * edges + 16) % 16;
      }
      startup_result_t r = startup_evaluate(rows, 384, &c);
      CHECK_NEAR(r.max_speed_rpm, 6500 + 50 * extra, 1e-10);
      CHECK_NEAR(r.steady_rpm, 5000, 1e-12);
      CHECK_NEAR(r.overshoot_ratio, 1.3 + 0.01 * extra, 1e-12);
      CHECK_EQ(r.verdict, extra ? STARTUP_FAIL : STARTUP_PASS);
      CHECK_EQ(r.rule, extra ? STARTUP_RULE_OVERSHOOT : STARTUP_RULE_NONE);
    }
    memset(rows, 0, sizeof rows);
    for (int i = 1; i < 384; ++i) {
      rows[i].ref_rpm = sign * 100;
      rows[i].edges_mod16 = ((sign * (i / 10)) % 16 + 16) % 16;
    }
    CHECK_EQ(startup_evaluate(rows, 210, &c).overshoot, STARTUP_NOT_MEASURABLE);
    CHECK_EQ(startup_evaluate(rows, 211, &c).overshoot, STARTUP_PASS);
  }
  /* No same-direction run spans 25 ms, despite a long recording. */
  for (int i = 1; i < 384; ++i) rows[i].edges_mod16 = (i / 10) % 2;
  CHECK_EQ(startup_evaluate(rows, 384, &c).overshoot, STARTUP_NOT_MEASURABLE);
}
static void plateaus(void)
{
  startup_calibration_t c = startup_nominal(0);
  const int signs[4] = {1, -1, -1, 1};
  for (int w = 0; w < 4; ++w) {
    /* Fixed 10 ms edge intervals give 125 RPM, independent of the reference. */
    memset(rows, 0, sizeof rows);
    for (int i = 1; i < 384; ++i) {
      rows[i].ref_rpm = signs[w] * wheel_plateaus[w];
      rows[i].edges_mod16 = (signs[w] * (i / 10) % 16 + 16) % 16;
    }
    startup_result_t r = nominal();
    CHECK_EQ(r.verdict, STARTUP_PASS);
    CHECK_NEAR(r.steady_rpm, wheel_plateaus[w], 1e-12);
    CHECK_NEAR(r.overshoot_ratio, 125.0 / wheel_plateaus[w], 1e-12);
    /* Both inclusive +/-1 boundaries, including the oldest of the 50 rows. */
    rows[334].ref_rpm += signs[w];
    rows[382].ref_rpm -= signs[w];
    CHECK_EQ(nominal().verdict, STARTUP_PASS);
    rows[334].ref_rpm += signs[w];
    r = nominal();
    CHECK_EQ(r.verdict, STARTUP_NOT_MEASURABLE);
    CHECK_EQ(r.overshoot, STARTUP_NOT_MEASURABLE);
    CHECK_EQ(r.rule, STARTUP_RULE_NO_PLATEAU);
    CHECK(isnan(r.steady_rpm)); CHECK(isnan(r.overshoot_ratio));
    rows[334].ref_rpm = signs[w] * wheel_plateaus[w];
    rows[333].ref_rpm += 2 * signs[w];
    CHECK_EQ(nominal().verdict, STARTUP_PASS); /* outside the last 50 rows */
    for (int i = 334; i < 384; ++i)
      rows[i].ref_rpm = signs[w] * (wheel_plateaus[w] + i - 334);
    r = nominal();
    CHECK_EQ(r.tracking, STARTUP_PASS);
    CHECK_EQ(r.verdict, STARTUP_NOT_MEASURABLE);
    CHECK_EQ(r.overshoot, STARTUP_NOT_MEASURABLE);
    CHECK_EQ(r.rule, STARTUP_RULE_NO_PLATEAU);
    for (int i = 334; i < 384; ++i) rows[i].ref_rpm = signs[w];
    CHECK_EQ(nominal().rule, STARTUP_RULE_NO_PLATEAU);
    for (int i = 334; i < 384; ++i) rows[i].ref_rpm = signs[w] * 2;
    CHECK_NEAR(nominal().steady_rpm, 2, 1e-12);
    /* Enough early edges for tracking, but fewer than 50 plateau samples. */
    for (int i = 1; i < 50; ++i) {
      rows[i].ref_rpm = signs[w] * 3000;
      rows[i].edges_mod16 = (rows[i-1].edges_mod16 + signs[w] * 2 + 16) % 16;
    }
    r = startup_evaluate(rows, 49, &c);
    CHECK_EQ(r.tracking, STARTUP_PASS);
    CHECK_EQ(r.rule, STARTUP_RULE_NO_PLATEAU);
    CHECK(isnan(r.steady_rpm));
  }
}
static void calibrated(void)
{
  startup_calibration_t c = {{5, 10, 6, 9, 7, 8}, 0, 0};
  for (int s = 0; s < 6; ++s) for (int sign = -1; sign <= 1; sign += 2) {
    c.starting_sector = s;
    trace(sign, 0, 0, &c);
    startup_result_t r = startup_evaluate(rows, 384, &c);
    CHECK_EQ(r.verdict, STARTUP_PASS);
    CHECK_NEAR(r.transitions[0].distance_deg, c.widths_deg[s], 1e-12);
    CHECK_NEAR(r.transitions[1].distance_deg, c.widths_deg[s] + c.widths_deg[(s+sign+6)%6], 1e-12);
    double tolerance = r.transitions[2].tolerance_ms;
    c.spread_deg = 1;
    r = startup_evaluate(rows, 384, &c);
    CHECK(r.transitions[2].tolerance_ms > tolerance);
    c.spread_deg = 0;
  }
  for (int sign = -1; sign <= 1; sign += 2) {
    c.starting_sector = 0;
    memset(rows, 0, sizeof rows);
    int next_ms = 1, sector = 0, counter = 0;
    for (int t = 1; t < 384; ++t) {
      rows[t].ref_rpm = sign * 100;
      if (t == next_ms) {
        counter = (counter + sign + 16) % 16;
        sector = (sector + sign + 6) % 6;
        next_ms += (int)(c.widths_deg[sector] * 4);
      }
      rows[t].edges_mod16 = counter;
    }
    startup_result_t r = startup_evaluate(rows, 384, &c);
    CHECK_EQ(r.overshoot, STARTUP_PASS);
    CHECK_NEAR(r.max_speed_rpm, 1.0 / 4 / 0.006, 1e-12);
  }
  c.starting_sector = -1;
  CHECK_EQ(startup_evaluate(rows, 384, &c).rule, STARTUP_RULE_SECTOR);
  c = (startup_calibration_t){{10,10,10,10,10,10},0,0};
  CHECK_EQ(startup_evaluate(rows, 384, &c).rule, STARTUP_RULE_INPUT);
}
static void hall_states(void)
{
  for (int h = 0; h < 8; ++h) {
    int k = -1;
    for (int s = 0; s < 6; ++s) if (HALL_ORDER[s] == h) k = s;
    CHECK_EQ(startup_hall_sector(h), k); /* 0 and 7 are invalid */
  }
  CHECK_EQ(startup_hall_sector(-1), -1);
  CHECK_EQ(startup_hall_sector(8), -1);
  trace_hall = true;
  const startup_calibration_t map = {{5, 10, 6, 9, 7, 8}, 0, -1};
  for (int s = 0; s < 6; ++s) for (int sign = -1; sign <= 1; sign += 2) {
    startup_calibration_t c = map;
    c.starting_sector = s; /* the trace's true start; the evaluation only sees the Hall state */
    trace(sign, 0, 0, &c);
    CHECK_EQ(rows[0].hall_state, HALL_ORDER[s]);
    startup_result_t r = startup_evaluate(rows, 384, &map);
    CHECK_EQ(r.verdict, STARTUP_PASS);
    CHECK_EQ(r.sign, sign);
    CHECK_NEAR(r.transitions[0].distance_deg, map.widths_deg[s], 1e-12);
    CHECK_NEAR(r.transitions[1].distance_deg, map.widths_deg[s] + map.widths_deg[(s+sign+6)%6], 1e-12);
    CHECK_EQ(startup_evaluate(rows, 384, &c).verdict, STARTUP_PASS); /* agreeing given sector */
    c.starting_sector = (s + 1) % 6;                                 /* disagreeing one */
    CHECK_EQ(startup_evaluate(rows, 384, &c).rule, STARTUP_RULE_HALL_SEQUENCE);
    startup_calibration_t nominal_cal = startup_nominal(0);
    CHECK_EQ(startup_evaluate(rows, 384, &nominal_cal).verdict, STARTUP_PASS);
    size_t first = 1;
    while (rows[first].edges_mod16 == rows[first - 1].edges_mod16) ++first;
    int saved = rows[first].hall_state;
    rows[first].hall_state = HALL_ORDER[(s - sign + 6) % 6]; /* edge counted one way, Hall went the other */
    r = startup_evaluate(rows, 384, &map);
    CHECK_EQ(r.verdict, STARTUP_NOT_MEASURABLE);
    CHECK_EQ(r.rule, STARTUP_RULE_HALL_SEQUENCE);
    CHECK(!strcmp(startup_rule_name(r.rule), "hall_sequence"));
    CHECK_EQ(startup_evaluate(rows, 384, &nominal_cal).rule, STARTUP_RULE_HALL_SEQUENCE);
    rows[first].hall_state = saved;
    saved = rows[200].hall_state;
    rows[200].hall_state = 7;                                /* invalid state mid-run */
    CHECK_EQ(startup_evaluate(rows, 384, &map).rule, STARTUP_RULE_HALL_SEQUENCE);
    rows[200].hall_state = saved;
    CHECK_EQ(startup_evaluate(rows, 384, &map).verdict, STARTUP_PASS);
    rows[201].edges_mod16 = (rows[201].edges_mod16 + 1) % 16; /* edge without a Hall change */
    CHECK_EQ(startup_evaluate(rows, 384, &map).rule, STARTUP_RULE_HALL_SEQUENCE);
  }
  trace_hall = false;
  trace(1, 0, 0, &map);                                      /* no Hall state recorded */
  CHECK_EQ(startup_evaluate(rows, 384, &map).rule, STARTUP_RULE_SECTOR);
}
static void synchronization(void)
{
  double times[4] = {10, 10.001, 10.002, 10.0005};
  CHECK_EQ(startup_synchronization(times, 4).verdict, STARTUP_PASS);
  CHECK_NEAR(startup_synchronization(times, 4).skew_ms, 2, 1e-9);
  times[3] = 10.00201;
  CHECK_EQ(startup_synchronization(times, 4).verdict, STARTUP_FAIL);
  for (int i = 0; i < 4; ++i) {
    times[i] = NAN;
    CHECK_EQ(startup_synchronization(times, 4).verdict, STARTUP_NOT_MEASURABLE);
    times[i] = 10;
  }
  CHECK_EQ(startup_synchronization(times, 1).verdict, STARTUP_PASS);
  CHECK_EQ(startup_synchronization(times, 0).verdict, STARTUP_NOT_MEASURABLE);
  CHECK_EQ(startup_synchronization(times, 5).verdict, STARTUP_NOT_MEASURABLE);
}
static void report_file(const char *path, const char *verdict, const char *rule,
                        int k, const char *ratio, int wheel_count)
{
  FILE *f = fopen(path, "w");
  CHECK(f != NULL);
  const char *wheel[] = {"FR", "BR", "BL", "FL"};
  for (int i = 0; i < wheel_count; ++i)
    fprintf(f, "test-run\tvx\t+\t%s\t%s\t%d\t%d\t%d\t%s\t%s\t%s\t1800\n", wheel[i], verdict,
            k, !strcmp(verdict, "INACTIVE") ? 0 : 8, !strcmp(verdict, "INACTIVE") ? 0 : 5,
            ratio, !strcmp(verdict, "INACTIVE") || !strcmp(rule, "no_plateau") ? "na" : "120", rule);
  CHECK_EQ(fclose(f), 0);
}
static void report(void)
{
  char dir[] = "/tmp/startup-report-XXXXXX", path[256];
  CHECK(mkdtemp(dir) != NULL);
  snprintf(path, sizeof path, "%s/startup.tsv", dir);
  char *args[] = {"startup_report", dir, dir};
  FILE *capture = tmpfile();
  CHECK(capture != NULL);
  fflush(stdout);
  int saved = dup(STDOUT_FILENO);
  CHECK(saved >= 0);
  CHECK(dup2(fileno(capture), STDOUT_FILENO) >= 0);
  report_file(path, "PASS", "none", 0, "1.3", 4);
  CHECK_EQ(startup_report_main(2, args), 0);
  CHECK_EQ(startup_report_main(3, args), 1); /* duplicate input is not another start */
  report_file(path, "PASS", "none", 0, "1.3", 3);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "FAIL", "deadline", 2, "1", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "FAIL", "wrong_direction", 1, "na", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "FAIL", "overshoot", 0, "1.31", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "NOT_MEASURABLE", "short_record", 0, "na", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "NOT_MEASURABLE", "no_plateau", 0, "na", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "INACTIVE", "none", 0, "na", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "PASS", "none", 0, "na", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "PASS", "none", 0, "1.31", 4);
  CHECK_EQ(startup_report_main(2, args), 1);
  report_file(path, "PASS", "none", 0, "1", 0);
  CHECK_EQ(startup_report_main(2, args), 1);
  const char *invalid_steady[] = {"na", "nan", "inf", "1", "-120", "120rpm", ""};
  for (size_t i = 0; i < sizeof invalid_steady / sizeof *invalid_steady; ++i) {
    FILE *f = fopen(path, "w");
    CHECK(f != NULL);
    for (int w = 0; w < 4; ++w)
      fprintf(f, "test-run\tvx\t+\t%s\tPASS\t0\t8\t5\t1\t%s\tnone\n",
              IK_WHEEL_POS[w], invalid_steady[i]);
    CHECK_EQ(fclose(f), 0);
    CHECK_EQ(startup_report_main(2, args), 1);
  }
  FILE *bad = fopen(path, "w");
  CHECK(bad != NULL);
  fputs("broken\trow\n", bad);
  CHECK_EQ(fclose(bad), 0);
  CHECK_EQ(startup_report_main(2, args), 1);
  CHECK_EQ(unlink(path), 0);
  CHECK_EQ(startup_report_main(2, args), 1);
  CHECK_EQ(rmdir(dir), 0);
  fflush(stdout);
  CHECK_EQ(fseek(capture, 0, SEEK_SET), 0);
  char output[32768];
  size_t n = fread(output, 1, sizeof output - 1, capture);
  output[n] = 0;
  CHECK(strstr(output, "deadline\t2\t4") != NULL);
  CHECK(strstr(output, "wrong_direction\t1\t4") != NULL);
  CHECK(strstr(output, "overshoot\t0\t4") != NULL);
  CHECK(dup2(saved, STDOUT_FILENO) >= 0);
  close(saved);
  fclose(capture);
}
static void move_integration(void)
{
  char dir[] = "/tmp/startup-move-XXXXXX", path[256];
  CHECK(mkdtemp(dir) != NULL);
  char *args[] = {"move_test", "--startup-metrics", "--out", dir,
                  "--record", "--first-back"};
  FILE *capture = tmpfile();
  CHECK(capture != NULL);
  fflush(stdout);
  int saved = dup(STDOUT_FILENO);
  CHECK(saved >= 0);
  CHECK(dup2(fileno(capture), STDOUT_FILENO) >= 0);
  CHECK_EQ(startup_move_test_main(4, args), 1); /* missing record */
  char *removed[] = {"move_test", "--steady-rpm", "120"};
  CHECK_EQ(startup_move_test_main(3, removed), 1);
  CHECK_EQ(opens, 0);
  startup_calibration_t cal = startup_nominal(0);
  trace(1, 0, 0, &cal);
  CHECK_EQ(startup_move_test_main(5, args), 0);
  CHECK_EQ(startup_move_test_main(6, args), 0);
  char *report_args[] = {"startup_report", dir};
  CHECK_EQ(startup_report_main(2, report_args), 0);
  snprintf(path, sizeof path, "%s/startup.tsv", dir);
  FILE *tsv = fopen(path, "r");
  CHECK(tsv != NULL);
  char line[1024];
  CHECK(fgets(line, sizeof line, tsv) != NULL);
  CHECK(strstr(line, "\tsteady_rpm\tfailed_rule\tapply_skew_us\n") != NULL);
  for (int run = 0; run < 2; ++run) for (int w = 0; w < 4; ++w) {
    CHECK(fgets(line, sizeof line, tsv) != NULL);
    char *field = strtok(line, "\t");
    for (int col = 0; col < 9; ++col) {
      CHECK(field != NULL);
      field = strtok(NULL, "\t");
    }
    CHECK(field != NULL);
    CHECK_NEAR(strtod(field, NULL), wheel_plateaus[w], 1e-12);
  }
  CHECK_EQ(fclose(tsv), 0);
  snprintf(path, sizeof path, "%s/result.json", dir);
  FILE *f = fopen(path, "r");
  CHECK(f != NULL);
  char json[32768];
  size_t count = fread(json, 1, sizeof json - 1, f);
  json[count] = 0;
  fclose(f);
  CHECK(strstr(json, "\"startup\":{\"run_id\":") != NULL);
  CHECK(strstr(json, "\"sign\":\"-\"") != NULL);
  for (int w = 0; w < 4; ++w) {
    char expected[128];
    snprintf(expected, sizeof expected, "\"%s\":{\"verdict\":\"PASS\",\"tracking\":\"PASS\",\"overshoot\":\"PASS\"", IK_WHEEL_POS[w]);
    CHECK(strstr(json, expected) != NULL);
    const char *wheel_json = strstr(json, expected);
    snprintf(expected, sizeof expected, "\"steady_rpm\":%d,", wheel_plateaus[w]);
    const char *steady = strstr(wheel_json, expected);
    CHECK(steady != NULL && steady < strchr(wheel_json, '}'));
    CHECK_EQ(arms[w], 2); CHECK_EQ(reads[w], 2);
  }
  for (int w = 0; w < 4; ++w) {
    missing_wheel = w;
    CHECK_EQ(startup_move_test_main(5, args), 1);
    CHECK_EQ(startup_report_main(2, report_args), 1);
  }
  missing_wheel = -1;
  CHECK_EQ(unlink(path), 0);
  snprintf(path, sizeof path, "%s/startup.tsv", dir);
  CHECK_EQ(unlink(path), 0);
  CHECK_EQ(rmdir(dir), 0);
  fflush(stdout);
  CHECK(dup2(saved, STDOUT_FILENO) >= 0);
  close(saved); fclose(capture);
}
int main(void)
{
  tracking(); interpolation_and_incomplete(); multi_edge_and_wrap();
  overshoot(); plateaus(); calibrated(); hall_states(); synchronization(); report(); move_integration();
  puts("startup metrics: all cases passed");
  return 0;
}

int __wrap_ik_link_rows(ik_link_t *link, ik_link_row_t links[4])
{
  (void)link;
  for (int w = 0; w < 4; w++) links[w] = (ik_link_row_t){.valid = true,
    .first_start_valid = true, .first_start_reply_us = UINT32_MAX - 500U + (uint32_t)w * 600U};
  return 0;
}
