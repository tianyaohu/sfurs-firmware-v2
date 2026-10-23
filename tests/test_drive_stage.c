#define _POSIX_C_SOURCE 200809L
#include "check.h"
#include "drive_ops.h"
#include "elfsym.h"
#include "esc_defaults.h"
#include <limits.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>

static void live_rows(ik_row_t before[4], ik_row_t rows[4])
{
  for (int w = 0; w < 4; w++) {
    before[w] = (ik_row_t){.valid = true, .status_count = 10};
    rows[w] = (ik_row_t){.valid = true, .status_count = 11, .mc_state = "IDLE", .vbus_v = 12};
  }
}

static void test_preflight_live_idle_vbus_and_faults(void)
{
  ik_row_t before[4], rows[4];
  char problems[2048];
  live_rows(before, rows);
  CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 0);
  for (int w = 0; w < 4; w++) {
    live_rows(before, rows); rows[w].status_count = before[w].status_count;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    CHECK(strstr(problems, IK_WHEEL_POS[w])); CHECK(strstr(problems, "no live reply"));
    live_rows(before, rows); rows[w].valid = false;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    live_rows(before, rows); before[w].valid = false;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    live_rows(before, rows); rows[w].mc_state = "RUN";
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    CHECK(strstr(problems, "RUN faults -"));
    live_rows(before, rows); strcpy(rows[w].fault_names, "overvolt");
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    CHECK(strstr(problems, "overvolt"));
    live_rows(before, rows); rows[w].vbus_v = 10.99;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    CHECK(strstr(problems, "Vbus 10.99"));
    live_rows(before, rows); rows[w].vbus_v = 11;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 0);
    live_rows(before, rows); rows[w].vbus_v = NAN;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 1);
    live_rows(before, rows); before[w].status_count = UINT32_MAX; rows[w].status_count = 0;
    CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 0);
  }
  for (int w = 0; w < 4; w++) rows[w].valid = false;
  CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, sizeof problems), 4);
  CHECK_EQ(drive_preflight_rows(before, rows, 11, problems, 2), 4);
  CHECK_EQ(drive_preflight_rows(before, rows, 11, NULL, 0), 4);
}

static void test_defaults_and_settings_all_wheels(void)
{
  drive_settings_t settings;
  CHECK_EQ(drive_settings_defaults(&settings), 0);
  const char *const want[] = {"kp", "ki", "stall_ramp_ma_per_s", "stall_ms"};
  CHECK_EQ(DRIVE_KEYS, 4);
  for (int k = 0; k < DRIVE_KEYS; k++) {
    int def;
    CHECK(!strcmp(DRIVE_SETTING_KEYS[k], want[k]));
    CHECK_EQ(esc_default(DRIVE_SETTING_KEYS[k], &def), 0);
    for (int w = 0; w < 4; w++) CHECK_EQ(settings.value[w][k], def);
  }
  CHECK_EQ(drive_setting_set(&settings, "stall_ramp_ma_per_s", 12000), 0);
  for (int w = 0; w < 4; w++) CHECK_EQ(settings.value[w][2], 12000);
  CHECK(drive_setting_set(&settings, "ff_static_ma", 1234) != 0); /* retired */
  CHECK(drive_setting_set(&settings, "hall_shift_deg", 10) != 0); /* valid CONFIG key, not a drive setting */
  CHECK(drive_setting_set(&settings, "not_a_setting", 1) != 0);
  CHECK(drive_setting_set(&settings, "kp", 32768) != 0);
  CHECK_EQ(drive_setting_set(&settings, "kp", -32768), 0);
  for (int w = 0; w < 4; w++) CHECK_EQ(settings.value[w][0], -32768);
}

static drive_stage_t sample_stage(void)
{
  drive_stage_t s = {0};
  s.pulse.n = 6;
  s.pulse.t0_monotonic_s = 100.25;
  s.pulse.log = calloc(s.pulse.n, sizeof *s.pulse.log); CHECK(s.pulse.log);
  for (size_t k = 0; k < s.pulse.n; k++) {
    pulse_entry_t *p = &s.pulse.log[k];
    p->e.t = k * 0.1;
    for (int w = 0; w < 4; w++) {
      p->e.target_rpm[w] = k >= 1 && k < 5 ? 50 : 0;
      p->e.rpm[w] = k >= 2 && k < 5 ? 50 + w : 0;
      p->e.rpm_ok[w] = p->e.iq_ok[w] = true;
      p->e.iq_ref_a[w] = w * 0.1 + 0.4;
      p->mc_state[w] = "IDLE";
      p->vbus_v[w] = 12 - w * 0.1;
    }
  }
  strcpy(s.pulse.log[0].e.faults[0], "overvolt overcurrent");
  strcpy(s.pulse.log[1].e.faults[3], "overcurrent undervolt");
  CHECK_EQ(drive_stage_analyze(&s), 0);
  return s;
}

static void test_stage_conversion_and_telemetry_response(void)
{
  drive_stage_t s = sample_stage();
  CHECK_NEAR(s.vbus_min_v, 11.7, 1e-9);
  CHECK(!strcmp(s.faults, "overcurrent overvolt undervolt"));
  for (int w = 0; w < 4; w++) {
    CHECK_EQ(s.response.wheels[w].starts, 1); CHECK_EQ(s.response.wheels[w].moved, 1);
    CHECK_NEAR(s.response.wheels[w].breakaway_s, 0.1, 1e-9);
    CHECK_NEAR(s.response.wheels[w].overshoot, (50 + w) / 50.0, 1e-9);
    CHECK_NEAR(s.analysis[2].iq_ref_a[w], 0.4 + w * 0.1, 1e-9);
    CHECK(s.analysis[2].rpm_valid[w]); CHECK(s.analysis[2].iq_ref_valid[w]);
  }
  s.pulse.log[2].e.rpm_ok[2] = false;
  s.pulse.log[2].e.iq_ok[1] = false;
  CHECK_EQ(drive_stage_analyze(&s), 0);
  CHECK(!s.analysis[2].rpm_valid[2]); CHECK(!s.analysis[2].iq_ref_valid[1]);
  drive_stage_free(&s);
  CHECK_EQ(s.pulse.n, 0); CHECK(!s.analysis);
}

static void test_stage_json_plan_windows_and_nulls(void)
{
  drive_stage_t s = sample_stage();
  drive_settings_t settings;
  CHECK_EQ(drive_settings_defaults(&settings), 0);
  esc_move_t plan[] = {{0.5, 1.5, {0, 200, 0}}, {3, 4, {0, -200, 0}}};
  FILE *f = tmpfile(); CHECK(f);
  drive_stage_json(f, "identify", &s, &settings, plan, 2);
  rewind(f); char text[16384] = "";
  CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  CHECK(strstr(text, "\"t0_monotonic_s\":100.250000000"));
  CHECK(strstr(text, "\"start_s\":0.500000000,\"end_s\":1.500000000"));
  CHECK(strstr(text, "\"start_monotonic_s\":100.750000000,\"end_monotonic_s\":101.750000000"));
  CHECK(strstr(text, "\"start_monotonic_s\":103.250000000,\"end_monotonic_s\":104.250000000"));
  CHECK(strstr(text, "\"settings\":[{")); CHECK(strstr(text, "\"aborted\":null"));
  drive_stage_free(&s);
  CHECK_EQ(drive_stage_analyze(&s), 0);
  CHECK(isnan(s.vbus_min_v));
  f = tmpfile(); CHECK(f); drive_stage_json(f, "drive", &s, &settings, plan, 2);
  rewind(f); memset(text, 0, sizeof text); CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  CHECK(strstr(text, "\"settings\":{\"kp\":1024,\"ki\":553,\"stall_ramp_ma_per_s\":10000,\"stall_ms\":500,"
                    "\"ff\":{\"enable\":false,\"coulomb_x_n\":0.000,\"coulomb_y_n\":0.000,"
                    "\"viscous_xy_n_per_mps\":0.000,\"coulomb_w_nm\":0.000,\"viscous_w_nm_per_radps\":0.000,"
                    "\"breakaway_x_n\":0.000,\"breakaway_y_n\":0.000,\"breakaway_w_nm\":0.000,\"breakaway_ms\":0,"
                    "\"a_per_nm\":0.000,\"scale_pct\":[100,100,100,100]}}"));
  CHECK(strstr(text, "\"vbus_min\":null"));
  CHECK(strstr(text, "\"breakaway_s\":null")); CHECK(!strstr(text, "nan"));
  drive_stage_free(&s);
}

static void test_log_monotonic_origin_every_row(void)
{
  drive_stage_t s = sample_stage();
  char path[] = "/tmp/drive-log-XXXXXX";
  int fd = mkstemp(path); CHECK(fd >= 0); close(fd);
  CHECK_EQ(drive_log_write(path, &s.pulse), 0);
  FILE *f = fopen(path, "r"); CHECK(f);
  char text[8192];
  for (size_t k = 0; k < s.pulse.n; k++) {
    CHECK(fgets(text, sizeof text, f));
    char want[128];
    CHECK(strstr(text, "\"t0_monotonic_s\":100.250000000"));
    snprintf(want, sizeof want, "\"t_monotonic_s\":%.9f", 100.25 + k * 0.1);
    CHECK(strstr(text, want)); CHECK(strstr(text, "\"target_rpm\":[")); CHECK(strstr(text, "\"faults\":["));
  }
  CHECK(!fgets(text, sizeof text, f)); fclose(f); unlink(path);
  CHECK(drive_log_write("/nonexistent-directory/log.jsonl", &s.pulse) != 0);
  drive_stage_free(&s);
}

static void test_cli_parser_and_args(void)
{
  const char *s = NULL;
  int i = INT_MIN; double d = 0.2; bool b = false;
  drive_option options[] = {DRIVE_OPT("file-name", STRING, &s), DRIVE_OPT("int", INT, &i),
                            DRIVE_OPT("double", DOUBLE, &d), DRIVE_OPT("bool", BOOL, &b)};
  char *args[] = {"tool", "--file-name", "a\"b", "--int=-2", "--double", "0.75", "--bool"};
  CHECK_EQ(drive_parse(7, args, options, 4), 0);
  CHECK(!strcmp(s, "a\"b")); CHECK_EQ(i, -2); CHECK_NEAR(d, 0.75, 1e-9); CHECK(b);
  FILE *f = tmpfile(); CHECK(f); drive_args_json(f, options, 4); rewind(f);
  char text[4096] = ""; CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  CHECK(strstr(text, "\"file_name\":\"a\\\"b\"")); CHECK(strstr(text, "\"bool\":\"True\""));
  const char *bad[][2] = {{"--int", "1x"}, {"--int", "2147483648"}, {"--double", "nan"},
                          {"--double", "inf"}, {"--double", "0.3bad"}, {"--no-such", "1"}, {"--bool=1", "x"}};
  for (size_t k = 0; k < sizeof bad / sizeof *bad; k++) {
    char *a[] = {"tool", (char *)bad[k][0], (char *)bad[k][1]};
    CHECK(drive_parse(3, a, options, 4) < 0);
  }
  char *missing[] = {"tool", "--int"}; CHECK(drive_parse(2, missing, options, 4) < 0);
  char *help[] = {"tool", "--help"}; CHECK_EQ(drive_parse(2, help, options, 4), 1);
}

/* Parse argv through a tool-style table holding DRIVE_FF_OPTIONS. */
static int ff_parse(char **argv, int argc, drive_ff_t *ff, char *err)
{
  drive_ff_args_t args = {0};
  const char *other = NULL;
  drive_option options[] = {DRIVE_OPT("out", STRING, &other), DRIVE_FF_OPTIONS(&args)};
  size_t n = sizeof options / sizeof *options;
  if (drive_parse(argc, argv, options, n)) return -2;
  return drive_ff_from_options(options, n, ff, err, 160);
}

static void test_ff_options_convert_si_to_mailbox_words(void)
{
  drive_ff_t ff; char err[160];
  char *none[] = {"tool", "--out", "x"};
  CHECK_EQ(ff_parse(none, 3, &ff, err), 0);
  CHECK(!ff.enable); CHECK_EQ(ff.coulomb_x_mn, 0); CHECK_EQ(ff.coulomb_y_mn, 0); CHECK_EQ(ff.a_per_nm_milli, 0);
  CHECK_EQ(ff.breakaway_ms, 0);
  for (int w = 0; w < 4; w++) CHECK_EQ(ff.scale_pct[w], DRIVE_FF_SCALE_NOMINAL_PCT);
  char *all[] = {"tool", "--ff-coulomb-x-n", "6.5", "--ff-coulomb-y-n", "4.25", "--ff-viscous-xy-n-per-mps=1.5",
                 "--ff-coulomb-w-nm", "0.3", "--ff-viscous-w-nm-per-radps", "0.0126", "--ff-breakaway-x-n", "20",
                 "--ff-breakaway-y-n", "12.5", "--ff-breakaway-w-nm", "0.9", "--ff-breakaway-ms", "250",
                 "--ff-a-per-nm", "34.8", "--ff-scale", "90,110,100,200"};
  CHECK_EQ(ff_parse(all, (int)(sizeof all / sizeof *all), &ff, err), 0);
  CHECK(ff.enable);
  CHECK_EQ(ff.coulomb_x_mn, 6500); CHECK_EQ(ff.coulomb_y_mn, 4250); CHECK_EQ(ff.viscous_xy_mn_per_mps, 1500);
  CHECK_EQ(ff.breakaway_x_mn, 20000); CHECK_EQ(ff.breakaway_y_mn, 12500); CHECK_EQ(ff.breakaway_w_mnm, 900);
  CHECK_EQ(ff.breakaway_ms, 250);
  CHECK_EQ(ff.coulomb_w_mnm, 300); CHECK_EQ(ff.viscous_w_mnm_per_radps, 13); /* nearest mN*m per rad/s */
  CHECK_EQ(ff.a_per_nm_milli, 34800);
  const int32_t scale[] = {90, 110, 100, 200};
  for (int w = 0; w < 4; w++) CHECK_EQ(ff.scale_pct[w], scale[w]);
  /* One term enables the model; trims stay nominal. */
  char *one[] = {"tool", "--ff-coulomb-y-n", "3", "--ff-a-per-nm", "35"};
  CHECK_EQ(ff_parse(one, 5, &ff, err), 0);
  CHECK(ff.enable); CHECK_EQ(ff.coulomb_y_mn, 3000); CHECK_EQ(ff.coulomb_x_mn, 0); CHECK_EQ(ff.viscous_xy_mn_per_mps, 0);
  CHECK_EQ(ff.breakaway_ms, 0); /* no breakaway level: no boost time */
  for (int w = 0; w < 4; w++) CHECK_EQ(ff.scale_pct[w], 100);
  FILE *f = tmpfile(); CHECK(f); drive_ff_json(f, &ff); rewind(f);
  char text[512] = ""; CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  CHECK(strstr(text, "\"enable\":true,\"coulomb_x_n\":0.000,\"coulomb_y_n\":3.000"));
  CHECK(strstr(text, "\"a_per_nm\":35.000"));
  char *level[] = {"tool", "--ff-breakaway-x-n", "20", "--ff-a-per-nm", "35"}; /* a level alone: default time */
  CHECK_EQ(ff_parse(level, 5, &ff, err), 0);
  CHECK_EQ(ff.breakaway_x_mn, 20000); CHECK_EQ(ff.breakaway_ms, DRIVE_FF_BREAKAWAY_MS_DEFAULT);
  const char *bad[][4] = {
    {"--ff-coulomb-x-n", "3", NULL, NULL},                       /* no 1/Kt */
    {"--ff-a-per-nm", "0", NULL, NULL},
    {"--ff-a-per-nm", "0.0004", NULL, NULL},                      /* rounds to 0 mA per N*m */
    {"--ff-coulomb-x-n", "-1", "--ff-a-per-nm", "35"},
    {"--ff-coulomb-y-n", "-1", "--ff-a-per-nm", "35"},
    {"--ff-breakaway-x-n", "-1", "--ff-a-per-nm", "35"},
    {"--ff-breakaway-y-n", "-1", "--ff-a-per-nm", "35"},
    {"--ff-breakaway-w-nm", "-1", "--ff-a-per-nm", "35"},
    {"--ff-breakaway-ms", "0", "--ff-a-per-nm", "35"},
    {"--ff-breakaway-ms", "2001", "--ff-a-per-nm", "35"},
    {"--ff-viscous-xy-n-per-mps", "-0.1", "--ff-a-per-nm", "35"},
    {"--ff-coulomb-w-nm", "-0.2", "--ff-a-per-nm", "35"},
    {"--ff-viscous-w-nm-per-radps", "-1", "--ff-a-per-nm", "35"},
    {"--ff-coulomb-y-n", "3e6", "--ff-a-per-nm", "35"},           /* int32 mN overflow */
    {"--ff-scale", "100,100,100", "--ff-a-per-nm", "35"},
    {"--ff-scale", "100,100,100,100,100", "--ff-a-per-nm", "35"},
    {"--ff-scale", "0,100,100,100", "--ff-a-per-nm", "35"},       /* 0 switches the whole model off */
    {"--ff-scale", "100,201,100,100", "--ff-a-per-nm", "35"},
    {"--ff-scale", "100,1.5,100,100", "--ff-a-per-nm", "35"},
    {"--ff-scale", "100,,100,100", "--ff-a-per-nm", "35"}};
  for (size_t k = 0; k < sizeof bad / sizeof *bad; k++) {
    char *argv[] = {"tool", (char *)bad[k][0], (char *)bad[k][1], (char *)bad[k][2], (char *)bad[k][3]};
    err[0] = 0;
    CHECK_EQ(ff_parse(argv, bad[k][2] ? 5 : 3, &ff, err), -1);
    CHECK(err[0]);
  }
  char *nonfinite[] = {"tool", "--ff-coulomb-x-n", "nan", "--ff-a-per-nm", "35"};
  CHECK_EQ(ff_parse(nonfinite, 5, &ff, err), -2); /* drive_parse rejects non-finite numbers */
}

static void test_paths_median_and_json(void)
{
  CHECK_NEAR(drive_median((double[]){3, 1, 2}, 3), 2, 1e-9);
  CHECK_NEAR(drive_median((double[]){4, 1, 2, 3}, 4), 2.5, 1e-9);
  CHECK(isnan(drive_median(NULL, 0)));
  char path[] = "/tmp/drive-dir-XXXXXX"; CHECK(mkdtemp(path));
  char nested[256]; CHECK_EQ(drive_path(nested, sizeof nested, path, "a/b"), 0);
  CHECK_EQ(drive_mkdir(nested), 0); CHECK_EQ(drive_mkdir(nested), 0);
  CHECK(drive_path(nested, 2, path, "long") != 0);
  CHECK_EQ(drive_path(nested, sizeof nested, path, "a/b"), 0); CHECK_EQ(rmdir(nested), 0);
  CHECK_EQ(drive_path(nested, sizeof nested, path, "a"), 0); CHECK_EQ(rmdir(nested), 0); CHECK_EQ(rmdir(path), 0);
  CHECK(drive_mkdir("") != 0); CHECK(drive_mkdir("/etc/hostname/sub") != 0);
}

static void test_stage_refuses_static_replies_without_writes(void)
{
  setenv("OPENOCD", FAKE_OCD, 1);
  uint32_t addresses[IK_SYM_COUNT]; char err[256];
  CHECK_EQ(ik_resolve(FAKE_ELF, addresses, err, sizeof err), 0);
  char mem[256];
  snprintf(mem, sizeof mem, "%x=%x,%x=1234,%x=dead,%x=0", addresses[IK_ABI_SYM], IK_ABI,
           addresses[IK_ENABLE], addresses[IK_RAW_SEQ], addresses[IK_FAULT]);
  setenv("FAKE_OCD_MEM", mem, 1);
  ik_link_t link; CHECK_EQ(ik_link_open(&link, "SN", FAKE_ELF), 0);
  drive_settings_t settings; CHECK_EQ(drive_settings_defaults(&settings), 0);
  drive_stage_t stage;
  esc_move_t plan = {0.5, 1, {0, 200, 0}};
  CHECK(drive_stage_run(&link, "/tmp", "unused", &plan, 1, 2, &settings, 0, &stage) != 0);
  CHECK(strstr(link.err, "preflight"));
  uint32_t value;
  CHECK_EQ(ik_read(&link, IK_ENABLE, &value), 0); CHECK_EQ(value, 0x1234);
  CHECK_EQ(ik_read(&link, IK_RAW_SEQ, &value), 0); CHECK_EQ(value, 0xdead);
  drive_stage_free(&stage); ik_link_close(&link);
  unsetenv("FAKE_OCD_MEM");
}

/* Mailbox words after drive_ff_write/drive_ff_clear, read back through the fake OpenOCD. */
static void ff_words(ik_link_t *l, uint32_t w[15])
{
  const ik_sym_t syms[] = {IK_FF_ENABLE, IK_FF_COULOMB_X_MN, IK_FF_COULOMB_Y_MN, IK_FF_VISCOUS_XY_MN_PER_MPS,
                           IK_FF_COULOMB_W_MNM, IK_FF_VISCOUS_W_MNM_PER_RADPS, IK_FF_BREAKAWAY_X_MN,
                           IK_FF_BREAKAWAY_Y_MN, IK_FF_BREAKAWAY_W_MNM, IK_FF_BREAKAWAY_MS, IK_FF_A_PER_NM_MILLI};
  for (int k = 0; k < 11; k++) CHECK_EQ(ik_read(l, syms[k], &w[k]), 0);
  for (int k = 0; k < 4; k++) CHECK_EQ(ocd_read32(l->ocd, l->addr[IK_FF_SCALE_PCT] + k * 4, &w[11 + k]), 0);
}

static void test_ff_write_only_stopped_and_clear(void)
{
  setenv("OPENOCD", FAKE_OCD, 1);
  uint32_t addresses[IK_SYM_COUNT]; char err[256];
  CHECK_EQ(ik_resolve(FAKE_ELF, addresses, err, sizeof err), 0);
  char mem[256];
  snprintf(mem, sizeof mem, "%x=%x,%x=1,%x=7", addresses[IK_ABI_SYM], IK_ABI, addresses[IK_STATE],
           addresses[IK_FF_COULOMB_X_MN]);
  setenv("FAKE_OCD_MEM", mem, 1);
  ik_link_t link; CHECK_EQ(ik_link_open(&link, "SN", FAKE_ELF), 0);
  drive_ff_t ff = {.enable = true, .coulomb_x_mn = 6500, .coulomb_y_mn = 4250, .viscous_xy_mn_per_mps = 1500,
                   .coulomb_w_mnm = 300, .viscous_w_mnm_per_radps = 13, .breakaway_x_mn = 20000,
                   .breakaway_y_mn = 12500, .breakaway_w_mnm = 900, .breakaway_ms = 250, .a_per_nm_milli = 34800,
                   .scale_pct = {90, 110, 100, 200}};
  uint32_t w[15];
  /* Running: refused, and nothing is touched. */
  CHECK(drive_ff_write(&link, &ff) != 0); CHECK(strstr(link.err, "not stopped"));
  CHECK_EQ(ik_read(&link, IK_FF_COULOMB_X_MN, &w[0]), 0); CHECK_EQ(w[0], 7);
  CHECK_EQ(ik_write(&link, IK_STATE, 0), 0);
  CHECK_EQ(drive_ff_write(&link, &ff), 0);
  ff_words(&link, w);
  const uint32_t on[15] = {1, 6500, 4250, 1500, 300, 13, 20000, 12500, 900, 250, 34800, 90, 110, 100, 200};
  for (int k = 0; k < 15; k++) CHECK_EQ(w[k], on[k]);
  /* Clear works in any state: enable and every term zero, trims back to nominal. */
  CHECK_EQ(ik_write(&link, IK_STATE, 1), 0);
  CHECK_EQ(drive_ff_clear(&link), 0);
  ff_words(&link, w);
  const uint32_t off[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100};
  for (int k = 0; k < 15; k++) CHECK_EQ(w[k], off[k]);
  /* A disabled model writes its words but leaves enable at 0. */
  CHECK_EQ(ik_write(&link, IK_STATE, 0), 0);
  drive_ff_off(&ff); ff.coulomb_x_mn = 5;
  CHECK_EQ(drive_ff_write(&link, &ff), 0);
  ff_words(&link, w); CHECK_EQ(w[0], 0); CHECK_EQ(w[1], 5);
  ik_link_close(&link);
  unsetenv("FAKE_OCD_MEM");
}

int main(int argc, char **argv)
{
  if (argc > 1 && !strcmp(argv[1], "--rpc")) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    bool available = fd >= 0 && bind(fd, (struct sockaddr *)&a, sizeof a) == 0;
    if (fd >= 0) close(fd);
    if (!available) { puts("SKIP: sandbox denies loopback sockets needed by OpenOCD RPC"); return 77; }
    test_stage_refuses_static_replies_without_writes();
    test_ff_write_only_stopped_and_clear();
    puts("drive stage RPC: 2 test cases passed");
    return 0;
  }
  test_preflight_live_idle_vbus_and_faults();
  test_defaults_and_settings_all_wheels();
  test_stage_conversion_and_telemetry_response();
  test_stage_json_plan_windows_and_nulls();
  test_log_monotonic_origin_every_row();
  test_cli_parser_and_args();
  test_paths_median_and_json();
  test_ff_options_convert_si_to_mailbox_words();
  puts("drive stage: 8 test cases passed");
  return 0;
}
