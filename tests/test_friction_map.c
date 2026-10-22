/* Exercise the real CLI/report orchestration with literal acquisition results. No robot model. */
#define _POSIX_C_SOURCE 200809L
#define main friction_map_main
#include "../src/tools/friction_map.c"
#undef main
#include "check.h"
#include <unistd.h>

static unsigned moves, ramps, preflights, stops, closes;
static int acquisition_error, preflight_error, cancel_at;
static double fake_time;
static char directory[] = "/tmp/friction-map-XXXXXX";

int __wrap_ik_link_open(ik_link_t *l, const char *sn, const char *elf)
{ (void)sn; (void)elf; memset(l, 0, sizeof *l); return 0; }
void __wrap_ik_link_close(ik_link_t *l) { (void)l; closes++; }
int __wrap_ik_read(ik_link_t *l, ik_sym_t s, uint32_t *value)
{ (void)l; CHECK_EQ(s, IK_STATE); *value = 0; return 0; }
int __wrap_drive_preflight(ik_link_t *l, double min)
{
  CHECK_EQ(min, 11); preflights++;
  if (preflight_error) { strcpy(l->err, "preflight failed"); return -1; }
  return 0;
}
int __wrap_friction_stop(ik_link_t *l) { (void)l; stops++; return 0; }
int __wrap_ik_set_mode(ik_link_t *l, unsigned mode) { (void)l; CHECK_EQ(mode, IK_MODE_SPEED); return 0; }
double __wrap_pulse_now_s(void) { return fake_time; }
void __wrap_pulse_sleep_s(double s) { fake_time += s; }
int __wrap_friction_speed_move(ik_link_t *l, const int32_t twist[3], double duration,
                               const friction_limits_t *limits, friction_point_t out[4])
{
  static const int reference[] = {-100, 100, -200, 200, 100, -100, 200, -200};
  CHECK(moves < 8); CHECK(duration > 0); CHECK_EQ(limits->min_vbus_v, 11);
  CHECK_EQ(twist[0], 0); CHECK_EQ(twist[2], 0);
  CHECK_EQ(twist[1], reference[moves]);
  for (int w = 0; w < 4; w++) {
    double v = reference[moves];
    out[w] = (friction_point_t){v, (100 + 10 * w) * (v > 0 ? 1 : -1) + 2 * v, 0, 10, FRICTION_STEADY};
  }
  moves++;
  if (cancel_at && moves == (unsigned)cancel_at) raise(SIGINT);
  if (acquisition_error) { strcpy(l->err, "fixture Vbus error"); return -1; }
  return 0;
}
int __wrap_friction_breakaway_run(ik_link_t *l, unsigned selected, int sign, double rate,
                                  double cap, double threshold, const friction_limits_t *limits,
                                  friction_breakaway_t out[4])
{
  (void)l; CHECK_EQ(selected, 15); CHECK_EQ(rate, 2000); CHECK_EQ(cap, 4000);
  CHECK_EQ(threshold, 20); CHECK_EQ(limits->min_vbus_v, 11);
  CHECK_EQ(sign, ramps < 3 ? -1 : 1);
  for (int w = 0; w < 4; w++) out[w] = (friction_breakaway_t){.moved = true,
    .command_ma = sign * (1000.0 + 100 * (ramps % 3)), .measured_ma = sign * 950, .attempted = true};
  ramps++;
  return 0;
}

static void reset_report(void)
{
  moves = ramps = preflights = stops = closes = 0;
  acquisition_error = preflight_error = cancel_at = 0; fake_time = 0;
}

static int invoke(const char *flag)
{
  char *argv[] = {"friction_map", "--out", directory, "--pause", "0", (char *)flag, NULL};
  return friction_map_main(flag ? 6 : 5, argv);
}

static void read_report(char *text, size_t capacity)
{
  char path[512]; CHECK_EQ(drive_path(path, sizeof path, directory, "result.json"), 0);
  FILE *f = fopen(path, "r"); CHECK(f);
  size_t n = fread(text, 1, capacity - 1, f); text[n] = 0; fclose(f);
  CHECK(!strstr(text, ":nan")); CHECK(!strstr(text, ":inf"));
  CHECK_EQ(unlink(path), 0);
}

static void speed_report(void)
{
  reset_report(); CHECK_EQ(invoke(NULL), 0);
  CHECK_EQ(moves, 8); CHECK_EQ(preflights, 9); CHECK_EQ(stops, 1); CHECK_EQ(closes, 1);
  char text[32768]; read_report(text, sizeof text);
  CHECK(strstr(text, "\"complete\":true")); CHECK(strstr(text, "\"sufficient_data\":true"));
  for (int w = 0; w < 4; w++) {
    char want[256];
    snprintf(want, sizeof want, "\"%s\":{\"both\":{\"valid\":true,\"points\":8,\"ic_ma\":%d,\"b_ma_per_rpm\":2,\"residual_rms_ma\":0}", IK_WHEEL_POS[w], 100 + w * 10);
    CHECK(strstr(text, want));
  }
  CHECK(strstr(text, "\"negative\"")); CHECK(strstr(text, "\"positive\""));
  CHECK(strstr(text, "\"samples\":10,\"verdict\":\"steady\""));
}

static void breakaway_report(void)
{
  reset_report(); CHECK_EQ(invoke("--breakaway"), 0);
  CHECK_EQ(ramps, 6); CHECK_EQ(preflights, 7); CHECK_EQ(stops, 1); CHECK_EQ(closes, 1);
  char text[32768]; read_report(text, sizeof text);
  CHECK(strstr(text, "\"sign\":-1,\"detected\":3,\"median_ma\":-1100,\"min_ma\":-1200,\"max_ma\":-1000"));
  CHECK(strstr(text, "\"sign\":1,\"detected\":3,\"median_ma\":1100,\"min_ma\":1000,\"max_ma\":1200"));
  CHECK(strstr(text, "\"attempted\":true,\"capped\":false,\"moved\":true"));
}

static void failures_and_check(void)
{
  reset_report(); CHECK_EQ(invoke("--check"), 0);
  CHECK_EQ(moves, 0); CHECK_EQ(ramps, 0); CHECK_EQ(stops, 0); CHECK_EQ(closes, 1);
  reset_report(); preflight_error = 1; CHECK_EQ(invoke(NULL), 1);
  CHECK_EQ(moves, 0); CHECK_EQ(stops, 0); CHECK_EQ(closes, 1);
  reset_report(); acquisition_error = 1; CHECK_EQ(invoke(NULL), 1);
  CHECK_EQ(moves, 1); CHECK_EQ(stops, 1); CHECK_EQ(closes, 1);
  char text[32768]; read_report(text, sizeof text);
  CHECK(strstr(text, "\"complete\":false,\"error\":\"fixture Vbus error\""));
  reset_report(); cancel_at = 1; CHECK_EQ(invoke(NULL), 1);
  CHECK_EQ(moves, 1); CHECK_EQ(stops, 1); CHECK_EQ(closes, 1);
  read_report(text, sizeof text); CHECK(strstr(text, "\"error\":\"interrupted; stopping\""));
}

int main(void)
{
  CHECK(mkdtemp(directory));
  speed_report(); breakaway_report(); failures_and_check();
  CHECK_EQ(rmdir(directory), 0);
  puts("friction_map: 3 test groups passed");
  return 0;
}
