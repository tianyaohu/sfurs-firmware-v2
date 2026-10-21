#include "check.h"
#include "drive_record.h"
#include <string.h>

static void test_observed_handoff(void)
{
  ik_record_unpacked_t rows[17];
  for (int k = 0; k < 5; k++) rows[k] = (ik_record_unpacked_t){.state = "stopped", .reason = "stop"};
  for (int k = 5; k < 15; k++) rows[k] = (ik_record_unpacked_t){.state = "starting", .reason = "armed", .iq_ref_a = 1 + (k - 5) / 10.0};
  rows[15] = (ik_record_unpacked_t){.state = "running", .reason = "edge", .iq_ref_a = 0.6, .rpm = 40};
  rows[16] = (ik_record_unpacked_t){.state = "running", .reason = "edge", .iq_ref_a = 0.7, .rpm = -60};
  drive_record_summary_t s = drive_record_summary(rows, 17, 5);
  CHECK(s.handoff); CHECK_EQ(s.handoff_ms, 10); CHECK_NEAR(s.handoff_iq_a, 1.9, 1e-9); CHECK_EQ(s.peak_rpm, 60);
  CHECK_EQ(s.samples, 17); CHECK_EQ(s.trigger, 5);
}

static void test_delayed_window_has_no_handoff(void)
{
  ik_record_unpacked_t rows[20];
  for (int k = 0; k < 20; k++) rows[k] = (ik_record_unpacked_t){.state = "running", .reason = "edge", .iq_ref_a = 0.6, .rpm = 50};
  drive_record_summary_t s = drive_record_summary(rows, 20, 0);
  CHECK(!s.handoff); CHECK_EQ(s.peak_rpm, 50);
  drive_records_t all = {0};
  for (int w = 0; w < 4; w++) { all.valid[w] = true; all.wheels[w] = s; }
  FILE *f = tmpfile(); CHECK(f); drive_records_json(f, &all); rewind(f);
  char text[2048] = ""; CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  CHECK(strstr(text, "\"handoff_ms\":null,\"handoff_iq_a\":null"));
  for (int w = 0; w < 4; w++) CHECK(strstr(text, IK_WHEEL_POS[w]));
}

static void test_first_handoff_only_and_edge_reason(void)
{
  ik_record_unpacked_t rows[] = {
    {.state = "starting", .reason = "armed", .iq_ref_a = 1.234},
    {.state = "running", .reason = "moving"},
    {.state = "starting", .reason = "armed", .iq_ref_a = -1.236},
    {.state = "running", .reason = "edge"},
    {.state = "starting", .reason = "armed", .iq_ref_a = 3},
    {.state = "running", .reason = "edge"}
  };
  drive_record_summary_t s = drive_record_summary(rows, 6, 4);
  CHECK(s.handoff); CHECK_EQ(s.handoff_ms, -1); CHECK_NEAR(s.handoff_iq_a, -1.24, 1e-9);
  s = drive_record_summary(NULL, 0, 0);
  CHECK(!s.handoff); CHECK_EQ(s.samples, 0); CHECK_EQ(s.peak_rpm, 0);
}

static void test_record_errors_escape_and_all_wheels(void)
{
  drive_records_t r = {0};
  for (int w = 0; w < 4; w++) snprintf(r.errors[w], sizeof r.errors[w], "error \"%s\"\n", IK_WHEEL_POS[w]);
  FILE *f = tmpfile(); CHECK(f); drive_records_json(f, &r); rewind(f);
  char text[2048] = ""; CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  for (int w = 0; w < 4; w++) {
    char want[128]; snprintf(want, sizeof want, "\"%s\":\"error \\\"%s\\\"\\u000a\"", IK_WHEEL_POS[w], IK_WHEEL_POS[w]);
    CHECK(strstr(text, want));
  }
}

int main(void)
{
  test_observed_handoff();
  test_delayed_window_has_no_handoff();
  test_first_handoff_only_and_edge_reason();
  test_record_errors_escape_and_all_wheels();
  puts("drive record: 4 test cases passed");
  return 0;
}
