#include "check.h"
#include "drive_record.h"
#include <string.h>

static void test_summary_peak_and_window(void)
{
  ik_record_unpacked_t rows[17];
  for (int k = 0; k < 17; k++) rows[k] = (ik_record_unpacked_t){.mode = "run", .rpm = k == 11 ? -60 : 40};
  drive_record_summary_t s = drive_record_summary(rows, 17, 5);
  CHECK_EQ(s.peak_rpm, 60); /* |RPM|: reverse counts */
  CHECK_EQ(s.samples, 17); CHECK_EQ(s.trigger, 5);
  s = drive_record_summary(NULL, 0, 0);
  CHECK_EQ(s.samples, 0); CHECK_EQ(s.peak_rpm, 0);
}

static void test_summary_json_all_wheels(void)
{
  ik_record_unpacked_t rows[20];
  for (int k = 0; k < 20; k++) rows[k] = (ik_record_unpacked_t){.mode = "run", .rpm = 50};
  drive_record_summary_t s = drive_record_summary(rows, 20, 3);
  drive_records_t all = {0};
  for (int w = 0; w < 4; w++) { all.valid[w] = true; all.wheels[w] = s; }
  FILE *f = tmpfile(); CHECK(f); drive_records_json(f, &all); rewind(f);
  char text[2048] = ""; CHECK(fread(text, 1, sizeof text - 1, f) > 0); fclose(f);
  for (int w = 0; w < 4; w++) {
    char want[128];
    snprintf(want, sizeof want, "\"%s\":{\"samples\":20,\"trigger\":3,\"peak_rpm\":50}", IK_WHEEL_POS[w]);
    CHECK(strstr(text, want));
  }
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
  test_summary_peak_and_window();
  test_summary_json_all_wheels();
  test_record_errors_escape_and_all_wheels();
  puts("drive record: 3 test cases passed");
  return 0;
}
