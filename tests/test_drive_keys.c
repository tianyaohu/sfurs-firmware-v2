#include "check.h"
#include "drive_keyboard.h"
#include <string.h>

static void vector(drive_keys_t *keys, double now_s, int vx, int vy, int yaw)
{
  int32_t v[3];
  drive_key_vector(keys, now_s, 0.5, 2, v);
  CHECK_EQ(v[0], vx); CHECK_EQ(v[1], vy); CHECK_EQ(v[2], yaw);
}

static void test_keys_combine_and_expire(void)
{
  drive_keys_t keys = {.hold_s = 0.6};
  drive_key_press(&keys, 'w', 0);
  drive_key_press(&keys, 'd', 0.1);
  drive_key_press(&keys, 'q', 0.2);
  vector(&keys, 0.3, 500, 500, 2000);
  vector(&keys, 0.6, 500, 0, 2000);
  vector(&keys, 0.7, 0, 0, 2000);
  vector(&keys, 0.8, 0, 0, 0);
  keys = (drive_keys_t){.hold_s = 0.6};
  drive_key_press(&keys, 'w', 0);
  drive_key_press(&keys, 'd', 0.1);
  drive_key_press(&keys, 'q', 0.2);
  vector(&keys, 0.3, 500, 500, 2000);
  drive_key_press(&keys, 's', 0.4);
  vector(&keys, 0.5, 500, -500, 2000);
  vector(&keys, 0.85, 0, -500, 0);
  drive_key_press(&keys, ' ', 0.9);
  vector(&keys, 0.9, 0, 0, 0);
}

static void test_opposite_repeat_space_and_unknown(void)
{
  drive_keys_t keys = {.hold_s = 0.6};
  drive_key_press(&keys, 'w', 1);
  drive_key_press(&keys, 's', 1.1);
  drive_key_press(&keys, 'a', 1.1);
  drive_key_press(&keys, 'e', 1.1);
  vector(&keys, 1.2, -500, -500, -2000);
  drive_key_press(&keys, 's', 1.5);
  drive_key_press(&keys, '?', 1.6);
  vector(&keys, 1.8, 0, -500, 0);
  drive_key_press(&keys, ' ', 1.9);
  vector(&keys, 2, 0, 0, 0);
  drive_key_press(&keys, 0, 2);
  vector(&keys, 2.1, 0, 0, 0);
}

static void test_quit_disables_and_zeros(void)
{
  for (int key = 0; key < 2; key++) {
    drive_keyboard_t s = {.keys = {.hold_s = 0.6}, .speed_mps = 0.5, .max_speed_mps = 2.5, .yaw_radps = 2};
    int32_t v[3];
    drive_keyboard_press(&s, 'W', 1);
    CHECK(drive_keyboard_vector(&s, 1, v)); CHECK_EQ(v[1], 500);
    drive_keyboard_press(&s, key ? 27 : 'x', 1.1);
    drive_keyboard_press(&s, 'd', 1.2);
    CHECK(s.quitting); CHECK_NEAR(s.quit_deadline_s, 4.1, 1e-9);
    CHECK(!drive_keyboard_vector(&s, 1.2, v));
    CHECK_EQ(v[0], 0); CHECK_EQ(v[1], 0); CHECK_EQ(v[2], 0);
  }
}

static void test_speed_limits_and_rounding(void)
{
  drive_keyboard_t s = {.keys = {.hold_s = 0.6}, .speed_mps = 0.5, .max_speed_mps = 0.7, .yaw_radps = 2};
  for (int k = 0; k < 10; k++) drive_keyboard_press(&s, '+', 0);
  CHECK_NEAR(s.speed_mps, 0.7, 1e-12);
  for (int k = 0; k < 10; k++) drive_keyboard_press(&s, '-', 0);
  CHECK_NEAR(s.speed_mps, 0.1, 1e-12);
  drive_keys_t keys = {.hold_s = 1};
  drive_key_press(&keys, 'd', 0);
  int32_t v[3];
  drive_key_vector(&keys, 0, 0.1005, 2, v);
  CHECK_EQ(v[0], 100);
}

static void test_keyboard_log_all_wheels_and_missing(void)
{
  FILE *f = tmpfile(); CHECK(f);
  ik_status_t status = {.state = 2, .telem_now = 9, .fault = 0xa5, .armed = 15, .target_rpm = {50, 51, 52, 53}};
  ik_row_t rows[4];
  for (int w = 0; w < 4; w++) rows[w] = (ik_row_t){.valid = true, .rpm = 61 + w, .state = 6,
    .status_count = 10, .mc_state = "RUN", .stop_reason = "none", .align = "ok", .iq_a = 0.5, .vbus_v = 12};
  strcpy(rows[3].fault_names, "overcurrent");
  CHECK_EQ(drive_keyboard_log(f, 100.125, 90.25, (int32_t[]){0, 500, 0}, &status, rows), 0);
  rewind(f);
  char text[16384] = ""; CHECK(fread(text, 1, sizeof text - 1, f) > 0);
  CHECK(strstr(text, "\"t0_monotonic_s\":90.250000000"));
  CHECK(strstr(text, "\"t_monotonic_s\":100.125000000"));
  CHECK(strstr(text, "\"state_name\":\"STOPPING\""));
  CHECK(strstr(text, "\"fault\":165,\"armed\":15"));
  CHECK(strstr(text, "\"vector\":[0,500,0]"));
  for (int w = 0; w < 4; w++) { char want[32]; snprintf(want, sizeof want, "\"rpm\":%d", 61 + w); CHECK(strstr(text, want)); }
  CHECK(strstr(text, "\"fault_names\":[\"overcurrent\"]"));
  fclose(f);
  f = tmpfile(); CHECK(f);
  memset(rows, 0, sizeof rows);
  CHECK_EQ(drive_keyboard_log(f, 1, 1, (int32_t[]){0, 0, 0}, &status, rows), 0);
  rewind(f); memset(text, 0, sizeof text); CHECK(fread(text, 1, sizeof text - 1, f) > 0);
  CHECK(strstr(text, "\"rows\":[null,null,null,null]"));
  fclose(f);
}

int main(void)
{
  test_keys_combine_and_expire();
  test_opposite_repeat_space_and_unknown();
  test_quit_disables_and_zeros();
  test_speed_limits_and_rounding();
  test_keyboard_log_all_wheels_and_missing();
  puts("drive keys: 5 test cases passed");
  return 0;
}
