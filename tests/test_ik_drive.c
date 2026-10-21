#include "ik_drive.h"
#include "can_datatypes.h"
#include "check.h"
#include <stdlib.h>
#include <string.h>
static unsigned count, fail;
static uint32_t ids[4096]; static uint8_t payload[4096][4];
static int send(uint32_t id, const uint8_t d[4]) {
  if (fail) return 0;
  CHECK(count < 4096); ids[count] = id; memcpy(payload[count++], d, 4); return 1;
}
static uint32_t now;
static void run(unsigned ms, int beat) {
  for (unsigned i = 0; i < ms; ++i) { if (beat && now % 50 == 0) ++ik_drive_heartbeat; ik_drive_tick(now++, send); }
}
static int rpm_of(unsigned k) { return (int16_t)(payload[k][0] | payload[k][1] << 8); }
static int last_rpm(unsigned i) {
  for (unsigned k = count; k-- > 0;)
    if (ids[k] == (SPEED_WRITE | (1U << i))) return rpm_of(k);
  return 99999;
}
static unsigned count_id(uint32_t id) { unsigned n = 0; for (unsigned k = 0; k < count; ++k) n += ids[k] == id; return n; }
static const int fwd1[4] = {253, 239, -239, -253}; /* 1 m/s forward, raw motor RPM */


static void test_needs_enable_and_heartbeat(void) {
  /* nothing without enable + heartbeat */
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(1000, 0); CHECK(count == 0 && ik_drive_state == IK_STOPPED); /* no heartbeat yet */
  ik_drive_enable = 0; run(1000, 1); CHECK(count == 0); /* heartbeat, not enabled */
}

static void test_streams_and_slews_to_target(void) {
  /* stream at 50 Hz, accelerate along the twist, reach target */
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(20, 1); CHECK(ik_drive_state == IK_DRIVING && count == 4);
  for (unsigned i = 0; i < 4; ++i) {
    CHECK(ids[i] == (SPEED_WRITE | (1U << i)) && payload[i][2] == 20 && payload[i][3] == 0);
    /* first step: largest wheel moves 900 RPM/s * 20 ms = 18 RPM, others in proportion */
    CHECK(abs(rpm_of(i)) <= 18 && abs(rpm_of(i)) >= 16);
  }
  run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) {
    unsigned frames = count_id(SPEED_WRITE | (1U << i));
    CHECK(frames >= 50 && frames <= 52);
    CHECK(last_rpm(i) == fwd1[i] && ik_drive_target_rpm[i] == fwd1[i]);
  }
}

static void test_release_brakes_then_stops(void) {
  /* release: decelerate, hold, then 3 STOP broadcasts */
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(1000, 1);
  ik_drive_enable = 0; unsigned mark = count;
  run(320, 1); CHECK(ik_drive_state == IK_DRIVING); /* 253 RPM at 900 RPM/s */
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == 0);
  for (unsigned k = mark; k < count; ++k) CHECK(ids[k] != (MOTOR_STOP | 0xFU));
  run(400, 1); CHECK(ik_drive_state == IK_STOPPED && count_id(MOTOR_STOP | 0xFU) == 3);
  mark = count; run(500, 1); CHECK(count == mark); /* quiet once stopped */
}

static void test_lost_heartbeat_brakes_then_stops(void) {
  /* host dies: heartbeat ages out -> same brake + STOP */
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(1000, 1);
  run(240, 0); /* last beat at t=950: fresh until 1250 */
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i]);
  run(1000, 0); CHECK(ik_drive_state == IK_STOPPED && count_id(MOTOR_STOP | 0xFU) == 3);
}

static void test_reverses_without_stop(void) {
  /* reverse direction without stopping in between */
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(1000, 1);
  ik_drive_vy_mmps = -1000; run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == -fwd1[i]);
  CHECK(count_id(MOTOR_STOP | 0xFU) == 0);
}

static void test_zero_twist_holds(void) {
  /* enable with zero twist holds zero, no STOP */
  ik_drive_enable = 1; run(1000, 1);
  CHECK(ik_drive_state == IK_DRIVING && count_id(MOTOR_STOP | 0xFU) == 0);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == 0);
}

static void test_bad_input_and_saturation(void) {
  /* out-of-range input drives zero and counts */
  ik_drive_vx_mmps = 5001; ik_drive_enable = 1; run(200, 1);
  CHECK(ik_drive_bad_input > 0);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == 0);
  ik_drive_vx_mmps = 0; ik_drive_vy_mmps = 5000; run(3000, 1);
  const int sat[4] = {900, 851, -851, -900};
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == sat[i]);
}

static void test_raw_frames_only_when_stopped(void) {
  /* raw frames only while stopped */
  for (unsigned i = 0; i < 4; ++i) {
    ik_drive_raw_id = HALL_ALIGN | (1U << i);
    ik_drive_raw_data = HALL_ALIGN_MAGIC;
    ik_drive_raw_seq = i + 1;
    run(1, 0);
    CHECK(count == i + 1 && ids[i] == (HALL_ALIGN | (1U << i)) && ik_drive_raw_done == i + 1);
    CHECK(memcmp(payload[i], "HALN", 4) == 0);
  }
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(100, 1);
  for (unsigned i = 0; i < 4; ++i) {
    ik_drive_raw_id = HALL_ALIGN | (1U << i);
    ik_drive_raw_seq = i + 5;
    run(100, 1);
    CHECK(ik_drive_raw_done == 4 && count_id(HALL_ALIGN | (1U << i)) == 1);
  }
}

static void test_tx_failures_counted(void) {
  /* TX failures are counted, never fatal */
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1; fail = 1; run(100, 1);
  CHECK(ik_drive_tx_fail >= 16 && ik_drive_state == IK_DRIVING);
  fail = 0; run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i]);
}

static void test_custom_acceleration(void) {
  /* custom acceleration limit */
  ik_drive_accel_rpmps = 2530; ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(120, 1); /* 253 RPM in 100 ms */
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i]);
}

int main(int argc, char **argv) {
  CHECK(argc <= 2);
  /* CTest starts a fresh process for each case because the drive retains state. */
  const char *name = argc == 2 ? argv[1] : "needs_enable_and_heartbeat";
  if (strcmp(name, "needs_enable_and_heartbeat") == 0) { test_needs_enable_and_heartbeat(); return 0; }
  if (strcmp(name, "streams_and_slews_to_target") == 0) { test_streams_and_slews_to_target(); return 0; }
  if (strcmp(name, "release_brakes_then_stops") == 0) { test_release_brakes_then_stops(); return 0; }
  if (strcmp(name, "lost_heartbeat_brakes_then_stops") == 0) { test_lost_heartbeat_brakes_then_stops(); return 0; }
  if (strcmp(name, "reverses_without_stop") == 0) { test_reverses_without_stop(); return 0; }
  if (strcmp(name, "zero_twist_holds") == 0) { test_zero_twist_holds(); return 0; }
  if (strcmp(name, "bad_input_and_saturation") == 0) { test_bad_input_and_saturation(); return 0; }
  if (strcmp(name, "raw_frames_only_when_stopped") == 0) { test_raw_frames_only_when_stopped(); return 0; }
  if (strcmp(name, "tx_failures_counted") == 0) { test_tx_failures_counted(); return 0; }
  if (strcmp(name, "custom_acceleration") == 0) { test_custom_acceleration(); return 0; }
  CHECK(0);
  return 1;
}
