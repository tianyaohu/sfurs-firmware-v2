/* Nucleo drive loop (ik_drive.c) end to end over protocol v1. The four ESCs are the real
 * command lifecycle (motor_cmd.c), so both ends of the protocol are production code; only
 * MCSDK's state machine is faked, at its documented boundary (IDLE/start/RUN/stop/fault). */
#include "ik_drive.h"
#include "can_datatypes.h"
#include "check.h"
#include "motor_cmd.h"
#include "motor_protocol.h"
#include <stdlib.h>
#include <string.h>

#define MAX_FRAMES 16384U
static unsigned count, fail;
static uint32_t ids[MAX_FRAMES], lens[MAX_FRAMES];
static uint8_t payload[MAX_FRAMES][8];
static uint32_t now;

static struct motor_cmd esc[4];
static struct motor_cmd_inputs esc_in[4];
static int drop_replies[4];

static int send(uint32_t id, const uint8_t *d, uint32_t len)
{
  if (fail) return 0;
  CHECK(count < MAX_FRAMES && (len == 4U || len == 8U));
  ids[count] = id; lens[count] = len; memset(payload[count], 0, 8); memcpy(payload[count], d, len); ++count;
  for (unsigned w = 0; w < 4U; ++w) /* the ESC hardware filter: frames carrying its select bit */
    if ((id & (1U << w)) && (id & MOTOR_CAN_ID_BASE_MASK) == MOTOR_CAN_ID_CONTROL)
      motor_cmd_rx(&esc[w], id, d, len, now);
  return 1;
}

/* One ESC millisecond: MCSDK settles (start -> RUN, stop -> IDLE take 1 ms), the lifecycle
 * ticks, its actions move MCSDK, and its replies go back to the Nucleo. */
static void esc_step(unsigned w)
{
  struct motor_cmd_inputs *in = &esc_in[w];
  if (in->mc == MOTOR_CMD_MC_STARTING) in->mc = MOTOR_CMD_MC_RUN;
  else if (in->mc == MOTOR_CMD_MC_STOPPING) in->mc = MOTOR_CMD_MC_IDLE;
  in->now_ms = now;
  struct motor_cmd_actions a;
  motor_cmd_tick(&esc[w], in, &a);
  if (a.stop && in->mc != MOTOR_CMD_MC_FAULT) in->mc = MOTOR_CMD_MC_STOPPING;
  if (a.ack_fault && in->mc == MOTOR_CMD_MC_FAULT && !in->mc_fault_present) in->mc = MOTOR_CMD_MC_IDLE;
  if (a.start) in->mc = MOTOR_CMD_MC_STARTING;
  uint32_t id; uint8_t d[8];
  while (motor_cmd_reply_peek(&esc[w], &id, d)) {
    if (!drop_replies[w]) CHECK(ik_drive_rx(id, d, 8U, now * 1000U + 100U * w) == 1);
    motor_cmd_reply_pop(&esc[w]);
  }
}

static void run(unsigned ms, int beat)
{
  for (unsigned i = 0; i < ms; ++i) {
    if (beat && now % 50U == 0U) ++ik_drive_heartbeat;
    ik_drive_tick(now, send);
    for (unsigned w = 0; w < 4U; ++w) esc_step(w);
    ++now;
  }
}

static void setup(void)
{
  for (unsigned w = 0; w < 4U; ++w) {
    const struct motor_cmd_config cfg = { .select = 1U << w, .max_rpm = 2976, .max_ma = 6000,
                                          .ff_supported = 0U, .current_supported = 0U };
    motor_cmd_init(&esc[w], &cfg, 100U + w);
    esc_in[w] = (struct motor_cmd_inputs){ .mc = MOTOR_CMD_MC_IDLE, .aligned = 1U };
  }
}

static int decode(unsigned k, struct motor_control *c)
{
  return lens[k] == 8U && motor_control_decode(ids[k], payload[k], 8U, c) == MOTOR_R_NONE;
}

/* Setpoint of the last SPEED frame sent to wheel i, or 99999. */
static int last_rpm(unsigned i)
{
  struct motor_control c;
  for (unsigned k = count; k-- > 0;)
    if (ids[k] == (MOTOR_CAN_ID_CONTROL | (1U << i)) && decode(k, &c) && c.mode == MOTOR_MODE_SPEED) return c.setpoint;
  return 99999;
}

static unsigned count_speed(unsigned i)
{
  unsigned n = 0; struct motor_control c;
  for (unsigned k = 0; k < count; ++k)
    n += ids[k] == (MOTOR_CAN_ID_CONTROL | (1U << i)) && decode(k, &c) && c.mode == MOTOR_MODE_SPEED;
  return n;
}

/* Plain DISABLE broadcasts sent since frame @p from. */
static unsigned count_disable_all(unsigned from)
{
  unsigned n = 0; struct motor_control c;
  for (unsigned k = from; k < count; ++k)
    n += ids[k] == (MOTOR_CAN_ID_CONTROL | 0xFU) && decode(k, &c) && c.mode == MOTOR_MODE_DISABLE && c.flags == 0U;
  return n;
}

static const int fwd1[4] = {253, 239, -239, -253}; /* 1 m/s forward, raw motor RPM */

static void drive_forward(void)
{
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i] && esc[i].lifecycle == MOTOR_LC_RUNNING);
}

static void test_needs_enable_and_heartbeat(void)
{
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(1000, 0); CHECK(count == 0 && ik_drive_state == IK_STOPPED); /* no heartbeat yet */
  ik_drive_enable = 0; run(1000, 1); CHECK(count == 0); /* heartbeat, not enabled */
}

static void test_links_then_starts_together(void)
{
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(1, 1); /* period 1: every wheel probed with an ARM carrying session 0 */
  for (unsigned i = 0; i < 4; ++i) {
    struct motor_control c;
    CHECK(decode(i, &c) && c.mode == MOTOR_MODE_DISABLE && c.flags == MOTOR_FLAG_ARM && c.session == 0U);
    CHECK(ik_drive_link[i].lifecycle == MOTOR_LC_DISARMED && ik_drive_link[i].reason == MOTOR_R_BAD_SESSION);
    CHECK(ik_drive_link[i].session == esc[i].session);
  }
  run(20, 1); /* period 2: armed with the announced session */
  CHECK(ik_drive_armed == 0xFU);
  for (unsigned i = 0; i < 4; ++i) CHECK(esc[i].lifecycle == MOTOR_LC_READY && count_speed(i) == 0);
  run(20, 1); /* period 3: motion, every wheel in the same period */
  for (unsigned i = 0; i < 4; ++i) {
    CHECK(count_speed(i) == 1 && abs(last_rpm(i)) >= 16 && abs(last_rpm(i)) <= 18); /* 900 RPM/s * 20 ms */
    CHECK(ik_drive_link[i].start_count == 1U && ik_drive_link[i].outcome == MOTOR_OUT_INHIBITED);
    CHECK(ik_drive_link[i].reason == MOTOR_R_STARTING && esc[i].lifecycle == MOTOR_LC_STARTING);
  }
  run(1, 1); /* MCSDK reaches RUN one ms after the start */
  for (unsigned i = 0; i < 4; ++i) CHECK(esc[i].lifecycle == MOTOR_LC_RUNNING);
  for (unsigned i = 0; i < 4; ++i) /* the start reply's Nucleo timestamp is latched once */
    CHECK(ik_drive_link[i].first_start_reply_us == ik_drive_link[i].start_reply_us &&
          ik_drive_link[i].first_start_valid == 1U);
}

static void test_streams_and_slews_to_target(void)
{
  drive_forward();
  const unsigned mark[4] = {count_speed(0), count_speed(1), count_speed(2), count_speed(3)};
  run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) {
    const unsigned frames = count_speed(i) - mark[i];
    CHECK(frames >= 49 && frames <= 51);
    CHECK(last_rpm(i) == fwd1[i] && ik_drive_target_rpm[i] == fwd1[i]);
    CHECK(ik_drive_link[i].outcome == MOTOR_OUT_APPLIED && ik_drive_link[i].misses == 0U);
    CHECK(ik_drive_link[i].applied_seq == ik_drive_link[i].request_seq);
  }
}

static void test_release_brakes_then_stops(void)
{
  drive_forward();
  ik_drive_enable = 0; unsigned mark = count;
  run(320, 1); CHECK(ik_drive_state == IK_DRIVING); /* 253 RPM at 900 RPM/s */
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == 0);
  CHECK(count_disable_all(mark) == 0);
  run(400, 1); CHECK(ik_drive_state == IK_STOPPED && count_disable_all(mark) == IK_DRIVE_STOP_FRAMES);
  mark = count; run(500, 1); CHECK(count == mark); /* quiet once stopped */
  for (unsigned i = 0; i < 4; ++i) CHECK(esc[i].lifecycle == MOTOR_LC_DISARMED); /* their watchdog */
}

static void test_lost_heartbeat_brakes_then_stops(void)
{
  drive_forward();
  run(240, 0); /* last beat at t=950: fresh until 1250 */
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i]);
  const unsigned mark = count;
  run(1000, 0); CHECK(ik_drive_state == IK_STOPPED && count_disable_all(mark) == IK_DRIVE_STOP_FRAMES);
}

static void test_reverses_without_stop(void)
{
  drive_forward();
  uint32_t first[4];
  for (unsigned i = 0; i < 4; ++i) first[i] = ik_drive_link[i].first_start_reply_us;
  const unsigned mark = count;
  ik_drive_vy_mmps = -1000; run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) {
    CHECK(last_rpm(i) == -fwd1[i] && ik_drive_link[i].start_count == 2U); /* a reversal is a new start */
    CHECK(esc[i].lifecycle == MOTOR_LC_RUNNING);
    CHECK(ik_drive_link[i].first_start_reply_us == first[i]);  /* the session's first start is kept */
    CHECK(ik_drive_link[i].start_reply_us > first[i]);         /* the latest start moved on */
  }
  CHECK(count_disable_all(mark) == 0);
}

static void test_zero_twist_holds(void)
{
  ik_drive_enable = 1; run(1000, 1);
  CHECK(ik_drive_state == IK_DRIVING && count_disable_all(0) == 0 && ik_drive_armed == 0xFU);
  for (unsigned i = 0; i < 4; ++i) {
    CHECK(last_rpm(i) == 0 && count_speed(i) >= 45 && esc[i].lifecycle == MOTOR_LC_READY);
    CHECK(ik_drive_link[i].start_count == 0U);
  }
}

static void test_bad_input_and_saturation(void)
{
  ik_drive_vx_mmps = 5001; ik_drive_enable = 1; run(200, 1);
  CHECK(ik_drive_bad_input > 0);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == 0);
  ik_drive_vx_mmps = 0; ik_drive_vy_mmps = 5000; run(3000, 1);
  const int sat[4] = {900, 851, -851, -900};
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == sat[i]);
}

static void test_raw_frames_only_when_stopped(void)
{
  for (unsigned i = 0; i < 4; ++i) {
    ik_drive_raw_id = HALL_ALIGN | (1U << i);
    ik_drive_raw_data = HALL_ALIGN_MAGIC;
    ik_drive_raw_len = 4U;
    ik_drive_raw_seq = i + 1;
    run(1, 0);
    CHECK(count == i + 1 && ids[i] == (HALL_ALIGN | (1U << i)) && lens[i] == 4U && ik_drive_raw_done == i + 1);
    CHECK(memcmp(payload[i], "HALN", 4) == 0);
  }
  /* an 8-byte raw frame: a CONTROL probe sent by a tool */
  ik_drive_raw_id = MOTOR_CAN_ID_CONTROL | 0x2U;
  ik_drive_raw_data = 0x00000110U;    /* version 1, DISABLE, ARM */
  ik_drive_raw_data_hi = 0x00000000U; /* session 0 */
  ik_drive_raw_len = 8U;
  ik_drive_raw_seq = 9U;
  run(1, 0);
  CHECK(lens[count - 1] == 8U && ik_drive_raw_done == 9U);
  CHECK(ik_drive_link[1].reason == MOTOR_R_BAD_SESSION); /* the ESC answered it */
  ik_drive_raw_len = 4U;
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(100, 1);
  for (unsigned i = 0; i < 4; ++i) {
    ik_drive_raw_id = HALL_ALIGN | (1U << i);
    ik_drive_raw_seq = i + 20;
    run(100, 1);
    CHECK(ik_drive_raw_done == 9U); /* not while driving */
  }
}

static void test_tx_failures_counted(void)
{
  drive_forward();
  fail = 1; run(40, 1); fail = 0; /* two periods unsent: tolerated */
  CHECK(ik_drive_tx_fail >= 8 && ik_drive_state == IK_DRIVING && ik_drive_fault == 0U);
  run(1000, 1);
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i] && ik_drive_link[i].misses == 0U);
  fail = 1; run(20 * (IK_DRIVE_LOST_PERIODS + 1U), 1); /* a wheel that cannot be commanded is lost */
  CHECK(ik_drive_fault == 0xF0U && ik_drive_state != IK_DRIVING);
  fail = 0; run(200, 1);
  CHECK(ik_drive_state == IK_STOPPED);
}

static void test_custom_acceleration(void)
{
  ik_drive_accel_rpmps = 2530; ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(160, 1); /* 40 ms to arm, then 253 RPM in 100 ms */
  for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i]);
}

/* For every wheel: lose its replies; the robot stops and stays stopped until enable goes 0. */
static void test_lost_wheel_stops_the_robot(void)
{
  for (unsigned w = 0; w < 4; ++w) {
    drive_forward();
    drop_replies[w] = 1;
    unsigned before = count;
    for (unsigned t = 0; t < 20 * (IK_DRIVE_LOST_PERIODS + 1U) && ik_drive_fault == 0U; ++t) {
      before = count;
      run(1, 1);
    }
    CHECK(ik_drive_fault == (1U << (4U + w)) && ik_drive_link[w].misses >= IK_DRIVE_LOST_PERIODS);
    /* the detecting period sends DISABLE and no further motion */
    CHECK(count_disable_all(before) == 1U);
    for (unsigned i = 0; i < 4; ++i) {
      struct motor_control c;
      for (unsigned k = before; k < count; ++k)
        CHECK(!(ids[k] == (MOTOR_CAN_ID_CONTROL | (1U << i)) && decode(k, &c) && c.mode == MOTOR_MODE_SPEED));
    }
    CHECK(ik_drive_link[w].fault_reason == 0U); /* LOST has no ESC reason */
    const unsigned mark = count;
    run(200, 1);
    CHECK(ik_drive_state == IK_STOPPED && count_disable_all(mark) >= 2U);
    for (unsigned i = 0; i < 4; ++i) CHECK(esc[i].lifecycle == MOTOR_LC_READY || esc[i].lifecycle == MOTOR_LC_DISARMED);
    run(500, 1); CHECK(ik_drive_state == IK_STOPPED); /* latched while enabled */
    drop_replies[w] = 0;
    ik_drive_enable = 0; run(20, 1); CHECK(ik_drive_fault == 0U);
  }
}

/* For every wheel: an ESC fault stops the robot; clearing needs enable 0 and clear_seq. */
static void test_faulted_wheel_stops_until_cleared(void)
{
  for (unsigned w = 0; w < 4; ++w) {
    drive_forward();
    esc_in[w].mc = MOTOR_CMD_MC_FAULT; esc_in[w].mc_fault_present = 1U;
    run(60, 1);
    CHECK(ik_drive_fault == (1U << w) && esc[w].lifecycle == MOTOR_LC_FAULTED);
    run(200, 1); CHECK(ik_drive_state == IK_STOPPED);
    CHECK(ik_drive_link[w].fault_reason == MOTOR_R_FAULT_MCSDK); /* kept while later replies arrive */
    ik_drive_enable = 0; run(20, 1); CHECK(ik_drive_fault == 0U);
    ++ik_drive_clear_seq; run(5, 1); /* cause still present: the ESC refuses */
    CHECK(ik_drive_clear_done == ik_drive_clear_seq && esc[w].lifecycle == MOTOR_LC_FAULTED);
    CHECK(ik_drive_link[w].reason == MOTOR_R_NOT_IDLE);
    esc_in[w].mc_fault_present = 0U; /* MCSDK FAULT_OVER */
    ++ik_drive_clear_seq; run(5, 1);
    CHECK(esc[w].lifecycle == MOTOR_LC_DISARMED && esc_in[w].mc == MOTOR_CMD_MC_IDLE);
    ik_drive_enable = 1; run(1000, 1);
    for (unsigned i = 0; i < 4; ++i) CHECK(last_rpm(i) == fwd1[i] && esc[i].lifecycle == MOTOR_LC_RUNNING);
    ik_drive_enable = 0; run(800, 1);
  }
}

/* After a Nucleo restart the ESC still holds another sequence baseline: re-arm, no fault. */
static void test_resyncs_a_moved_baseline(void)
{
  drive_forward();
  for (unsigned w = 0; w < 4; ++w) esc[w].last_seq = (uint8_t)(esc[w].last_seq + 100U); /* far ahead */
  run(100, 1);
  CHECK(ik_drive_fault == 0U && ik_drive_state == IK_DRIVING);
  for (unsigned i = 0; i < 4; ++i) {
    CHECK(ik_drive_link[i].outcome == MOTOR_OUT_APPLIED && esc[i].lifecycle == MOTOR_LC_RUNNING);
    CHECK(ik_drive_link[i].rejects >= 1U);
  }
}

static void test_feed_forward_rides_in_speed_frames(void)
{
  ik_drive_ff_enable = 1; ik_drive_ff_coulomb_xy_mn = 3000; ik_drive_ff_a_per_nm_milli = 40000;
  drive_forward();
  for (unsigned i = 0; i < 4; ++i) {
    struct motor_control c = {0};
    for (unsigned k = count; k-- > 0;)
      if (ids[k] == (MOTOR_CAN_ID_CONTROL | (1U << i)) && decode(k, &c) && c.mode == MOTOR_MODE_SPEED) break;
    CHECK(c.ff_ma == ik_drive_ff_ma[i] && c.ff_ma != 0 && (c.ff_ma > 0) == (fwd1[i] > 0)); /* along the motion */
    CHECK(ik_drive_link[i].reason == MOTOR_R_FF_UNUSED); /* this ESC's loop ignores it, and says so */
  }
}

/* A lost reply to the first start leaves no first-start time, even after the 8-bit sequence
 * wraps many times (a later reply with the same seq must not fill it in). */
static void test_lost_start_reply_stays_invalid(void)
{
  ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
  run(40, 1);                                   /* probe at t=0, ARM at t=20 */
  drop_replies[0] = 1; run(1, 1); drop_replies[0] = 0; /* t=40: the first motion frame */
  run(6000, 1);                                 /* ~300 frames: the sequence wraps */
  CHECK(ik_drive_fault == 0U && ik_drive_link[0].start_count == 1U);
  CHECK(ik_drive_link[0].first_start_valid == 0U);
  for (unsigned i = 1; i < 4; ++i) CHECK(ik_drive_link[i].first_start_valid == 1U);
}

int main(int argc, char **argv)
{
  CHECK(argc <= 2);
  setup();
  /* CTest starts a fresh process for each case because the drive retains state. */
  static const struct { const char *name; void (*fn)(void); } cases[] = {
    { "needs_enable_and_heartbeat", test_needs_enable_and_heartbeat },
    { "links_then_starts_together", test_links_then_starts_together },
    { "streams_and_slews_to_target", test_streams_and_slews_to_target },
    { "release_brakes_then_stops", test_release_brakes_then_stops },
    { "lost_heartbeat_brakes_then_stops", test_lost_heartbeat_brakes_then_stops },
    { "reverses_without_stop", test_reverses_without_stop },
    { "zero_twist_holds", test_zero_twist_holds },
    { "bad_input_and_saturation", test_bad_input_and_saturation },
    { "raw_frames_only_when_stopped", test_raw_frames_only_when_stopped },
    { "tx_failures_counted", test_tx_failures_counted },
    { "custom_acceleration", test_custom_acceleration },
    { "lost_wheel_stops_the_robot", test_lost_wheel_stops_the_robot },
    { "faulted_wheel_stops_until_cleared", test_faulted_wheel_stops_until_cleared },
    { "resyncs_a_moved_baseline", test_resyncs_a_moved_baseline },
    { "feed_forward_rides_in_speed_frames", test_feed_forward_rides_in_speed_frames },
    { "lost_start_reply_stays_invalid", test_lost_start_reply_stays_invalid },
  };
  const char *name = argc == 2 ? argv[1] : cases[0].name;
  for (size_t i = 0; i < sizeof cases / sizeof *cases; ++i)
    if (strcmp(name, cases[i].name) == 0) { cases[i].fn(); return 0; }
  CHECK(0);
  return 1;
}
