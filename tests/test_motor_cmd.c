/* ESC command lifecycle (motor_cmd.c) against B-lifecycle.md: one process per scenario. The fake
 * motor-control state is just the inputs struct; the caller-side actions are checked directly. */
#include "motor_cmd.h"
#include "check.h"

#include <stdlib.h>
#include <string.h>

static uint32_t select_bit = 0x1U; /* overridden per wheel by main() */
static struct motor_cmd m;
static struct motor_cmd_inputs in;
static struct motor_cmd_actions act, acts; /* last tick, and OR of all ticks since clear_acts() */
static uint8_t seq;

static void clear_acts(void) { memset(&acts, 0, sizeof acts); }

static void tick(void)
{
  motor_cmd_tick(&m, &in, &act);
  acts.abort_busy |= act.abort_busy; acts.stop |= act.stop; acts.ack_fault |= act.ack_fault;
  acts.set_target |= act.set_target; acts.start |= act.start;
  if (act.stop) acts.stop_cause = act.stop_cause;
  if (act.set_target) { acts.mode = act.mode; acts.setpoint = act.setpoint; acts.ff_ma = act.ff_ma; }
  /* the fake MCSDK follows the actions one tick later, like the real state machine */
  if (act.stop && in.mc != MOTOR_CMD_MC_FAULT) in.mc = MOTOR_CMD_MC_STOPPING;
  if (act.ack_fault) in.mc = MOTOR_CMD_MC_IDLE;
  if (act.start) in.mc = MOTOR_CMD_MC_STARTING;
  ++in.now_ms;
}

static void ticks(unsigned n) { while (n--) tick(); }

/* MCSDK settles: STARTING -> RUN, STOPPING -> IDLE. */
static void settle(void)
{
  if (in.mc == MOTOR_CMD_MC_STARTING) in.mc = MOTOR_CMD_MC_RUN;
  else if (in.mc == MOTOR_CMD_MC_STOPPING) in.mc = MOTOR_CMD_MC_IDLE;
}

static void send_raw(uint32_t id, const uint8_t d[8], uint32_t dlc) { motor_cmd_rx(&m, id, d, dlc, in.now_ms); }

static void send(uint8_t mode, uint8_t flags, int16_t sp, int16_t ff, uint8_t s, uint8_t session, uint32_t sel)
{
  const struct motor_control c = { mode, flags, sp, ff, s, session };
  uint32_t id; uint8_t d[8];
  CHECK_EQ(motor_control_encode(&c, sel, &id, d), MOTOR_R_NONE);
  send_raw(id, d, 8);
}

static void speed(int16_t rpm) { send(MOTOR_MODE_SPEED, 0, rpm, 0, ++seq, m.session, select_bit); }
static void disable(void) { send(MOTOR_MODE_DISABLE, 0, 0, 0, seq, 0, 0xFU); }
static void arm(void) { send(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, seq, m.session, select_bit); }
static void clear_fault(void) { send(MOTOR_MODE_DISABLE, MOTOR_FLAG_CLEAR_FAULT, 0, 0, seq, 0, select_bit); }

/* Next reply, decoded; fails when none is pending. */
static struct motor_reply next_reply(void)
{
  uint32_t id; uint8_t d[8]; struct motor_reply r;
  CHECK(motor_cmd_reply_peek(&m, &id, d));
  CHECK_EQ(id, MOTOR_CAN_ID_CONTROL_REPLY | select_bit);
  motor_cmd_reply_pop(&m);
  CHECK_EQ(motor_reply_decode(d, 8, &r), MOTOR_R_NONE);
  return r;
}

static void no_reply(void) { uint32_t id; uint8_t d[8]; CHECK(!motor_cmd_reply_peek(&m, &id, d)); }
static void drain_replies(void) { uint32_t id; uint8_t d[8]; while (motor_cmd_reply_peek(&m, &id, d)) motor_cmd_reply_pop(&m); }

static void expect(uint8_t s, uint8_t outcome, uint8_t reason, uint8_t lifecycle)
{
  const struct motor_reply r = next_reply();
  CHECK_EQ(r.request_seq, s);
  CHECK_EQ(r.outcome, outcome);
  CHECK_EQ(r.reason, reason);
  CHECK_EQ(r.lifecycle, lifecycle);
  CHECK_EQ(r.session, m.session);
}

static void setup(void)
{
  const struct motor_cmd_config cfg = { .select = select_bit, .max_rpm = 2976, .max_ma = 6000,
                                        .ff_supported = 0, .current_supported = 0 };
  motor_cmd_init(&m, &cfg, 1234U);
  memset(&in, 0, sizeof in);
  in.now_ms = 1000U;
  in.aligned = 1U;
  seq = 10U;
  clear_acts();
}

/* Armed and running at @p rpm, replies drained. */
static void run_at(int16_t rpm)
{
  arm(); tick();
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_READY);
  speed(rpm); tick();
  CHECK(act.start && act.set_target && act.setpoint == rpm && act.mode == MOTOR_MODE_SPEED);
  expect(seq, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING, MOTOR_LC_STARTING);
  settle();
  speed(rpm); tick();
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_RUNNING);
  clear_acts();
}

static void test_boot_is_disarmed_and_rejects_motion(void)
{
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
  CHECK(m.session != 0U);
  speed(100); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_NOT_ARMED, MOTOR_LC_DISARMED);
  CHECK(!acts.start && !acts.set_target);
  CHECK(motor_cmd_output_off(&m));
}

static void test_arm_needs_the_current_session(void)
{
  send(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, seq, (uint8_t)(m.session + 1U), select_bit); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION, MOTOR_LC_DISARMED);
  arm(); tick();
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_READY);
  /* motion from another session is foreign even when armed */
  send(MOTOR_MODE_SPEED, 0, 100, 0, ++seq, (uint8_t)(m.session ^ 0x80U), select_bit); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION, MOTOR_LC_READY);
}

static void test_start_run_and_target_updates(void)
{
  run_at(120);
  CHECK_EQ(m.lifecycle, MOTOR_LC_RUNNING);
  speed(-80); tick();
  CHECK(act.set_target && act.setpoint == -80 && !act.start);
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_RUNNING);
  no_reply();
  CHECK_EQ(m.applied_seq, seq);
}

static void test_sequence_rules(void)
{
  arm(); tick(); drain_replies();
  const uint8_t base = seq;
  speed(50); tick(); drain_replies(); /* base + 1 accepted */
  send(MOTOR_MODE_SPEED, 0, 50, 0, (uint8_t)(base + 1U), m.session, select_bit); tick();
  expect((uint8_t)(base + 1U), MOTOR_OUT_DUPLICATE, MOTOR_R_NONE, m.lifecycle);
  send(MOTOR_MODE_SPEED, 0, 51, 0, (uint8_t)(base + 1U), m.session, select_bit); tick();
  expect((uint8_t)(base + 1U), MOTOR_OUT_REJECTED, MOTOR_R_CONFLICT, m.lifecycle);
  send(MOTOR_MODE_SPEED, 0, 50, 0, base, m.session, select_bit); tick();
  expect(base, MOTOR_OUT_REJECTED, MOTOR_R_STALE, m.lifecycle);
  send(MOTOR_MODE_SPEED, 0, 50, 0, (uint8_t)(base + 1U + 128U), m.session, select_bit); tick();
  expect((uint8_t)(base + 129U), MOTOR_OUT_REJECTED, MOTOR_R_SEQ_AMBIGUOUS, m.lifecycle);
  send(MOTOR_MODE_SPEED, 0, 50, 0, (uint8_t)(base + 1U + 17U), m.session, select_bit); tick();
  expect((uint8_t)(base + 18U), MOTOR_OUT_REJECTED, MOTOR_R_SEQ_JUMP, m.lifecycle);
  send(MOTOR_MODE_SPEED, 0, 50, 0, (uint8_t)(base + 1U + 16U), m.session, select_bit); tick();
  const struct motor_reply r = next_reply();
  CHECK_EQ(r.request_seq, (uint8_t)(base + 17U));
  CHECK(r.outcome == MOTOR_OUT_APPLIED || r.outcome == MOTOR_OUT_INHIBITED);
  CHECK_EQ(m.last_seq, (uint8_t)(base + 17U));
}

static void test_rejected_frames_do_not_feed_the_watchdog(void)
{
  run_at(100);
  for (unsigned t = 0; t < MOTOR_WATCHDOG_MS - 1U; ++t) {
    if (t % 20U == 0U) send(MOTOR_MODE_SPEED, 0, 100, 0, seq, m.session, select_bit); /* duplicates only */
    tick();
  }
  CHECK_EQ(m.lifecycle, MOTOR_LC_RUNNING);
  drain_replies();
  tick(); /* 100 ms since the last fresh frame */
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
  CHECK(acts.stop && acts.stop_cause == MOTOR_CMD_STOP_TIMEOUT);
}

static void test_watchdog_tolerates_three_lost_frames(void)
{
  run_at(100);
  ticks(79); /* frames at +20, +40, +60 lost; the one at +80 arrives */
  speed(100); tick(); drain_replies();
  ticks(99);
  CHECK_EQ(m.lifecycle, MOTOR_LC_RUNNING);
  const uint8_t old = m.session;
  tick();
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
  CHECK(m.session != old && m.session != 0U);
  CHECK_EQ(m.applied_seq, 0U);
  /* a frame of the old session can never restart the motor */
  send(MOTOR_MODE_SPEED, 0, 100, 0, ++seq, old, select_bit); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_NOT_ARMED, MOTOR_LC_DISARMED);
}

static void test_armed_but_silent_disarms(void)
{
  arm(); tick(); drain_replies(); /* ARM received at t = 0 */
  ticks(MOTOR_WATCHDOG_MS - 1U);   /* t = 1 .. 99 */
  CHECK_EQ(m.lifecycle, MOTOR_LC_READY);
  tick();                          /* t = 100: trips */
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
  CHECK(!acts.stop); /* output was already off */
}

static void test_stale_receive_time_expires(void)
{
  arm(); tick(); drain_replies();
  speed(100);
  m.inbox[(atomic_load(&m.inbox_head) - 1U) % MOTOR_CMD_INBOX].rx_ms = in.now_ms - MOTOR_WATCHDOG_MS;
  tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_TIMEOUT, MOTOR_LC_READY);
  CHECK(!acts.start);
}

static void test_disable_stops_and_keeps_the_session(void)
{
  run_at(100);
  const uint8_t s = m.session;
  disable(); tick();
  CHECK(act.stop && act.stop_cause == MOTOR_CMD_STOP_DISABLE);
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_READY);
  CHECK_EQ(m.session, s);
  settle();
  speed(100); tick(); /* resumes without a new ARM */
  CHECK(act.start);
}

static void test_disable_honoured_with_any_session_and_sequence(void)
{
  run_at(100);
  send(MOTOR_MODE_DISABLE, 0, 0, 0, (uint8_t)(seq - 50U), (uint8_t)(m.session + 7U), 0xFU); tick();
  CHECK(act.stop);
  CHECK_EQ(m.lifecycle, MOTOR_LC_READY);
}

static void test_ordering_within_one_tick(void)
{
  run_at(100);
  /* motion then DISABLE: the DISABLE wins, the motion is answered SUPERSEDED */
  speed(200); disable(); tick();
  expect((uint8_t)(seq), MOTOR_OUT_SUPERSEDED, MOTOR_R_NONE, MOTOR_LC_RUNNING);
  CHECK(act.stop && !act.set_target);
  CHECK_EQ(m.lifecycle, MOTOR_LC_READY);
  CHECK_EQ(m.setpoint, 0);
  drain_replies();
  settle(); ticks(2);
  CHECK(!acts.start);
  /* DISABLE then motion: the newer motion wins and restarts once MCSDK is idle */
  clear_acts();
  speed(100); disable(); speed(150); tick();
  drain_replies();
  CHECK_EQ(m.setpoint, 150);
  CHECK(act.start);
  /* two motion frames in one tick: the older is SUPERSEDED */
  settle(); tick(); drain_replies();
  const uint8_t first = (uint8_t)(seq + 1U);
  speed(160); speed(170); tick();
  expect(first, MOTOR_OUT_SUPERSEDED, MOTOR_R_NONE, MOTOR_LC_RUNNING);
  expect((uint8_t)(first + 1U), MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_RUNNING);
  CHECK_EQ(act.setpoint, 170);
}

static void test_mcsdk_fault_latches_until_cleared(void)
{
  run_at(100);
  in.mc = MOTOR_CMD_MC_FAULT; in.mc_fault_present = 1U;
  tick();
  CHECK_EQ(m.lifecycle, MOTOR_LC_FAULTED);
  CHECK_EQ(m.latched, MOTOR_R_FAULT_MCSDK);
  speed(100); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_FAULT_MCSDK, MOTOR_LC_FAULTED);
  arm(); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_FAULT_MCSDK, MOTOR_LC_FAULTED);
  clear_fault(); tick(); /* cause still present */
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_NOT_IDLE, MOTOR_LC_FAULTED);
  in.mc_fault_present = 0U;
  const uint8_t old = m.session;
  clear_fault(); tick();
  CHECK(act.ack_fault);
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_DISARMED);
  CHECK(m.session != old);
  clear_acts();
  ticks(MOTOR_WATCHDOG_MS * 3U); /* no automatic restart: DISARMED until a fresh ARM */
  CHECK(!acts.start);
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
}

static void test_fault_at_boot_with_output_off(void)
{
  in.mc = MOTOR_CMD_MC_FAULT; /* e.g. undervoltage latched on USB power before the battery */
  tick();
  CHECK_EQ(m.lifecycle, MOTOR_LC_FAULTED);
  CHECK(!act.stop);
  clear_fault(); tick();
  CHECK(act.ack_fault);
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
}

static void test_start_timeout_faults(void)
{
  arm(); tick(); drain_replies();
  speed(100); tick();
  CHECK(act.start);
  for (unsigned t = 0; t < MOTOR_CMD_START_TIMEOUT_MS; t += 20U) { speed(100); ticks(20); } /* never reaches RUN */
  CHECK_EQ(m.lifecycle, MOTOR_LC_FAULTED);
  CHECK_EQ(m.latched, MOTOR_R_FAULT_MCSDK);
  CHECK(acts.stop);
}

static void test_mcsdk_leaving_run_faults(void)
{
  run_at(100);
  in.mc = MOTOR_CMD_MC_IDLE;
  tick();
  CHECK_EQ(m.lifecycle, MOTOR_LC_FAULTED);
  CHECK_EQ(m.latched, MOTOR_R_FAULT_MCSDK);
}

static void test_loop_failure_faults_as_stall(void)
{
  run_at(100);
  in.loop_failed = 1U;
  tick();
  CHECK_EQ(m.lifecycle, MOTOR_LC_FAULTED);
  CHECK_EQ(m.latched, MOTOR_R_FAULT_STALL);
  CHECK(act.stop && act.stop_cause == MOTOR_CMD_STOP_LOOP_FAILED);
}

static void test_parks_at_zero_and_restarts(void)
{
  run_at(100);
  for (unsigned t = 0; t < 400U; t += 20U) {
    speed(0);
    in.rpm = 30; /* below the park threshold */
    ticks(20);
    drain_replies();
    if (m.lifecycle != MOTOR_LC_RUNNING) break;
  }
  CHECK_EQ(m.lifecycle, MOTOR_LC_READY);
  CHECK(acts.stop && acts.stop_cause == MOTOR_CMD_STOP_PARKED);
  settle();
  clear_acts();
  speed(100); tick();
  CHECK(act.start);
}

static void test_capabilities_and_limits(void)
{
  arm(); tick(); drain_replies();
  speed(2977); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_OUT_OF_RANGE, MOTOR_LC_READY);
  speed(-2977); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_OUT_OF_RANGE, MOTOR_LC_READY);
  send(MOTOR_MODE_SPEED, 0, 100, 6001, ++seq, m.session, select_bit); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_OUT_OF_RANGE, MOTOR_LC_READY);
  send(MOTOR_MODE_CURRENT, 0, 1000, 0, ++seq, m.session, select_bit); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_UNSUPPORTED, MOTOR_LC_READY);
  CHECK(!acts.start);
  /* ff is accepted but this loop does not apply it */
  send(MOTOR_MODE_SPEED, 0, 2976, 1500, ++seq, m.session, select_bit); tick();
  expect(seq, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING, MOTOR_LC_STARTING);
  settle();
  send(MOTOR_MODE_SPEED, 0, 2976, 1500, ++seq, m.session, select_bit); tick();
  expect(seq, MOTOR_OUT_INHIBITED, MOTOR_R_FF_UNUSED, MOTOR_LC_RUNNING);
}

static void test_not_aligned_and_busy(void)
{
  in.aligned = 0U;
  arm(); tick(); drain_replies();
  speed(100); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_NOT_ALIGNED, MOTOR_LC_READY);
  speed(0); tick(); /* a zero target needs no alignment */
  expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_READY);
  in.aligned = 1U; in.busy = 1U;
  speed(100); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_BUSY, MOTOR_LC_READY);
  disable(); tick(); /* DISABLE aborts an alignment or capture */
  CHECK(act.abort_busy);
  CHECK(!acts.start);
}

static void test_structural_rejects_are_answered(void)
{
  const uint8_t bad_version[8] = { 0x21, 0, 100, 0, 0, 0, 77, 0 };
  send_raw(MOTOR_CAN_ID_CONTROL | select_bit, bad_version, 8);
  const uint8_t short_frame[4] = { 0x11, 0, 100, 0 };
  send_raw(MOTOR_CAN_ID_CONTROL | select_bit, short_frame, 4);
  tick();
  expect(77, MOTOR_OUT_REJECTED, MOTOR_R_BAD_VERSION, MOTOR_LC_DISARMED);
  no_reply();
  CHECK_EQ(m.count.bad_length, 1U);
  CHECK_EQ(m.count.rejected, 1U);
  CHECK_EQ(m.count.last_reject, MOTOR_R_BAD_VERSION);
}

static void test_inbox_overflow_disarms(void)
{
  run_at(100);
  for (unsigned k = 0; k <= MOTOR_CMD_INBOX; ++k) speed(100);
  tick();
  CHECK_EQ(m.count.overflow, 1U);
  CHECK(act.stop && act.stop_cause == MOTOR_CMD_STOP_OVERFLOW);
  CHECK_EQ(m.lifecycle, MOTOR_LC_DISARMED);
}

static void test_reply_ring_drops_newest(void)
{
  arm(); tick();
  speed(0); tick(); speed(0); tick(); speed(0); tick();
  speed(0); tick(); /* fifth reply: ring of four is full */
  CHECK_EQ(m.count.replies_dropped, 1U);
  const struct motor_reply r = next_reply();
  CHECK_EQ(r.outcome, MOTOR_OUT_APPLIED); /* the oldest (ARM) is still first */
  drain_replies();
}

static void test_first_edge_and_reply_measurements(void)
{
  run_at(100);
  in.rpm = 95; in.iq_ma = 1234;
  speed(100); tick();
  struct motor_reply r = next_reply();
  CHECK(!r.first_edge);
  CHECK_EQ(r.rpm, 95);
  CHECK_EQ(r.iq_ma, 1250); /* 50 mA steps */
  in.edges_neg = (uint16_t)(in.edges_neg + 1U); /* an edge the wrong way does not count */
  speed(100); tick();
  CHECK(!next_reply().first_edge);
  in.edges_pos = (uint16_t)(in.edges_pos + 1U);
  speed(100); tick();
  CHECK(next_reply().first_edge);
  speed(-100); tick(); /* a reversal is a new start */
  CHECK(!next_reply().first_edge);
  in.edges_neg = (uint16_t)(in.edges_neg + 1U);
  speed(-100); tick();
  CHECK(next_reply().first_edge);
  disable(); tick();
  r = next_reply();
  CHECK_EQ(r.rpm, 0); /* output off: no measurements */
  CHECK_EQ(r.iq_ma, 0);
}

static void test_seq_wraps_through_255(void)
{
  seq = 250U;
  run_at(100);
  for (unsigned k = 0; k < 20U; ++k) { speed(100); tick(); expect(seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE, MOTOR_LC_RUNNING); }
}

static void test_mode_change_needs_disable(void)
{
  const struct motor_cmd_config cfg = { .select = select_bit, .max_rpm = 2976, .max_ma = 6000,
                                        .ff_supported = 1, .current_supported = 1 };
  motor_cmd_init(&m, &cfg, 99U);
  run_at(100);
  send(MOTOR_MODE_CURRENT, 0, 1000, 0, ++seq, m.session, select_bit); tick();
  expect(seq, MOTOR_OUT_REJECTED, MOTOR_R_MODE_CHANGE, MOTOR_LC_RUNNING);
  disable(); tick(); drain_replies(); settle();
  send(MOTOR_MODE_CURRENT, 0, 1000, 0, ++seq, m.session, select_bit); tick();
  CHECK(act.start && act.mode == MOTOR_MODE_CURRENT && act.setpoint == 1000);
}

static const struct { const char *name; void (*fn)(void); } cases[] = {
  { "boot_is_disarmed_and_rejects_motion", test_boot_is_disarmed_and_rejects_motion },
  { "arm_needs_the_current_session", test_arm_needs_the_current_session },
  { "start_run_and_target_updates", test_start_run_and_target_updates },
  { "sequence_rules", test_sequence_rules },
  { "rejected_frames_do_not_feed_the_watchdog", test_rejected_frames_do_not_feed_the_watchdog },
  { "watchdog_tolerates_three_lost_frames", test_watchdog_tolerates_three_lost_frames },
  { "armed_but_silent_disarms", test_armed_but_silent_disarms },
  { "stale_receive_time_expires", test_stale_receive_time_expires },
  { "disable_stops_and_keeps_the_session", test_disable_stops_and_keeps_the_session },
  { "disable_honoured_with_any_session_and_sequence", test_disable_honoured_with_any_session_and_sequence },
  { "ordering_within_one_tick", test_ordering_within_one_tick },
  { "mcsdk_fault_latches_until_cleared", test_mcsdk_fault_latches_until_cleared },
  { "fault_at_boot_with_output_off", test_fault_at_boot_with_output_off },
  { "start_timeout_faults", test_start_timeout_faults },
  { "mcsdk_leaving_run_faults", test_mcsdk_leaving_run_faults },
  { "loop_failure_faults_as_stall", test_loop_failure_faults_as_stall },
  { "parks_at_zero_and_restarts", test_parks_at_zero_and_restarts },
  { "capabilities_and_limits", test_capabilities_and_limits },
  { "not_aligned_and_busy", test_not_aligned_and_busy },
  { "structural_rejects_are_answered", test_structural_rejects_are_answered },
  { "inbox_overflow_disarms", test_inbox_overflow_disarms },
  { "reply_ring_drops_newest", test_reply_ring_drops_newest },
  { "first_edge_and_reply_measurements", test_first_edge_and_reply_measurements },
  { "seq_wraps_through_255", test_seq_wraps_through_255 },
  { "mode_change_needs_disable", test_mode_change_needs_disable },
};

int main(int argc, char **argv)
{
  static const uint32_t wheels[4] = { 0x1U, 0x2U, 0x4U, 0x8U };
  for (unsigned w = 0; w < 4U; ++w) { /* every scenario on every wheel's select bit */
    select_bit = wheels[w];
    for (size_t i = 0; i < sizeof cases / sizeof *cases; ++i) {
      if (argc > 1 && strcmp(argv[1], cases[i].name) != 0) continue;
      setup();
      cases[i].fn();
    }
  }
  return 0;
}
