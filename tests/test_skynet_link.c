/* skynet serial command link (skynet_link.c) into the real drive mailbox (ik_drive.c). Records are built
 * the way skynet's ControlWire.cpp / ControlFormat.h build them, so this pins the wire contract. */
#include "skynet_link.h"
#include "ik_drive.h"
#include "check.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint32_t t = 1000U; /* Nucleo ms: byte stamps and poll time */

/* skynet: encodeScaledVelocity = lround(v * 127 / 6), clamped to +-127 */
static int8_t enc(double v)
{
  const double s = lround(v * 127.0 / 6.0);
  return (int8_t)(s > 127 ? 127 : s < -127 ? -127 : s);
}

static int32_t milli(int8_t v) { return (v * 6000 + (v >= 0 ? 63 : -63)) / 127; }
static int32_t clamp_i(int32_t v, int32_t lim) { return v > lim ? lim : v < -lim ? -lim : v; }

/* One ControlWire record from raw int8 velocities: CRC (host little-endian) then the packed ControlFormat. */
static void record_raw(uint8_t out[7], unsigned id, int dribble, int kick, int8_t vf, int8_t vl, int8_t vphi)
{
  uint8_t p[5] = {(uint8_t)((id & 0x0FU) | ((dribble & 1) << 4)), (uint8_t)((kick & 0x0F) << 1),
                  (uint8_t)vf, (uint8_t)vl, (uint8_t)vphi};
  const uint16_t crc = skynet_link_crc16(p, 5U);
  out[0] = (uint8_t)crc;
  out[1] = (uint8_t)(crc >> 8);
  memcpy(out + 2, p, 5);
}

static void feed(const uint8_t *b, unsigned n)
{
  for (unsigned i = 0; i < n; ++i) skynet_link_isr_byte(b[i], t);
}

/* A full-team batch as sendAsync() writes it; robot `id` gets the twist, the others zero. */
static void batch(unsigned team, unsigned id, double vf, double vl, double vphi)
{
  for (unsigned i = 0; i < team; ++i) {
    uint8_t r[7];
    record_raw(r, i, i == 1U, i == 2U ? 9 : 0, i == id ? enc(vf) : 0, i == id ? enc(vl) : 0,
               i == id ? enc(vphi) : 0);
    feed(r, 7U);
  }
}

static void poll(void) { skynet_link_poll(t, 0); }

/* Let the batch close on silence, as after skynet's last send. */
static void settle(void)
{
  poll();
  t += SKYNET_LINK_GAP_MS;
  poll();
}

static void reset_counters(void)
{
  skynet_link_bytes = skynet_link_records = skynet_link_batches = skynet_link_commands = 0U;
  skynet_link_skipped = skynet_link_broken = skynet_link_stale = skynet_link_refused = 0U;
}

/* The first batch of a run only tells the link the team size; it is not applied. */
static void prime(unsigned team)
{
  batch(team, 15U, 0, 0, 0);
  settle();
  CHECK_EQ(skynet_link_commands, 0);
  CHECK_EQ(ik_drive_heartbeat, 0);
  reset_counters();
}

static void crc_matches_ccitt_false(void)
{
  CHECK_EQ(skynet_link_crc16((const uint8_t *)"123456789", 9U), 0x29B1); /* catalogue check value */
  CHECK_EQ(skynet_link_abi, SKYNET_LINK_ABI);
}

static void batch_drives_this_robot(void)
{
  skynet_link_robot_id = 2U;
  batch(6U, 2U, 0.5, 0.25, -1.0);
  settle();
  CHECK_EQ(skynet_link_commands, 0); /* first batch: team size only */
  batch(6U, 2U, 0.5, 0.25, -1.0);
  poll();
  CHECK_EQ(skynet_link_records, 12);
  CHECK_EQ(skynet_link_commands, 0); /* open until framed by silence or the next batch */
  t += SKYNET_LINK_GAP_MS;
  poll();
  CHECK_EQ(skynet_link_batches, 1);
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_EQ(skynet_link_skipped, 0);
  CHECK_EQ(ik_drive_vy_mmps, milli(enc(0.5)));    /* forward -> +vy */
  CHECK_EQ(ik_drive_vx_mmps, -milli(enc(0.25)));  /* left -> -vx */
  CHECK_EQ(ik_drive_omega_mradps, milli(enc(-1.0)));
  CHECK_NEAR(ik_drive_vy_mmps, 500, 6000.0 / 127 / 2 + 1);
  CHECK_NEAR(ik_drive_vx_mmps, -250, 6000.0 / 127 / 2 + 1);
  CHECK_NEAR(ik_drive_omega_mradps, -1000, 6000.0 / 127 / 2 + 1);
  CHECK_EQ(ik_drive_enable, 1);
  CHECK_EQ(ik_drive_heartbeat, 1);
  CHECK_EQ(skynet_link_active, 1);
  /* back-to-back batches: each next ID 0 frames the previous batch immediately */
  t += 5U;
  batch(6U, 2U, -0.5, 0, 0);
  t += 5U;
  batch(6U, 2U, 0, 0, 0);
  poll();
  CHECK_EQ(skynet_link_commands, 2);
  CHECK_EQ(ik_drive_vy_mmps, milli(enc(-0.5)));
}

static void other_robots_leave_mailbox_alone(void)
{
  skynet_link_robot_id = 7U;
  prime(6U);
  ik_drive_vx_mmps = 123;
  batch(6U, 2U, 1.0, 1.0, 1.0);
  settle();
  CHECK_EQ(skynet_link_batches, 1);
  CHECK_EQ(skynet_link_commands, 0);
  CHECK_EQ(ik_drive_vx_mmps, 123);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK_EQ(ik_drive_heartbeat, 0);
  CHECK_EQ(skynet_link_active, 0);
}

static void finds_batches_after_garbage_and_splits(void)
{
  skynet_link_robot_id = 1U;
  const uint8_t junk[] = {0x00, 0xFF, 0x13, 0x37, 0x42}; /* e.g. the tail of a batch from before we booted */
  feed(junk, sizeof junk);
  uint8_t r0[7], r1[7];
  record_raw(r0, 0U, 0, 0, 0, 0, 0);
  record_raw(r1, 1U, 0, 0, enc(1.0), 0, 0);
  feed(r0, 7U);
  feed(r1, 7U);
  settle(); /* team size 2 learnt */
  CHECK_EQ(skynet_link_skipped, sizeof junk);
  feed(r0, 7U);
  feed(r1, 3U); /* a record split across two polls */
  poll();
  feed(r1 + 3, 4U);
  settle();
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_NEAR(ik_drive_vy_mmps, 1000, 25);
}

static void corrupt_record_discards_its_batch(void)
{
  skynet_link_robot_id = 1U;
  prime(6U);
  uint8_t r[7];
  for (unsigned i = 0; i < 6U; ++i) { /* robot 1's record is valid, robot 3's is corrupt */
    record_raw(r, i, 0, 0, i == 1U ? enc(2.0) : 0, 0, 0);
    if (i == 3U) r[4] ^= 0x10U;
    feed(r, 7U);
  }
  settle();
  CHECK_EQ(skynet_link_commands, 0);
  CHECK(skynet_link_broken >= 1U);
  CHECK_EQ(ik_drive_enable, 0);
  batch(6U, 1U, 0.2, 0, 0); /* damage costs the team size: the next clean batch relearns it */
  settle();
  batch(6U, 1U, 0.2, 0, 0);
  settle();
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_NEAR(ik_drive_vy_mmps, 200, 25);
}

static void ids_out_of_order_discard_batch(void)
{
  skynet_link_robot_id = 2U;
  prime(3U);
  uint8_t r[7];
  const unsigned ids[] = {0U, 2U, 3U}; /* robot 1 missing: not a ControlWire batch */
  for (unsigned i = 0; i < 3U; ++i) {
    record_raw(r, ids[i], 0, 0, enc(1.0), 0, 0);
    feed(r, 7U);
  }
  settle();
  CHECK_EQ(skynet_link_records, 3);
  CHECK_EQ(skynet_link_commands, 0);
  CHECK(skynet_link_broken >= 1U);
}

/* A lost byte can leave a CRC-valid "ID 0" record made of two neighbours' bytes at a batch's tail; silence
 * then closes it as a one-robot batch. It is not as long as the team, so it is not applied. */
static void truncated_tail_is_not_a_batch(void)
{
  skynet_link_robot_id = 0U;
  prime(6U);
  uint8_t r[7];
  for (unsigned id = 0; id < 4U; ++id) { record_raw(r, id, 0, 0, 0, 0, 0); feed(r, 7U); }
  const uint8_t tail[] = {0x98, 0x04, 0x00, 0x00, 0x00, 0x00, 0x99, 0x1a}; /* ID 4 lost a byte, ID 5 its payload */
  CHECK_EQ(skynet_link_crc16(tail + 3, 5U), (uint16_t)(tail[1] | (tail[2] << 8))); /* the false record is valid */
  feed(tail, sizeof tail);
  settle();
  CHECK_EQ(skynet_link_commands, 0);
  CHECK(ik_drive_vx_mmps == 0 && ik_drive_vy_mmps == 0 && ik_drive_omega_mradps == 0);
}

static void reserved_bits_reject_record(void)
{
  skynet_link_robot_id = 0U;
  uint8_t p[5] = {0x20, 0, 40, 0, 0}; /* bit 5 of byte 0 set: not a ControlFormat */
  const uint16_t crc = skynet_link_crc16(p, 5U);
  const uint8_t r[7] = {(uint8_t)crc, (uint8_t)(crc >> 8), p[0], p[1], p[2], p[3], p[4]};
  feed(r, 7U);
  settle();
  feed(r, 7U);
  settle();
  CHECK_EQ(skynet_link_records, 0);
  CHECK_EQ(ik_drive_enable, 0);
}

static void full_scale_is_clamped_to_drive_limits(void)
{
  skynet_link_robot_id = 3U;
  prime(4U);
  batch(4U, 3U, 9.0, -9.0, 9.0); /* skynet clamps to +-127 = 6 m/s, rad/s */
  settle();
  CHECK_EQ(ik_drive_vy_mmps, IK_DRIVE_MAX_LINEAR_MMPS);
  CHECK_EQ(ik_drive_vx_mmps, IK_DRIVE_MAX_LINEAR_MMPS); /* left -6 m/s = right +6 */
  CHECK_EQ(ik_drive_omega_mradps, IK_DRIVE_MAX_OMEGA_MRADPS);
}

static void silence_stops_and_restart_drives(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  batch(1U, 0U, 0.4, 0, 0);
  const uint32_t sent = t;
  settle();
  CHECK_EQ(skynet_link_last_ms, sent); /* the command's arrival, not when it was applied */
  CHECK_EQ(ik_drive_enable, 1);
  t = sent + SKYNET_LINK_TIMEOUT_MS - 1U;
  poll();
  CHECK_EQ(skynet_link_active, 1);
  CHECK(ik_drive_vy_mmps != 0);
  t = sent + SKYNET_LINK_TIMEOUT_MS;
  poll();
  CHECK_EQ(skynet_link_active, 0);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK_EQ(ik_drive_vx_mmps, 0);
  CHECK_EQ(ik_drive_vy_mmps, 0);
  CHECK_EQ(ik_drive_omega_mradps, 0);
  const uint32_t beats = ik_drive_heartbeat;
  t += 5000U;
  poll(); /* idle link: nothing more written */
  CHECK_EQ(ik_drive_heartbeat, beats);
  batch(1U, 0U, 0.4, 0, 0);
  settle();
  CHECK_EQ(ik_drive_enable, 1);
  CHECK_EQ(skynet_link_active, 1);
  CHECK_EQ(ik_drive_heartbeat, beats + 1U);
}

static void stale_bytes_are_dropped(void)
{
  skynet_link_robot_id = 0U;
  prime(2U);
  batch(2U, 0U, 1.0, 0, 0); /* queued, but the main loop does not run for a second */
  t += 1000U;
  settle();
  CHECK_EQ(skynet_link_stale, 14);
  CHECK_EQ(skynet_link_commands, 0);
  CHECK_EQ(ik_drive_enable, 0);
}

static void decoded_command_does_not_outlive_a_stall(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  batch(1U, 0U, 1.0, 0, 0);
  poll();                  /* decoded, batch still open */
  t += 1000U;              /* main loop stalls before silence can close it */
  poll();
  CHECK_EQ(skynet_link_commands, 0);
  CHECK_EQ(skynet_link_stale, 1);
  CHECK_EQ(ik_drive_enable, 0);
}

static void record_cut_by_silence_is_not_completed(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  uint8_t old[7], fresh[7];
  record_raw(old, 0U, 0, 0, enc(2.0), 0, 0);
  feed(old, 6U);                      /* last byte never arrives */
  poll();
  t += 1000U;
  poll();
  record_raw(fresh, 0U, 0, 0, 0, 0, 0); /* a halt, whose first byte would complete the old record */
  feed(fresh, 7U);
  settle();
  feed(fresh, 7U);
  settle();
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_EQ(ik_drive_vy_mmps, 0);
}

static void ring_overflow_is_counted(void)
{
  skynet_link_robot_id = 0U;
  for (unsigned i = 0; i < SKYNET_LINK_RING + 10U; ++i) skynet_link_isr_byte(0xAA, t);
  CHECK_EQ(skynet_link_overruns, 10);
  poll();
  CHECK_EQ(skynet_link_bytes, SKYNET_LINK_RING);
  t += 50U;
  prime(1U); /* the ring is usable again and the decoder resyncs */
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_commands, 1);
  skynet_link_isr_overrun();
  CHECK_EQ(skynet_link_overruns, 11);
}

static void overrun_discards_open_batch(void)
{
  skynet_link_robot_id = 0U;
  prime(2U);
  uint8_t r[7];
  record_raw(r, 0U, 0, 0, enc(1.0), 0, 0);
  feed(r, 7U);
  poll();
  skynet_link_isr_overrun(); /* the UART lost bytes after robot 0's record */
  record_raw(r, 1U, 0, 0, 0, 0, 0);
  feed(r, 7U);
  settle();
  CHECK_EQ(skynet_link_commands, 0);
}

static void refuses_current_mode_and_busy_drive(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  ik_drive_mode = IK_MODE_CURRENT; /* a diagnostic's session left behind */
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_refused, 1);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK_EQ(ik_drive_heartbeat, 0);
  ik_drive_mode = IK_MODE_SPEED;
  ik_drive_state = IK_DRIVING; /* another host is driving */
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_refused, 2);
  ik_drive_state = IK_STOPPED;
  ik_drive_enable = 1U; /* another host has enabled a session that has not started yet */
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_refused, 3);
  CHECK_EQ(ik_drive_heartbeat, 0);
  ik_drive_enable = 0U;
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_commands, 1);
}

static void another_host_writing_takes_over(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_active, 1);
  ik_drive_enable = 0U; /* an SWD tool stops the robot */
  poll();
  CHECK_EQ(skynet_link_fault, SKYNET_LINK_FAULT_EXTERNAL);
  CHECK_EQ(skynet_link_active, 0);
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(ik_drive_enable, 0); /* skynet does not undo the stop */
  ++skynet_link_ack_seq;
  poll();
  CHECK_EQ(skynet_link_fault, 0);
  CHECK_EQ(ik_drive_enable, 0); /* an external stop's acknowledge does not touch the mailbox */
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(ik_drive_enable, 1);
  ik_drive_vy_mmps = 77; /* a twist written by someone else also counts */
  poll();
  CHECK_EQ(skynet_link_fault, SKYNET_LINK_FAULT_EXTERNAL);
  CHECK_EQ(ik_drive_vy_mmps, 77);
}

static void other_writer_seen_before_next_batch(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  ik_drive_enable = 0U; /* a host stops the robot ... */
  ++ik_drive_heartbeat;
  batch(1U, 0U, 1.0, 0, 0); /* ... and skynet's next batch is decoded in the same poll */
  settle();
  CHECK_EQ(skynet_link_fault, SKYNET_LINK_FAULT_EXTERNAL);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK(ik_drive_vy_mmps != milli(enc(1.0)));
}

static void tick_wrap_and_early_stamps(void)
{
  skynet_link_robot_id = 0U;
  t = 0xFFFFFFF0U; /* HAL_GetTick wraps after 49.7 days */
  prime(1U);
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK(t < 0x100U);
  CHECK_EQ(skynet_link_commands, 1);
  t += 5000U;
  poll();
  CHECK_EQ(ik_drive_enable, 0); /* the timeout across the wrap */
  uint8_t r[7];
  record_raw(r, 0U, 0, 0, enc(0.5), 0, 0);
  for (unsigned i = 0; i < 7U; ++i) skynet_link_isr_byte(r[i], t + 1U); /* ISR ran after `now` was read */
  skynet_link_poll(t, 0);
  t += SKYNET_LINK_GAP_MS + 1U;
  poll();
  CHECK_EQ(skynet_link_commands, 2);
}

/* The link behind the real ik_drive with no ESC answering: every wheel goes LOST. */
static int sent;
static int send(uint32_t id, const uint8_t *d, uint32_t len) { (void)id; (void)d; (void)len; ++sent; return 1; }

static void step(int stream, int button)
{
  if (stream) batch(6U, 0U, 0.3, 0, 0);
  skynet_link_poll(t, button);
  ik_drive_tick(t, t * 1000U, send);
  t += 10U;
}

static void press(void)
{
  for (unsigned i = 0; i < 3U; ++i) step(1, 0); /* stable released */
  for (unsigned i = 0; i < 3U; ++i) step(1, 1); /* stable pressed: one acknowledge */
}

static void fault_latches_until_acknowledged(void)
{
  skynet_link_robot_id = 0U;
  for (unsigned i = 0; i < 3U; ++i) step(1, 0);
  CHECK_EQ(ik_drive_state, IK_DRIVING);
  CHECK(sent > 0);
  for (uint32_t end = t + 1000U; t < end;) step(1, 0);
  CHECK_EQ(ik_drive_state, IK_STOPPED);
  CHECK_EQ(ik_drive_fault & 0xF0U, 0xF0U); /* all four LOST */
  CHECK_EQ(skynet_link_fault & 0xF0U, 0xF0U);
  CHECK_EQ(skynet_link_active, 0);
  const uint32_t applied = skynet_link_commands;
  for (uint32_t end = t + 2U * SKYNET_LINK_TIMEOUT_MS; t < end;) step(0, 0); /* silence: no auto-acknowledge */
  CHECK_EQ(ik_drive_enable, 1);
  CHECK_EQ(ik_drive_fault & 0xF0U, 0xF0U);
  for (uint32_t end = t + 500U; t < end;) step(1, 0); /* reconnect: still stopped */
  CHECK_EQ(skynet_link_commands, applied);
  CHECK_EQ(ik_drive_state, IK_STOPPED);
  const uint32_t clears = ik_drive_clear_seq;
  press();
  CHECK_EQ(skynet_link_fault, 0);
  CHECK_EQ(ik_drive_clear_seq, clears + 1U);
  for (unsigned i = 0; i < 5U; ++i) step(1, 1); /* held button is one acknowledge */
  CHECK_EQ(ik_drive_clear_seq, clears + 1U);
  CHECK_EQ(ik_drive_state, IK_DRIVING);
  CHECK(skynet_link_commands > applied);
  while (ik_drive_fault == 0U) step(1, 0); /* LOST again: the SWD acknowledge works too */
  step(1, 0);
  CHECK(skynet_link_fault != 0U);
  ++skynet_link_ack_seq;
  step(1, 0);
  CHECK_EQ(skynet_link_fault, 0);
  step(1, 0);
  step(1, 0);
  CHECK_EQ(ik_drive_state, IK_DRIVING);
}

static void acknowledge_before_fault_does_not_clear_it(void)
{
  skynet_link_robot_id = 0U;
  for (unsigned i = 0; i < 3U; ++i) step(1, 0);
  CHECK_EQ(ik_drive_state, IK_DRIVING);
  /* the acknowledge lands between a poll and the tick that finds the fault */
  while (ik_drive_fault == 0U) {
    batch(6U, 0U, 0.3, 0, 0);
    skynet_link_poll(t, 0);
    ik_drive_tick(t, t * 1000U, send);
    if (ik_drive_fault != 0U) ++skynet_link_ack_seq;
    t += 10U;
  }
  step(1, 0);
  CHECK(skynet_link_fault != 0U);
  for (unsigned i = 0; i < 10U; ++i) step(1, 0);
  CHECK(skynet_link_fault != 0U);
  CHECK_EQ(ik_drive_state, IK_STOPPED);
}

static void button_bounce_is_one_acknowledge(void)
{
  skynet_link_robot_id = 0U;
  for (uint32_t end = t + 1000U; t < end;) step(1, 0); /* LOST fault */
  CHECK(skynet_link_fault != 0U);
  press();
  CHECK_EQ(skynet_link_fault, 0);
  const uint32_t clears = ik_drive_clear_seq;
  while (skynet_link_fault == 0U) step(1, 1); /* still held while the drive faults again */
  /* contact bounce: 1 ms low, 1 ms high, then held */
  skynet_link_poll(t, 0); ++t;
  skynet_link_poll(t, 1); ++t;
  for (unsigned i = 0; i < 5U; ++i) step(1, 1);
  CHECK(skynet_link_fault != 0U);
  CHECK_EQ(ik_drive_clear_seq, clears);
}

static void resume_during_stop_keeps_driving(void)
{
  skynet_link_robot_id = 0U;
  prime(1U);
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  ik_drive_state = IK_DRIVING; /* as ik_drive reports while the link drives */
  t += SKYNET_LINK_TIMEOUT_MS;
  poll();
  CHECK_EQ(ik_drive_enable, 0);
  ik_drive_state = IK_STOPPING; /* decelerating and holding */
  batch(1U, 0U, 0.5, 0, 0);
  settle();
  CHECK_EQ(skynet_link_refused, 0);
  CHECK_EQ(ik_drive_enable, 1);
  CHECK(ik_drive_vy_mmps != 0);
}

/* Random team batches with dropped, inserted and flipped bytes: whatever the stream, every command the link
 * applies must be one that skynet actually sent for this robot. */
static void damaged_streams_apply_only_sent_commands(void)
{
  srand(82);
  skynet_link_robot_id = 2U;
  enum { HISTORY = 4096 };
  static int8_t sent_cmd[HISTORY][3];
  unsigned nsent = 0, damaged = 0, clean = 0;
  int32_t last_vy = 0x7FFFFFFF, last_vx = 0, last_w = 0;
  uint32_t last_commands = 0;
  unsigned team = 6U;
  for (unsigned b = 0; b < 8000U; ++b) {
    if (b % 500U == 0U) team = 1U + (unsigned)rand() % 16U; /* skynet's team size changes rarely */
    uint8_t stream[16 * 7 + 1];
    unsigned n = 0;
    for (unsigned i = 0; i < team; ++i) {
      const int8_t v[3] = {(int8_t)(rand() % 255 - 127), (int8_t)(rand() % 255 - 127), (int8_t)(rand() % 255 - 127)};
      if (i == skynet_link_robot_id) { memcpy(sent_cmd[nsent % HISTORY], v, 3); ++nsent; }
      record_raw(stream + n, i, rand() & 1, rand() % 16, v[0], v[1], v[2]);
      n += 7U;
    }
    const int kind = rand() % 4; /* 0: clean, 1: drop, 2: insert, 3: flip */
    const unsigned at = (unsigned)rand() % n;
    if (kind == 1) { memmove(stream + at, stream + at + 1, n - at - 1); --n; ++damaged; }
    if (kind == 2) { memmove(stream + at + 1, stream + at, n - at); stream[at] = (uint8_t)rand(); ++n; ++damaged; }
    if (kind == 3) { stream[at] ^= (uint8_t)(1U << (rand() % 8)); ++damaged; }
    if (kind == 0) ++clean;
    feed(stream, n);
    t += (rand() % 3 == 0) ? SKYNET_LINK_GAP_MS + 1U : 5U; /* mix back-to-back and silence-framed batches */
    skynet_link_poll(t, 0);
    if (skynet_link_commands != last_commands) {
      last_commands = skynet_link_commands;
      if (ik_drive_vy_mmps == last_vy && ik_drive_vx_mmps == last_vx && ik_drive_omega_mradps == last_w) continue;
      int found = 0;
      for (unsigned k = 0; k < 8U && k < nsent && !found; ++k) { /* one of the last few sent */
        const int8_t *c = sent_cmd[(nsent - 1U - k) % HISTORY];
        found = ik_drive_vy_mmps == clamp_i(milli(c[0]), IK_DRIVE_MAX_LINEAR_MMPS) &&
                ik_drive_vx_mmps == clamp_i(-milli(c[1]), IK_DRIVE_MAX_LINEAR_MMPS) &&
                ik_drive_omega_mradps == clamp_i(milli(c[2]), IK_DRIVE_MAX_OMEGA_MRADPS);
      }
      CHECK(found);
      last_vy = ik_drive_vy_mmps; last_vx = ik_drive_vx_mmps; last_w = ik_drive_omega_mradps;
    }
  }
  printf("fuzz: 8000 batches (%u damaged, %u clean), %u commands applied, %u broken, %u skipped\n", damaged,
         clean, (unsigned)skynet_link_commands, (unsigned)skynet_link_broken, (unsigned)skynet_link_skipped);
  /* Clean batches after a damaged one only relearn the team size; with one in four clean, about a third of
   * the clean batches follow another clean one. */
  CHECK(skynet_link_commands * 10U >= clean * 2U);
}

int main(int argc, char **argv)
{
  static const struct { const char *name; void (*fn)(void); } cases[] = {
    {"crc_matches_ccitt_false", crc_matches_ccitt_false},
    {"batch_drives_this_robot", batch_drives_this_robot},
    {"other_robots_leave_mailbox_alone", other_robots_leave_mailbox_alone},
    {"finds_batches_after_garbage_and_splits", finds_batches_after_garbage_and_splits},
    {"corrupt_record_discards_its_batch", corrupt_record_discards_its_batch},
    {"ids_out_of_order_discard_batch", ids_out_of_order_discard_batch},
    {"truncated_tail_is_not_a_batch", truncated_tail_is_not_a_batch},
    {"reserved_bits_reject_record", reserved_bits_reject_record},
    {"full_scale_is_clamped_to_drive_limits", full_scale_is_clamped_to_drive_limits},
    {"silence_stops_and_restart_drives", silence_stops_and_restart_drives},
    {"stale_bytes_are_dropped", stale_bytes_are_dropped},
    {"decoded_command_does_not_outlive_a_stall", decoded_command_does_not_outlive_a_stall},
    {"record_cut_by_silence_is_not_completed", record_cut_by_silence_is_not_completed},
    {"ring_overflow_is_counted", ring_overflow_is_counted},
    {"overrun_discards_open_batch", overrun_discards_open_batch},
    {"refuses_current_mode_and_busy_drive", refuses_current_mode_and_busy_drive},
    {"another_host_writing_takes_over", another_host_writing_takes_over},
    {"other_writer_seen_before_next_batch", other_writer_seen_before_next_batch},
    {"tick_wrap_and_early_stamps", tick_wrap_and_early_stamps},
    {"fault_latches_until_acknowledged", fault_latches_until_acknowledged},
    {"acknowledge_before_fault_does_not_clear_it", acknowledge_before_fault_does_not_clear_it},
    {"button_bounce_is_one_acknowledge", button_bounce_is_one_acknowledge},
    {"resume_during_stop_keeps_driving", resume_during_stop_keeps_driving},
    {"damaged_streams_apply_only_sent_commands", damaged_streams_apply_only_sent_commands},
  };
  CHECK(argc == 2);
  for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i)
    if (strcmp(argv[1], cases[i].name) == 0) { cases[i].fn(); printf("ok %s\n", argv[1]); return 0; }
  printf("unknown case %s\n", argv[1]);
  return 1;
}
