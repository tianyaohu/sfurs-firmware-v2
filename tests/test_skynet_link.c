/* skynet serial command link (skynet_link.c) into the real drive mailbox (ik_drive.c). Records are built
 * the way skynet's ControlWire.cpp / ControlFormat.h build them, so this pins the wire contract. */
#include "skynet_link.h"
#include "ik_drive.h"
#include "check.h"
#include <math.h>
#include <string.h>

/* skynet: encodeScaledVelocity = lround(v * 127 / 6), clamped to +-127 */
static int8_t enc(double v)
{
  const double s = lround(v * 127.0 / 6.0);
  return (int8_t)(s > 127 ? 127 : s < -127 ? -127 : s);
}

/* One ControlWire record: CRC (host little-endian) then the packed ControlFormat. */
static void record(uint8_t out[7], unsigned id, int dribble, int chip, int kick, double vf, double vl, double vphi)
{
  uint8_t p[5];
  p[0] = (uint8_t)((id & 0x0FU) | ((dribble & 1) << 4));
  p[1] = (uint8_t)((chip & 1) | ((kick & 0x0F) << 1));
  p[2] = (uint8_t)enc(vf);
  p[3] = (uint8_t)enc(vl);
  p[4] = (uint8_t)enc(vphi);
  const uint16_t crc = skynet_link_crc16(p, 5U);
  out[0] = (uint8_t)crc;
  out[1] = (uint8_t)(crc >> 8);
  memcpy(out + 2, p, 5);
}

static void feed(const uint8_t *b, unsigned n)
{
  for (unsigned i = 0; i < n; ++i) skynet_link_isr_byte(b[i]);
}

/* A full-team batch as sendAsync() writes it; robot `id` gets the twist, the others zero. */
static void batch(unsigned team, unsigned id, double vf, double vl, double vphi)
{
  for (unsigned i = 0; i < team; ++i) {
    uint8_t r[7];
    record(r, i, i == 1U, 0, i == 2U ? 9 : 0, i == id ? vf : 0, i == id ? vl : 0, i == id ? vphi : 0);
    feed(r, 7U);
  }
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
  skynet_link_poll(1000U);
  CHECK_EQ(skynet_link_records, 6);
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_EQ(skynet_link_skipped, 0);
  /* forward -> +vy, left -> -vx; int8 steps of 6000/127 mm/s */
  CHECK_EQ(ik_drive_vy_mmps, (enc(0.5) * 6000 + 63) / 127);
  CHECK_EQ(ik_drive_vx_mmps, -((enc(0.25) * 6000 + 63) / 127));
  CHECK_EQ(ik_drive_omega_mradps, (enc(-1.0) * 6000 - 63) / 127);
  CHECK_NEAR(ik_drive_vy_mmps, 500, 6000.0 / 127 / 2 + 1);
  CHECK_NEAR(ik_drive_vx_mmps, -250, 6000.0 / 127 / 2 + 1);
  CHECK_NEAR(ik_drive_omega_mradps, -1000, 6000.0 / 127 / 2 + 1);
  CHECK_EQ(ik_drive_enable, 1);
  CHECK_EQ(ik_drive_heartbeat, 1);
  CHECK_EQ(skynet_link_active, 1);
}

static void other_robots_leave_mailbox_alone(void)
{
  skynet_link_robot_id = 7U;
  ik_drive_vx_mmps = 123;
  batch(6U, 2U, 1.0, 1.0, 1.0);
  skynet_link_poll(5U);
  CHECK_EQ(skynet_link_records, 6);
  CHECK_EQ(skynet_link_commands, 0);
  CHECK_EQ(ik_drive_vx_mmps, 123);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK_EQ(ik_drive_heartbeat, 0);
  CHECK_EQ(skynet_link_active, 0);
}

static void finds_records_after_garbage_and_splits(void)
{
  skynet_link_robot_id = 0U;
  const uint8_t junk[] = {0x00, 0xFF, 0x13, 0x37, 0x42}; /* e.g. the tail of a batch from before we booted */
  feed(junk, sizeof junk);
  skynet_link_poll(1U);
  uint8_t r[7];
  record(r, 0U, 0, 0, 0, 1.0, 0, 0);
  feed(r, 3U); /* a record split across two polls */
  skynet_link_poll(2U);
  CHECK_EQ(skynet_link_commands, 0);
  feed(r + 3, 4U);
  skynet_link_poll(3U);
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_EQ(skynet_link_skipped, sizeof junk);
  CHECK_NEAR(ik_drive_vy_mmps, 1000, 25);
  batch(6U, 0U, -0.3, 0, 0); /* aligned now: no further skips */
  skynet_link_poll(4U);
  CHECK_EQ(skynet_link_skipped, sizeof junk);
  CHECK_EQ(skynet_link_commands, 2);
  CHECK_NEAR(ik_drive_vy_mmps, -300, 25);
}

static void corrupt_record_is_dropped(void)
{
  skynet_link_robot_id = 1U;
  uint8_t bad[7], good[7];
  record(bad, 1U, 0, 0, 0, 2.0, 0, 0);
  bad[4] ^= 0x10; /* flipped bit in forward velocity */
  record(good, 1U, 0, 0, 0, 0.2, 0, 0);
  feed(bad, 7U);
  feed(good, 7U);
  skynet_link_poll(10U);
  CHECK_EQ(skynet_link_commands, 1);
  CHECK_NEAR(ik_drive_vy_mmps, 200, 25);
  CHECK(skynet_link_skipped >= 7U);
}

static void reserved_bits_reject_record(void)
{
  skynet_link_robot_id = 0U;
  uint8_t p[5] = {0x20, 0, 40, 0, 0}; /* bit 5 of byte 0 set: not a ControlFormat */
  const uint16_t crc = skynet_link_crc16(p, 5U);
  const uint8_t r[7] = {(uint8_t)crc, (uint8_t)(crc >> 8), p[0], p[1], p[2], p[3], p[4]};
  feed(r, 7U);
  skynet_link_poll(1U);
  CHECK_EQ(skynet_link_records, 0);
  CHECK_EQ(ik_drive_enable, 0);
}

static void full_scale_is_clamped_to_drive_limits(void)
{
  skynet_link_robot_id = 3U;
  batch(4U, 3U, 9.0, -9.0, 9.0); /* skynet clamps to +-127 = 6 m/s, rad/s */
  skynet_link_poll(1U);
  CHECK_EQ(ik_drive_vy_mmps, IK_DRIVE_MAX_LINEAR_MMPS);
  CHECK_EQ(ik_drive_vx_mmps, IK_DRIVE_MAX_LINEAR_MMPS); /* left -6 m/s = right +6 */
  CHECK_EQ(ik_drive_omega_mradps, IK_DRIVE_MAX_OMEGA_MRADPS);
}

static void silence_stops_and_restart_drives(void)
{
  skynet_link_robot_id = 0U;
  batch(1U, 0U, 0.4, 0, 0);
  skynet_link_poll(100U);
  CHECK_EQ(ik_drive_enable, 1);
  skynet_link_poll(100U + SKYNET_LINK_TIMEOUT_MS - 1U);
  CHECK_EQ(skynet_link_active, 1);
  CHECK(ik_drive_vy_mmps != 0);
  ik_drive_fault = 0x11U; /* a latched fault is acknowledged by the enable = 0 below */
  skynet_link_poll(100U + SKYNET_LINK_TIMEOUT_MS);
  CHECK_EQ(skynet_link_active, 0);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK_EQ(ik_drive_vx_mmps, 0);
  CHECK_EQ(ik_drive_vy_mmps, 0);
  CHECK_EQ(ik_drive_omega_mradps, 0);
  const uint32_t beats = ik_drive_heartbeat;
  skynet_link_poll(5000U); /* idle link: nothing more written */
  CHECK_EQ(ik_drive_heartbeat, beats);
  batch(1U, 0U, 0.4, 0, 0);
  skynet_link_poll(5001U);
  CHECK_EQ(ik_drive_enable, 1);
  CHECK_EQ(skynet_link_active, 1);
  CHECK_EQ(ik_drive_heartbeat, beats + 1U);
}

static void ring_overflow_is_counted(void)
{
  skynet_link_robot_id = 0U;
  for (unsigned i = 0; i < SKYNET_LINK_RING + 10U; ++i) skynet_link_isr_byte(0xAA);
  CHECK_EQ(skynet_link_overruns, 10);
  skynet_link_poll(1U);
  CHECK_EQ(skynet_link_bytes, SKYNET_LINK_RING);
  batch(1U, 0U, 0.5, 0, 0); /* the ring is usable again and the decoder resyncs */
  skynet_link_poll(2U);
  CHECK_EQ(skynet_link_commands, 1);
  skynet_link_isr_overrun();
  CHECK_EQ(skynet_link_overruns, 11);
}

/* ik_drive sees the link's heartbeat like the SWD host's. With no ESC answering, every wheel goes LOST:
 * the fault stays latched while skynet keeps streaming, and only a pause in the stream acknowledges it. */
static int sent;
static int send(uint32_t id, const uint8_t *d, uint32_t len) { (void)id; (void)d; (void)len; ++sent; return 1; }

static void step(uint32_t t, int stream)
{
  if (stream) batch(6U, 0U, 0.3, 0, 0);
  skynet_link_poll(t);
  ik_drive_tick(t, t * 1000U, send);
}

static void link_heartbeat_starts_drive_session(void)
{
  skynet_link_robot_id = 0U;
  step(0U, 1);
  CHECK_EQ(ik_drive_state, IK_DRIVING);
  CHECK(sent > 0);
  uint32_t t = 10U;
  for (; t < 1000U; t += 10U) step(t, 1);
  CHECK_EQ(ik_drive_state, IK_STOPPED);
  CHECK_EQ(ik_drive_fault & 0xF0U, 0xF0U); /* all four LOST */
  CHECK_EQ(ik_drive_enable, 1);
  const uint32_t end = t + SKYNET_LINK_TIMEOUT_MS + 10U;
  for (; t < end; t += 10U) step(t, 0);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK_EQ(ik_drive_fault, 0); /* acknowledged by the pause */
  step(t, 1);
  CHECK_EQ(ik_drive_state, IK_DRIVING); /* a restarted stream starts a new session */
}

int main(int argc, char **argv)
{
  static const struct { const char *name; void (*fn)(void); } cases[] = {
    {"crc_matches_ccitt_false", crc_matches_ccitt_false},
    {"batch_drives_this_robot", batch_drives_this_robot},
    {"other_robots_leave_mailbox_alone", other_robots_leave_mailbox_alone},
    {"finds_records_after_garbage_and_splits", finds_records_after_garbage_and_splits},
    {"corrupt_record_is_dropped", corrupt_record_is_dropped},
    {"reserved_bits_reject_record", reserved_bits_reject_record},
    {"full_scale_is_clamped_to_drive_limits", full_scale_is_clamped_to_drive_limits},
    {"silence_stops_and_restart_drives", silence_stops_and_restart_drives},
    {"ring_overflow_is_counted", ring_overflow_is_counted},
    {"link_heartbeat_starts_drive_session", link_heartbeat_starts_drive_session},
  };
  CHECK(argc == 2);
  for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i)
    if (strcmp(argv[1], cases[i].name) == 0) { cases[i].fn(); printf("ok %s\n", argv[1]); return 0; }
  printf("unknown case %s\n", argv[1]);
  return 1;
}
