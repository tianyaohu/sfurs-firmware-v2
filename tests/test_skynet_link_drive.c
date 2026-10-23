/* skynet_link.c against the whole drive: the real ik_drive.c and four real ESC command lifecycles
 * (motor_cmd.c) from test_ik_drive.c's fixture. Each case is an attack found in review of the first link:
 * forged commands from a dropped byte, a stale or cut record, an inherited current-mode session, and a
 * fault acknowledged by mere silence. */
#define main ik_drive_fixture_main
#include "test_ik_drive.c"
#undef main
#include "skynet_link.h"

static void feed_bytes(const uint8_t *r, unsigned len)
{
  for (unsigned i = 0; i < len; ++i) skynet_link_isr_byte(r[i], now);
}

static void wire(unsigned id, int vf, int vl, int omega)
{
  uint8_t r[7] = {0, 0, (uint8_t)id, 0, (uint8_t)vf, (uint8_t)vl, (uint8_t)omega};
  const uint16_t crc = skynet_link_crc16(r + 2, 5);
  r[0] = (uint8_t)crc;
  r[1] = (uint8_t)(crc >> 8);
  feed_bytes(r, 7);
}

static void tick_link(int button)
{
  skynet_link_poll(now, button);
  ik_drive_tick(now, now * 1000U, send);
  for (unsigned w = 0; w < 4; w++) esc_step(w);
  ++now;
}

/* A one-robot team at skynet's ~100 Hz: every record is its own batch. */
static void stream(unsigned ms, int vf)
{
  for (const unsigned end = now + ms; now < end;) {
    if (now % 10 == 0) wire(0, vf, 0, 0);
    tick_link(0);
  }
}

static void all_running(void)
{
  for (unsigned w = 0; w < 4; w++) CHECK(esc[w].lifecycle == MOTOR_LC_RUNNING);
}

static void fault_waits_for_operator(unsigned target)
{
  stream(1000, 21);
  all_running();
  drop_replies[target] = 1;
  stream(100, 21);
  CHECK(ik_drive_fault & (1U << (4U + target)));
  drop_replies[target] = 0;
  stream(100, 21);
  CHECK(ik_drive_fault & (1U << (4U + target)));
  for (const unsigned end = now + 310; now < end;) tick_link(0); /* silence */
  stream(1000, 21);                                            /* skynet reconnects */
  CHECK(ik_drive_fault & (1U << (4U + target)));
  CHECK(skynet_link_fault & (1U << (4U + target)));
  CHECK_EQ(ik_drive_state, IK_STOPPED);
  tick_link(1); /* operator presses B1 */
  CHECK_EQ(skynet_link_fault, 0);
  stream(1000, 21);
  CHECK_EQ(ik_drive_fault, 0);
  all_running();
  CHECK(last_rpm(0) != 0);
}

static void inherited_current_session_is_refused(void)
{
  ik_drive_mode = IK_MODE_CURRENT;
  for (unsigned w = 0; w < 4; w++) ik_drive_current_ma[w] = (w < 2 ? 2000 : -2000);
  stream(1000, 0); /* a diagnostic host has gone; only serial halts follow */
  for (unsigned k = 0; k < count; ++k) {
    struct motor_control c;
    CHECK(!(decode(k, &c) && c.mode == MOTOR_MODE_CURRENT && c.setpoint != 0));
  }
  CHECK(skynet_link_refused > 0U);
  CHECK_EQ(ik_drive_state, IK_STOPPED);
}

static void queued_record_goes_stale(void)
{
  wire(0, 21, 0, 0);
  now = 1000;
  tick_link(0);
  for (const unsigned end = now + 100; now < end;) tick_link(0);
  CHECK_EQ(ik_drive_enable, 0);
  CHECK(skynet_link_stale > 0U);
}

static void cut_record_does_not_absorb_halt(void)
{
  uint8_t r[7] = {0, 0, 0, 0, 21, 0, 12};
  const uint16_t crc = skynet_link_crc16(r + 2, 5);
  r[0] = (uint8_t)crc;
  r[1] = (uint8_t)(crc >> 8);
  stream(100, 21);
  feed_bytes(r, 6);
  tick_link(0);
  for (const unsigned end = now + 1000; now < end;) tick_link(0);
  CHECK_EQ(ik_drive_enable, 0);
  wire(0, 0, 0, 0); /* a halt whose first CRC byte 0x0c would complete the cut record */
  tick_link(0);
  for (const unsigned end = now + 200; now < end;) tick_link(0);
  CHECK_EQ(ik_drive_vy_mmps, 0);
  for (unsigned w = 0; w < 4; w++) CHECK(esc[w].lifecycle != MOTOR_LC_RUNNING || last_rpm(w) == 0);
}

static void dropped_byte_forges_nothing(void)
{
  stream(1000, 0); /* serial halts have armed all four wheels */
  for (unsigned id = 0; id < 4; id++) wire(id, 0, 0, 0);
  const uint8_t dropped4[] = {0x98, 4, 0, 0, 0, 0}; /* idle ID 4 record missing its first CRC byte 0x0a */
  feed_bytes(dropped4, sizeof dropped4);
  wire(5, -18, 5, -23);
  for (const unsigned end = now + 50; now < end;) tick_link(0);
  CHECK(ik_drive_vx_mmps == 0 && ik_drive_vy_mmps == 0 && ik_drive_omega_mradps == 0);
}

int main(int argc, char **argv)
{
  CHECK(argc == 2);
  setup();
  if (!strcmp(argv[1], "dropped_byte_forges_nothing")) dropped_byte_forges_nothing();
  else if (!strncmp(argv[1], "fault_waits_for_operator_", 25)) {
    const unsigned w = (unsigned)atoi(argv[1] + 25);
    CHECK(w < 4);
    fault_waits_for_operator(w);
  } else if (!strcmp(argv[1], "inherited_current_session_is_refused")) inherited_current_session_is_refused();
  else if (!strcmp(argv[1], "queued_record_goes_stale")) queued_record_goes_stale();
  else if (!strcmp(argv[1], "cut_record_does_not_absorb_halt")) cut_record_does_not_absorb_halt();
  else CHECK(0);
  printf("ok %s\n", argv[1]);
  return 0;
}
