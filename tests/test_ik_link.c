/* ik_link.c: command words, config frames, telemetry decode, and the SWD
 * mailbox through the fake OpenOCD with a host ELF that carries the ik_drive_*
 * symbols. The constants cannot drift from the firmware: ik_link.c includes can_datatypes.h. */
#include <stdlib.h>
#include <string.h>

#include "can_datatypes.h"
#include "check.h"
#include "elfsym.h"
#include "ik_link.h"
#include "motor_protocol.h"

static void test_record_words(void)
{
  uint32_t w;
  CHECK_EQ(ik_record_start_word(50, 1000, &w), 0);
  CHECK_EQ(w, 0x5EU << 24 | 1000U << 9 | 50);
  CHECK_EQ(ik_record_read_word(0, 32, 32, true, &w), 0);
  CHECK_EQ(w, 0x3DU << 24 | 1U << 18 | 32U << 10 | 32);
  CHECK_EQ(ik_record_read_word(1, 0, 1, false, &w), 0);
  CHECK_EQ(w, 0x3DU << 24 | 1U << 16 | 1U << 10);
  CHECK(ik_record_start_word(384, 0, &w) != 0);
  CHECK(ik_record_start_word(-1, 0, &w) != 0);
  CHECK(ik_record_start_word(0, 4096, &w) != 0);
  CHECK(ik_record_read_word(1, 0, 1, true, &w) != 0); /* ch67 only on bank 0 */
  CHECK(ik_record_read_word(2, 0, 1, false, &w) != 0);
}

static void test_capture_words(void)
{
  uint32_t w;
  CHECK_EQ(ik_capture_start_word(3000, &w), 0);
  CHECK_EQ(w, 0xC5U << 24 | 3000);
  CHECK(ik_capture_start_word(6001, &w) != 0);
  CHECK_EQ(ik_capture_read_word(3, 0, 1, &w), 0); /* info bank */
  CHECK_EQ(w, 0x3DU << 24 | 3U << 16 | 1U << 10);
  CHECK(ik_capture_read_word(2, 0, 1, &w) != 0);
  CHECK(ik_capture_read_word(0, 512, 1, &w) != 0);
  CHECK(ik_capture_read_word(0, 0, 33, &w) != 0);
}

static void test_config_frames_for_all_wheels(void)
{
  for (int i = 0; i < IK_WHEELS; i++) {
    uint32_t id, word;
    CHECK_EQ(ik_config_frame(IK_WHEEL_SELECT[i], "kp", -2, &id, &word), 0);
    CHECK_EQ(id, CONFIG_SET | IK_WHEEL_SELECT[i]);
    CHECK_EQ(word, 1U | 0xFFFEU << 16); /* int16 two's complement in the high half */
  }
  uint32_t id, word;
  CHECK(ik_config_frame(1, "nope", 0, &id, &word) != 0);
  CHECK(ik_config_frame(1, "kp", 40000, &id, &word) != 0);
  /* Settable keys are exactly can_datatypes.h's ESC_CFG_*; retired 7-24 are not offered. */
  const struct { const char *name; int number; } want[] = {
      {"kp", ESC_CFG_SPEED_KP}, {"ki", ESC_CFG_SPEED_KI}, {"shift", ESC_CFG_HALL_SHIFT_DEG},
      {"iqmax_ma", ESC_CFG_IQ_MAX_MA}, {"current_kp", ESC_CFG_CURRENT_KP}, {"current_ki", ESC_CFG_CURRENT_KI},
      {"stall_ramp_ma_per_s", ESC_CFG_STALL_RAMP_MA_PER_S}, {"stall_ms", ESC_CFG_STALL_MS}};
  CHECK_EQ(ik_config_key_count(), sizeof want / sizeof want[0]);
  for (size_t k = 0; k < ik_config_key_count(); k++) {
    int number;
    CHECK(!strcmp(ik_config_key_name(k, &number), want[k].name));
    CHECK_EQ(number, want[k].number);
    CHECK(number < 7 || number > 24);
  }
  CHECK(ik_config_key_name(ik_config_key_count(), NULL) == NULL);
  CHECK_EQ(ESC_CFG_STALL_RAMP_MA_PER_S, 25);
  CHECK_EQ(ESC_CFG_STALL_MS, 26);
  for (int i = 0; i < IK_WHEELS; i++) {
    CHECK_EQ(ik_config_frame(IK_WHEEL_SELECT[i], "stall_ramp_ma_per_s", 30000, &id, &word), 0);
    CHECK_EQ(id, CONFIG_SET | IK_WHEEL_SELECT[i]);
    CHECK_EQ(word, 25U | 30000U << 16);
    CHECK_EQ(ik_config_frame(IK_WHEEL_SELECT[i], "stall_ms", 500, &id, &word), 0);
    CHECK_EQ(word, 26U | 500U << 16);
  }
  CHECK(ik_config_frame(1, "ff_static_ma", 0, &id, &word) != 0); /* retired key 7 */
}

static void test_unpack_record(void)
{
  unsigned packed = ESC_REC_MODE_HOLD | 0x04 | 0x10 | 3 << 8 | 5 << 12 | 1 << 15;
  ik_record_sample_t s = {51, 40, 272, 0, 326, 598, -544, (int16_t)packed};
  ik_record_unpacked_t u;
  ik_unpack_record(&s, &u);
  CHECK(!strcmp(u.mode, "hold"));
  CHECK(u.valid && !u.saturated && u.bleed && !u.stall_timer && u.loop);
  CHECK_EQ(u.edges_mod16, 3);
  CHECK_EQ(u.hall_state, 5);
  CHECK_NEAR(u.p_a, 0.5, 1e-9);
  CHECK_NEAR(u.iq_a, -1.0, 1e-9);
  const char *const modes[] = {"off", "run", "hold", "stall_ramp"};
  for (unsigned m = 0; m < 4; m++) {
    s.packed = (int16_t)(m | 0x08 | 0x20);
    ik_unpack_record(&s, &u);
    CHECK(!strcmp(u.mode, modes[m]));
    CHECK(!u.valid && u.saturated && !u.bleed && u.stall_timer && !u.loop);
    CHECK_EQ(u.edges_mod16, 0);
    CHECK_EQ(u.hall_state, 0);
  }
}

static void test_decode_row(void)
{
  uint32_t w[IK_ROW_WORDS] = {2, (uint32_t)-60, (uint32_t)(-544), 1088, 6, 0x10 | 0x80 | 2, 0, 0, 1210, 0x42,
                              (uint32_t)-30, 1, 0, 0, 0, 0, 0, 0, 0};
  ik_row_t r;
  ik_decode_row(w, &r);
  CHECK_EQ(r.rpm, -60);
  CHECK_NEAR(r.iq_a, -1.0, 1e-9);
  CHECK_NEAR(r.iq_ref_a, 2.0, 1e-9);
  CHECK_NEAR(r.vbus_v, 12.1, 1e-9);
  CHECK(!strcmp(r.mc_state, "RUN") && !strcmp(r.stop_reason, "timeout") && !strcmp(r.align, "ok"));
  CHECK(r.active && r.aligned && !r.derated && !r.aligning);
  CHECK(!strcmp(r.fault_names, "overvolt overcurrent"));
  CHECK_EQ(r.hall_shift_deg, -30);
  w[5] = 0x10 | 5;
  ik_decode_row(w, &r);
  CHECK(!strcmp(r.stop_reason, "capture"));
  for (uint32_t retired = 6; retired <= 7; retired++) { /* old parked / start-failed: shown as numbers */
    char want[4];
    snprintf(want, sizeof want, "%u", retired);
    w[5] = 0x10 | retired;
    ik_decode_row(w, &r);
    CHECK(!strcmp(r.stop_reason, want));
  }
}

static uint32_t sym(const char *name)
{
  uint32_t a;
  char full[64];
  snprintf(full, sizeof full, "ik_drive_%s", name);
  CHECK_EQ(elfsym_lookup(FAKE_ELF, full, &a, NULL), 0);
  return a;
}

static void set_mem(const char *spec) { setenv("FAKE_OCD_MEM", spec, 1); }

static void test_mailbox_over_fake_openocd(void)
{
  setenv("OPENOCD", FAKE_OCD, 1);
  char mem[512];
  ik_link_t l;
  /* wrong ABI word: refuse to talk to an unknown image */
  set_mem("");
  CHECK(ik_link_open(&l, "SN", FAKE_ELF) != 0);
  CHECK(strstr(l.err, "flash"));

  snprintf(mem, sizeof mem, "%x=%x,%x=0,%x=5,%x=6", sym("abi"), IK_ABI, sym("state"), sym("raw_seq"),
           sym("raw_done"));
  set_mem(mem);
  CHECK_EQ(ik_link_open(&l, "SN", FAKE_ELF), 0);
  CHECK_EQ(ik_drive(&l, 200, -100, 300, true), 0);
  uint32_t v;
  CHECK_EQ(ik_read(&l, IK_VX_MMPS, &v), 0);
  CHECK_EQ((int32_t)v, 200);
  CHECK_EQ(ik_read(&l, IK_VY_MMPS, &v), 0);
  CHECK_EQ((int32_t)v, -100);
  CHECK_EQ(ik_read(&l, IK_ENABLE, &v), 0);
  CHECK_EQ(v, 1);
  uint32_t hb;
  CHECK_EQ(ik_read(&l, IK_HEARTBEAT, &hb), 0);
  CHECK_EQ(ik_drive(&l, 0, 0, 0, false), 0);
  CHECK_EQ(ik_read(&l, IK_HEARTBEAT, &v), 0);
  CHECK_EQ(v, hb + 1); /* every setpoint bumps the heartbeat */
  /* raw frame: raw_done == raw_seq + 1 in the preset, so the Nucleo "sent" it */
  CHECK_EQ(ik_send_raw(&l, CAPTURE | MOTOR_NE_CAN_ID, 0x12345678U, 0.5), 0);
  CHECK_EQ(ik_read(&l, IK_RAW_ID, &v), 0);
  CHECK_EQ(v, CAPTURE | MOTOR_NE_CAN_ID);
  CHECK_EQ(ik_read(&l, IK_RAW_LEN, &v), 0); CHECK_EQ(v, 4);
  CHECK_EQ(ik_read(&l, IK_RAW_DATA_HI, &v), 0); CHECK_EQ(v, 0);
  uint32_t id;
  uint8_t data[8];
  const struct motor_control disable = {.mode = MOTOR_MODE_DISABLE};
  CHECK_EQ(motor_control_encode(&disable, 15, &id, data), MOTOR_R_NONE);
  CHECK_EQ(ik_write(&l, IK_RAW_DONE, 7), 0);
  CHECK_EQ(ik_send_raw8(&l, id, data, 0.5), 0);
  CHECK_EQ(ik_read(&l, IK_RAW_LEN, &v), 0); CHECK_EQ(v, 8);
  CHECK_EQ(ik_read(&l, IK_RAW_DATA, &v), 0); CHECK_EQ(v, 0x10);
  CHECK_EQ(ik_read(&l, IK_RAW_DATA_HI, &v), 0); CHECK_EQ(v, 0);
  CHECK_EQ(ik_write(&l, IK_FAULT, 0xa5), 0); CHECK_EQ(ik_write(&l, IK_ARMED, 0xf), 0);
  ik_status_t status;
  CHECK_EQ(ik_status(&l, &status), 0); CHECK_EQ(status.fault, 0xa5); CHECK_EQ(status.armed, 15);
  /* A frame never completes: raw_done stays 7, expected 8. */
  CHECK(ik_send_raw(&l, CAPTURE, 1, 0.05) != 0);
  CHECK(strstr(l.err, "not sent"));
  /* Read the real cumulative-counter decoder through fake SWD, without simulating ESC behaviour. */
  uint32_t rx_addr;
  CHECK_EQ(elfsym_lookup(FAKE_ELF, "can_rx_frame_count", &rx_addr, NULL), 0);
  CHECK_EQ(ocd_write32(l.ocd, rx_addr, UINT32_MAX), 0);
  CHECK_EQ(ik_write(&l, IK_TX_COUNT, 123), 0); CHECK_EQ(ik_write(&l, IK_TX_FAIL, 2), 0);
  for (int w = 0; w < IK_WHEELS; w++) {
    for (int b = 0; b < IK_LAT_BINS; b++)
      CHECK_EQ(ocd_write32(l.ocd, l.addr[IK_REPLY_LAT_HIST] + (uint32_t)(w * IK_LAT_BINS + b) * 4U,
                           (uint32_t)(w * 100 + b)), 0);
    CHECK_EQ(ocd_write32(l.ocd, l.addr[IK_REPLY_LAT_MAX_US] + (uint32_t)w * 4U, (uint32_t)(w + 1) * 1000U), 0);
  }
  ik_counters_t counters;
  CHECK_EQ(ik_counters(&l, rx_addr, &counters), 0);
  CHECK_EQ(counters.tx_count, 123); CHECK_EQ(counters.tx_fail, 2); CHECK_EQ(counters.can_rx, UINT32_MAX);
  CHECK_EQ(counters.fault, 0xa5);
  for (int w = 0; w < IK_WHEELS; w++) {
    CHECK(counters.rows[w].valid); CHECK_EQ(counters.max_us[w], (uint32_t)(w + 1) * 1000U);
    for (int b = 0; b < IK_LAT_BINS; b++) CHECK_EQ(counters.hist[w][b], (uint32_t)(w * 100 + b));
  }
  ik_link_close(&l);

  /* raw frames are refused unless the drive is STOPPED */
  snprintf(mem, sizeof mem, "%x=%x,%x=1", sym("abi"), IK_ABI, sym("state"));
  set_mem(mem);
  CHECK_EQ(ik_link_open(&l, "SN", FAKE_ELF), 0);
  CHECK(ik_send_raw(&l, CAPTURE, 1, 0.05) != 0);
  CHECK(strstr(l.err, "not stopped"));
  ik_link_close(&l);
  set_mem("");
}

static void test_missing_symbols_are_reported(void)
{
  uint32_t addr[IK_SYM_COUNT];
  char err[128];
  CHECK(ik_resolve("/nonexistent.elf", addr, err, sizeof err) != 0);
  CHECK(ik_resolve("/etc/hostname", addr, err, sizeof err) != 0); /* not an ELF */
  CHECK(strstr(err, "missing"));
}

int main(void)
{
  test_record_words();
  test_capture_words();
  test_config_frames_for_all_wheels();
  test_unpack_record();
  test_decode_row();
  test_mailbox_over_fake_openocd();
  test_missing_symbols_are_reported();
  printf("ok\n");
  return 0;
}
