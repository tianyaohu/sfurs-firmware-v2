/* Exercise real control/mailbox code with scripted replies at the SWD boundary. */
#include "check.h"
#include "esc_ctl.h"
#include "gen_esc_defaults.h"
#include "motor_protocol.h"
#include "startup_metrics.h"
#include "drive_ops.h"
#include <signal.h>
#include <string.h>

#define main startup_report_main
#include "../src/tools/startup_report.c"
#undef main
#include <unistd.h>

static double spin_clock_s, spin_sleep_s;
static int spin_signal;
static double spin_now(void) { return spin_clock_s; }
static void spin_sleep(double s)
{
  spin_clock_s += spin_sleep_s > 0 ? spin_sleep_s : s;
  if (spin_signal) { int sig = spin_signal; spin_signal = 0; raise(sig); }
}
#define pulse_now_s spin_now
#define pulse_sleep_s spin_sleep
#define main esc_tool_main
#include "../src/tools/esc_tool.c"
#undef main
#undef pulse_now_s
#undef pulse_sleep_s
#undef now_s
#undef sleep_s

static ik_link_t mailbox;
static esc_ctl_t ctl;
static uint32_t memory[IK_SYM_COUNT][2048];
static int fail_sym, tear_wheel, tear_reads, mutate_wheel;
static unsigned sent;
static bool live_telem;
static uint32_t inject_fault;
static bool raw_ack, clear_ack, have_clear_reply;
static struct motor_reply clear_reply[4];
static unsigned clear_mask;
static int config_wheel, config_reads_until_match;
static bool config_sent;
static uint32_t config_reply_words[6];
static struct {
  uint32_t id;
  uint8_t data[8];
  int wheel;
  bool reply;
  struct motor_reply response;
} script[16];
static size_t script_n, script_pos;

static uint32_t *word(uint32_t addr)
{
  unsigned sym = addr / 8192 - 1;
  CHECK(sym < IK_SYM_COUNT);
  CHECK_EQ(addr % 4, 0);
  return &memory[sym][(addr % 8192) / 4];
}

static void publish(int w, const struct motor_reply *r)
{
  uint32_t *row = &memory[IK_LINK][w * IK_LINK_WORDS];
  row[0] += 2;
  row[1] = r->lifecycle; row[2] = r->outcome; row[3] = r->reason;
  row[4] = r->session; row[5] = r->request_seq; row[6] = r->applied_seq; row[7] = r->first_edge;
  row[8] = (uint32_t)r->rpm; row[9] = (uint32_t)r->iq_ma;
  row[10]++; row[11] += 123;
}

static void reset(void)
{
  memset(&mailbox, 0, sizeof mailbox);
  memset(memory, 0, sizeof memory);
  for (int s = 0; s < IK_SYM_COUNT; s++) mailbox.addr[s] = (uint32_t)(s + 1) * 8192;
  for (int w = 0; w < 4; w++) memory[IK_LINK][w * IK_LINK_WORDS + 1] = MOTOR_LC_DISARMED;
  fail_sym = tear_wheel = mutate_wheel = -1;
  tear_reads = sent = script_n = script_pos = 0;
  raw_ack = clear_ack = true; have_clear_reply = false; clear_mask = 15;
  live_telem = false; inject_fault = 0; config_wheel = -1;
  config_sent = false; config_reads_until_match = 0;
  esc_ctl_init(&ctl, &mailbox, 0);
}

ocd_t *ocd_open(const char *binary, const char *serial, const char *chip, char *err, size_t cap)
{
  (void)binary; (void)serial; (void)chip; (void)err; (void)cap;
  return (ocd_t *)&mailbox;
}
void ocd_close(ocd_t *o) { (void)o; }
const char *ocd_error(const ocd_t *o) { (void)o; return "scripted SWD error"; }
int ocd_read(ocd_t *o, uint32_t addr, unsigned width, size_t count, uint32_t *out)
{
  (void)o; CHECK_EQ(width, 32);
  if (fail_sym >= 0 && addr / 8192 == (unsigned)fail_sym + 1) return -1;
  if (live_telem && count == IK_ROW_WORDS && addr >= mailbox.addr[IK_TELEM] &&
      addr < mailbox.addr[IK_TELEM] + 4 * IK_ROW_WORDS * 4) {
    word(addr)[7]++;
    word(addr)[8] = 1200;
  }
  if (config_sent && config_reads_until_match && count == IK_LINK_WORDS &&
      addr == mailbox.addr[IK_LINK] + (uint32_t)config_wheel * IK_LINK_WORDS * 4U &&
      --config_reads_until_match == 0) {
    word(addr)[0] += 2; word(addr)[20] = 201;
  }
  for (size_t i = 0; i < count; i++) out[i] = *word(addr + (uint32_t)i * 4);
  if (count == IK_LINK_WORDS && addr == mailbox.addr[IK_LINK] + (uint32_t)mutate_wheel * IK_LINK_WORDS * 4) {
    *word(addr) += 2; /* A complete update between the two generation reads. */
    mutate_wheel = -1;
  }
  return 0;
}
int ocd_read32(ocd_t *o, uint32_t addr, uint32_t *value)
{
  if (ocd_read(o, addr, 32, 1, value)) return -1;
  if (tear_reads && addr == mailbox.addr[IK_LINK] + (uint32_t)tear_wheel * IK_LINK_WORDS * 4) {
    *value |= 1; tear_reads--;
  }
  return 0;
}
int ocd_read_bytes(ocd_t *o, uint32_t addr, uint8_t *buf, size_t n)
{
  (void)o;
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)(*word((addr + (uint32_t)i) & ~3U) >> (8 * ((addr + i) % 4)));
  return 0;
}
int ocd_write32(ocd_t *o, uint32_t addr, uint32_t value)
{
  (void)o;
  if (fail_sym >= 0 && addr / 8192 == (unsigned)fail_sym + 1) return -1;
  *word(addr) = value;
  if (addr == mailbox.addr[IK_ENABLE] && value) memory[IK_FAULT][0] = inject_fault;
  if (addr == mailbox.addr[IK_RAW_SEQ]) {
    sent++;
    if (script_pos < script_n) {
      size_t k = script_pos++;
      CHECK_EQ(memory[IK_RAW_ID][0], script[k].id);
      CHECK_EQ(memory[IK_RAW_LEN][0], 8);
      uint8_t bytes[8];
      for (int b = 0; b < 8; b++)
        bytes[b] = (uint8_t)(memory[b < 4 ? IK_RAW_DATA : IK_RAW_DATA_HI][0] >> (8 * (b % 4)));
      CHECK_EQ(memcmp(bytes, script[k].data, 8), 0);
      if (script[k].reply) publish(script[k].wheel, &script[k].response);
    }
    if (config_wheel >= 0) {
      uint32_t *row = &memory[IK_LINK][config_wheel * IK_LINK_WORDS];
      row[0] += 2;
      memcpy(row + 19, config_reply_words, sizeof config_reply_words);
      config_sent = true;
    }
    if (raw_ack) memory[IK_RAW_DONE][0] = value;
  }
  if (addr == mailbox.addr[IK_CLEAR_SEQ] && clear_ack) {
    memory[IK_CLEAR_DONE][0] = value;
    if (have_clear_reply) for (int w = 0; w < 4; w++) if (clear_mask & (1U << w)) publish(w, &clear_reply[w]);
  }
  return 0;
}

static void expect_control(int w, struct motor_control c, const struct motor_reply *r)
{
  CHECK(script_n < 16);
  CHECK_EQ(motor_control_encode(&c, w < 0 ? 15 : IK_WHEEL_SELECT[w],
                               &script[script_n].id, script[script_n].data), MOTOR_R_NONE);
  script[script_n].wheel = w;
  script[script_n].reply = r != NULL;
  if (r) script[script_n].response = *r;
  script_n++;
}

static void arm_script(int w, uint8_t seq, unsigned reason)
{
  uint8_t session = (uint8_t)(100 + w);
  struct motor_reply probe = {.request_seq = (uint8_t)(seq + 1), .session = session,
    .lifecycle = MOTOR_LC_DISARMED, .outcome = MOTOR_OUT_REJECTED, .reason = MOTOR_R_BAD_SESSION};
  struct motor_reply arm = {.request_seq = (uint8_t)(seq + 2), .session = session,
    .lifecycle = reason ? MOTOR_LC_FAULTED : MOTOR_LC_READY,
    .outcome = reason ? MOTOR_OUT_REJECTED : MOTOR_OUT_APPLIED, .reason = (uint8_t)reason};
  expect_control(w, (struct motor_control){.flags = MOTOR_FLAG_ARM, .seq = (uint8_t)(seq + 1)}, &probe);
  expect_control(w, (struct motor_control){.flags = MOTOR_FLAG_ARM, .seq = (uint8_t)(seq + 2), .session = session}, &arm);
}

static void test_arm_speed_disable(void)
{
  reset();
  for (int w = 0; w < 4; w++) {
    arm_script(w, 0, 0);
    CHECK_EQ(esc_ctl_arm(&ctl, w), 0);
    CHECK_EQ(ctl.wheel[w].session, 100 + w); CHECK_EQ(ctl.wheel[w].seq, 2); CHECK(ctl.wheel[w].armed);
  }
  CHECK_EQ(script_pos, script_n);
  script_n = script_pos = 0;
  for (int w = 0; w < 4; w++) {
    for (int n = 3; n <= 4; n++) {
      int rpm = n == 3 ? -120 : 120;
      expect_control(w, (struct motor_control){.mode = MOTOR_MODE_SPEED, .setpoint = rpm,
                      .seq = n, .session = 100 + w}, NULL);
      CHECK_EQ(esc_ctl_speed(&ctl, w, rpm), 0);
      CHECK_EQ(ctl.wheel[w].seq, n);
    }
  }
  expect_control(-1, (struct motor_control){.mode = MOTOR_MODE_DISABLE}, NULL);
  CHECK_EQ(esc_ctl_disable_all(&ctl), 0);
  for (int w = 0; w < 4; w++) { CHECK(!ctl.wheel[w].armed); CHECK(esc_ctl_speed(&ctl, w, 10) != 0); }
  CHECK_EQ(script_pos, script_n);
  for (int w = 0; w < 4; w++) {
    reset(); ctl.wheel[w].seq = 254;
    arm_script(w, 254, 0); CHECK_EQ(esc_ctl_arm(&ctl, w), 0);
    CHECK_EQ(ctl.wheel[w].seq, 0);
    expect_control(w, (struct motor_control){.mode = MOTOR_MODE_SPEED, .setpoint = 120,
                    .seq = 1, .session = 100 + w}, NULL);
    CHECK_EQ(esc_ctl_speed(&ctl, w, 120), 0);
    CHECK_EQ(script_pos, script_n);
  }
}

static void test_arm_failures(void)
{
  const unsigned reasons[] = {MOTOR_R_NOT_ALIGNED, MOTOR_R_BUSY, MOTOR_R_BAD_SESSION, MOTOR_R_FAULT_MCSDK,
                             MOTOR_R_FAULT_STALL, MOTOR_R_NOT_IDLE};
  for (int w = 0; w < 4; w++) {
    for (size_t i = 0; i < sizeof reasons / sizeof *reasons; i++) {
      reset(); arm_script(w, 0, reasons[i]);
      CHECK(esc_ctl_arm(&ctl, w) != 0); CHECK(!ctl.wheel[w].armed);
      CHECK(strstr(mailbox.err, IK_WHEEL_POS[w])); CHECK(strstr(mailbox.err, ik_reason_name(reasons[i])));
      CHECK_EQ(script_pos, script_n);
    }
    reset();
    memory[IK_LINK][w * IK_LINK_WORDS + 4] = 77; /* Cached replies cannot satisfy a new probe. */
    CHECK(esc_ctl_arm(&ctl, w) != 0); CHECK(strstr(mailbox.err, "timed out")); CHECK_EQ(sent, 1);
    reset(); tear_wheel = w; tear_reads = 3;
    CHECK(esc_ctl_arm(&ctl, w) != 0); CHECK_EQ(sent, 0);
    reset(); memory[IK_STATE][0] = 1;
    CHECK(esc_ctl_arm(&ctl, w) != 0); CHECK(strstr(mailbox.err, "not stopped")); CHECK_EQ(sent, 0);
    reset(); CHECK(esc_ctl_speed(&ctl, w, 120) != 0); CHECK_EQ(sent, 0);
    ctl.wheel[w].armed = true;
    CHECK(esc_ctl_speed(&ctl, w, 32768) != 0); CHECK_EQ(sent, 0);
  }
  reset(); CHECK(esc_ctl_arm(&ctl, -1) != 0); CHECK(esc_ctl_arm(&ctl, 4) != 0);
  CHECK(esc_ctl_speed(&ctl, 4, 1) != 0); CHECK_EQ(sent, 0);
}

static void test_spin_stops(void)
{
  const int signals[] = {SIGINT, SIGTERM, SIGHUP};
  for (int w = 0; w < 4; w++) for (int mode = 0; mode < 6; mode++) {
    reset(); spin_clock_s = spin_sleep_s = 0; spin_signal = 0;
    arm_script(w, 0, 0);
    struct motor_reply fault = {.request_seq = 3, .session = 100 + w, .lifecycle = MOTOR_LC_FAULTED,
      .outcome = MOTOR_OUT_REJECTED, .reason = MOTOR_R_FAULT_MCSDK};
    expect_control(w, (struct motor_control){.mode = MOTOR_MODE_SPEED, .setpoint = 120,
      .seq = 3, .session = 100 + w}, mode == 1 ? &fault : NULL);
    if (mode == 0) for (int seq = 4; seq <= 7; seq++)
      expect_control(w, (struct motor_control){.mode = MOTOR_MODE_SPEED, .setpoint = 120,
        .seq = seq, .session = 100 + w}, NULL);
    if (mode >= 3) spin_signal = signals[mode - 3];
    if (mode == 2) spin_sleep_s = 0.05;
    expect_control(-1, (struct motor_control){.mode = MOTOR_MODE_DISABLE}, NULL);
    if (mode == 0) expect_control(-1, (struct motor_control){.mode = MOTOR_MODE_DISABLE}, NULL);
    wheel_t wheel = {IK_WHEEL_POS[w], w, IK_WHEEL_SELECT[w]};
    spin_args_t args = {.rpm = 120, .seconds = mode == 0 || mode == 2 ? 0.1 : 0.01};
    struct sigaction before, after;
    CHECK_EQ(sigaction(SIGINT, NULL, &before), 0);
    CHECK_EQ(cmd_spin(&mailbox, &wheel, &args), mode == 1 || mode == 2 ? 1 : 0);
    CHECK_EQ(sigaction(SIGINT, NULL, &after), 0); CHECK(before.sa_handler == after.sa_handler);
    CHECK_EQ(script_pos, script_n);
    if (mode == 1) CHECK(strstr(mailbox.err, "FAULT_MCSDK"));
    if (mode == 2) CHECK(strstr(mailbox.err, "40 ms deadline"));
  }
}

static void test_clear(void)
{
  reset(); have_clear_reply = true;
  memory[IK_ENABLE][0] = 1;
  memory[IK_CLEAR_SEQ][0] = UINT32_MAX;
  memory[IK_CLEAR_DONE][0] = UINT32_MAX;
  for (int w = 0; w < 4; w++) {
    memory[IK_LINK][w * IK_LINK_WORDS + 1] = MOTOR_LC_FAULTED;
    clear_reply[w] = (struct motor_reply){.lifecycle = MOTOR_LC_DISARMED, .outcome = MOTOR_OUT_APPLIED};
    ctl.wheel[w].armed = true;
  }
  CHECK_EQ(esc_ctl_clear_faults(&ctl), 0); CHECK_EQ(memory[IK_CLEAR_SEQ][0], 0);
  CHECK_EQ(memory[IK_ENABLE][0], 0);
  for (int w = 0; w < 4; w++) CHECK(!ctl.wheel[w].armed);
  for (int w = 0; w < 4; w++) {
    reset(); have_clear_reply = true;
    clear_reply[w] = (struct motor_reply){.lifecycle = MOTOR_LC_FAULTED,
      .outcome = MOTOR_OUT_REJECTED, .reason = MOTOR_R_NOT_IDLE};
    CHECK(esc_ctl_clear_faults(&ctl) != 0);
    CHECK(strstr(mailbox.err, IK_WHEEL_POS[w])); CHECK(strstr(mailbox.err, "NOT_IDLE"));
    clear_reply[w] = (struct motor_reply){0};
    reset(); have_clear_reply = true; clear_mask = 15 & ~(1U << w);
    CHECK(esc_ctl_clear_faults(&ctl) != 0); CHECK(strstr(mailbox.err, "no fresh reply"));
  }
  reset(); clear_ack = false;
  CHECK(esc_ctl_clear_faults(&ctl) != 0); CHECK(strstr(mailbox.err, "clear_done"));
  reset(); memory[IK_STATE][0] = 1;
  CHECK(esc_ctl_clear_faults(&ctl) != 0); CHECK_EQ(memory[IK_CLEAR_SEQ][0], 0);
  reset(); fail_sym = IK_CLEAR_SEQ;
  CHECK(esc_ctl_clear_faults(&ctl) != 0); CHECK(strstr(mailbox.err, "SWD error"));
}

static void test_raw_mailbox(void)
{
  reset();
  const uint8_t data[] = {0x10, 0, 0x78, 0x56, 0x12, 0x34, 0xab, 0xcd};
  CHECK_EQ(ik_send_raw8(&mailbox, 0x1f, data, 0), 0);
  CHECK_EQ(memory[IK_RAW_DATA][0], 0x56780010); CHECK_EQ(memory[IK_RAW_DATA_HI][0], 0xcdab3412);
  CHECK_EQ(memory[IK_RAW_LEN][0], 8);
  CHECK_EQ(ik_send_raw(&mailbox, 0x501, 0xdeadbeef, 0), 0);
  CHECK_EQ(memory[IK_RAW_LEN][0], 4); CHECK_EQ(memory[IK_RAW_DATA_HI][0], 0);
  CHECK_EQ(memory[IK_RAW_DATA][0], 0xdeadbeef); CHECK_EQ(memory[IK_RAW_SEQ][0], 2);
  reset(); fail_sym = IK_RAW_DATA_HI;
  CHECK(ik_send_raw8(&mailbox, 0x1f, data, 0) != 0); CHECK_EQ(sent, 0);
  reset(); raw_ack = false;
  CHECK(ik_send_raw8(&mailbox, 0x1f, data, 0) != 0); CHECK(strstr(mailbox.err, "not sent"));
  reset(); memory[IK_STATE][0] = 1;
  CHECK(ik_send_raw8(&mailbox, 0x1f, data, 0) != 0); CHECK_EQ(sent, 0);
  reset(); memory[IK_FAULT][0] = 0xa5; memory[IK_ARMED][0] = 0xf;
  ik_status_t s; CHECK_EQ(ik_status(&mailbox, &s), 0); CHECK_EQ(s.fault, 0xa5); CHECK_EQ(s.armed, 15);
}

static void test_link_rows_faults(void)
{
  for (int w = 0; w < 4; w++) {
    reset();
    uint32_t *r = &memory[IK_LINK][w * IK_LINK_WORDS];
    uint32_t expected[] = {2, MOTOR_LC_RUNNING, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING, 42, 11, 10, 1,
                          (uint32_t)-120, (uint32_t)-1250, 12, 1234, 3, 4, 2, 1001, 999, 1, MOTOR_R_FAULT_STALL};
    memcpy(r, expected, sizeof expected);
    ik_link_row_t rows[4]; CHECK_EQ(ik_link_rows(&mailbox, rows), 0);
    ik_link_row_t *x = &rows[w];
    CHECK(x->valid); CHECK_EQ(x->rpm, -120); CHECK_EQ(x->iq_ma, -1250); CHECK_EQ(x->first_start_reply_us, 999);
    CHECK_EQ(x->misses, 3); CHECK_EQ(x->rejects, 4); CHECK_EQ(x->request_seq, 11); CHECK_EQ(x->applied_seq, 10);
    CHECK_EQ(x->reply_count, 12); CHECK_EQ(x->reply_us, 1234); CHECK_EQ(x->start_reply_us, 1001); CHECK_EQ(x->start_count, 2);
    CHECK(!strcmp(x->lifecycle_name, "RUNNING")); CHECK(!strcmp(x->outcome_name, "INHIBITED"));
    CHECK(!strcmp(x->reason_name, "STARTING")); CHECK(x->first_start_valid);
    CHECK_EQ(x->fault_reason, MOTOR_R_FAULT_STALL); CHECK(!strcmp(x->fault_reason_name, "FAULT_STALL"));
    tear_wheel = w; tear_reads = 3;
    CHECK_EQ(ik_link_rows(&mailbox, rows), 0); CHECK(!rows[w].valid);
    tear_reads = 1; CHECK_EQ(ik_link_rows(&mailbox, rows), 0); CHECK(rows[w].valid);
    mutate_wheel = w; CHECK_EQ(ik_link_rows(&mailbox, rows), 0); CHECK(rows[w].valid); CHECK_EQ(rows[w].generation, 4);
    r[3] = MOTOR_R_NONE; /* a later reply overwrote reason; the latched cause is reported */
    CHECK(ik_check_fault(&mailbox, 1U << w) != 0); CHECK(strstr(mailbox.err, IK_WHEEL_POS[w]));
    CHECK(strstr(mailbox.err, "FAULTED")); CHECK(strstr(mailbox.err, "FAULT_STALL")); CHECK(strstr(mailbox.err, "esc_tool clear"));
    r[18] = 0;
    CHECK(ik_check_fault(&mailbox, 1U << (w + 4)) != 0); CHECK(strstr(mailbox.err, "LOST (no replies)"));
    r[1] = MOTOR_LC_FAULTED; r[3] = MOTOR_R_FAULT_MCSDK;
    CHECK(ik_check_fault(&mailbox, 0) != 0); CHECK(strstr(mailbox.err, "FAULTED (FAULT_MCSDK)"));
    /* Faulted before this session: the probe reply carries BAD_SESSION or NONE, never a cause. */
    r[3] = MOTOR_R_BAD_SESSION; r[18] = MOTOR_R_BAD_SESSION;
    CHECK(ik_check_fault(&mailbox, 1U << w) != 0); CHECK(strstr(mailbox.err, "faulted before this session"));
    CHECK(!strstr(mailbox.err, "BAD_SESSION"));
    r[3] = MOTOR_R_NONE; r[18] = MOTOR_R_NONE;
    CHECK(ik_check_fault(&mailbox, 1U << w) != 0); CHECK(strstr(mailbox.err, "faulted before this session"));
    r[1] = MOTOR_LC_DISARMED;
    CHECK_EQ(ik_check_fault(&mailbox, 0), 0);
    fail_sym = IK_LINK; CHECK(ik_link_rows(&mailbox, rows) != 0);
  }
  CHECK(!strcmp(ik_reason_name(255), "UNKNOWN"));
}

static void test_wrapping_skew(void)
{
  startup_result_t starts[4] = {{0}};
  ik_link_row_t links[4] = {{0}};
  for (int first = 0; first < 4; first++) {
    for (int w = 0; w < 4; w++) {
      links[w] = (ik_link_row_t){.valid = true, .start_count = 1, .first_start_valid = true,
        .first_start_reply_us = UINT32_MAX - 500U + (uint32_t)((w - first + 4) % 4) * 600U};
      starts[w].verdict = STARTUP_PASS;
    }
    startup_sync_t s = startup_link_synchronization(starts, links);
    CHECK_EQ(s.verdict, STARTUP_PASS); CHECK_NEAR(s.skew_ms, 1.8, 1e-9);
    for (int w = 0; w < 4; w++) {
      uint32_t saved = links[w].first_start_reply_us;
      links[w].first_start_valid = false; /* start_count alone may count a start whose reply is lost */
      CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_NOT_MEASURABLE);
      links[w].first_start_valid = true; links[w].valid = false;
      CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_NOT_MEASURABLE);
      links[w].valid = true; links[w].first_start_reply_us += 5000;
      CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_FAIL);
      starts[w].verdict = STARTUP_INACTIVE;
      CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_PASS);
      starts[w].verdict = STARTUP_PASS; links[w].first_start_reply_us = saved;
    }
  }
  for (int w = 0; w < 4; w++) {
    starts[w].verdict = STARTUP_INACTIVE;
    links[w].first_start_reply_us = 0;
  }
  CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_NOT_MEASURABLE);
  for (int w = 0; w < 4; w++) {
    starts[w].verdict = STARTUP_PASS;
    CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_PASS);
    links[w].first_start_reply_us = 2000;
    CHECK_EQ(startup_link_synchronization(starts, links).verdict, STARTUP_PASS);
  }
}

static void test_drive_fault_stops(void)
{
  for (int w = 0; w < 4; w++) {
    for (int lost = 0; lost < 2; lost++) {
      reset(); live_telem = true;
      uint32_t fault = 1U << (w + (lost ? 4 : 0));
      memory[IK_FAULT][0] = fault;
      memory[IK_LINK][w * IK_LINK_WORDS + 3] = MOTOR_R_NONE; /* overwritten by later replies */
      memory[IK_LINK][w * IK_LINK_WORDS + 18] = lost ? 0 : MOTOR_R_FAULT_STALL;
      CHECK(drive_preflight(&mailbox, 11) != 0);
      CHECK(strstr(mailbox.err, IK_WHEEL_POS[w])); CHECK(strstr(mailbox.err, lost ? "LOST" : "FAULTED"));
      CHECK(strstr(mailbox.err, lost ? "no replies" : "FAULT_STALL")); CHECK(strstr(mailbox.err, "esc_tool clear"));
      CHECK_EQ(sent, 0); CHECK_EQ(memory[IK_ENABLE][0], 0);
      memory[IK_FAULT][0] = 0;
      inject_fault = fault;
      pulse_result_t result;
      pulse_settings_t settings = {0};
      esc_move_t plan = {0, 1, {0, 200, 0}};
      CHECK(pulse_run(&mailbox, &plan, 1, 1, &settings, 0, &result) != 0);
      CHECK(strstr(result.aborted, IK_WHEEL_POS[w])); CHECK(strstr(result.aborted, lost ? "LOST" : "FAULTED"));
      CHECK(strstr(result.aborted, lost ? "no replies" : "FAULT_STALL"));
      CHECK_EQ(memory[IK_ENABLE][0], 0);
      pulse_result_free(&result);
    }
    reset(); live_telem = true;
    memory[IK_LINK][w * IK_LINK_WORDS + 1] = MOTOR_LC_FAULTED;
    memory[IK_LINK][w * IK_LINK_WORDS + 3] = MOTOR_R_FAULT_MCSDK;
    CHECK(drive_preflight(&mailbox, 11) != 0); CHECK(strstr(mailbox.err, "esc_tool clear"));
  }
}

static void test_report_skew(void)
{
  char dir[] = "/tmp/esc-ctl-report-XXXXXX", path[256]; CHECK(mkdtemp(dir));
  snprintf(path, sizeof path, "%s/startup.tsv", dir);
  char *args[] = {"startup_report", dir};
  const char *skews[] = {"2000", "2001", "na", NULL, "-1", "nan", "1oops", "", "1000", "2000.0001"};
  for (size_t i = 0; i < sizeof skews / sizeof *skews; i++) {
    FILE *f = fopen(path, "w"); CHECK(f);
    for (int w = 0; w < 4; w++) {
      fprintf(f, "run\tvx\t+\t%s\tPASS\t0\t8\t5\t1.1\t120\tnone", IK_WHEEL_POS[w]);
      if (skews[i]) fprintf(f, "\t%s", i == 8 && w == 3 ? "1001" : skews[i]);
      fputc('\n', f);
    }
    CHECK_EQ(fclose(f), 0);
    FILE *capture = tmpfile(); CHECK(capture);
    fflush(stdout);
    int saved = dup(STDOUT_FILENO); CHECK(saved >= 0);
    CHECK(dup2(fileno(capture), STDOUT_FILENO) >= 0);
    CHECK_EQ(startup_report_main(2, args), i == 0 ? 0 : 1);
    fflush(stdout); rewind(capture);
    char output[8192] = ""; CHECK(fread(output, 1, sizeof output - 1, capture) > 0);
    CHECK(strstr(output, i == 0 ? "overall: PASS" : i == 2 || i == 3 ? "overall: NOT_MEASURABLE" : "overall: FAIL"));
    CHECK(dup2(saved, STDOUT_FILENO) >= 0); close(saved); fclose(capture);
  }
  CHECK_EQ(unlink(path), 0); CHECK_EQ(rmdir(dir), 0);
}

static void test_reject_vectors(void)
{
  /* Literal wire bytes: protocol.md CONTROL layout, structural validation and sequence golden vectors.
   * Baseline is SPEED 0, seq 250, session 0x3c. */
  static const uint8_t bytes[][8] = {
    {0x21, 0, 0, 0, 0, 0, 251, 0x3c},
    {0x13, 0, 0, 0, 0, 0, 251, 0x3c},
    {0x11, 1, 0, 0, 0, 0, 251, 0x3c},
    {0x11, 4, 0, 0, 0, 0, 251, 0x3c},
    {0x11, 0, 0, 0, 0, 0, 251, 0x3c},
    {0x10, 1, 0, 0, 0, 0, 251, 0},
    {0x11, 0, 0, 0, 0, 0, 251, 0x3c},
    {0x11, 0, 0xa1, 0x0b, 0, 0, 251, 0x3c},
    {0x11, 0, 0, 0, 0x71, 0x17, 251, 0x3c},
    {0x11, 0, 0, 0, 0, 0, 249, 0x3c},
    {0x11, 0, 0, 0, 0, 0, 122, 0x3c},
    {0x11, 0, 0, 0, 0, 0, 11, 0x3c},
    {0x11, 0, 0, 0, 0, 0, 250, 0x3c},
    {0x11, 0, 0xa1, 0x0b, 0, 0, 250, 0x3c},
  };
  static const unsigned reasons[] = {1, 2, 3, 4, 5, 8, 7, 6, 6, 9, 10, 11, 0, 19};
  size_t count;
  const esc_reject_case_t *cases = esc_ctl_reject_cases(&count);
  CHECK_EQ(count, sizeof bytes / sizeof *bytes);
  for (int w = 0; w < 4; w++) for (size_t k = 0; k < count; k++) {
    uint32_t id;
    uint8_t data[8];
    CHECK_EQ(cases[k].build(w, 250, 0x3c, &id, data), 0);
    CHECK_EQ(memcmp(data, bytes[k], 8), 0);
    CHECK_EQ(id, 0x10U | (1U << w) | (k == 4 ? 1U << ((w + 1) % 4) : 0));
    CHECK_EQ(cases[k].reason, reasons[k]);
    CHECK_EQ(cases[k].outcome, k == 12 ? 4 : 3);
    CHECK_EQ(cases[k].disarmed, k == 6);
    struct motor_control c;
    CHECK_EQ(motor_control_decode(id, data, 8, &c), k < 5 ? reasons[k] : 0);
    CHECK_EQ(cases[k].build(w, 255, 1, &id, data), 0);
    CHECK_EQ(data[6], k == 9 ? 254 : k == 10 ? 127 : k == 11 ? 16 : k == 12 || k == 13 ? 255 : 0);
  }
  uint32_t id;
  uint8_t data[8];
  CHECK(cases[0].build(-1, 0, 1, &id, data) != 0);
  CHECK(cases[0].build(4, 0, 1, &id, data) != 0);
}

static void test_h2_arithmetic(void)
{
  uint64_t hist[IK_LAT_BINS] = {0};
  esc_latency_t s = esc_ctl_latency(hist);
  CHECK_EQ(s.count, 0); CHECK_EQ(s.p50_us, 0); CHECK_EQ(s.p99_us, 0); CHECK_EQ(s.max_us, 0);
  for (int b = 0; b < IK_LAT_BINS; b++) {
    memset(hist, 0, sizeof hist); hist[b] = 1;
    s = esc_ctl_latency(hist);
    uint32_t edge = b == 15 ? UINT32_MAX : (uint32_t)(b + 1) * 250;
    CHECK_EQ(s.count, 1); CHECK_EQ(s.p50_us, edge); CHECK_EQ(s.p99_us, edge); CHECK_EQ(s.max_us, edge);
  }
  memset(hist, 0, sizeof hist); hist[0] = 50; hist[14] = 49; hist[15] = 1;
  s = esc_ctl_latency(hist);
  CHECK_EQ(s.count, 100); CHECK_EQ(s.p50_us, 250); CHECK_EQ(s.p99_us, 3750); CHECK_EQ(s.max_us, UINT32_MAX);
  hist[0] = 49; hist[14] = 49; hist[15] = 2;
  s = esc_ctl_latency(hist); CHECK_EQ(s.p50_us, 3750); CHECK_EQ(s.p99_us, UINT32_MAX);
  memset(hist, 0, sizeof hist); hist[2] = (uint64_t)UINT32_MAX + 2;
  s = esc_ctl_latency(hist); CHECK_EQ(s.count, (uint64_t)UINT32_MAX + 2); CHECK_EQ(s.p99_us, 750);
  CHECK_NEAR(esc_ctl_bus_load(400, 200, 1), 14.4, 1e-9);
  CHECK_NEAR(esc_ctl_bus_load(800, 400, 2), 14.4, 1e-9);
  CHECK_NEAR(esc_ctl_bus_load(0, 200, 1), 4, 1e-9);
  CHECK_NEAR(esc_ctl_bus_load(0, 0, 1), 0, 1e-9);
  CHECK_EQ(esc_ctl_bus_load(1, 1, 0), 0); CHECK_EQ(esc_ctl_bus_load(1, 1, -1), 0);
  uint32_t per_wheel;
  CHECK_EQ(esc_ctl_stream_tx(403, &per_wheel), 0); CHECK_EQ(per_wheel, 103);
  CHECK_EQ(esc_ctl_stream_tx(3, &per_wheel), 0); CHECK_EQ(per_wheel, 3);
  CHECK(esc_ctl_stream_tx(0, &per_wheel) != 0); CHECK(esc_ctl_stream_tx(402, &per_wheel) != 0);
}

static void test_counter_snapshot(void)
{
  reset();
  memory[IK_TX_COUNT][0] = UINT32_MAX - 2U;
  memory[IK_TX_FAIL][0] = 7;
  memory[IK_FAULT][0] = 0x21;
  /* A separate fake SWD word stands in for the ELF-resolved RX counter address. */
  memory[IK_BAD_INPUT][0] = UINT32_MAX;
  for (int w = 0; w < IK_WHEELS; w++) {
    memory[IK_LINK][w * IK_LINK_WORDS + 10] = (uint32_t)w + 100;
    memory[IK_REPLY_LAT_MAX_US][w] = (uint32_t)w * 1000;
    for (int b = 0; b < IK_LAT_BINS; b++) memory[IK_REPLY_LAT_HIST][w * IK_LAT_BINS + b] = (uint32_t)(w * 100 + b);
    ik_link_row_t row;
    CHECK_EQ(ik_link_row(&mailbox, w, &row), 0); CHECK(row.valid); CHECK_EQ(row.reply_count, (uint32_t)w + 100);
  }
  ik_counters_t before, after;
  CHECK_EQ(ik_counters(&mailbox, mailbox.addr[IK_BAD_INPUT], &before), 0);
  CHECK_EQ(before.tx_count, UINT32_MAX - 2U); CHECK_EQ(before.tx_fail, 7); CHECK_EQ(before.fault, 0x21);
  CHECK_EQ(before.can_rx, UINT32_MAX);
  for (int w = 0; w < IK_WHEELS; w++) {
    CHECK_EQ(before.max_us[w], (uint32_t)w * 1000); CHECK_EQ(before.rows[w].reply_count, (uint32_t)w + 100);
    for (int b = 0; b < IK_LAT_BINS; b++) CHECK_EQ(before.hist[w][b], (uint32_t)(w * 100 + b));
  }
  memory[IK_BAD_INPUT][0] = 1;
  memory[IK_TX_COUNT][0] = 2;
  memory[IK_REPLY_LAT_HIST][0] = UINT32_MAX;
  CHECK_EQ(ik_counters(&mailbox, mailbox.addr[IK_BAD_INPUT], &after), 0);
  CHECK_EQ((uint32_t)(after.tx_count - before.tx_count), 5);
  CHECK_EQ((uint32_t)(after.can_rx - before.can_rx), 2);
  CHECK_EQ(after.hist[0][0], UINT32_MAX);
  ik_link_row_t row;
  CHECK(ik_link_row(&mailbox, -1, &row) != 0); CHECK(ik_link_row(&mailbox, 4, &row) != 0);
  for (int w = 0; w < IK_WHEELS; w++) {
    tear_wheel = w; tear_reads = 3;
    CHECK(ik_counters(&mailbox, mailbox.addr[IK_BAD_INPUT], &after) != 0);
    CHECK(strstr(mailbox.err, IK_WHEEL_POS[w]));
  }
  fail_sym = IK_REPLY_LAT_HIST;
  CHECK(ik_counters(&mailbox, mailbox.addr[IK_BAD_INPUT], &after) != 0);
}

static void test_config_exchange_all_wheels(void)
{
  const unsigned statuses[] = {0, MOTOR_R_NOT_IDLE, MOTOR_CFG_UNKNOWN_KEY, MOTOR_CFG_BAD_VALUE, MOTOR_R_BAD_VERSION};
  for (int w = 0; w < 4; w++) for (size_t k = 0; k < sizeof statuses / sizeof *statuses; k++) {
    reset(); mailbox.config_seq = 200; config_wheel = w;
    uint32_t *baseline = &memory[IK_LINK][w * IK_LINK_WORDS];
    baseline[19] = UINT32_MAX; baseline[20] = 200;
    uint32_t fixture[] = {0, 201, statuses[k], MOTOR_CFG_HALL_SHIFT_DEG, (uint32_t)-64, (uint32_t)-63};
    memcpy(config_reply_words, fixture, sizeof fixture);
    struct motor_config_reply r;
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_SET, MOTOR_CFG_HALL_SHIFT_DEG, -64, 0.05, &r), statuses[k] ? -1 : 0);
    CHECK_EQ(r.status, statuses[k]); CHECK_EQ(r.seq, 201); CHECK_EQ(r.key, MOTOR_CFG_HALL_SHIFT_DEG);
    CHECK_EQ(r.stored, -64); CHECK_EQ(r.effective, -63);
    if (statuses[k]) CHECK(strstr(mailbox.err, ik_config_status_name(statuses[k])));
    CHECK_EQ(memory[IK_RAW_ID][0], MOTOR_CAN_ID_CONFIG | IK_WHEEL_SELECT[w]);
    CHECK_EQ(memory[IK_RAW_LEN][0], 8); CHECK_EQ(memory[IK_RAW_DATA][0], 0x0002c911);
    CHECK_EQ(memory[IK_RAW_DATA_HI][0], 0x0000ffc0);
  }
  for (int w = 0; w < 4; w++) for (int mismatch = 0; mismatch < 4; mismatch++) {
    reset(); mailbox.config_seq = 200; config_wheel = mismatch == 3 ? (w + 1) % 4 : w;
    uint32_t fixture[] = {1, 201, 0, MOTOR_CFG_SPEED_KP, 1234, 1234};
    if (mismatch == 0) fixture[1] = 202;
    if (mismatch == 1) fixture[3] = MOTOR_CFG_SPEED_KI;
    if (mismatch == 2) fixture[0] = 0;
    memcpy(config_reply_words, fixture, sizeof fixture);
    struct motor_config_reply r;
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0, 0.01, &r), -1);
    CHECK(strstr(mailbox.err, "timeout")); CHECK_EQ(r.seq, 0);
  }
  for (int w = 0; w < 4; w++) {
    reset(); config_wheel = w; mailbox.config_seq = 255;
    uint32_t fixture[] = {1, 0, 0, MOTOR_CFG_SPEED_KP, 1234, 1234};
    memory[IK_LINK][w * IK_LINK_WORDS + 20] = 255;
    memcpy(config_reply_words, fixture, sizeof fixture);
    struct motor_config_reply r;
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0, 0.05, &r), 0);
    CHECK_EQ(r.seq, 0); CHECK_EQ(r.stored, 1234);
    CHECK_EQ(memory[IK_RAW_DATA_HI][0], 0); /* GET value and reserved bytes zero */
  }
  for (int w = 0; w < 4; w++) {
    reset(); mailbox.config_seq = 200; config_wheel = w; config_reads_until_match = 2;
    uint32_t fixture[] = {1, 202, 0, MOTOR_CFG_SPEED_KP, 1234, 1234};
    memcpy(config_reply_words, fixture, sizeof fixture);
    struct motor_config_reply r;
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0, 0.05, &r), 0);
    CHECK_EQ(r.seq, 201); CHECK_EQ(r.stored, 1234);
    reset(); raw_ack = false;
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0, 0.01, &r), -1);
    CHECK(strstr(mailbox.err, "not sent"));
    reset(); tear_wheel = w; tear_reads = 3;
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0, 0.01, &r), -1);
    CHECK(strstr(mailbox.err, "baseline")); CHECK_EQ(sent, 0);
    reset();
    CHECK_EQ(ik_config_exchange(&mailbox, w, MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 1, 0.01, &r), -1);
    CHECK(strstr(mailbox.err, "RESERVED_NONZERO")); CHECK_EQ(sent, 0);
  }
}

static void test_current_mailbox_helpers(void)
{
  reset();
  CHECK_EQ(ik_set_mode(&mailbox, IK_MODE_CURRENT), 0); CHECK_EQ(memory[IK_MODE][0], IK_MODE_CURRENT);
  const int32_t currents[] = {INT32_MIN, -500, 500, INT32_MAX};
  const int32_t expected[] = {-ESC_IQMAX_A * 1000, -500, 500, ESC_IQMAX_A * 1000};
  CHECK_EQ(ik_drive_current(&mailbox, currents, true), 0);
  for (int w = 0; w < 4; w++) CHECK_EQ((int32_t)memory[IK_CURRENT_MA][w], expected[w]);
  CHECK_EQ(memory[IK_ENABLE][0], 1); CHECK_EQ(memory[IK_HEARTBEAT][0], 1);
  for (unsigned state = 1; state <= 2; state++) {
    memory[IK_STATE][0] = state;
    CHECK_EQ(ik_set_mode(&mailbox, IK_MODE_SPEED), -1); CHECK(strstr(mailbox.err, "not stopped"));
    CHECK_EQ(memory[IK_MODE][0], IK_MODE_CURRENT);
  }
  memory[IK_STATE][0] = 0;
  CHECK_EQ(ik_set_mode(&mailbox, 2), -1);
  fail_sym = IK_CURRENT_MA;
  CHECK_EQ(ik_drive_current(&mailbox, currents, true), -1);
  CHECK_EQ(memory[IK_HEARTBEAT][0], 1);
}

int main(void)
{
  test_config_exchange_all_wheels(); test_current_mailbox_helpers();
  test_reject_vectors(); test_h2_arithmetic(); test_counter_snapshot();
  test_arm_speed_disable(); test_arm_failures(); test_spin_stops(); test_clear(); test_raw_mailbox();
  test_link_rows_faults(); test_wrapping_skew(); test_report_skew(); test_drive_fault_stops();
  puts("esc_ctl: all tests passed");
  return 0;
}
