#include "drive_app.h"
#include "can_datatypes.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_config_common.h"
#include "check.h"
#include "motor_controller_can.h"
#include "motor_protocol.h"
#include "parameters_conversion.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OTHER_WHEEL (MOTOR_CAN_ID == 1U ? 2U : 1U)

PID_Handle_t PIDIqHandle_M1, PIDIdHandle_M1;
HALL_Handle_t HALL_M1;
RDivider_Handle_t BusVoltageSensor_M1 = { ._Super = { .ConversionFactor = 34, .raw = 23831 } };

/* ---- fake MCSDK state machine ---- */
static int state = IDLE, start_pending, stop_pending, starts, acks;
static uint16_t cur_faults, occ_faults;
static int16_t speed;
static qd_t iqd, iqdref;
bool MC_StartMotor1(void) { if (state != IDLE) return false; start_pending = 1; ++starts; return true; }
bool MC_StopMotor1(void) { stop_pending = 1; return true; }
void MC_SetCurrentReferenceMotor1(qd_t r) { iqdref = r; }
int MC_GetSTMStateMotor1(void) { return state; }
uint16_t MC_GetCurrentFaultsMotor1(void) { return cur_faults; }
uint16_t MC_GetOccurredFaultsMotor1(void) { return occ_faults; }
bool MC_AcknowledgeFaultMotor1(void) { if (state != FAULT_OVER) return false; state = IDLE; occ_faults = 0; ++acks; return true; }
int16_t MC_GetMecSpeedAverageMotor1(void) { return speed; }
qd_t MC_GetIqdMotor1(void) { return iqd; }
qd_t MC_GetIqdrefMotor1(void) { return iqdref; }

/* ---- fake motor + Hall decoder (hall_speed_pos_fdbk.c table) ---- */
static const uint8_t SEQ[6] = {5, 1, 3, 2, 6, 4};
static double true_shift_deg = 77.0; static int reversed, stuck, hall_dead;
static double rotor_deg;
static int sector_of(uint8_t s) { for (int k = 0; k < 6; ++k) if (SEQ[k] == s) return k; return -1; }
static uint8_t hall_at(double el_deg) {
  double a = (reversed ? -el_deg : el_deg) - true_shift_deg;
  int k = (int)floor(a / 60.0); k %= 6; if (k < 0) k += 6;
  return SEQ[k];
}
static void hall_isr(uint8_t next) {
  uint8_t prev = HALL_M1.HallState; int sp = sector_of(prev), sn = sector_of(next);
  HALL_M1.HallState = next;
  if (sn == (sp + 1) % 6) { HALL_M1.Direction = 1; HALL_M1.MeasuredElAngle = (int16_t)(HALL_M1.PhaseShift + sn * 65536 / 6); }
  else if (sp == (sn + 1) % 6) { HALL_M1.Direction = -1; HALL_M1.MeasuredElAngle = (int16_t)(HALL_M1.PhaseShift + sp * 65536 / 6); }
}
static void motor_step(void) {
  if (hall_dead) { HALL_M1.HallState = 7; return; } /* lost Hall harness: all lines read high */
  if (state == RUN && drive_align_force && iqdref.d > 300 && !stuck)
    rotor_deg = (uint16_t)drive_align_angle * 360.0 / 65536.0;
  uint8_t h = hall_at(rotor_deg);
  if (h != HALL_M1.HallState) hall_isr(h);
}
static void mf_step(void) {
  if (stop_pending) { stop_pending = 0; start_pending = 0; if (state == RUN || state == CHARGE_BOOT_CAP) state = IDLE; }
  if (start_pending && state == IDLE) { start_pending = 0; state = CHARGE_BOOT_CAP; }
  else if (state == CHARGE_BOOT_CAP) state = RUN;
}
/* The main loop sends CONTROL replies as fast as they come; keep the latest for the checks so
 * explicit drive_app_poll() calls below only see the service replies they ask for. */
static struct motor_reply ctl_last; static int ctl_seen;
static void take_replies(void) {
  uint32_t id; uint8_t d[8];
  while (motor_cmd_reply_peek(&drive_cmd, &id, d)) {
    CHECK(id == (MOTOR_CAN_ID_CONTROL_REPLY | MOTOR_CAN_ID));
    CHECK(motor_reply_decode(d, 8, &ctl_last) == MOTOR_R_NONE);
    ctl_seen = 1;
    motor_cmd_reply_pop(&drive_cmd);
  }
}
static void tick(int n) { while (n--) { mf_step(); drive_app_tick(); take_replies(); motor_step(); } }
static void rx(uint32_t id, uint32_t word) { uint8_t d[4]; CAN_PAYLOAD_FROM_U32(d, word); drive_app_can_rx(id, d, 4); }
uint32_t fake_validity[8] = { 0x56414C49U, ~0x56414C49U, 45344U, 0xDEADBEEFU, 0x08004000U, 2U };
static uint32_t sent_id; static uint8_t sent[8];
static unsigned frames, send_budget = 1000000;
static uint32_t frame_id[64]; static uint8_t frame[64][8];
static int send(uint32_t id, const uint8_t d[8]) {
  if (send_budget == 0) return 0;
  --send_budget; sent_id = id; memcpy(sent, d, 8);
  if (frames < 64) { frame_id[frames] = id; memcpy(frame[frames], d, 8); } ++frames; return 1;
}
static int16_t f16(const uint8_t *d) { return (int16_t)(d[0] | d[1] << 8); }
/* FOC ISR: 16 periods per ms; stores the reference as Id and 2x it as Vd, only to tell the
 * capture channels apart in the readout. */
static void foc_isr(int n) {
  while (n--) {
    int16_t ref_d = iqdref.d; drive_capture_isr_ref(&ref_d);
    drive_capture_isr_store(ref_d, 0, (int16_t)(2 * ref_d), 0, ref_d, drive_align_angle);
  }
}
static void tick_isr(int n) { while (n--) { mf_step(); drive_app_tick(); take_replies(); foc_isr(16); motor_step(); } }
/* Protocol v1 CONTROL frames, encoded by the real codec. The session is read from the ESC
 * state, as the Nucleo learns it from the ESC's replies. ARM sets the sequence baseline. */
static uint8_t ctl_seq = 1;
static void ctl(uint8_t mode, uint8_t flags, int16_t sp, uint8_t seq, uint32_t sel) {
  const struct motor_control c = { mode, flags, sp, 0, seq, drive_cmd.session };
  uint32_t id; uint8_t d[8];
  CHECK(motor_control_encode(&c, sel, &id, d) == MOTOR_R_NONE);
  drive_app_can_rx(id, d, 8);
}
static void arm(void) { ctl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, ctl_seq, MOTOR_CAN_ID); }
static void speed_cmd(int rpm) { ctl(MOTOR_MODE_SPEED, 0, (int16_t)rpm, ++ctl_seq, MOTOR_CAN_ID); }
static void disable(uint32_t sel) { ctl(MOTOR_MODE_DISABLE, 0, 0, ctl_seq, sel); }
static void clear_fault(void) { ctl(MOTOR_MODE_DISABLE, MOTOR_FLAG_CLEAR_FAULT, 0, ctl_seq, MOTOR_CAN_ID); }
/* Send everything pending (CONTROL replies first) and forget it. */
static void flush(void) { drive_app_poll(send); sent_id = 0; frames = 0; }
/* Most recent CONTROL reply; fails if none was sent since the last call. */
static struct motor_reply last_reply(void) { CHECK(ctl_seen); ctl_seen = 0; return ctl_last; }
#define last_reply_rpm (ctl_last.rpm)
static double shift_deg(void) { return HALL_M1.PhaseShift * 360.0 / 65536.0; }
/* One CONFIG frame (real codec), one tick, then the CONFIG_REPLY it produced. */
static uint8_t cfg_seq;
static struct motor_config_reply cfg_raw(uint32_t id, const uint8_t d[8])
{
  drive_app_can_rx(id, d, 8); tick(1);
  frames = 0; drive_app_poll(send);
  struct motor_config_reply r = { .status = 0xFF };
  int seen = 0;
  for (unsigned k = 0; k < frames && k < 64; ++k)
    if (frame_id[k] == (MOTOR_CAN_ID_CONFIG_REPLY | MOTOR_CAN_ID)) {
      CHECK(motor_config_reply_decode(frame[k], 8, &r) == MOTOR_R_NONE); ++seen;
    }
  CHECK(seen == 1);
  return r;
}
static struct motor_config_reply cfg(uint8_t op, uint16_t key, int16_t value)
{
  const struct motor_config c = { op, ++cfg_seq, key, value };
  uint32_t id; uint8_t d[8];
  CHECK(motor_config_encode(&c, MOTOR_CAN_ID, &id, d) == MOTOR_R_NONE);
  const struct motor_config_reply r = cfg_raw(id, d);
  CHECK(r.seq == cfg_seq && r.key == key);
  return r;
}

static void setup(int scenario)
{
  drive_app_init();
  HALL_M1.PhaseShift = (int16_t)(120 * 65536 / 360);
  HALL_M1.HallState = hall_at(0);
  if (scenario >= 3 && scenario <= 6) drive_app.aligned = 1; /* as if built with --hall-shift */
}

static void alignment_scenario(int scenario)
{
  setup(scenario); /* alignment: 1 reversed wiring, 2 rotor stuck */
  reversed = scenario == 1; stuck = scenario == 2;
  HALL_M1.HallState = hall_at(0);
  tick(5); rx(HALL_ALIGN | MOTOR_CAN_ID, 0x12345678U); tick(5); CHECK(drive_app.align_phase == 0); /* bad magic */
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); tick(2);
  CHECK(drive_align_force && drive_app.align_result == ESC_ALIGN_RUNNING);
  arm(); speed_cmd(100); tick(1); CHECK(starts == 1); /* armed, but motion is refused while aligning */
  CHECK(last_reply().reason == MOTOR_R_BUSY);
  tick(5000);
  CHECK(!drive_align_force && state == IDLE && drive_app.align_phase == 0);
  CHECK(drive_app.align_result == (scenario == 1 ? ESC_ALIGN_REVERSED : ESC_ALIGN_NO_EDGES));
  CHECK(fabs(shift_deg() - 120.0) < 0.01 && !drive_app.aligned);
  arm(); speed_cmd(100); tick(3); CHECK(starts == 1 && state == IDLE); /* refused */
  CHECK(last_reply().reason == MOTOR_R_NOT_ALIGNED);
}

static void test_alignment_detects_reversed_wiring(void)
{
  alignment_scenario(1);
}

static void test_alignment_detects_stuck_rotor(void)
{
  alignment_scenario(2);
}

static void test_streaming_and_watchdog(void)
{
  setup(3); /* stream, watchdog */
  speed_cmd(150); tick(3); CHECK(starts == 0); /* not armed */
  CHECK(last_reply().reason == MOTOR_R_NOT_ARMED);
  arm(); speed_cmd(0); tick(3); CHECK(starts == 0); /* zero does not start */
  speed_cmd(150); tick(3); CHECK(starts == 1 && state == RUN && drive_app.active);
  CHECK(drive_speed.ref_to_rpm == 150); /* the speed loop interpolates to it over 10 ms */
  for (int i = 0; i < 25; ++i) { speed_cmd(-90); tick(20); } /* < 1 s: the edge-less fake wheel would stall */
  CHECK(state == RUN && drive_speed.ref_to_rpm == -90 && drive_speed.ref_rpm == -90 && starts == 1);
  tick(79); CHECK(state == RUN); /* 99 ms since the last command */
  tick(2); CHECK(state == IDLE && !drive_app.active && drive_app.stop_reason == ESC_STOP_TIMEOUT);
  CHECK(drive_cmd.lifecycle == MOTOR_LC_DISARMED);
  speed_cmd(100); tick(3); CHECK(starts == 1); /* disarmed: needs a new ARM */
  arm(); speed_cmd(5000); tick(3); CHECK(starts == 1); /* beyond MAX_APPLICATION_SPEED_RPM: */
  CHECK(last_reply().reason == MOTOR_R_OUT_OF_RANGE);    /* rejected, not clamped */
}

static void test_stop_and_status_reply(void)
{
  setup(4); /* DISABLE, status reply */
  arm(); speed_cmd(200); tick(5); CHECK(state == RUN);
  iqd.q = 1234;
  HALL_M1.EdgesPos++; tick(10); HALL_M1.EdgesPos++; tick(1); /* edges 10 ms apart: 1250 / 10 = 125 RPM */
  speed_cmd(200); tick(1); /* the lifecycle answers with the loop's estimate of the tick before */ CHECK(last_reply().iq_ma == 2250); /* 1234 / 544 A in 50 mA steps */
  CHECK(last_reply_rpm == 125);
  rx(STATUS_READ | MOTOR_CAN_ID, 0); drive_app_poll(send);
  CHECK(sent_id == (STATUS_REPLY | MOTOR_CAN_ID));
  CHECK((int16_t)(sent[0] | sent[1] << 8) == 125 && (int16_t)(sent[2] | sent[3] << 8) == 1234);
  CHECK((int16_t)(sent[4] | sent[5] << 8) == iqdref.q && iqdref.q != 0 && sent[6] == RUN);
  CHECK(sent[7] == (ESC_FLAG_ACTIVE | ESC_FLAG_ALIGNED));
  disable(0xFU); tick(2);
  CHECK(state == IDLE && drive_app.stop_reason == ESC_STOP_COMMAND && !drive_app.active);
  iqd.q = 13270; /* bridge off: the SDK's Iq is garbage (uncalibrated offsets), reported as 0 */
  rx(STATUS_READ | MOTOR_CAN_ID, 0); drive_app_poll(send);
  CHECK(sent_id == (STATUS_REPLY | MOTOR_CAN_ID) && (int16_t)(sent[2] | sent[3] << 8) == 0);
  disable(MOTOR_CAN_ID); tick(1); CHECK(last_reply().iq_ma == 0);
  ctl(MOTOR_MODE_SPEED, 0, 100, ++ctl_seq, OTHER_WHEEL); tick(3); CHECK(starts == 1); /* other wheel's */
  rx((1U << 7) | MOTOR_CAN_ID, 100); tick(3); CHECK(starts == 1); /* retired SPEED_WRITE: ignored */
  speed_cmd(100); tick(3); CHECK(starts == 2 && state == RUN);   /* DISABLE kept the session */
  rx((15U << 7) | MOTOR_CAN_ID, 0); tick(3); CHECK(state == RUN); /* retired MOTOR_STOP: ignored */
  const uint8_t short_ctl[4] = { 0x11, 0, 100, 0 };
  drive_app_can_rx(MOTOR_CAN_ID_CONTROL | MOTOR_CAN_ID, short_ctl, 4); tick(1);
  CHECK(drive_cmd.count.bad_length == 1); /* wrong length: counted, never answered */
}

static void test_fault_latches_until_stop(void)
{
  setup(5); /* fault latch: off until DISABLE + CLEAR_FAULT, then a fresh ARM */
  arm(); speed_cmd(200); tick(5); CHECK(state == RUN);
  state = FAULT_NOW; cur_faults = 2; occ_faults = 2; tick(1);
  CHECK(drive_cmd.lifecycle == MOTOR_LC_FAULTED && drive_cmd.latched == MOTOR_R_FAULT_MCSDK);
  CHECK(drive_app.stop_reason == ESC_STOP_FAULT && !drive_app.active);
  clear_fault(); tick(1); CHECK(acks == 0); /* cause still present */
  CHECK(last_reply().reason == MOTOR_R_NOT_IDLE);
  cur_faults = 0; /* MCSDK still in FAULT_NOW with an empty fault word between safety passes */
  clear_fault(); tick(1); CHECK(acks == 0 && drive_cmd.lifecycle == MOTOR_LC_FAULTED);
  CHECK(last_reply().reason == MOTOR_R_NOT_IDLE && drive_cmd.latched == MOTOR_R_FAULT_MCSDK);
  state = FAULT_OVER; cur_faults = 0;
  for (int i = 0; i < 10; ++i) { speed_cmd(200); tick(20); }
  CHECK(state == FAULT_OVER && starts == 1); /* stays off while commands stream */
  CHECK(last_reply().reason == MOTOR_R_FAULT_MCSDK);
  disable(MOTOR_CAN_ID); tick(1); CHECK(drive_cmd.lifecycle == MOTOR_LC_FAULTED); /* DISABLE alone: no */
  clear_fault(); tick(1); CHECK(acks == 1 && state == IDLE && drive_cmd.lifecycle == MOTOR_LC_DISARMED);
  speed_cmd(200); tick(3); CHECK(starts == 1); /* no restart without ARM */
  arm(); speed_cmd(200); tick(3); CHECK(starts == 2 && state == RUN);
}

static void test_i2t_derates_and_recovers(void)
{
  setup(6); /* I2T derate and recovery */
  arm(); speed_cmd(300); tick(5); iqd.q = 6 * 544;
  int t = 0; /* Hall edges keep coming (2 per 10 ms), so this is heating, not a stall */
  while (!drive_app.derated && t < 3000) { speed_cmd(300); tick(10); HALL_M1.EdgesPos += 2; t += 10; }
  printf("derated after %d ms\n", t);
  CHECK(t > 1000 && t < 1400 && drive_app.iq_limit_counts == 3 * 544);
  tick(1); CHECK(iqdref.q <= 3 * 544); /* the loop obeys the derated limit at once */
  iqd.q = 2 * 544; t = 0;
  while (drive_app.derated && t < 10000) { speed_cmd(300); tick(10); HALL_M1.EdgesPos += 2; t += 10; }
  CHECK(!drive_app.derated && drive_app.iq_limit_counts == (int32_t)IQMAX);
}

static void test_config_only_when_idle(void)
{
  setup(7); /* CONFIG: acknowledged, read back, only when idle */
  struct motor_config_reply r = cfg(MOTOR_CONFIG_SET, MOTOR_CFG_SPEED_KP, 5000);
  CHECK(r.status == 0 && r.stored == 5000 && r.effective == 5000);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_HALL_SHIFT_DEG, -30).stored == -30);
  r = cfg(MOTOR_CONFIG_SET, MOTOR_CFG_IQ_LIMIT_MA, 4000);
  CHECK(r.status == 0 && r.stored == 4000 && r.effective == 4000);
  CHECK(drive_speed.cfg.kp == 5000 && fabs(shift_deg() + 30.0) < 0.01 && drive_app.aligned);
  CHECK(drive_app.iq_limit_counts == 2176);
  flush(); drive_app_poll(send); CHECK(sent_id == 0); /* nothing unsolicited */
  rx(DIAG_READ | MOTOR_CAN_ID, 0); drive_app_poll(send);
  CHECK(sent_id == (DIAG_REPLY | MOTOR_CAN_ID) && f16(sent + 4) == -30 && sent[6] == ESC_ALIGN_NONE);
  CHECK((sent[0] | sent[1] << 8) == 1236); /* 23831 * 34 * 100 / 65536 centivolts */
  r = cfg(MOTOR_CONFIG_SET, MOTOR_CFG_IQ_LIMIT_MA, (int16_t)(IQMAX_A * 1000 + 1)); /* above IQMAX: refused, nothing changes */
  CHECK(r.status == MOTOR_CFG_BAD_VALUE && r.stored == 4000 && drive_app.iq_limit_counts == 2176);
  CHECK(drive_app.rejected == 1);
  drive_app.derated = 1; drive_app.i2t_a2s = 40.0f; /* derated and still hot: the tick keeps it derated */
  r = cfg(MOTOR_CONFIG_GET, MOTOR_CFG_IQ_LIMIT_MA, 0);
  CHECK(r.status == 0 && r.stored == 4000 && r.effective == 3000);
  drive_app.iq_limit_counts = 3 * 544; /* as the I2T derate leaves it */
  r = cfg(MOTOR_CONFIG_SET, MOTOR_CFG_IQ_LIMIT_MA, 1000); /* lower than the derate: in force at once */
  CHECK(r.status == 0 && r.stored == 1000 && r.effective == 1000 && drive_app.iq_limit_counts == 544);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_IQ_LIMIT_MA, 4000).effective == 3000 && drive_app.iq_limit_counts == 3 * 544);
  drive_app.derated = 0; drive_app.i2t_a2s = 0.0f;
  drive_app.iq_limit_counts = drive_app.iq_max_counts;
  CHECK(cfg(MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0).stored == 5000);
  CHECK(cfg(MOTOR_CONFIG_SET, 0x0777, 1).status == MOTOR_CFG_UNKNOWN_KEY && drive_app.rejected == 2);
  arm(); speed_cmd(100); tick(3); CHECK(state == RUN);
  r = cfg(MOTOR_CONFIG_SET, MOTOR_CFG_SPEED_KP, 7000); /* running: refused, value reported */
  CHECK(r.status == MOTOR_R_NOT_IDLE && r.stored == 5000 && drive_speed.cfg.kp == 5000);
  CHECK(cfg(MOTOR_CONFIG_GET, MOTOR_CFG_SPEED_KP, 0).status == MOTOR_R_NOT_IDLE && drive_app.rejected == 4);
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); tick(1); CHECK(drive_app.align_phase == 0);
  disable(MOTOR_CAN_ID); tick(3); CHECK(state == IDLE);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_CURRENT_KP, 1500).status == 0);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_CURRENT_KI, 900).stored == 900);
  CHECK(PIDIqHandle_M1.kp == 1500 && PIDIdHandle_M1.kp == 1500);
  CHECK(PIDIqHandle_M1.ki == 900 && PIDIdHandle_M1.ki == 900);
  r = cfg(MOTOR_CONFIG_SET, MOTOR_CFG_CURRENT_KP, -5);
  CHECK(r.status == MOTOR_CFG_BAD_VALUE && r.stored == 1500 && PIDIqHandle_M1.kp == 1500);
  CHECK(drive_app.rejected == 6); /* 4 config refusals + the align refused while running + this */
  /* structural rejects are answered with their reason; a short frame is not answered at all */
  const uint8_t bad_op[8] = { 0x13, 0x41, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
  r = cfg_raw(MOTOR_CAN_ID_CONFIG | MOTOR_CAN_ID, bad_op);
  CHECK(r.status == MOTOR_R_BAD_MODE && r.seq == 0x41 && r.key == MOTOR_CFG_SPEED_KP && r.stored == 5000);
  drive_app_can_rx(MOTOR_CAN_ID_CONFIG | MOTOR_CAN_ID, bad_op, 4); tick(1);
  frames = 0; drive_app_poll(send); CHECK(frames == 0);
  /* two CONFIG frames before one tick: both applied and answered intact, in order; a third is dropped */
  uint32_t id; uint8_t d1[8], d2[8], d3[8];
  const struct motor_config c1 = { MOTOR_CONFIG_SET, 0x51, MOTOR_CFG_IQ_LIMIT_MA, 2000 };
  const struct motor_config c2 = { MOTOR_CONFIG_SET, 0x52, MOTOR_CFG_SPEED_KP, 4096 };
  const struct motor_config c3 = { MOTOR_CONFIG_SET, 0x53, MOTOR_CFG_SPEED_KI, 100 };
  CHECK(motor_config_encode(&c1, MOTOR_CAN_ID, &id, d1) == MOTOR_R_NONE);
  CHECK(motor_config_encode(&c2, MOTOR_CAN_ID, &id, d2) == MOTOR_R_NONE);
  CHECK(motor_config_encode(&c3, MOTOR_CAN_ID, &id, d3) == MOTOR_R_NONE);
  drive_app_can_rx(id, d1, 8); drive_app_can_rx(id, d2, 8); drive_app_can_rx(id, d3, 8);
  tick(1); frames = 0; drive_app_poll(send);
  struct motor_config_reply a, b;
  CHECK(frames == 2 && frame_id[0] == (MOTOR_CAN_ID_CONFIG_REPLY | MOTOR_CAN_ID));
  CHECK(motor_config_reply_decode(frame[0], 8, &a) == MOTOR_R_NONE && motor_config_reply_decode(frame[1], 8, &b) == MOTOR_R_NONE);
  CHECK(a.seq == 0x51 && a.status == 0 && a.key == MOTOR_CFG_IQ_LIMIT_MA && a.stored == 2000);
  CHECK(b.seq == 0x52 && b.status == 0 && b.key == MOTOR_CFG_SPEED_KP && b.stored == 4096);
  CHECK(drive_app.iq_max_ma == 2000 && drive_speed.cfg.kp == 4096 && drive_speed.cfg.ki != 100);
}

static void test_stop_aborts_alignment(void)
{
  setup(8); /* STOP aborts alignment; idle check for boot entry */
  CHECK(drive_app_idle());
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); CHECK(!drive_app_idle()); tick(600);
  disable(MOTOR_CAN_ID); tick(5);
  CHECK(drive_app.align_result == ESC_ALIGN_ABORTED && !drive_align_force && state == IDLE);
  CHECK(fabs(shift_deg() - 120.0) < 0.01 && drive_app_idle());
  /* An align or capture request queued before a DISABLE in the same tick never starts. */
  const int s0 = starts;
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); disable(0xFU); tick(20);
  CHECK(starts == s0 && drive_app.align_phase == 0 && !drive_align_force);
  rx(CAPTURE | MOTOR_CAN_ID, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); disable(0xFU); tick(20);
  CHECK(starts == s0 && drive_app.capture_status == ESC_CAPTURE_REJECTED && drive_app_idle());
}

static void test_stop_clears_finished_fault(void)
{
  setup(9); /* undervolt latched on USB power: CLEAR_FAULT clears it once gone */
  state = FAULT_OVER; cur_faults = 4; occ_faults = 4; tick(1);
  CHECK(drive_cmd.lifecycle == MOTOR_LC_FAULTED);
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); tick(2);
  CHECK(drive_app.align_phase == 0 && drive_app.rejected == 1); /* align needs IDLE, no fault */
  disable(0xFU); tick(1); CHECK(acks == 0 && state == FAULT_OVER); /* DISABLE does not clear */
  clear_fault(); tick(1); CHECK(acks == 0 && state == FAULT_OVER); /* cause still present */
  cur_faults = 0;
  clear_fault(); tick(1); CHECK(acks == 1 && state == IDLE && starts == 0);
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); tick(2); CHECK(drive_align_force && starts == 1);
}

static void test_ident_reports_validity_record(void)
{
  setup(10); /* IDENT_READ reports the bootloader validity record */
  rx(IDENT_READ | MOTOR_CAN_ID, 0); drive_app_poll(send);
  CHECK(sent_id == (IDENT_REPLY | MOTOR_CAN_ID));
  CHECK((uint32_t)(sent[0] | sent[1] << 8 | sent[2] << 16 | (uint32_t)sent[3] << 24) == 0xDEADBEEFU);
  CHECK((uint32_t)(sent[4] | sent[5] << 8 | sent[6] << 16 | (uint32_t)sent[7] << 24) == 45344U);
  rx(IDENT_READ | OTHER_WHEEL, 0); frames = 0; drive_app_poll(send); CHECK(frames == 0); /* other wheel */
}

static void test_capture_steps_id_and_reads_out(void)
{
  setup(11); /* capture: hold, sample-exact Id step, readout */
  const uint32_t start = (uint32_t)ESC_CAPTURE_START << 24;
  rx(CAPTURE | MOTOR_CAN_ID, start | (uint32_t)(IQMAX_A * 1000 + 1)); tick_isr(2); /* above IQMAX */
  CHECK(drive_app.capture_status == ESC_CAPTURE_REJECTED && starts == 0);
  rx(CAPTURE | MOTOR_CAN_ID, start | 3000U); tick_isr(2);
  CHECK(drive_align_force && drive_align_angle == 0 && starts == 1);
  CHECK(drive_app.capture_status == ESC_CAPTURE_RUNNING && !drive_app_idle());
  arm(); speed_cmd(100); tick_isr(1); CHECK(starts == 1); /* motion refused while capturing */
  rx(HALL_ALIGN | MOTOR_CAN_ID, HALL_ALIGN_MAGIC); tick_isr(1); CHECK(drive_app.align_phase == 0);
  int t = 0; while (drive_app.capture_status == ESC_CAPTURE_RUNNING && t < 2000) { tick_isr(1); ++t; }
  printf("capture done after %d ms\n", t);
  CHECK(drive_app.capture_status == ESC_CAPTURE_DONE && t > 500 && t < 600);
  tick_isr(3);
  CHECK(state == IDLE && !drive_align_force && drive_app_idle() && drive_app.stop_reason == ESC_STOP_CAPTURE);
  const uint32_t read = (uint32_t)ESC_CAPTURE_READ << 24;
  rx(CAPTURE | MOTOR_CAN_ID, read | (1U << 16) | (8U << 10) | 60U); frames = 0; drive_app_poll(send);
  CHECK(frames == 8 && frame_id[0] == (CAPTURE_REPLY | MOTOR_CAN_ID));
  for (int k = 0; k < 8; ++k) {
    CHECK((frame[k][0] | frame[k][1] << 8) == (1 << 12 | (60 + k)));
    CHECK(f16(frame[k] + 4) == (60 + k < 64 ? 544 : 1632)); /* Id ref: hold 1 A, step 3 A */
  }
  rx(CAPTURE | MOTOR_CAN_ID, read | (0U << 16) | (32U << 10) | 64U); frames = 0; drive_app_poll(send);
  CHECK(frames == 32);
  CHECK(f16(frame[31] + 6) == 2 * 1632); /* Vd */
  rx(CAPTURE | MOTOR_CAN_ID, read | (0U << 16) | (63U << 10) | 500U); frames = 0; drive_app_poll(send);
  CHECK(frames == 12); /* clamped to the buffer end */
  send_budget = 3; rx(CAPTURE | MOTOR_CAN_ID, read | (20U << 10)); frames = 0; drive_app_poll(send);
  CHECK(frames == 3); send_budget = 1000; drive_app_poll(send); CHECK(frames == 20); /* resumes */
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16)); frames = 0; drive_app_poll(send);
  CHECK(frames == 1 && (frame[0][0] | frame[0][1] << 8) == 3 << 12);
  CHECK(f16(frame[0] + 2) == ESC_CAPTURE_DONE && f16(frame[0] + 4) == 1236 && f16(frame[0] + 6) == 512);
}

static void test_capture_stop_and_busy(void)
{
  setup(12); /* STOP aborts a capture; busy ESC rejects one */
  const uint32_t start = (uint32_t)ESC_CAPTURE_START << 24;
  drive_app.aligned = 1; arm(); speed_cmd(100); tick_isr(3); CHECK(state == RUN);
  rx(CAPTURE | MOTOR_CAN_ID, start | 2000U); tick_isr(1);
  CHECK(drive_app.capture_status == ESC_CAPTURE_REJECTED && !drive_align_force);
  disable(MOTOR_CAN_ID); tick_isr(3); CHECK(state == IDLE);
  rx(CAPTURE | MOTOR_CAN_ID, start | 2000U); tick_isr(300);
  CHECK(drive_app.capture_status == ESC_CAPTURE_RUNNING && iqdref.d > 0);
  disable(0xFU); tick_isr(3);
  CHECK(drive_app.capture_status == ESC_CAPTURE_ABORTED && state == IDLE && !drive_align_force);
}

/* Stall keys with their bounds; the retired 4-byte CONFIG_SET frame is ignored. */
static void test_speed_config_keys(void)
{
  setup(13);
  rx((10U << 7) | MOTOR_CAN_ID, (1000U << 16) | 1U); tick(1); /* old CONFIG_SET Kp = 1000 */
  frames = 0; drive_app_poll(send);
  CHECK(frames == 0 && drive_speed.cfg.kp == PID_SPEED_KP_DEFAULT && drive_app.rejected == 0);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_STALL_RAMP_MA_PER_S, 1999).status == MOTOR_CFG_BAD_VALUE);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_STALL_RAMP_MA_PER_S, 30001).status == MOTOR_CFG_BAD_VALUE);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_STALL_MS, 199).status == MOTOR_CFG_BAD_VALUE);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_STALL_MS, 2001).status == MOTOR_CFG_BAD_VALUE);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_SPEED_KI, -1).status == MOTOR_CFG_BAD_VALUE);
  CHECK(drive_app.rejected == 5);
  CHECK(drive_speed.cfg.stall_ramp_ma_per_s == SPEED_STALL_RAMP_MA_PER_S_DEFAULT);
  CHECK(drive_speed.cfg.stall_ms == SPEED_STALL_MS_DEFAULT && drive_speed.cfg.ki == PID_SPEED_KI_DEFAULT);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_STALL_RAMP_MA_PER_S, 12000).stored == 12000);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_STALL_MS, 800).stored == 800);
  CHECK(cfg(MOTOR_CONFIG_SET, MOTOR_CFG_SPEED_KP, 2048).stored == 2048);
  CHECK(drive_app.rejected == 5 && drive_speed.cfg.stall_ms == 800 && drive_speed.cfg.kp == 2048);
  CHECK(drive_speed.stall_step_q14 == 106955); /* 12 A/s: 12000 * 544 * 16384 / 1e6, rounded */
  CHECK(cfg(MOTOR_CONFIG_GET, MOTOR_CFG_STALL_RAMP_MA_PER_S, 0).stored == 12000);
}

/* Zero target at standstill: HOLD keeps the bridge on at Iq 0 and resumes without an MCSDK start. */
static void test_holds_at_zero_command(void)
{
  setup(14);
  drive_app.aligned = 1;
  arm(); speed_cmd(100); tick(3); CHECK(state == RUN);
  for (int i = 0; i < 7; ++i) { speed_cmd(0); tick(20); } /* no Hall edges: standstill */
  CHECK(drive_cmd.lifecycle == MOTOR_LC_RUNNING); /* 140 ms */
  speed_cmd(0); tick(20);
  CHECK(drive_cmd.lifecycle == MOTOR_LC_HOLD && state == RUN && drive_app.active && starts == 1);
  CHECK(iqdref.q == 0 && iqdref.d == 0 && drive_app.stop_reason == ESC_STOP_NONE);
  flush(); rx(STATUS_READ | MOTOR_CAN_ID, 0); drive_app_poll(send);
  CHECK((sent[7] & ESC_FLAG_ACTIVE) && (sent[7] & 0x0F) == ESC_STOP_NONE);
  speed_cmd(80); tick(1);
  CHECK(drive_cmd.lifecycle == MOTOR_LC_RUNNING && starts == 1 && drive_speed.ref_to_rpm == 80);
  tick(5); CHECK(iqdref.q > 0); /* P on the error again */
  for (int i = 0; i < 9; ++i) { speed_cmd(0); tick(20); }
  CHECK(drive_cmd.lifecycle == MOTOR_LC_HOLD);
  disable(MOTOR_CAN_ID); tick(3);
  CHECK(state == IDLE && !drive_app.active && drive_app.stop_reason == ESC_STOP_COMMAND);
}

/* CURRENT mode: the setpoint is the q-current, clamped to the (derated) limit; the speed loop and its stall
 * monitor stay out, so a wheel held still never faults. */
static void test_current_mode_drives_q_current(void)
{
  setup(19);
  drive_app.aligned = 1;
  arm(); ctl(MOTOR_MODE_CURRENT, 0, 1500, ++ctl_seq, MOTOR_CAN_ID); tick(5);
  CHECK(state == RUN && drive_cmd.lifecycle == MOTOR_LC_RUNNING && drive_cmd.mode == MOTOR_MODE_CURRENT);
  CHECK(iqdref.q == 1500 * 544 / 1000 && iqdref.d == 0);
  for (int i = 0; i < 100; ++i) { ctl(MOTOR_MODE_CURRENT, 0, 1500, ++ctl_seq, MOTOR_CAN_ID); tick(20); }
  CHECK(drive_cmd.lifecycle == MOTOR_LC_RUNNING && iqdref.q == 816); /* 2 s without an edge: no stall */
  drive_app.iq_limit_counts = 3 * 544; /* as after an I2T derate */
  ctl(MOTOR_MODE_CURRENT, 0, 5000, ++ctl_seq, MOTOR_CAN_ID); tick(2);
  CHECK(iqdref.q == 3 * 544 && last_reply().reason == MOTOR_R_LIMITED);
  ctl(MOTOR_MODE_CURRENT, 0, -5000, ++ctl_seq, MOTOR_CAN_ID); tick(2);
  CHECK(iqdref.q == -3 * 544);
  for (int i = 0; i < 10; ++i) { ctl(MOTOR_MODE_CURRENT, 0, 0, ++ctl_seq, MOTOR_CAN_ID); tick(20); }
  CHECK(drive_cmd.lifecycle == MOTOR_LC_RUNNING && iqdref.q == 0); /* zero current is a command, not HOLD */
  /* I2T in current mode: a held 6 A request derates the clamp to 3 A on its own, then recovers. */
  drive_app.iq_limit_counts = drive_app.iq_max_counts; /* undo the manual derate above */
  iqd.q = 6 * 544; /* what the current loop then measures */
  int t = 0;
  while (!drive_app.derated && t < 3000) { ctl(MOTOR_MODE_CURRENT, 0, 6000, ++ctl_seq, MOTOR_CAN_ID); tick(20); t += 20; }
  printf("current mode derated after %d ms\n", t);
  CHECK(drive_app.derated && t > 1000 && t < 1400 && iqdref.q == 3 * 544);
  iqd.q = 2 * 544; t = 0;
  while (drive_app.derated && t < 10000) { ctl(MOTOR_MODE_CURRENT, 0, 6000, ++ctl_seq, MOTOR_CAN_ID); tick(20); t += 20; }
  CHECK(!drive_app.derated);
  ctl(MOTOR_MODE_CURRENT, 0, 6000, ++ctl_seq, MOTOR_CAN_ID); tick(2);
  CHECK(iqdref.q == 6 * 544); /* full limit back */
  disable(MOTOR_CAN_ID); tick(3); CHECK(state == IDLE);
}

/* Torque mode: Iq 0 programmed before the start, the loop's request published in RUN; stall and Hall
 * loss end as latched faults with the output off. */
static void test_speed_loop_drives_and_faults(void)
{
  setup(15);
  drive_app.aligned = 1;
  iqdref = (qd_t){ .q = 777, .d = 55 };
  arm(); speed_cmd(100); tick(1);
  CHECK(starts == 1 && iqdref.q == 0 && iqdref.d == 0);
  tick(4); CHECK(state == RUN && iqdref.q > 0 && iqdref.d == 0);
  int t = 0; /* the wheel never moves (no Hall edges): stall ramp, then the stall timer */
  while (drive_cmd.lifecycle != MOTOR_LC_FAULTED && t < 3000) { speed_cmd(100); tick(20); t += 20; }
  printf("stall fault after %d ms\n", t);
  CHECK(drive_cmd.latched == MOTOR_R_FAULT_STALL && t > 800 && t < 1300);
  CHECK(state == IDLE && drive_app.stop_reason == ESC_STOP_FAULT);

  setup(15);
  drive_app.aligned = 1;
  arm(); speed_cmd(100); tick(5); CHECK(state == RUN);
  const uint8_t good = HALL_M1.HallState;
  hall_dead = 1;
  speed_cmd(100); tick(20); CHECK(drive_cmd.lifecycle == MOTOR_LC_RUNNING); /* within the stale window */
  speed_cmd(100); tick(4); /* the line reads 7 from the next tick: 21 ms bad, fault, MCSDK stops */
  CHECK(drive_cmd.latched == MOTOR_R_FAULT_HALL_LOST && state == IDLE);
  hall_dead = 0; HALL_M1.HallState = good;

  setup(15);
  drive_app.aligned = 1;
  arm(); speed_cmd(100); tick(5); CHECK(state == RUN);
  state = FAULT_NOW; cur_faults = MC_SPEED_FDBK; occ_faults = MC_SPEED_FDBK; tick(1);
  CHECK(drive_cmd.latched == MOTOR_R_FAULT_HALL_LOST);
}

static void test_speed_loop_recorder(void)
{
  setup(16); /* speed-loop recorder */
  drive_app.aligned = 1;
  const uint32_t rec = (uint32_t)ESC_RECORD_START << 24, read = (uint32_t)ESC_CAPTURE_READ << 24;
  rx(CAPTURE | MOTOR_CAN_ID, rec | 384U); tick(1);                  /* pre out of range */
  CHECK(drive_app.rejected == 1);
  rx(CAPTURE | MOTOR_CAN_ID, rec | 10U); tick(1);                   /* pre 10, no delay */
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
  CHECK(frames == 1 && (frame[0][0] | frame[0][1] << 8) == (3 << 12 | 1));
  CHECK(f16(frame[0] + 2) == ESC_RECORD_ARMED);
  rx(CAPTURE | MOTOR_CAN_ID, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); tick(2);
  CHECK(starts == 0 && drive_app.capture_status == ESC_CAPTURE_REJECTED); /* shares RAM */
  tick(20);
  iqd.q = 99;
  arm(); speed_cmd(100);
  for (int i = 0; i < 20; ++i) { tick(20); speed_cmd(100); }
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 4) == 384 && f16(frame[0] + 6) == 10);
  /* trigger: the first loop tick with |ref| >= 10 RPM; 100 RPM over 10 ticks starts at 10 */
  rx(CAPTURE | MOTOR_CAN_ID, read | (0U << 16) | (2U << 10) | 9U); frames = 0; drive_app_poll(send);
  CHECK(frames == 2 && f16(frame[0] + 2) == 0 && f16(frame[1] + 2) == 10); /* ref around the trigger */
  CHECK(f16(frame[1] + 4) == 0 && f16(frame[1] + 6) == 40);              /* estimate (no edges), P = 4 x 10 */
  rx(CAPTURE | MOTOR_CAN_ID, read | (1U << 16) | (1U << 10) | 10U); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == 0 && f16(frame[0] + 4) == 0 && f16(frame[0] + 6) == 40); /* I (Q14 < 1), ff, Iq ref */
  rx(CAPTURE | MOTOR_CAN_ID, read | ESC_RECORD_READ_CH67 | (1U << 10) | 10U); frames = 0; drive_app_poll(send);
  CHECK(frames == 1 && f16(frame[0] + 2) == 99 && f16(frame[0] + 6) == 0); /* measured Iq; pad */
  const uint16_t packed = (uint16_t)f16(frame[0] + 4);
  CHECK((packed & 3U) == ESC_REC_MODE_RUN && !(packed & 0x3CU) && (packed & 0x8000U));
  CHECK(((packed >> 12) & 7U) == HALL_M1.HallState);
  rx(CAPTURE | MOTOR_CAN_ID, read | (2U << 16) | (1U << 10) | 10U); frames = 0; drive_app_poll(send);
  CHECK(frames == 0);                                                     /* no bank 2 */
  rx(CAPTURE | MOTOR_CAN_ID, read | (0U << 16) | (32U << 10) | 380U); frames = 0; drive_app_poll(send);
  CHECK(frames == 4);                                       /* clamped to 384 samples */
  disable(MOTOR_CAN_ID); tick(5); CHECK(state == IDLE);
  rx(CAPTURE | MOTOR_CAN_ID, rec | (100U << 9)); tick(1);              /* delay 100 ms, pre 0 */
  arm(); speed_cmd(100);
  for (int k = 0; k < 100; ++k) { iqd.q = (int16_t)k; tick(1); if (k % 20 == 19) speed_cmd(100); }
  iqd.q = 1000;
  for (int i = 0; i < 20; ++i) { tick(20); speed_cmd(100); }
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 6) == 0);
  rx(CAPTURE | MOTOR_CAN_ID, read | ESC_RECORD_READ_CH67 | (1U << 10) | 0U); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == 1000);                         /* window starts ~100 ms after the trigger */
  disable(MOTOR_CAN_ID); tick(5); CHECK(state == IDLE);
  rx(CAPTURE | MOTOR_CAN_ID, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); tick(2);
  CHECK(drive_app.capture_status == ESC_CAPTURE_RUNNING);   /* recorder done: allowed */
  rx(CAPTURE | MOTOR_CAN_ID, rec | 10U); tick(1);
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == ESC_RECORD_REJECTED);          /* capture running */
}

static void test_recorder_idle_arm_and_early_stop(void)
{
  setup(17); /* recorder armed while IDLE, real start, early stop */
  const uint32_t rec = (uint32_t)ESC_RECORD_START << 24, read = (uint32_t)ESC_CAPTURE_READ << 24;
  drive_app.aligned = 1;
  rx(CAPTURE | MOTOR_CAN_ID, rec | 20U); tick(5);    /* IDLE: loop not running, samples read ref 0 */
  arm(); speed_cmd(51);                    /* ARM and motion in the same tick */
  int start_ms = 0; while (state != RUN && start_ms < 10) { tick(1); ++start_ms; }
  CHECK(state == RUN);
  /* samples before the trigger: IDLE and motor start-up (its last tick runs the loop at ref 5);
   * the next loop tick triggers at 10 RPM (51 / 10 per tick) */
  const int pre = 5 + start_ms;
  CHECK(pre < 20);
  tick(40);
  disable(MOTOR_CAN_ID); tick(3); CHECK(state == IDLE);   /* stopped before the window filled */
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == ESC_RECORD_TRIGGERED);
  tick(400);                               /* keeps sampling while stopped, then done */
  rx(CAPTURE | MOTOR_CAN_ID, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
  /* fewer samples before the trigger than pre 20: the window starts at sample 0 and holds
   * them plus (384 - 20) samples */
  CHECK(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 6) == pre && f16(frame[0] + 4) == pre + 384 - 20);
  rx(CAPTURE | MOTOR_CAN_ID, read | (0U << 16) | (2U << 10) | (uint32_t)(pre - 1)); frames = 0; drive_app_poll(send);
  CHECK(f16(frame[0] + 2) == 5 && f16(frame[1] + 2) == 10); /* trigger at the first |ref| >= 10 RPM */
  rx(CAPTURE | MOTOR_CAN_ID, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); tick(2);
  CHECK(drive_app.capture_status == ESC_CAPTURE_RUNNING); /* buffer released */
}

int main(int argc, char **argv)
{
  static const struct { const char *name; void (*run)(void); } cases[] = {
    { "test_alignment_detects_reversed_wiring", test_alignment_detects_reversed_wiring },
    { "test_alignment_detects_stuck_rotor", test_alignment_detects_stuck_rotor },
    { "test_streaming_and_watchdog", test_streaming_and_watchdog },
    { "test_stop_and_status_reply", test_stop_and_status_reply },
    { "test_fault_latches_until_stop", test_fault_latches_until_stop },
    { "test_i2t_derates_and_recovers", test_i2t_derates_and_recovers },
    { "test_config_only_when_idle", test_config_only_when_idle },
    { "test_stop_aborts_alignment", test_stop_aborts_alignment },
    { "test_stop_clears_finished_fault", test_stop_clears_finished_fault },
    { "test_ident_reports_validity_record", test_ident_reports_validity_record },
    { "test_capture_steps_id_and_reads_out", test_capture_steps_id_and_reads_out },
    { "test_capture_stop_and_busy", test_capture_stop_and_busy },
    { "test_speed_config_keys", test_speed_config_keys },
    { "test_holds_at_zero_command", test_holds_at_zero_command },
    { "test_speed_loop_drives_and_faults", test_speed_loop_drives_and_faults },
    { "test_current_mode_drives_q_current", test_current_mode_drives_q_current },
    { "test_speed_loop_recorder", test_speed_loop_recorder },
    { "test_recorder_idle_arm_and_early_stop", test_recorder_idle_arm_and_early_stop },
  };
  if (argc != 2) { fprintf(stderr, "usage: %s <case>\n", argv[0]); return 1; }
  const char *name = argv[1];
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    if (strcmp(name, cases[i].name) == 0) {
      cases[i].run();
      puts(name);
      return 0;
    }
  }
  fprintf(stderr, "Unknown test: %s\n", name);
  return 1;
}
