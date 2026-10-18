#!/usr/bin/env python3
"""Compile the real ESC drive_app.c on the host against a simulated MCSDK.

The fake motor follows the forced alignment angle and produces Hall edges
with the same angle table as hall_speed_pos_fdbk.c, so the alignment math,
watchdog, fault latch, I2T derate and CAN decoding are all exercised.
"""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
ESC = ROOT / "HW/B-G431B-ESC1"

STUBS = {
"mc_api.h": r'''
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct { int16_t q, d; } qd_t;
enum { IDLE = 0, CHARGE_BOOT_CAP = 16, OFFSET_CALIB = 17, RUN = 6, STOP = 8,
       FAULT_NOW = 10, FAULT_OVER = 11 };
#define MC_NO_FAULTS 0U
#define U_RPM 60
#define SPEED_UNIT U_RPM
#define RPM_2_SPEED_UNIT(rpm) ((int16_t)(((rpm)*SPEED_UNIT)/U_RPM))
#define SPEED_UNIT_2_RPM(speed) ((int16_t)(((speed)*U_RPM)/SPEED_UNIT))
bool MC_StartMotor1(void); bool MC_StopMotor1(void);
void MC_ProgramSpeedRampMotor1(int16_t, uint16_t);
void MC_SetCurrentReferenceMotor1(qd_t);
int MC_GetSTMStateMotor1(void);
uint16_t MC_GetCurrentFaultsMotor1(void); uint16_t MC_GetOccurredFaultsMotor1(void);
bool MC_AcknowledgeFaultMotor1(void);
int16_t MC_GetMecSpeedAverageMotor1(void);
int16_t MC_GetMecSpeedReferenceMotor1(void);
qd_t MC_GetIqdMotor1(void); qd_t MC_GetIqdrefMotor1(void);
''',
"pid_regulator.h": r'''
#pragma once
#include <stdint.h>
typedef struct { int16_t kp, ki; int16_t up, low; int32_t iup, ilow; uint16_t kidiv; int32_t wIntegralTerm; } PID_Handle_t;
int16_t PI_Controller(PID_Handle_t *p, int32_t err);
''',
"mc_config.h": r'''
#pragma once
#include <stdint.h>
#include "pid_regulator.h"
typedef struct { int16_t PhaseShift; int8_t Direction; uint8_t HallState; int16_t MeasuredElAngle;
                 uint16_t EdgesPos, EdgesNeg, Captures; } HALL_Handle_t;
extern PID_Handle_t PIDSpeedHandle_M1, PIDIqHandle_M1, PIDIdHandle_M1; extern HALL_Handle_t HALL_M1;
static inline void PID_SetKP(PID_Handle_t *p, int16_t v) { p->kp = v; }
static inline void PID_SetKI(PID_Handle_t *p, int16_t v) { p->ki = v; }
static inline uint16_t PID_GetKIDivisor(PID_Handle_t *p) { return p->kidiv; }
static inline void PID_SetUpperOutputLimit(PID_Handle_t *p, int16_t v) { p->up = v; }
static inline void PID_SetLowerOutputLimit(PID_Handle_t *p, int16_t v) { p->low = v; }
static inline void PID_SetUpperIntegralTermLimit(PID_Handle_t *p, int32_t v) { p->iup = v; }
static inline void PID_SetLowerIntegralTermLimit(PID_Handle_t *p, int32_t v) { p->ilow = v; }
''',
"mc_config_common.h": r'''
#pragma once
#include <stdint.h>
typedef struct { uint16_t ConversionFactor; uint16_t raw; } BusVoltageSensor_Handle_t;
typedef struct { BusVoltageSensor_Handle_t _Super; } RDivider_Handle_t;
extern RDivider_Handle_t BusVoltageSensor_M1;
static inline uint16_t VBS_GetAvBusVoltage_d(const BusVoltageSensor_Handle_t *p) { return p->raw; }
''',
"test_hooks.h": r'''
#pragma once
#include <stdint.h>
extern uint32_t fake_validity[8];
''',
"parameters_conversion.h": r'''
#pragma once
#define CURRENT_CONV_FACTOR 544
#define IQMAX_A 6
#define IQMAX (IQMAX_A * CURRENT_CONV_FACTOR)
#define MAX_APPLICATION_SPEED_RPM 2976
#define FRICTION_FF_STATIC_MA_DEFAULT 0
#define FRICTION_FF_KINETIC_MA_DEFAULT 0
#define DSC_ENABLE_DEFAULT 0
#define DSC_FF_RUN_MA_DEFAULT 600
#define DSC_FF_PER_KRPM_MA_DEFAULT 0
#define DSC_START_FROM_MA_DEFAULT 500
#define DSC_START_CAP_MA_DEFAULT 3000
#define DSC_START_SLOPE_MA_DEFAULT 16
#define DSC_START_HOLD_MS_DEFAULT 150
#define DSC_PCAP_MA_DEFAULT 500
#define DSC_PCAP_MS_DEFAULT 50
#define DSC_PCAP_UPDATES_DEFAULT 2
#define DSC_PCAP_FALLBACK_MS_DEFAULT 150
#define SPEED_REV_FILTER_DEFAULT 1
''',
}

HARNESS = r'''
#include "drive_app.h"
#include "can_datatypes.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_config_common.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

PID_Handle_t PIDSpeedHandle_M1 = { .kidiv = 16384 }, PIDIqHandle_M1, PIDIdHandle_M1;
HALL_Handle_t HALL_M1;
static int pi_calls, dsc_calls, dsc_resets, fake_next_state = -1; static dsc_in_t dsc_last_in;
int16_t PI_Controller(PID_Handle_t *p, int32_t err) { (void)p; ++pi_calls; return (int16_t)err; }
void dsc_reset(dsc_state_t *st) { *st = (dsc_state_t){0}; ++dsc_resets; }
static int rebases;
void dsc_rev_rebase(dsc_rev_t *f, uint16_t p, uint16_t n, uint16_t c) { (void)f; (void)p; (void)n; (void)c; ++rebases; }
int16_t dsc_reversal_filter(dsc_rev_t *f, uint16_t pos, uint16_t neg, uint16_t caps, int16_t rpm) {
  f->qpos = (uint16_t)(pos + 100); f->qneg = (uint16_t)(neg + 100); f->qcaps = (uint16_t)(caps + 100);
  return (int16_t)(rpm / 2); /* marks "filtered" */
}
int16_t dsc_step(const dsc_cfg_t *c, dsc_state_t *st, const dsc_in_t *in, PID_Handle_t *p) {
  (void)c; (void)p; ++dsc_calls; dsc_last_in = *in;
  if (fake_next_state >= 0) st->state = (uint8_t)fake_next_state;
  return 123;
}
RDivider_Handle_t BusVoltageSensor_M1 = { ._Super = { .ConversionFactor = 34, .raw = 23831 } };

/* ---- fake MCSDK state machine ---- */
static int state = IDLE, start_pending, stop_pending, starts, acks;
static uint16_t cur_faults, occ_faults;
static int16_t ramp_rpm, speed; static uint16_t ramp_ms;
static qd_t iqd, iqdref;
bool MC_StartMotor1(void) { if (state != IDLE) return false; start_pending = 1; ++starts; return true; }
bool MC_StopMotor1(void) { stop_pending = 1; return true; }
void MC_ProgramSpeedRampMotor1(int16_t rpm, uint16_t ms) { ramp_rpm = rpm; ramp_ms = ms; }
void MC_SetCurrentReferenceMotor1(qd_t r) { iqdref = r; }
int MC_GetSTMStateMotor1(void) { return state; }
uint16_t MC_GetCurrentFaultsMotor1(void) { return cur_faults; }
uint16_t MC_GetOccurredFaultsMotor1(void) { return occ_faults; }
bool MC_AcknowledgeFaultMotor1(void) { if (state != FAULT_OVER) return false; state = IDLE; occ_faults = 0; ++acks; return true; }
int16_t MC_GetMecSpeedAverageMotor1(void) { return speed; }
static int16_t speed_ref;
int16_t MC_GetMecSpeedReferenceMotor1(void) { return speed_ref; }
qd_t MC_GetIqdMotor1(void) { return iqd; }
qd_t MC_GetIqdrefMotor1(void) { return iqdref; }

/* ---- fake motor + Hall decoder (hall_speed_pos_fdbk.c table) ---- */
static const uint8_t SEQ[6] = {5, 1, 3, 2, 6, 4};
static double true_shift_deg = 77.0, lag_deg = 6.0; static int reversed, stuck;
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
static double last_forced;
static void motor_step(void) {
  if (state == RUN && drive_align_force && iqdref.d > 300 && !stuck) {
    double forced = (uint16_t)drive_align_angle * 360.0 / 65536.0;
    while (forced - last_forced > 180) forced -= 360;
    while (forced - last_forced < -180) forced += 360;
    double dir = forced > last_forced ? 1 : forced < last_forced ? -1 : 0;
    last_forced = forced;
    rotor_deg = forced - dir * lag_deg;
  }
  uint8_t h = hall_at(rotor_deg);
  if (h != HALL_M1.HallState) hall_isr(h);
}
static int feed_rec; static int16_t rec_pi = 300, rec_iq = 500;
static void mf_step(void) {
  if (feed_rec || state == RUN) drive_record_tick(rec_pi, rec_iq);
  if (stop_pending) { stop_pending = 0; start_pending = 0; if (state == RUN || state == CHARGE_BOOT_CAP) state = IDLE; }
  if (start_pending && state == IDLE) { start_pending = 0; state = CHARGE_BOOT_CAP; }
  else if (state == CHARGE_BOOT_CAP) state = RUN;
}
static void tick(int n) { while (n--) { mf_step(); drive_app_tick(); motor_step(); } }
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
/* FOC ISR: 16 periods per ms, Id follows its reference with lag; Vd = 0.5 V/A-ish */
static int16_t foc_id;
static void foc_isr(int n) {
  while (n--) {
    int16_t ref_d = iqdref.d; drive_capture_isr_ref(&ref_d);
    if (state == RUN) foc_id = (int16_t)(foc_id + (ref_d - foc_id) / 4); else foc_id = 0;
    drive_capture_isr_store(foc_id, 0, (int16_t)(2 * ref_d), 0, ref_d, drive_align_angle);
  }
}
static void tick_isr(int n) { while (n--) { mf_step(); drive_app_tick(); foc_isr(16); motor_step(); } }
static void speed_cmd(int rpm, int ms) { rx(SPEED_WRITE | 1U, ((uint32_t)(uint16_t)ms << 16) | (uint16_t)rpm); }
static double shift_deg(void) { return HALL_M1.PhaseShift * 360.0 / 65536.0; }

int main(int argc, char **argv) {
  const int scenario = atoi(argv[1]);
  drive_app_init();
  HALL_M1.PhaseShift = (int16_t)(120 * 65536 / 360);
  HALL_M1.HallState = hall_at(0);
  if (scenario >= 3 && scenario <= 6) drive_app.aligned = 1; /* as if built with --hall-shift */
  if (scenario <= 2) { /* alignment: 0 ok, 1 reversed wiring, 2 rotor stuck */
    reversed = scenario == 1; stuck = scenario == 2;
    HALL_M1.HallState = hall_at(0);
    tick(5); rx(HALL_ALIGN | 1U, 0x12345678U); tick(5); assert(drive_app.align_phase == 0); /* bad magic */
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); tick(2);
    assert(drive_align_force && drive_app.align_result == ESC_ALIGN_RUNNING);
    speed_cmd(100, 100); tick(1); assert(starts == 1); /* speed ignored while aligning */
    tick(5000);
    assert(!drive_align_force && state == IDLE && drive_app.align_phase == 0);
    if (scenario == 0) {
      assert(drive_app.align_result == ESC_ALIGN_OK);
      assert(drive_app.align_edges_fwd >= 11 && drive_app.align_edges_back >= 11);
      double err = shift_deg() - true_shift_deg;
      printf("shift %.2f true %.2f fwd %u back %u\n", shift_deg(), true_shift_deg,
             drive_app.align_edges_fwd, drive_app.align_edges_back);
      assert(fabs(err) < 1.0 && drive_app.aligned);
      speed_cmd(100, 100); tick(3); assert(starts == 2 && state == RUN); /* now allowed */
      drive_app_poll(send); assert(sent_id == 0);
      rx(DIAG_READ | 1U, 0); drive_app_poll(send);
      assert(sent_id == (DIAG_REPLY | 1U) && (int16_t)(sent[4] | sent[5] << 8) == 77 && sent[6] == ESC_ALIGN_OK);
      assert((sent[0] | sent[1] << 8) == 1236); /* 23831 * 34 * 100 / 65536 centivolts */
    } else {
      assert(drive_app.align_result == (scenario == 1 ? ESC_ALIGN_REVERSED : ESC_ALIGN_NO_EDGES));
      assert(fabs(shift_deg() - 120.0) < 0.01 && !drive_app.aligned);
      speed_cmd(100, 100); tick(3); assert(starts == 1 && state == IDLE); /* refused */
    }
  } else if (scenario == 3) { /* stream, watchdog */
    speed_cmd(0, 100); tick(3); assert(starts == 0); /* zero does not start */
    speed_cmd(150, 200); tick(3); assert(starts == 1 && state == RUN && drive_app.active);
    assert(ramp_rpm == 150 && ramp_ms == 200);
    for (int i = 0; i < 50; ++i) { speed_cmd(-90, 100); tick(20); }
    assert(state == RUN && ramp_rpm == -90 && starts == 1);
    tick(225); assert(state == RUN); /* 245 ms since the last command */
    tick(10); assert(state == IDLE && !drive_app.active && drive_app.stop_reason == ESC_STOP_TIMEOUT);
    speed_cmd(5000, 0); tick(3); assert(ramp_rpm == 2976 && starts == 2); /* clamped */
  } else if (scenario == 4) { /* STOP, status reply */
    speed_cmd(200, 100); tick(5); assert(state == RUN);
    iqd.q = 1234; iqdref.q = 1300; speed = 187;
    rx(STATUS_READ | 1U, 0); drive_app_poll(send);
    assert(sent_id == (STATUS_REPLY | 1U));
    assert((int16_t)(sent[0] | sent[1] << 8) == 187 && (int16_t)(sent[2] | sent[3] << 8) == 1234);
    assert((int16_t)(sent[4] | sent[5] << 8) == 1300 && sent[6] == RUN);
    assert(sent[7] == (ESC_FLAG_ACTIVE | ESC_FLAG_ALIGNED));
    rx(MOTOR_STOP | 0xFU, 0); tick(2);
    assert(state == IDLE && drive_app.stop_reason == ESC_STOP_COMMAND && !drive_app.active);
    rx(SPEED_WRITE | 2U, 100); tick(3); assert(starts == 1); /* other wheel's address */
  } else if (scenario == 5) { /* fault latch */
    speed_cmd(200, 100); tick(5); assert(state == RUN);
    state = FAULT_NOW; cur_faults = 2; occ_faults = 2; tick(1);
    assert(drive_app.fault_latched && drive_app.stop_reason == ESC_STOP_FAULT);
    state = FAULT_OVER; cur_faults = 0;
    for (int i = 0; i < 10; ++i) { speed_cmd(200, 100); tick(20); }
    assert(state == FAULT_OVER && starts == 1); /* stays off while commands stream */
    rx(MOTOR_STOP | 1U, 0); tick(1); assert(!drive_app.fault_latched);
    speed_cmd(200, 100); tick(3); assert(acks == 1 && starts == 2 && state == RUN);
  } else if (scenario == 6) { /* I2T derate and recovery */
    speed_cmd(300, 100); tick(5); iqd.q = 6 * 544;
    int t = 0; while (!drive_app.derated && t < 3000) { speed_cmd(300, 100); tick(10); t += 10; }
    printf("derated after %d ms\n", t);
    assert(t > 1000 && t < 1400 && PIDSpeedHandle_M1.up == 3 * 544 && PIDSpeedHandle_M1.low == -3 * 544);
    assert(PIDSpeedHandle_M1.iup == 3 * 544 * 16384);
    iqd.q = 2 * 544; t = 0;
    while (drive_app.derated && t < 10000) { speed_cmd(300, 100); tick(10); t += 10; }
    assert(!drive_app.derated && PIDSpeedHandle_M1.up == 6 * 544);
  } else if (scenario == 7) { /* CONFIG_SET only when idle */
    rx(CONFIG_SET | 1U, ((uint32_t)(uint16_t)5000 << 16) | ESC_CFG_SPEED_KP); tick(1);
    rx(CONFIG_SET | 1U, ((uint32_t)(uint16_t)-30 << 16) | ESC_CFG_HALL_SHIFT_DEG); tick(1);
    rx(CONFIG_SET | 1U, ((uint32_t)4000 << 16) | ESC_CFG_IQ_MAX_MA); tick(1);
    assert(PIDSpeedHandle_M1.kp == 5000 && fabs(shift_deg() + 30.0) < 0.01 && drive_app.aligned);
    assert(PIDSpeedHandle_M1.up == 2176);
    rx(CONFIG_SET | 1U, ((uint32_t)9000 << 16) | ESC_CFG_IQ_MAX_MA); tick(1);
    assert(PIDSpeedHandle_M1.up == 2176 && drive_app.rejected == 1);
    speed_cmd(100, 100); tick(3);
    rx(CONFIG_SET | 1U, ((uint32_t)(uint16_t)7000 << 16) | ESC_CFG_SPEED_KP); tick(1);
    assert(PIDSpeedHandle_M1.kp == 5000 && drive_app.rejected == 2);
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); tick(1); assert(drive_app.align_phase == 0);
    rx(MOTOR_STOP | 1U, 0); tick(3); assert(state == IDLE);
    rx(CONFIG_SET | 1U, ((uint32_t)1500 << 16) | ESC_CFG_CURRENT_KP); tick(1);
    rx(CONFIG_SET | 1U, ((uint32_t)900 << 16) | ESC_CFG_CURRENT_KI); tick(1);
    assert(PIDIqHandle_M1.kp == 1500 && PIDIdHandle_M1.kp == 1500);
    assert(PIDIqHandle_M1.ki == 900 && PIDIdHandle_M1.ki == 900);
    rx(CONFIG_SET | 1U, ((uint32_t)(uint16_t)-5 << 16) | ESC_CFG_CURRENT_KP); tick(1);
    assert(PIDIqHandle_M1.kp == 1500 && drive_app.rejected == 4);
  } else if (scenario == 8) { /* STOP aborts alignment; idle check for boot entry */
    assert(drive_app_idle());
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); assert(!drive_app_idle()); tick(600);
    rx(MOTOR_STOP | 1U, 0); tick(5);
    assert(drive_app.align_result == ESC_ALIGN_ABORTED && !drive_align_force && state == IDLE);
    assert(fabs(shift_deg() - 120.0) < 0.01 && drive_app_idle());
  } else if (scenario == 10) { /* IDENT_READ reports the bootloader validity record */
    rx(IDENT_READ | 1U, 0); drive_app_poll(send);
    assert(sent_id == (IDENT_REPLY | 1U));
    assert((uint32_t)(sent[0] | sent[1] << 8 | sent[2] << 16 | (uint32_t)sent[3] << 24) == 0xDEADBEEFU);
    assert((uint32_t)(sent[4] | sent[5] << 8 | sent[6] << 16 | (uint32_t)sent[7] << 24) == 45344U);
    rx(IDENT_READ | 2U, 0); frames = 0; drive_app_poll(send); assert(frames == 0); /* other wheel */
  } else if (scenario == 11) { /* capture: hold, sample-exact Id step, readout */
    const uint32_t start = (uint32_t)ESC_CAPTURE_START << 24;
    rx(CAPTURE | 1U, start | 7000U); tick_isr(2); /* above IQMAX */
    assert(drive_app.capture_status == ESC_CAPTURE_REJECTED && starts == 0);
    rx(CAPTURE | 1U, start | 3000U); tick_isr(2);
    assert(drive_align_force && drive_align_angle == 0 && starts == 1);
    assert(drive_app.capture_status == ESC_CAPTURE_RUNNING && !drive_app_idle());
    speed_cmd(100, 100); tick_isr(1); assert(starts == 1); /* speed ignored */
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); tick_isr(1); assert(drive_app.align_phase == 0);
    int t = 0; while (drive_app.capture_status == ESC_CAPTURE_RUNNING && t < 2000) { tick_isr(1); ++t; }
    printf("capture done after %d ms\n", t);
    assert(drive_app.capture_status == ESC_CAPTURE_DONE && t > 500 && t < 600);
    tick_isr(3);
    assert(state == IDLE && !drive_align_force && drive_app_idle() && drive_app.stop_reason == ESC_STOP_CAPTURE);
    const uint32_t read = (uint32_t)ESC_CAPTURE_READ << 24;
    rx(CAPTURE | 1U, read | (1U << 16) | (8U << 10) | 60U); frames = 0; drive_app_poll(send);
    assert(frames == 8 && frame_id[0] == (CAPTURE_REPLY | 1U));
    for (int k = 0; k < 8; ++k) {
      assert((frame[k][0] | frame[k][1] << 8) == (1 << 12 | (60 + k)));
      assert(f16(frame[k] + 4) == (60 + k < 64 ? 544 : 1632)); /* Id ref: hold 1 A, step 3 A */
    }
    rx(CAPTURE | 1U, read | (0U << 16) | (32U << 10) | 64U); frames = 0; drive_app_poll(send);
    assert(frames == 32 && f16(frame[0] + 2) < f16(frame[31] + 2)); /* Id rises after the step */
    assert(f16(frame[31] + 6) == 2 * 1632); /* Vd */
    rx(CAPTURE | 1U, read | (0U << 16) | (63U << 10) | 500U); frames = 0; drive_app_poll(send);
    assert(frames == 12); /* clamped to the buffer end */
    send_budget = 3; rx(CAPTURE | 1U, read | (20U << 10)); frames = 0; drive_app_poll(send);
    assert(frames == 3); send_budget = 1000; drive_app_poll(send); assert(frames == 20); /* resumes */
    rx(CAPTURE | 1U, read | (3U << 16)); frames = 0; drive_app_poll(send);
    assert(frames == 1 && (frame[0][0] | frame[0][1] << 8) == 3 << 12);
    assert(f16(frame[0] + 2) == ESC_CAPTURE_DONE && f16(frame[0] + 4) == 1236 && f16(frame[0] + 6) == 512);
  } else if (scenario == 12) { /* STOP aborts a capture; busy ESC rejects one */
    const uint32_t start = (uint32_t)ESC_CAPTURE_START << 24;
    drive_app.aligned = 1; speed_cmd(100, 100); tick_isr(3); assert(state == RUN);
    rx(CAPTURE | 1U, start | 2000U); tick_isr(1);
    assert(drive_app.capture_status == ESC_CAPTURE_REJECTED && !drive_align_force);
    rx(MOTOR_STOP | 1U, 0); tick_isr(3); assert(state == IDLE);
    rx(CAPTURE | 1U, start | 2000U); tick_isr(300);
    assert(drive_app.capture_status == ESC_CAPTURE_RUNNING && iqdref.d > 0);
    rx(MOTOR_STOP | 0xFU, 0); tick_isr(3);
    assert(drive_app.capture_status == ESC_CAPTURE_ABORTED && state == IDLE && !drive_align_force);
  } else if (scenario == 13) { /* friction feed-forward */
    const uint32_t key_s = ESC_CFG_FF_STATIC_MA, key_k = ESC_CFG_FF_KINETIC_MA;
    assert(drive_speed_iq_ref(100, 50, 0) == 100); /* idle: PI output untouched */
    rx(CONFIG_SET | 1U, (2500U << 16) | key_s); tick(1);
    rx(CONFIG_SET | 1U, (500U << 16) | key_k); tick(1);
    assert(drive_app.ff_static_counts == 1360 && drive_app.ff_kinetic_counts == 272);
    assert(drive_speed_iq_ref(100, 50, 0) == 100); /* still not driving */
    drive_app.aligned = 1; speed_cmd(50, 100); tick(3); assert(state == RUN && drive_app.active);
    assert(drive_speed_iq_ref(100, 50, 0) == 100 + 1360);     /* standstill: full static */
    assert(drive_speed_iq_ref(100, 50, -20) == 100 + 1360);   /* pushed backwards: full static */
    assert(drive_speed_iq_ref(100, 50, 15) == 100 + 816);     /* halfway through the fade */
    assert(drive_speed_iq_ref(100, 50, 5) == 100 + 816);      /* fade only goes one way */
    assert(drive_speed_iq_ref(100, 50, 30) == 100 + 272);     /* broken free: kinetic only */
    for (int k = 0; k < 99; ++k) /* Hall scatter / brief dips below zero do not re-arm */
      assert(drive_speed_iq_ref(100, 50, k % 3 ? 10 : -5) == 100 + 272);
    for (int k = 0; k < 99; ++k) assert(drive_speed_iq_ref(100, 50, 0) == 100 + 272);
    assert(drive_speed_iq_ref(100, 50, 0) == 100 + 1360);     /* stalled 100 ms: static again */
    assert(drive_speed_iq_ref(100, 50, 400) == 100 + 272);
    assert(drive_speed_iq_ref(7, 1, 400) == 7 && drive_speed_iq_ref(7, -1, 0) == 7); /* stop band, re-arms */
    assert(drive_speed_iq_ref(-100, -50, 0) == -100 - 1360);  /* reverse mirrors */
    assert(drive_speed_iq_ref(-100, -50, -15) == -100 - 816);
    assert(drive_speed_iq_ref(-100, -50, -40) == -100 - 272);
    assert(drive_speed_iq_ref(3000, 50, 0) == 6 * 544);       /* clamped to the limit */
    assert(drive_speed_iq_ref(-3000, -50, 0) == -6 * 544);
    rx(CONFIG_SET | 1U, (3000U << 16) | key_s); tick(1);
    assert(drive_app.ff_static_counts == 1360 && drive_app.rejected == 1); /* only when idle */
    rx(MOTOR_STOP | 1U, 0); tick(3); assert(state == IDLE);
    assert(drive_speed_iq_ref(100, 50, 0) == 100); /* stopped: off */
    rx(CONFIG_SET | 1U, (7000U << 16) | key_s); tick(1);
    rx(CONFIG_SET | 1U, ((uint32_t)(uint16_t)-1 << 16) | key_k); tick(1);
    assert(drive_app.ff_static_counts == 1360 && drive_app.ff_kinetic_counts == 272 && drive_app.rejected == 3);
    drive_app.iq_limit_counts = 3 * 544; /* I2T derate also caps the feed-forward */
    drive_app.active = 1; assert(drive_speed_iq_ref(1000, 50, 0) == 3 * 544);
  } else if (scenario == 14) { /* parking at zero command */
    drive_app.aligned = 1;
    speed_cmd(100, 100); tick(3); assert(state == RUN); speed = 100;
    for (int i = 0; i < 20; ++i) { speed_cmd(0, 100); tick(20); } /* decelerating */
    assert(state == RUN && drive_app.active);
    speed = 60; for (int i = 0; i < 10; ++i) { speed_cmd(0, 100); tick(20); }
    assert(state == RUN); /* still rolling */
    for (int i = 0; i < 7; ++i) { speed = i % 2 ? -27 : 29; speed_cmd(0, 100); tick(20); } /* rocking */
    assert(state == RUN); /* 140 ms at standstill */
    speed_cmd(0, 100); tick(20);
    assert(state == IDLE && !drive_app.active && drive_app.stop_reason == ESC_STOP_PARKED);
    assert(!drive_app.fault_latched && starts == 1);
    for (int i = 0; i < 20; ++i) { speed_cmd(0, 100); tick(20); } /* zeros keep it off */
    assert(state == IDLE && starts == 1);
    speed = 0; speed_cmd(80, 100); tick(3);
    assert(state == RUN && drive_app.active && starts == 2 && ramp_rpm == 80);
    speed = 3; for (int i = 0; i < 20; ++i) { speed_cmd(5, 100); tick(20); }
    assert(state == RUN); /* a small non-zero command holds speed, no parking */
    speed_cmd(0, 100); tick(100); speed_cmd(-30, 100); tick(20);
    assert(state == RUN && starts == 2); /* parking timer reset by the new command */
    rx(STATUS_READ | 1U, 0); speed_cmd(0, 100); tick(200); drive_app_poll(send);
    assert(state == IDLE && (sent[7] & 0x0F) == ESC_STOP_PARKED && !(sent[7] & ESC_FLAG_ACTIVE));
  } else if (scenario == 15) { /* startup loop: config keys, dispatch, failed-start latch */
    const uint32_t ma = 544;
    assert(drive_dsc_cfg.enable == 0 && drive_dsc_cfg.ff_run == 600 * 544 / 1000);
    assert(drive_dsc_cfg.start_slope == 9 && drive_dsc_cfg.start_cap == 1632);
    assert(drive_speed_pi(&PIDSpeedHandle_M1, 50, 20) == 40 && pi_calls == 1 && dsc_calls == 0); /* filtered */
    rx(CONFIG_SET | 1U, (0U << 16) | ESC_CFG_SPEED_REV_FILTER); tick(1);
    assert(drive_rev_enable == 0 && drive_speed_pi(&PIDSpeedHandle_M1, 50, 20) == 30 && pi_calls == 2);
    rx(CONFIG_SET | 1U, (1U << 16) | ESC_CFG_SPEED_REV_FILTER); tick(1);
    assert(drive_rev_enable == 1);
    pi_calls = 1;
    rx(CONFIG_SET | 1U, (1U << 16) | ESC_CFG_DSC_ENABLE); tick(1);
    rx(CONFIG_SET | 1U, (700U << 16) | ESC_CFG_DSC_FF_RUN_MA); tick(1);
    rx(CONFIG_SET | 1U, (2500U << 16) | ESC_CFG_DSC_START_CAP_MA); tick(1);
    rx(CONFIG_SET | 1U, (1U << 16) | ESC_CFG_DSC_START_SLOPE_MA); tick(1);
    rx(CONFIG_SET | 1U, (200U << 16) | ESC_CFG_DSC_START_HOLD_MS); tick(1);
    rx(CONFIG_SET | 1U, (3U << 16) | ESC_CFG_DSC_PCAP_UPDATES); tick(1);
    assert(drive_dsc_cfg.enable == 1 && drive_dsc_enabled() && dsc_resets == 2);
    assert(drive_dsc_cfg.ff_run == 700 * 544 / 1000 && drive_dsc_cfg.start_cap == 2500 * 544 / 1000);
    assert(drive_dsc_cfg.start_slope == 1 && drive_dsc_cfg.start_hold_ms == 200);
    assert(drive_dsc_cfg.pcap_updates == 3 && drive_app.rejected == 0);
    rx(CONFIG_SET | 1U, (2U << 16) | ESC_CFG_DSC_ENABLE); tick(1);          /* not 0/1 */
    rx(CONFIG_SET | 1U, (0U << 16) | ESC_CFG_DSC_START_SLOPE_MA); tick(1);  /* never rises */
    rx(CONFIG_SET | 1U, (7000U << 16) | ESC_CFG_DSC_PCAP_MA); tick(1);      /* > IQMAX */
    rx(CONFIG_SET | 1U, (0U << 16) | ESC_CFG_DSC_PCAP_UPDATES); tick(1);
    rx(CONFIG_SET | 1U, (3000U << 16) | ESC_CFG_DSC_PCAP_FALLBACK_MS); tick(1);
    assert(drive_app.rejected == 5 && drive_dsc_cfg.enable == 1 && drive_dsc_cfg.pcap_updates == 3);
    (void)ma;
    drive_app.aligned = 1;
    HALL_M1.EdgesPos = 7; HALL_M1.EdgesNeg = 2; HALL_M1.Captures = 5;
    speed_cmd(100, 100); tick(3); assert(state == RUN && drive_app.active);
    assert(drive_speed_pi(&PIDSpeedHandle_M1, 50, 20) == 123 && pi_calls == 1);
    assert(dsc_last_in.edges_pos == 107 && dsc_last_in.edges_neg == 102 && dsc_last_in.captures == 105); /* qualified */
    assert(dsc_last_in.meas_rpm == 10);                                     /* filtered speed */
    assert(dsc_last_in.limit == 6 * 544 && dsc_last_in.active == 1 && dsc_last_in.ref_rpm == 50);
    { const int r0 = rebases;                       /* band origin: on entering STARTING only */
      drive_dsc.state = DSC_RUNNING;
      (void)drive_speed_pi(&PIDSpeedHandle_M1, 50, 20); assert(rebases == r0); /* fake step keeps RUNNING */
      drive_dsc.state = DSC_STOPPED;
      (void)drive_speed_pi(&PIDSpeedHandle_M1, 50, 20); assert(rebases == r0);
      fake_next_state = DSC_STARTING;
      (void)drive_speed_pi(&PIDSpeedHandle_M1, 50, 20); assert(rebases == r0 + 1); /* entered */
      (void)drive_speed_pi(&PIDSpeedHandle_M1, 50, 20); assert(rebases == r0 + 1); /* stays */
      fake_next_state = -1; drive_dsc.state = DSC_STOPPED;
    }
    rx(CONFIG_SET | 1U, (0U << 16) | ESC_CFG_DSC_ENABLE); tick(1);
    assert(drive_dsc_cfg.enable == 1 && drive_app.rejected == 6); /* only when idle */
    drive_dsc.state = DSC_FAILED; tick(3);
    assert(state == IDLE && !drive_app.active && drive_app.start_failed);
    assert(drive_app.stop_reason == ESC_STOP_START_FAILED);
    for (int i = 0; i < 10; ++i) { speed_cmd(100, 100); tick(20); }
    assert(state == IDLE && starts == 1);        /* no automatic retry */
    speed_cmd(0, 100); tick(1); assert(!drive_app.start_failed);
    speed_cmd(100, 100); tick(3); assert(state == RUN && starts == 2);
    assert(drive_dsc.state == DSC_STOPPED && drive_app.active); /* reset on start, not stopped again */
    drive_dsc.state = DSC_FAILED; tick(3); assert(drive_app.start_failed);
    rx(MOTOR_STOP | 1U, 0); tick(1); assert(!drive_app.start_failed);
    speed_cmd(100, 100); tick(3); assert(state == RUN && starts == 3);
    drive_dsc.state = DSC_FAILED; tick(3); assert(drive_app.start_failed);
    tick(260); assert(!drive_app.start_failed);  /* command gap clears it */
    speed_cmd(100, 100); tick(3); assert(state == RUN && starts == 4 && drive_dsc.state == DSC_STOPPED);
  } else if (scenario == 16) { /* speed-loop recorder */
    const uint32_t rec = (uint32_t)ESC_RECORD_START << 24, read = (uint32_t)ESC_CAPTURE_READ << 24;
    rx(CAPTURE | 1U, rec | 384U); tick(1);                  /* pre out of range */
    assert(drive_app.rejected == 1);
    rx(CAPTURE | 1U, rec | 10U); tick(1);                   /* pre 10, no delay */
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(frames == 1 && (frame[0][0] | frame[0][1] << 8) == (3 << 12 | 1));
    assert(f16(frame[0] + 2) == ESC_RECORD_ARMED);
    rx(CAPTURE | 1U, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); tick(2);
    assert(starts == 0 && drive_app.capture_status == ESC_CAPTURE_REJECTED); /* shares RAM */
    PIDSpeedHandle_M1.wIntegralTerm = 100 * 16384;
    /* rearm with the speed loop feeding (as if running), samples from the 1 kHz tick */
    feed_rec = 1; speed_ref = 51;
    rx(CAPTURE | 1U, rec | 10U); tick(1);                   /* sample 0, moving: no trigger */
    tick(19);                                               /* already moving: no trigger */
    speed_ref = 0; tick(30);
    speed_ref = 51; speed = 7; iqd.q = 99;
    tick(373);
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == ESC_RECORD_TRIGGERED);
    tick(1);                                                /* 374th after the trigger */
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 4) == 384 && f16(frame[0] + 6) == 10);
    rec_pi = 1; rec_iq = 1; tick(5); rec_pi = 300; rec_iq = 500; /* frozen once done */
    rx(CAPTURE | 1U, read | (0U << 16) | (2U << 10) | 9U); frames = 0; drive_app_poll(send);
    assert(frames == 2 && f16(frame[0] + 2) == 0 && f16(frame[1] + 2) == 51); /* ref around the trigger */
    assert(f16(frame[1] + 4) == 7 && f16(frame[1] + 6) == 200);              /* meas, P = 300 - I */
    rx(CAPTURE | 1U, read | (1U << 16) | (1U << 10) | 10U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == 100 && f16(frame[0] + 4) == 200 && f16(frame[0] + 6) == 500); /* I, ff, Iq ref */
    rx(CAPTURE | 1U, read | ESC_RECORD_READ_CH67 | (1U << 10) | 10U); frames = 0; drive_app_poll(send);
    assert(frames == 1 && f16(frame[0] + 2) == 99 && f16(frame[0] + 6) == 0); /* measured Iq; pad */
    assert((f16(frame[0] + 4) & 0x80) == 0);                                 /* legacy: not enabled */
    rx(CAPTURE | 1U, read | (2U << 16) | (1U << 10) | 10U); frames = 0; drive_app_poll(send);
    assert(frames == 0);                                                     /* no bank 2 */
    rx(CAPTURE | 1U, read | (0U << 16) | (32U << 10) | 380U); frames = 0; drive_app_poll(send);
    assert(frames == 4);                                       /* clamped to 384 samples */
    speed_ref = 0;
    rx(CAPTURE | 1U, rec | (100U << 9)); tick(1);              /* delay 100 ms, pre 0 */
    speed_ref = 51;
    for (int k = 0; k < 100; ++k) { speed = (int16_t)k; tick(1); }
    speed = 1000;
    tick(384);
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 6) == 0);
    rx(CAPTURE | 1U, read | (0U << 16) | (1U << 10) | 0U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 4) == 1000);                         /* window starts 100 ms after the start */
    rx(CAPTURE | 1U, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); tick(2);
    assert(starts == 1 && drive_app.capture_status == ESC_CAPTURE_RUNNING); /* recorder done: allowed */
    rx(CAPTURE | 1U, rec | 10U); tick(1);
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == ESC_RECORD_REJECTED);          /* capture running */
  } else if (scenario == 17) { /* recorder armed while IDLE, real start, early stop */
    const uint32_t rec = (uint32_t)ESC_RECORD_START << 24, read = (uint32_t)ESC_CAPTURE_READ << 24;
    drive_app.aligned = 1; speed_ref = 51;   /* the SDK would still hold a stale ref in IDLE */
    rx(CAPTURE | 1U, rec | 20U); tick(5);    /* IDLE: loop not running, samples read ref 0 */
    speed_cmd(51, 20); tick(3); assert(state == RUN);
    tick(40);
    rx(MOTOR_STOP | 1U, 0); tick(3); assert(state == IDLE);   /* stopped before the window filled */
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == ESC_RECORD_TRIGGERED);
    tick(400);                               /* keeps sampling while stopped, then done */
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    /* only 8 samples before the trigger (5 IDLE + 3 motor start-up) < pre 20:
     * the window starts at sample 0 and holds 8 + (384 - 20) samples */
    assert(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 6) == 8 && f16(frame[0] + 4) == 372);
    rx(CAPTURE | 1U, read | (0U << 16) | (2U << 10) | 7U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == 0 && f16(frame[1] + 2) == 51); /* trigger at the first RUN sample */
    rx(CAPTURE | 1U, ((uint32_t)ESC_CAPTURE_START << 24) | 1000U); tick(2);
    assert(drive_app.capture_status == ESC_CAPTURE_RUNNING); /* buffer released */
  } else if (scenario == 18) { /* recorder, startup loop enabled: stopped samples drop loop terms */
    const uint32_t rec = (uint32_t)ESC_RECORD_START << 24, read = (uint32_t)ESC_CAPTURE_READ << 24;
    drive_app.aligned = 1; speed_ref = 51;
    rx(CONFIG_SET | 1U, (1U << 16) | ESC_CFG_DSC_ENABLE); tick(1);
    rx(CAPTURE | 1U, rec | 0U); tick(2);
    speed_cmd(51, 20); tick(3); assert(state == RUN);
    drive_dsc = (dsc_state_t){ .state = DSC_RUNNING, .reason = DSC_R_EDGE, .capped = 1,
                               .p = 200, .i = 100, .ff = 326 };
    tick(10);
    drive_dsc.state = DSC_FAILED; drive_dsc.reason = DSC_R_NO_EDGE;
    tick(3); assert(state == IDLE && drive_app.start_failed);
    tick(400);
    rx(CAPTURE | 1U, read | (3U << 16) | 1U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == ESC_RECORD_DONE && f16(frame[0] + 6) == 0);
    rx(CAPTURE | 1U, read | (1U << 16) | (1U << 10) | 5U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == 100 && f16(frame[0] + 4) == 326);           /* running: I, ff */
    rx(CAPTURE | 1U, read | ESC_RECORD_READ_CH67 | (1U << 10) | 5U); frames = 0; drive_app_poll(send);
    uint16_t packed = (uint16_t)f16(frame[0] + 4);
    assert((packed & 3U) == DSC_RUNNING && (packed & 0x40U) && (packed & 0x80U) && (packed & 0x8000U));
    rx(CAPTURE | 1U, read | (0U << 16) | (1U << 10) | 300U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == 0 && f16(frame[0] + 6) == 0);               /* stopped: ref, P */
    rx(CAPTURE | 1U, read | (1U << 16) | (1U << 10) | 300U); frames = 0; drive_app_poll(send);
    assert(f16(frame[0] + 2) == 0 && f16(frame[0] + 4) == 0 && f16(frame[0] + 6) == 0); /* I, ff, Iq ref */
    rx(CAPTURE | 1U, read | ESC_RECORD_READ_CH67 | (1U << 10) | 300U); frames = 0; drive_app_poll(send);
    packed = (uint16_t)f16(frame[0] + 4);
    assert((packed & 3U) == DSC_FAILED && ((packed >> 2) & 15U) == DSC_R_NO_EDGE); /* failure kept */
    assert(!(packed & 0x40U) && !(packed & 0x8000U));                     /* not capped, loop off */
  } else if (scenario == 9) { /* undervolt latched on USB power: STOP clears it once gone */
    state = FAULT_OVER; cur_faults = 4; occ_faults = 4; tick(1);
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); tick(2);
    assert(drive_app.align_phase == 0 && drive_app.rejected == 1); /* align needs IDLE */
    rx(MOTOR_STOP | 0xFU, 0); tick(1); assert(acks == 0 && state == FAULT_OVER); /* still present */
    cur_faults = 0;
    rx(MOTOR_STOP | 0xFU, 0); tick(1); assert(acks == 1 && state == IDLE && starts == 0);
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); tick(2); assert(drive_align_force && starts == 1);
  }
  return 0;
}
'''


class EscDriveAppTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = pathlib.Path(cls.tmp.name)
        for name, text in STUBS.items():
            (tmp / name).write_text(text)
        (tmp / "harness.c").write_text(HARNESS)
        cls.exe = tmp / "harness"
        subprocess.run(["gcc", "-std=gnu11", "-Wall", "-Werror", "-O1", "-DMOTOR_NE",
                        "-include", str(tmp / "test_hooks.h"), "-DDRIVE_APP_VALIDITY=fake_validity",
                        "-I", str(tmp), "-I", str(ESC / "motor_config/Inc"),
                        "-I", str(ESC / "Inc"), "-I", str(ROOT / "HW/common/CAN"),
                        str(tmp / "harness.c"), str(ESC / "motor_config/Src/drive_app.c"),
                        "-lm", "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_scenario(self, n):
        result = subprocess.run([str(self.exe), str(n)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_alignment_measures_shift(self):
        print(self.run_scenario(0), end="")

    def test_alignment_detects_reversed_wiring(self):
        self.run_scenario(1)

    def test_alignment_detects_stuck_rotor(self):
        self.run_scenario(2)

    def test_streaming_and_watchdog(self):
        self.run_scenario(3)

    def test_stop_and_status_reply(self):
        self.run_scenario(4)

    def test_fault_latches_until_stop(self):
        self.run_scenario(5)

    def test_i2t_derates_and_recovers(self):
        print(self.run_scenario(6), end="")

    def test_config_only_when_idle(self):
        self.run_scenario(7)

    def test_stop_aborts_alignment(self):
        self.run_scenario(8)

    def test_stop_clears_finished_fault(self):
        self.run_scenario(9)

    def test_ident_reports_validity_record(self):
        self.run_scenario(10)

    def test_capture_steps_id_and_reads_out(self):
        print(self.run_scenario(11), end="")

    def test_capture_stop_and_busy(self):
        self.run_scenario(12)

    def test_friction_feedforward(self):
        self.run_scenario(13)

    def test_parks_at_zero_command(self):
        self.run_scenario(14)

    def test_startup_loop_config_and_failed_latch(self):
        self.run_scenario(15)

    def test_speed_loop_recorder(self):
        self.run_scenario(16)

    def test_recorder_idle_arm_and_early_stop(self):
        self.run_scenario(17)

    def test_recorder_dsc_stop_drops_loop_terms(self):
        self.run_scenario(18)


if __name__ == "__main__":
    unittest.main()
