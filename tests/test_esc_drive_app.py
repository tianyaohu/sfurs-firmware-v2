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
qd_t MC_GetIqdMotor1(void); qd_t MC_GetIqdrefMotor1(void);
''',
"mc_config.h": r'''
#pragma once
#include <stdint.h>
typedef struct { int16_t kp, ki; int16_t up, low; int32_t iup, ilow; uint16_t kidiv; } PID_Handle_t;
typedef struct { int16_t PhaseShift; int8_t Direction; uint8_t HallState; int16_t MeasuredElAngle; } HALL_Handle_t;
extern PID_Handle_t PIDSpeedHandle_M1; extern HALL_Handle_t HALL_M1;
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
"parameters_conversion.h": r'''
#pragma once
#define CURRENT_CONV_FACTOR 544
#define IQMAX_A 6
#define IQMAX (IQMAX_A * CURRENT_CONV_FACTOR)
#define MAX_APPLICATION_SPEED_RPM 2976
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

PID_Handle_t PIDSpeedHandle_M1 = { .kidiv = 16384 };
HALL_Handle_t HALL_M1;
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
static void mf_step(void) {
  if (stop_pending) { stop_pending = 0; start_pending = 0; if (state == RUN || state == CHARGE_BOOT_CAP) state = IDLE; }
  if (start_pending && state == IDLE) { start_pending = 0; state = CHARGE_BOOT_CAP; }
  else if (state == CHARGE_BOOT_CAP) state = RUN;
}
static void tick(int n) { while (n--) { mf_step(); drive_app_tick(); motor_step(); } }
static void rx(uint32_t id, uint32_t word) { uint8_t d[4]; CAN_PAYLOAD_FROM_U32(d, word); drive_app_can_rx(id, d, 4); }
static uint32_t sent_id; static uint8_t sent[8];
static int send(uint32_t id, const uint8_t d[8]) { sent_id = id; memcpy(sent, d, 8); return 1; }
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
  } else if (scenario == 8) { /* STOP aborts alignment; idle check for boot entry */
    assert(drive_app_idle());
    rx(HALL_ALIGN | 1U, HALL_ALIGN_MAGIC); assert(!drive_app_idle()); tick(600);
    rx(MOTOR_STOP | 1U, 0); tick(5);
    assert(drive_app.align_result == ESC_ALIGN_ABORTED && !drive_align_force && state == IDLE);
    assert(fabs(shift_deg() - 120.0) < 0.01 && drive_app_idle());
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


if __name__ == "__main__":
    unittest.main()
