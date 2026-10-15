#!/usr/bin/env python3
"""Host checks of the actual bench-profile guard snippets; no hardware access."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent
source = (root / 'Src/main.c').read_text()
helpers = source.split('/* Explicit diagnostic request words', 1)[1].split('#endif', 1)[0]
helpers = '/* Explicit diagnostic request words' + helpers
commands = source.split('  if (can_rx_header.Identifier == (TEST_CURRENT_READ | MOTOR_CAN_ID))', 1)[1]
commands = '  if (can_rx_header.Identifier == (TEST_CURRENT_READ | MOTOR_CAN_ID))' + commands.split('#endif', 1)[0]

harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int HAL_StatusTypeDef;
typedef int FDCAN_HandleTypeDef;
typedef int FDCAN_TxHeaderTypeDef;
#include "can_datatypes.h"
#define MOTOR_CAN_ID MOTOR_NE_CAN_ID
#define IDLE 0
#define FAULT_OVER 11
#define RUN 6
#define OFFSET_CALIB 2
#define MC_NO_FAULTS 0
#define RPM_2_SPEED_UNIT(rpm) (rpm)
volatile uint32_t test_drive_armed, test_drive_run_count, test_drive_active;
volatile uint32_t test_drive_start_tick, test_drive_stop_reason;
static volatile uint32_t test_drive_start_permit;
static uint32_t test_drive_arm_tick, test_drive_peak_l1;
static int32_t test_drive_iq, test_drive_id;
static int32_t test_drive_iq_reference, test_drive_id_reference, test_drive_measured_rpm;
static int32_t test_drive_hall_net_edges;
static uint32_t test_drive_hall_edges, test_drive_hall_invalid;
static uint32_t test_drive_hall_last_tick;
static uint8_t test_monitor_ready = 1;
static uint8_t test_current_pending;
static uint16_t current_faults;
static uint32_t test_calibration_request, test_calibration_result, test_calibration_valid;
static uint32_t test_drive_current_valid, test_calibration_tick, calibrations;
static uint32_t test_direct_request, test_direct_result, test_direct_last_request;
static bool offsets_ready = true;
typedef int PolarizationOffsets_t;
bool MC_GetPolarizationOffsetsMotor1(PolarizationOffsets_t *p) {
  (void)p; return offsets_ready;
}
typedef struct { int16_t q, d; } qd_t;
static qd_t measured;
#define CURRENT_CONV_FACTOR 544
#include "test_profile.h"
#define SPEED_UNIT_2_RPM(speed) (speed)
static int16_t measured_speed;
static qd_t reference;
static int torque_planned;
static int speed_planned, speed_ramp_ms;
qd_t MC_GetIqdrefMotor1(void) { return reference; }
int16_t MC_GetMecSpeedAverageMotor1(void) { return measured_speed; }
qd_t MC_GetIqdMotor1(void) { return measured; }
static uint32_t tick, state, stops, starts;
static bool allow_start = true;
static struct { uint32_t Identifier; } can_rx_header;
static uint8_t can_rx_data[4];
uint32_t HAL_GetTick(void) { return tick; }
int MC_GetSTMStateMotor1(void) { return state; }
uint16_t MC_GetCurrentFaultsMotor1(void) { return current_faults; }
bool MC_AcknowledgeFaultMotor1(void) { state = IDLE; return true; }
bool MC_StopMotor1(void) { ++stops; return true; }
bool MC_StartPolarizationOffsetsMeasurementMotor1(void) {
  ++calibrations; offsets_ready = false; state = OFFSET_CALIB; return true;
}
void MC_ProgramSpeedRampMotor1(int rpm, int ms) {
  assert(ESC_TEST_RPM_PROFILE ? (rpm >= ESC_TEST_RPM_MIN && rpm <= ESC_TEST_RPM_MAX && ms == 500) : (rpm >= 12 && rpm <= 60 && ms >= 500 && ms <= 1000));
  speed_planned = rpm; speed_ramp_ms = ms;
}
void MC_ProgramTorqueRampMotor1(int counts, int ms) {
  assert(counts > 0 && counts <= (int)(ESC_TEST_REFERENCE_A * CURRENT_CONV_FACTOR) && ms >= 200 && ms <= 500);
  torque_planned = counts;
}
bool TestDriveConsumeStartPermit(void);
bool MC_StartMotor1(void) {
  assert(TestDriveConsumeStartPermit()); ++starts; return allow_start;
}
'''
harness += helpers + '\nvoid command(void) {\n' + commands + '\n}\n'
harness += r'''
static void reset(void) {
  test_drive_armed = test_drive_run_count = test_drive_active = 0;
  test_drive_start_tick = test_drive_stop_reason = test_drive_start_permit = 0;
  tick = state = stops = starts = 0; allow_start = true;
  test_drive_arm_tick = test_drive_peak_l1 = 0;
  measured.q = measured.d = 0; test_monitor_ready = 1;
  reference.q = reference.d = 0; measured_speed = 0; torque_planned = 0;
  current_faults = test_current_pending = 0;
  test_calibration_request = test_calibration_tick = calibrations = 0;
  test_calibration_result = 2; test_calibration_valid = 1;
  test_drive_current_valid = 0; offsets_ready = true;
  test_direct_request = test_direct_result = test_direct_last_request = 0;
  test_drive_hall_net_edges = 0;
  test_drive_hall_edges = test_drive_hall_invalid = 0;
  test_drive_hall_last_tick = 0;
  speed_planned = speed_ramp_ms = 0;
}
static void send(uint32_t id, int16_t rpm, uint16_t ms) {
  can_rx_header.Identifier = id;
  CAN_PAYLOAD_FROM_U32(can_rx_data, (uint32_t)(uint16_t)rpm | ((uint32_t)ms << 16));
  command();
}
int main(void) {
#if ESC_TEST_AUTO_CAL_ON_ARM
  reset(); test_calibration_valid = 0; test_calibration_result = 0; offsets_ready = false;
  tick = 23; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && test_calibration_result == 1 && calibrations == 1 && !starts);
  assert(state == OFFSET_CALIB);
  send(SPEED_WRITE | 1, 60, 1000); assert(!starts && !test_drive_active);
  test_direct_request = 0x4452554E; TestDirectService(); assert(!starts && test_direct_result == 2);
  send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 1 && test_calibration_result == 1);
  current_faults = 4; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 1 && test_calibration_result == 1);
  current_faults = 0;
  test_drive_active = 1; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 1 && test_calibration_result == 1);
  test_drive_active = 0;
  state = 1; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 1 && test_calibration_result == 1);
  reset(); test_calibration_valid = 0; test_calibration_result = 0; offsets_ready = false;
  current_faults = 4; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 0 && test_calibration_result == 0);
  current_faults = 0;
  test_drive_active = 1; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 0 && test_calibration_result == 0);
  test_drive_active = 0; state = 1; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && calibrations == 0 && test_calibration_result == 0);
  state = IDLE;
  reset(); test_calibration_valid = 0; test_calibration_result = 0; offsets_ready = false;
  tick = 40; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(test_calibration_result == 1 && !test_drive_armed);
  state = IDLE; offsets_ready = true;
  TestDriveSafetyTick(); TestCalibrationService();
  assert(test_calibration_valid && test_calibration_result == 2 && !starts && !test_drive_active);
  tick = 41; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(test_drive_armed == 0x54455354U && test_drive_arm_tick == 41);
#else
  reset(); test_calibration_valid = 0; test_calibration_result = 0; offsets_ready = false;
  send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(!test_drive_armed && test_calibration_result == 0 && calibrations == 0 && !starts);
#endif
  reset(); test_calibration_valid = 0; test_calibration_result = 0; offsets_ready = false;
  test_drive_armed = 0x54455354U;
  send(SPEED_WRITE | 1, 60, 1000); assert(!starts);
  test_drive_armed = 0;
  current_faults = 4; test_calibration_request = 0x43414C31;
  TestCalibrationService(); assert(calibrations == 0 && test_calibration_result == 3);
  current_faults = 0; state = FAULT_OVER;
  test_calibration_request = 0x43414C31; TestCalibrationService();
  assert(calibrations == 1 && test_calibration_result == 1 && !starts);
  TestDriveSafetyTick(); assert(!test_calibration_valid && !test_drive_current_valid);
  state = IDLE; offsets_ready = true;
  TestDriveSafetyTick(); TestCalibrationService();
  assert(test_calibration_valid && test_calibration_result == 2 && !starts);
  reset(); test_calibration_result = 1; offsets_ready = false; state = OFFSET_CALIB;
  tick = 1000; TestDriveSafetyTick(); assert(test_calibration_result == 4 && stops == 1);
  reset(); offsets_ready = false; measured.q = 30000; measured.d = -30000;
  test_drive_active = 1; TestDriveSafetyTick();
  assert(!test_drive_current_valid && test_drive_iq == 0 && test_drive_id == 0 && !stops);
  reset(); current_faults = 4;
  send(TEST_ARM | 1, 0x5354, 0x5445); assert(!test_drive_armed);
  current_faults = 0; state = FAULT_OVER;
  send(TEST_ARM | 3, 0x5354, 0x5445); assert(!test_drive_armed);
  send(TEST_ARM | 1, 0, 0); assert(!test_drive_armed);
  tick = 23; send(TEST_ARM | 1, 0x5354, 0x5445);
  assert(test_drive_armed == 0x54455354U && test_drive_arm_tick == 23 && state == IDLE);
  test_drive_armed = 0; test_drive_run_count = 5;
  send(TEST_ARM | 1, 0x5354, 0x5445); assert(test_drive_armed == 0x54455354U);
  send(TEST_CURRENT_READ | 1, 0, 0); assert(test_current_pending && starts == 0);
  reset(); assert(!TestDriveConsumeStartPermit());
  test_drive_start_permit = 1;
  assert(TestDriveConsumeStartPermit()); assert(!TestDriveConsumeStartPermit());
  reset(); send(SPEED_WRITE | 1, 60, 1000); assert(starts == 0);
  test_drive_armed = 0x54455354U;
  send(TORQUE_WRITE | 1, 60, 1000);
  send(SPEED_WRITE | 3, 60, 1000);
  send(SPEED_WRITE | 1, -60, 1000);
  send(SPEED_WRITE | 1, 0, 1000);
  send(SPEED_WRITE | 1, 11, 1000);
  send(SPEED_WRITE | 1, 61, 1000);
  send(SPEED_WRITE | 1, 60, 499);
  send(SPEED_WRITE | 1, 60, 1001);
  state = 1; send(SPEED_WRITE | 1, 60, 1000); state = 0;
  assert(starts == 0 && test_drive_run_count == 0);
  tick = 100; send(SPEED_WRITE | 1, 60, 1000);
  assert(starts == 1 && test_drive_active && test_drive_run_count == 1 && !test_drive_armed);
  test_drive_active = 0U; state = IDLE; test_drive_armed = 0x54455354U; tick = 200;
  send(SPEED_WRITE | 1, 60, 1000);
  assert(starts == 2 && test_drive_run_count == 2 && test_drive_start_tick == 200);
  tick = 1699; TestDriveSafetyTick(); assert(stops == 0);
  tick = 1700; TestDriveSafetyTick(); assert(stops == 1 && !test_drive_active);
  assert(test_drive_stop_reason == 1);
  TestDriveSafetyTick(); assert(stops == 1);
  reset(); test_drive_armed = 0x54455354U;
  send(SPEED_WRITE | 1, 12, 500); assert(starts == 1);
  send(MOTOR_STOP | 1, 0, 0);
  assert(stops == 1 && !test_drive_active && !test_drive_armed);
  reset(); test_drive_armed = 0x54455354U; allow_start = false;
  send(SPEED_WRITE | 1, 60, 1000);
  assert(test_drive_run_count && !test_drive_active && test_drive_stop_reason == 3);
  reset(); test_drive_active = 1; test_drive_start_tick = UINT32_MAX - 999U;
  tick = 499; TestDriveSafetyTick(); assert(stops == 0);
  tick = 500; TestDriveSafetyTick(); assert(stops == 1);
  reset(); test_drive_armed = 0x54455354U;
  tick = 4999; TestDriveSafetyTick(); assert(test_drive_armed);
  tick = 5000; TestDriveSafetyTick(); assert(!test_drive_armed);
  reset(); test_drive_active = 1; state = RUN; measured.q = 136;
  TestDriveSafetyTick(); assert(stops == 0 && test_drive_peak_l1 == 136);
  measured.d = -81; TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 4 && test_drive_peak_l1 == 217);
  reset(); test_monitor_ready = 0; test_drive_active = 1; tick = 2000;
  TestDriveSafetyTick(); assert(stops == 0);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='esc-guard-tests-', dir='/tmp/opencode') as folder:
    folder = pathlib.Path(folder)
    c_file = folder / 'test.c'
    c_file.write_text(harness)
    for extra in ([], ['-DESC_TEST_AUTO_CAL_ON_ARM=0']):
        binary = folder / ('test-optout' if extra else 'test')
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        *extra,
                        '-I' + str(root.parent.parent / 'common/CAN'),
                        '-I' + str(root / 'Inc'),
                        str(c_file), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS: explicit arming, auto-CAL pending/done + opt-out hard-reject, fault gates, command bounds, run counter, stop, current guard and tick wrap')
