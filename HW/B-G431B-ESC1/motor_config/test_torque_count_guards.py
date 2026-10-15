#!/usr/bin/env python3
"""Offline tests of torque turn-count profile; no hardware access."""
import pathlib
import runpy
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent
original = runpy.run_path(str(root / 'test_low_current_guards.py'))
harness = original['harness'].split('int main(void) {', 1)[0]
harness += r'''
static void direct(uint32_t request) {
  test_direct_request = request; TestDirectService();
}
static void start(void) {
  direct(0x4441524D); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  direct(0x4452554E);
  assert(starts == 1 && test_drive_run_count && test_drive_active);
  assert(torque_planned == 435 && !speed_planned);
}
int main(void) {
  assert(ESC_TEST_TORQUE_PROFILE && ESC_TEST_COUNT_PROFILE && !ESC_TEST_RPM_PROFILE);
  assert(ESC_TEST_REFERENCE_A == 0.8f && ESC_TEST_IQMAX_A == 3.0f);
  assert(ESC_TEST_SAMPLED_STOP_A == 3.0f && ESC_TEST_TIMEOUT_MS == 180000U);
  assert(ESC_TEST_TARGET_NET_EDGES == 14400 && ESC_TEST_MAX_MOTOR_RPM == 1000);
  reset(); send(TEST_ARM | 1, 0x5354, 0x5445); assert(!test_drive_armed);
  reset(); direct(0x4452554E); assert(!starts && test_direct_result == 2);
  direct(0x12345678); assert(!starts);
#if ESC_TEST_AUTO_CAL_ON_ARM
  reset(); test_calibration_valid=0; test_calibration_result=0; offsets_ready=false;
  direct(0x4441524D);
  assert(!test_drive_armed && test_direct_result==2 && test_calibration_result==1 && calibrations==1 && !starts);
  direct(0x4452554E); assert(!starts && !test_drive_active);
  current_faults=4; direct(0x4441524D); assert(!test_drive_armed && calibrations==1); current_faults=0;
  test_drive_active=1; direct(0x4441524D); assert(!test_drive_armed && calibrations==1); test_drive_active=0;
  state=1; direct(0x4441524D); assert(!test_drive_armed && calibrations==1 && test_calibration_result==1);
  state=IDLE; offsets_ready=true; TestDriveSafetyTick(); TestCalibrationService();
  assert(test_calibration_valid && test_calibration_result==2 && !starts);
  direct(0x4441524D); assert(test_drive_armed==ESC_TEST_ARM_MAGIC && test_direct_result==1);
  reset();
#else
  reset(); test_calibration_valid=0; test_calibration_result=0; offsets_ready=false;
  direct(0x4441524D);
  assert(!test_drive_armed && test_direct_result==2 && test_calibration_result==0 && calibrations==0);
  reset();
#endif
  test_calibration_valid = 1; current_faults = 4;
  direct(0x4441524D); assert(!test_drive_armed);
  current_faults = 0; start(); direct(0x4452554E); assert(starts == 1);
  tick = 179999; test_drive_hall_last_tick = tick; TestDriveSafetyTick(); assert(!stops);
  tick = 180000; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 1);
  reset(); start();
  for (int i = 0; i < 14399; ++i) TestDriveHallTransition(5, 4);
  TestDriveSafetyTick(); assert(!stops && test_drive_hall_net_edges == -14399);
  TestDriveHallTransition(5, 4); TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 8);
  reset(); start();
  for (int i = 0; i < 14400; ++i) TestDriveHallTransition(5, 1);
  TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 8);
  reset(); start(); TestDriveHallTransition(5, 4); TestDriveHallTransition(4, 5);
  assert(test_drive_hall_net_edges == 0 && test_drive_hall_edges == 2);
  TestDriveSafetyTick(); assert(!stops);
  reset(); start(); TestDriveHallTransition(4, 1); assert(test_drive_hall_invalid == 1);
  TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 7);
  reset(); start(); tick = 1999; TestDriveSafetyTick(); assert(!stops);
  tick = 2000; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 9);
  reset(); start(); state = RUN; measured_speed = 1000; TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 5);
  reset(); start(); state = RUN; measured_speed = -1000; TestDriveSafetyTick(); assert(stops == 1);
  reset(); start(); state = RUN; measured.q = 1600; measured.d = 32;
  TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 4);
  reset(); start(); direct(0x44535450); assert(stops == 1 && !test_drive_active);
  direct(0x4441524D); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  reset(); direct(0x4441524D); tick = 5000; TestDriveSafetyTick();
  direct(0x4452554E); assert(!starts);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='torque-count-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    for extra in ([], ['-DESC_TEST_AUTO_CAL_ON_ARM=0']):
        binary = tmp / ('test-optout' if extra else 'test')
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        '-DESC_TORQUE_COUNT_DIAGNOSTIC', *extra, '-I' + str(root / 'Inc'),
                        '-I' + str(root.parent.parent / 'common/CAN'),
                        str(tmp / 'test.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS: 0.8 A torque, auto-CAL pending/done + opt-out hard-reject, 14400-edge count stop, 3 A/1000 RPM/180 s/stall/invalid guards, run counter')
