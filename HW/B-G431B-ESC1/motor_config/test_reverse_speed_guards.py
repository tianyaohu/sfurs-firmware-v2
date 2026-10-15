#!/usr/bin/env python3
"""Offline tests of negated-gain speed profile (reverse-validated,
forward-proven-on-hardware-pending); no hardware access."""
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
  assert(speed_planned == -100 && speed_ramp_ms == 500 && !torque_planned);
}
int main(void) {
  assert(ESC_TEST_RPM_PROFILE && !ESC_TEST_TORQUE_PROFILE && ESC_TEST_COUNT_PROFILE);
  assert(ESC_TEST_TARGET_RPM == -100 && ESC_TEST_ARM_MAGIC == 0x52504D32U);
  assert(ESC_TEST_TIMEOUT_MS == 30000 && ESC_TEST_MAX_MOTOR_RPM == 300);
  assert(ESC_TEST_COUNT_PROFILE && ESC_TEST_TARGET_NET_EDGES == 480);
  reset(); send(TEST_ARM | 1, 0x4c46, 0x4841); assert(!test_drive_armed);
  send(SPEED_WRITE | 1, 60, 500); assert(!starts);
  reset(); direct(0x4452554E); assert(!starts && test_direct_result == 2);
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
  tick = 29999; test_drive_hall_last_tick = tick; TestDriveSafetyTick(); assert(!stops);
  tick = 30000; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 1);
  reset(); start();
  assert(!TestDriveStart(0, 500) && !TestDriveStart(-100, 499));
  assert(!TestDriveStart(301, 500) && !TestDriveStart(-301, 500));
  assert(!TestDriveStart(60, 501));
  assert(starts == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(-120, 500) == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(-30, 500) == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(60, 500) == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(300, 500) == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(-300, 500) == 1);
  reset(); start(); state = RUN; measured_speed = 300; TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 5);
  reset(); start(); state = RUN; measured_speed = -300; TestDriveSafetyTick(); assert(stops == 1);
  reset(); start();
  const uint8_t cycle[] = {5, 4, 6, 2, 3, 1, 5};
  for (int i = 0; i < 6; ++i) TestDriveHallTransition(cycle[i], cycle[i+1]);
  assert(test_drive_hall_edges == 6 && test_drive_hall_net_edges == -6 && !test_drive_hall_invalid);
  TestDriveHallTransition(4, 1); assert(test_drive_hall_invalid == 1);
  TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 7);
  reset(); start(); tick = 1999; TestDriveSafetyTick(); assert(!stops);
  tick = 2000; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 9);
  reset(); start();
  for (int i = 0; i < 479; ++i) TestDriveHallTransition(5, 4);
  TestDriveSafetyTick(); assert(!stops);
  TestDriveHallTransition(5, 4); TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 8);
  reset(); start(); direct(0x44535450); assert(stops == 1 && !test_drive_active);
  direct(0x4441524D); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='reverse-speed-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    for extra in ([], ['-DESC_TEST_AUTO_CAL_ON_ARM=0']):
        binary = tmp / ('test-optout' if extra else 'test')
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        '-DESC_REVERSE_SPEED_DIAGNOSTIC', *extra, '-I' + str(root / 'Inc'),
                        '-I' + str(root.parent.parent / 'common/CAN'),
                        str(tmp / 'test.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS: negated-gain +-300 RPM range, auto-CAL pending/done + opt-out hard-reject, 0/ramp/edge rejects, 10-turn count, run counter and safety stops')
