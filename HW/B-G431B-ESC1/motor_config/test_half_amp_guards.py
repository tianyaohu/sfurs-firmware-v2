#!/usr/bin/env python3
"""Host test actual half-amp diagnostic guards and retain old-profile regression."""
import pathlib
import runpy
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent
original = runpy.run_path(str(root / 'test_low_current_guards.py'))
harness = original['harness'].split('int main(void) {', 1)[0]
harness += r'''
static void arm(void) {
  send(TEST_ARM | 1, (int16_t)(ESC_TEST_ARM_MAGIC & 0xffff),
       (uint16_t)(ESC_TEST_ARM_MAGIC >> 16));
}
int main(void) {
  assert(ESC_TEST_REFERENCE_A == 1.5f && ESC_TEST_SAMPLED_STOP_A == 1.5f && ESC_TEST_TIMEOUT_MS == 2000);
  reset(); send(TEST_ARM | 1, 0x5354, 0x5445); assert(!test_drive_armed);
  arm(); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  send(SPEED_WRITE | 1, 30, 500); assert(!starts);
  send(TORQUE_WRITE | 1, 272, 200); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 3, 272, 200); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 1, -272, 200); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 1, 0, 200); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 1, 817, 200); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 1, 272, 199); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 1, 272, 501); assert(!starts);
  current_faults=4; send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200); assert(!starts);
  current_faults=0; state=1; send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200); assert(!starts);
  state=0; test_calibration_valid=0;
  send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200); assert(!starts);
  test_calibration_valid=1; test_calibration_result=0;
  send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200); assert(!starts);
  test_calibration_result=2; tick=100;
  send(ESC_TEST_TORQUE_COMMAND | 1, 816, 200);
  assert(starts==1 && torque_planned==816 && test_drive_active && test_drive_run_count == 1);
  send(MOTOR_STOP | 15, 0, 0); assert(!test_drive_active);
  arm(); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  tick=100; send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200);
  assert(starts==2 && test_drive_run_count == 2);
  stops = 0;
  tick=100 + ESC_TEST_TIMEOUT_MS - 1; TestDriveSafetyTick(); assert(!stops);
  tick=100 + ESC_TEST_TIMEOUT_MS; TestDriveSafetyTick(); assert(stops==1 && test_drive_stop_reason==1);
  reset(); arm(); send(ESC_TEST_TORQUE_COMMAND | 1, 1, 500); assert(starts==1);
  send(MOTOR_STOP | 15, 0, 0); assert(stops==1 && !test_drive_active);
  reset(); arm(); allow_start=false;
  send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200);
  assert(test_drive_run_count && !test_drive_active && test_drive_stop_reason==3);
  reset(); arm(); test_drive_active=1; state=RUN;
  measured.q=800; measured.d=-15; TestDriveSafetyTick(); assert(!stops);
  measured.d=-16; TestDriveSafetyTick();
  assert(stops==1 && test_drive_stop_reason==4 && test_drive_peak_l1==816);
  reset(); test_drive_active=1; measured_speed=120;
  TestDriveSafetyTick(); assert(stops==1 && test_drive_stop_reason==5);
  reset(); test_drive_active=1; measured_speed=-120;
  TestDriveSafetyTick(); assert(stops==1 && test_drive_stop_reason==5);
  reset(); test_drive_active=1; current_faults=4;
  TestDriveSafetyTick(); assert(stops==1 && test_drive_stop_reason==6);
  reset(); test_drive_active=1; offsets_ready=false;
  TestDriveSafetyTick(); assert(stops==1 && test_drive_stop_reason==6);
  reset(); test_drive_active=1; test_drive_start_tick=UINT32_MAX-499;
  tick=ESC_TEST_TIMEOUT_MS-501; TestDriveSafetyTick(); assert(!stops);
  tick=ESC_TEST_TIMEOUT_MS-500; TestDriveSafetyTick(); assert(stops==1);
  reset(); arm(); tick=5000; TestDriveSafetyTick(); assert(!test_drive_armed);
#if ESC_TEST_AUTO_CAL_ON_ARM
  reset(); test_calibration_valid=0; test_calibration_result=0; offsets_ready=false;
  arm(); assert(!test_drive_armed && !starts && test_calibration_result==1 && calibrations==1);
  send(ESC_TEST_TORQUE_COMMAND | 1, 272, 200); assert(!starts && !test_drive_active);
  test_direct_request=0x4452554E; TestDirectService(); assert(!starts && test_direct_result==2);
  current_faults=4; arm(); assert(!test_drive_armed && calibrations==1); current_faults=0;
  test_drive_active=1; arm(); assert(!test_drive_armed && calibrations==1); test_drive_active=0;
  state=1; arm(); assert(!test_drive_armed && calibrations==1 && test_calibration_result==1);
  state=IDLE; offsets_ready=true; TestDriveSafetyTick(); TestCalibrationService();
  assert(test_calibration_valid && test_calibration_result==2 && !starts);
  arm(); assert(test_drive_armed==ESC_TEST_ARM_MAGIC);
  reset();
#else
  reset(); test_calibration_valid=0; test_calibration_result=0;
  arm(); assert(!test_drive_armed && !starts && test_calibration_result==0 && calibrations==0);
#endif
  reset(); TestDirectService(); assert(!starts && !test_drive_armed);
  test_direct_request=0x4452554E; TestDirectService();
  assert(!starts && test_direct_result==2 && !test_drive_run_count);
  test_direct_request=0x12345678; TestDirectService(); assert(!starts && test_direct_result==2);
#if ESC_TEST_AUTO_CAL_ON_ARM
  test_calibration_valid=0; offsets_ready=false;
  test_direct_request=0x4441524D; TestDirectService();
  assert(!test_drive_armed && test_direct_result==2 && test_calibration_result==1);
  state=IDLE; offsets_ready=true; TestDriveSafetyTick(); TestCalibrationService();
  assert(test_calibration_valid && test_calibration_result==2 && !starts);
#else
  test_calibration_valid=0; test_direct_request=0x4441524D; TestDirectService();
  assert(!test_drive_armed && test_direct_result==2);
#endif
  test_calibration_valid=1; current_faults=4;
  test_direct_request=0x4441524D; TestDirectService(); assert(!test_drive_armed);
  current_faults=0; test_direct_request=0x4441524D; TestDirectService();
  assert(test_drive_armed==ESC_TEST_ARM_MAGIC && test_direct_result==1);
  test_direct_request=0x4452554E; TestDirectService();
  assert(starts==1 && torque_planned==816 && test_direct_result==3);
  assert(!test_direct_request && test_drive_run_count && test_direct_last_request==0x4452554E);
  TestDirectService(); assert(starts==1);
  test_direct_request=0x4452554E; TestDirectService(); assert(starts==1);
  test_direct_request=0x44535450; TestDirectService();
  assert(stops==1 && !test_drive_active && test_direct_result==5);
  test_direct_request=0x4441524D; TestDirectService();
  assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  reset(); test_direct_request=0x4441524D; TestDirectService();
  tick=5000; TestDriveSafetyTick(); test_direct_request=0x4452554E; TestDirectService();
  assert(!starts && test_direct_result==2);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='half-amp-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    for extra in ([], ['-DESC_TEST_AUTO_CAL_ON_ARM=0']):
        binary = tmp / ('test-optout' if extra else 'test')
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        '-DESC_HALF_AMP_DIAGNOSTIC', *extra, '-I' + str(root / 'Inc'),
                        '-I' + str(root.parent.parent / 'common/CAN'),
                        str(tmp / 'test.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS: half-amp torque bounds, auto-CAL pending/done + opt-out hard-reject, legacy rejection, run counter, calibration/fault/current/speed/time guards')
