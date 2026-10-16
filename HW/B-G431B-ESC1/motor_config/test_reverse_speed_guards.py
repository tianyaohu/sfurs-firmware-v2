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
harness = harness.replace('#define MOTOR_CAN_ID MOTOR_NE_CAN_ID',
                          '#include "motor_controller_can.h"')
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
  assert(ESC_TEST_TIMEOUT_MS == 30000 && ESC_TEST_MAX_MOTOR_RPM == 900);
  assert(ESC_TEST_RPM_MIN == -900 && ESC_TEST_RPM_MAX == 900);
  assert(ESC_TEST_REFERENCE_A == 3.0f && ESC_TEST_SAMPLED_STOP_A == 3.0f);
  assert(ESC_TEST_SPEED_KP == -18600);
#ifdef ESC_RPM2_KI4
  assert(ESC_TEST_SPEED_KI == -288);
#else
  assert(ESC_TEST_SPEED_KI == -72);
#endif
  assert(ESC_TEST_COUNT_PROFILE && ESC_TEST_TARGET_NET_EDGES == 480);
  reset(); send(TEST_ARM | MOTOR_CAN_ID, 0x4c46, 0x4841); assert(!test_drive_armed);
  send(SPEED_WRITE | MOTOR_CAN_ID, 60, 500); assert(!starts);
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
  const int bad_values[]={0,901,-901,32767,-32768};
  for (unsigned i=0;i<sizeof(bad_values)/sizeof(bad_values[0]);++i) {
    reset(); direct(0x4441524D);
    assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
    assert(!TestDriveStart(bad_values[i],500) && starts==0);
    assert(test_drive_armed == ESC_TEST_ARM_MAGIC); /* Range gate, not consumed ARM. */
  }
  reset(); direct(0x4441524D);
  assert(!TestDriveStart(-100,499) && !TestDriveStart(60,501) && starts==0);
  reset(); direct(0x4441524D); assert(TestDriveStart(-120, 500) == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(-30, 500) == 1);
  reset(); direct(0x4441524D); assert(TestDriveStart(60, 500) == 1);
  for (int sign=-1;sign<=1;sign+=2) {
    reset(); send(TEST_ARM | MOTOR_CAN_ID,0x4d32,0x5250);
    assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
    send(SPEED_WRITE | MOTOR_CAN_ID,sign*900,500);
    assert(starts==1 && test_drive_active && speed_planned==sign*900);
    state=RUN; measured_speed=sign*900; TestDriveSafetyTick(); assert(!stops);
    measured_speed=sign*901; TestDriveSafetyTick();
    assert(stops==1 && !test_drive_active && test_drive_stop_reason==5);
  }
  const unsigned wheels[]={1,2,4,8};
  for (unsigned i=0;i<4;++i) {
    if (wheels[i]==MOTOR_CAN_ID) continue;
    reset(); send(TEST_ARM | wheels[i],0x4d32,0x5250);
    assert(!test_drive_armed && !starts);
    direct(0x4441524D);
    send(SPEED_WRITE | wheels[i],900,500); assert(!starts);
  }
  reset(); start(); state=RUN; measured.q=1632; TestDriveSafetyTick();
  assert(stops==1 && test_drive_stop_reason==4); /* Same sampled-current stop. */
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
    for wheel in ('NE','SE','SW','NW'):
        for index, extra in enumerate(([], ['-DESC_TEST_AUTO_CAL_ON_ARM=0'],
                ['-DESC_RPM2_KI4', '-DESC_BOOT_OFFSET_APP'],
                ['-DESC_RPM2_KI4', '-DESC_BOOT_OFFSET_APP', '-DESC_TEST_AUTO_CAL_ON_ARM=0'])):
            binary = tmp / (wheel + '-' + str(index))
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-DMOTOR_' + wheel, '-DESC_LOW_CURRENT_TEST',
                            '-DESC_REVERSE_SPEED_DIAGNOSTIC', *extra,
                            '-I' + str(root / 'Inc'), '-I' + str(root.parent / 'Inc'),
                            '-I' + str(root.parent.parent / 'common/CAN'),
                            str(tmp / 'test.c'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
print('PASS: stock/Ki4 all four RPM2 selectors, inclusive +/-900 RPM range, overspeed beyond endpoints, fresh-ARM range/ramp rejection, unchanged CAL/current/stall/count/timeout stops')
