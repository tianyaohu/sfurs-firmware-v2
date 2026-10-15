#!/usr/bin/env python3
"""Offline tests of the short RPM validation profile; no hardware access."""
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
  direct(0x4441524d); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  direct(0x4452554e);
  assert(starts == 1 && test_drive_run_count && test_drive_active);
  assert(speed_planned == ESC_TEST_TARGET_RPM && speed_ramp_ms == 500 && !torque_planned);
}
int main(void) {
  assert(ESC_TEST_RPM_PROFILE && !ESC_TEST_TORQUE_PROFILE);
  assert(ESC_TEST_REFERENCE_A == 3.0f && ESC_TEST_SAMPLED_STOP_A == 3.0f);
  assert(ESC_TEST_TIMEOUT_MS == 10000 && ESC_TEST_TARGET_RPM == 60);
  reset(); send(TEST_ARM | 1, 0x4c46, 0x4841); assert(!test_drive_armed);
  send(SPEED_WRITE | 1, 60, 500); assert(!starts);
  send(ESC_TEST_TORQUE_COMMAND | 1, 816, 200); assert(!starts);
  reset(); direct(0x4452554e); assert(!starts && test_direct_result == 2);
  direct(0x12345678); assert(!starts && test_direct_result == 2);
  test_calibration_valid = 0; direct(0x4441524d); assert(!test_drive_armed);
  test_calibration_valid = 1; current_faults = 4;
  direct(0x4441524d); assert(!test_drive_armed);
  current_faults = 0; start(); direct(0x4452554e); assert(starts == 1);
  tick = 9999; test_drive_hall_last_tick = tick; TestDriveSafetyTick(); assert(!stops);
  tick = 10000; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 1);
  reset(); direct(0x4441524d);
  assert(!TestDriveStart(100, 500) && !TestDriveStart(60, 499) && !TestDriveStart(60, 1000));
  assert(!starts && !test_drive_run_count); direct(0x4452554e); assert(starts == 1);
  reset(); start(); state = RUN;
  measured.q = 1600; measured.d = -31; TestDriveSafetyTick(); assert(!stops);
  measured.d = -32; TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 4 && test_drive_peak_l1 == 1632);
  reset(); start(); measured_speed = 120; TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 5);
  reset(); start(); measured_speed = -120; TestDriveSafetyTick(); assert(stops == 1);
  reset(); start(); offsets_ready = false; TestDriveSafetyTick();
  assert(stops == 1 && test_drive_stop_reason == 6);
  reset(); start(); current_faults = 4; TestDriveSafetyTick(); assert(stops == 1);
  reset(); TestDriveHallTransition(5, 1); assert(!test_drive_hall_edges);
  start();
  const uint8_t cycle[] = {5, 1, 3, 2, 6, 4, 5};
  for (int revolution = 0; revolution < 8; ++revolution)
    for (int sector = 0; sector < 6; ++sector)
      TestDriveHallTransition(cycle[sector], cycle[sector+1]);
  assert(test_drive_hall_edges == 48 && test_drive_hall_net_edges == 48 && !test_drive_hall_invalid);
  TestDriveHallTransition(5, 4);
  assert(test_drive_hall_edges == 49 && test_drive_hall_net_edges == 47);
  TestDriveHallTransition(4, 1); assert(test_drive_hall_invalid == 1);
  TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 7);
  TestDriveHallTransition(1, 3); assert(test_drive_hall_edges == 49);
  reset(); start(); TestDriveHallTransition(0, 1); TestDriveHallTransition(1, 7);
  TestDriveHallTransition(1, 1); assert(test_drive_hall_invalid == 3);
  reset(); start(); direct(0x44535450); assert(stops == 1 && !test_drive_active);
  direct(0x4441524d); assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  reset(); direct(0x4441524d); tick = 5000; TestDriveSafetyTick();
  direct(0x4452554e); assert(!starts);
  reset(); allow_start = false; direct(0x4441524d); direct(0x4452554e);
  assert(test_drive_run_count && !test_drive_active && test_direct_result == 4);
  reset(); start(); test_drive_start_tick = UINT32_MAX - 499;
  tick = 9499; test_drive_hall_last_tick = tick; TestDriveSafetyTick(); assert(!stops);
  tick = 9500; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 1);
  reset(); start(); tick = 1999; TestDriveSafetyTick(); assert(!stops);
  tick = 2000; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 9);
  reset(); start(); tick = 1500; TestDriveHallTransition(5, 1);
  tick = 3499; TestDriveSafetyTick(); assert(!stops);
  tick = 3500; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 9);
  reset(); start(); test_drive_hall_last_tick = UINT32_MAX-499;
  tick = 1499; TestDriveSafetyTick(); assert(!stops);
  tick = 1500; TestDriveSafetyTick(); assert(stops == 1 && test_drive_stop_reason == 9);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='rpm-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-DESC_RPM_DIAGNOSTIC', '-I' + str(root / 'Inc'),
                    '-I' + str(root.parent.parent / 'common/CAN'),
                    str(tmp / 'test.c'), '-o', str(tmp / 'test')], check=True)
    subprocess.run([str(tmp / 'test')], check=True)
print('PASS: fixed 60 RPM, 3 A reference, 10-second timeout, Hall counts/stall/invalid transitions, run counter and safety gates')
