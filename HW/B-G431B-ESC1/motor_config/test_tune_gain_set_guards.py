#!/usr/bin/env python3
"""Offline tests of the RPM2 runtime GAIN_SET command (ESC_TUNE_GAIN_SET);
no hardware access. Reuses the shared guard harness with minimal PID fakes."""
import pathlib
import runpy
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent
original = runpy.run_path(str(root / 'test_low_current_guards.py'))
harness = original['harness'].split('int main(void) {', 1)[0]
harness = harness.replace('#define MOTOR_CAN_ID MOTOR_NE_CAN_ID',
                          '#include "motor_controller_can.h"')
PID_FAKES = r'''
#if defined(ESC_TUNE_GAIN_SET)
typedef struct { int16_t hKpGain; int16_t hKiGain; } PID_Handle_t;
static PID_Handle_t PIDSpeedHandle_M1;
void PID_SetKP(PID_Handle_t *p, int16_t v) { p->hKpGain = v; }
void PID_SetKI(PID_Handle_t *p, int16_t v) { p->hKiGain = v; }
#endif
'''
harness = harness.replace('/* Explicit diagnostic request words',
                          PID_FAKES + '\n/* Explicit diagnostic request words', 1)
harness += r'''
#if defined(ESC_TUNE_GAIN_SET)
static void arm(void) {
  send(TEST_ARM | MOTOR_CAN_ID, 0x4d32, 0x5250);
  assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
}
#endif
int main(void) {
#if defined(ESC_TUNE_GAIN_SET)
  assert(ESC_TEST_RPM_PROFILE && !ESC_TEST_TORQUE_PROFILE);
  assert(ESC_TEST_SPEED_KP == -18600 && ESC_TEST_SPEED_KI == -72);
  assert(TEST_GAIN_SET == 0x500U);
  reset();
  PIDSpeedHandle_M1.hKpGain = ESC_TEST_SPEED_KP;
  PIDSpeedHandle_M1.hKiGain = ESC_TEST_SPEED_KI;
  test_speed_kp_gain = ESC_TEST_SPEED_KP; test_speed_ki_gain = ESC_TEST_SPEED_KI;
  test_gain_set_result = 0; test_gain_set_count = 0;

  /* Unarmed: state rejection, nothing applied. */
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, -72);
  assert(test_gain_set_count == 1 && test_gain_set_result == 2);
  assert(PIDSpeedHandle_M1.hKpGain == ESC_TEST_SPEED_KP);

  /* Armed + IDLE + calibrated: applies, keeps the armed window, no start. */
  arm();
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, -864);
  assert(test_gain_set_count == 2 && test_gain_set_result == 1);
  assert(PIDSpeedHandle_M1.hKpGain == -22528 && PIDSpeedHandle_M1.hKiGain == -864);
  assert(test_speed_kp_gain == -22528 && test_speed_ki_gain == -864);
  assert(test_drive_armed == ESC_TEST_ARM_MAGIC && !starts && !test_drive_active);

  /* Bounds: sign, magnitude floor, int16 min; count increments, no apply. */
  {
    const int16_t bad_kp[] = {-999, -1, 0, 1, 18600, -32768};
    for (unsigned i = 0; i < sizeof(bad_kp)/sizeof(bad_kp[0]); ++i) {
      uint32_t before = test_gain_set_count;
      send(TEST_GAIN_SET | MOTOR_CAN_ID, bad_kp[i], -72);
      assert(test_gain_set_count == before + 1 && test_gain_set_result == 3);
      assert(PIDSpeedHandle_M1.hKpGain == -22528);
    }
    send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, 0);
    assert(test_gain_set_result == 3 && PIDSpeedHandle_M1.hKiGain == -864);
    send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, 72);
    assert(test_gain_set_result == 3 && PIDSpeedHandle_M1.hKiGain == -864);
    send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, -32768);
    assert(test_gain_set_result == 3 && PIDSpeedHandle_M1.hKiGain == -864);
  }
  /* Full int16 magnitude range applies (kp 32.0 / ki 2.0 ceiling). */
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -32767, -32767);
  assert(test_gain_set_result == 1);
  assert(PIDSpeedHandle_M1.hKpGain == -32767 && PIDSpeedHandle_M1.hKiGain == -32767);
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -1000, -1);
  assert(test_gain_set_result == 1);
  assert(PIDSpeedHandle_M1.hKpGain == -1000 && PIDSpeedHandle_M1.hKiGain == -1);

  /* State rejections: faults, active drive, non-IDLE, expired/other wheel. */
  current_faults = 4;
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -18600, -72);
  assert(test_gain_set_result == 2 && PIDSpeedHandle_M1.hKpGain == -1000);
  current_faults = 0;
  test_drive_active = 1;
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -18600, -72);
  assert(test_gain_set_result == 2 && PIDSpeedHandle_M1.hKpGain == -1000);
  test_drive_active = 0;
  state = RUN;
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -18600, -72);
  assert(test_gain_set_result == 2 && PIDSpeedHandle_M1.hKpGain == -1000);
  state = IDLE;
  {
    const unsigned wheels[] = {1, 2, 4, 8};
    uint32_t before = test_gain_set_count;
    for (unsigned i = 0; i < 4; ++i) {
      if (wheels[i] == MOTOR_CAN_ID) continue;
      send(TEST_GAIN_SET | wheels[i], -18600, -72);
    }
    assert(test_gain_set_count == before && PIDSpeedHandle_M1.hKpGain == -1000);
  }

  /* STOP clears armed; GAIN_SET then state-rejects; applied gains persist
   * across STOP/re-ARM (RAM-only, nothing restores defaults at runtime). */
  send(MOTOR_STOP | MOTOR_CAN_ID, 0, 0);
  assert(!test_drive_armed);
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -18600, -72);
  assert(test_gain_set_result == 2 && PIDSpeedHandle_M1.hKpGain == -1000);
  arm();
  assert(PIDSpeedHandle_M1.hKpGain == -1000 && PIDSpeedHandle_M1.hKiGain == -1);

  /* A started run still consumes ARM; GAIN_SET after start state-rejects. */
  send(SPEED_WRITE | MOTOR_CAN_ID, -100, 500);
  assert(starts == 1 && !test_drive_armed);
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -18600, -72);
  assert(test_gain_set_result == 2 && PIDSpeedHandle_M1.hKpGain == -1000);
  state = IDLE; test_drive_active = 0;
  arm();
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -18600, -72);
  assert(test_gain_set_result == 1);
  assert(PIDSpeedHandle_M1.hKpGain == ESC_TEST_SPEED_KP &&
         PIDSpeedHandle_M1.hKiGain == ESC_TEST_SPEED_KI);
  assert(test_speed_kp_gain == ESC_TEST_SPEED_KP &&
         test_speed_ki_gain == ESC_TEST_SPEED_KI);
#else
  /* Without ESC_TUNE_GAIN_SET the slot-10 frame is inert: no start, no arm. */
  reset();
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, -864);
  assert(!starts && test_drive_run_count == 0 && !test_drive_armed);
  send(TEST_ARM | MOTOR_CAN_ID, 0x4d32, 0x5250);
  assert(test_drive_armed == ESC_TEST_ARM_MAGIC);
  send(TEST_GAIN_SET | MOTOR_CAN_ID, -22528, -864);
  assert(!starts && test_drive_armed == ESC_TEST_ARM_MAGIC);
#endif
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='tune-gain-set-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    for wheel in ('NE', 'SE', 'SW', 'NW'):
        for flags, tag in ((['-DESC_TUNE_GAIN_SET'], 'tune'), ([], 'plain')):
            binary = tmp / (wheel + '-' + tag)
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-DMOTOR_' + wheel, '-DESC_REVERSE_SPEED_DIAGNOSTIC', *flags,
                            '-I' + str(root / 'Inc'), '-I' + str(root.parent / 'Inc'),
                            '-I' + str(root.parent.parent / 'common/CAN'),
                            str(tmp / 'test.c'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
print('PASS: GAIN_SET armed+IDLE gate, sign/floor/int16 bounds, per-wheel addressing, '
      'armed window preserved, STOP/start consumption, RAM persistence, inert without ESC_TUNE_GAIN_SET')
