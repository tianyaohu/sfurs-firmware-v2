#!/usr/bin/env python3
"""Offline SDK PI arithmetic check, not a motor/closed-loop stability model.

Uses the actual SDK PI implementation/type and current source constants. Simulates
stationary feedback with a 500 ms reference ramp; never accesses hardware.
"""
import pathlib
import argparse
import re
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trial', action='store_true', help='Check the opt-in experimental RPM gains')
args = parser.parse_args()
sdk = root / 'MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any'
header = (sdk / 'Inc/pid_regulator.h').read_text()
pid_type = 'typedef struct' + header.split('typedef struct', 1)[1].split('PID_Handle_t;', 1)[0] + 'PID_Handle_t;'
source = (sdk / 'Src/pid_regulator.c').read_text()
function = source.split('__weak int16_t PI_Controller(', 1)[1].split('\n}\n', 1)[0]
function = 'int16_t PI_Controller(' + function + '\n}\n'
params = (root / 'Inc/drive_parameters.h').read_text()

def constant(name):
    value = re.search(r'^#define\s+' + name + r'\s+(?:\(uint16_t\))?(\d+)', params, re.M)
    if not value:
        raise ValueError('Missing numeric speed parameter: ' + name)
    return int(value[1])

units = (root / 'Inc/mc_stm_types.h').read_text()
assert re.search(r'^#define\s+SPEED_UNIT\s+U_01HZ\s*$', units, re.M)
assert re.search(r'^#define\s+U_01HZ\s+10\s*$', units, re.M)
kp, ki = constant('PID_SPEED_KP_DEFAULT'), constant('PID_SPEED_KI_DEFAULT')
assert kp == 2325 and ki == 9
assert constant('SP_KPDIV') == 1024 and constant('SP_KIDIV') == 16384
assert constant('SPEED_LOOP_FREQUENCY_HZ') == 1000
if args.trial:
    profile = (root / 'Inc/test_profile.h').read_text()
    kp = int(re.search(r'^#define\s+ESC_TEST_SPEED_KP\s+(\d+)', profile, re.M)[1])
    ki = int(re.search(r'^#define\s+ESC_TEST_SPEED_KI\s+(\d+)', profile, re.M)[1])
    assert kp == 18600 and ki == 72
    assert '#define PID_SPEED_KP_DEFAULT (ESC_TEST_SPEED_KP/(SPEED_UNIT/10))' in params
    assert '#define PID_SPEED_KI_DEFAULT (ESC_TEST_SPEED_KI/(SPEED_UNIT/10))' in params
code = '#include <stdint.h>\n#include <limits.h>\n#include <assert.h>\n#include <stdio.h>\n'
code += pid_type + '\n' + function
code += r'''
static PID_Handle_t handle(void) {
  PID_Handle_t p = {0};
  p.hKpGain = CHECK_KP; p.hKiGain = CHECK_KI;
  p.hKpDivisor = 1024; p.hKiDivisor = 16384;
  p.hKpDivisorPOW2 = 10; p.hKiDivisorPOW2 = 14;
  p.hUpperOutputLimit = 1632; p.hLowerOutputLimit = -1632;
  p.wUpperIntegralLimit = 1632 * 16384;
  p.wLowerIntegralLimit = -1632 * 16384;
  return p;
}
int main(void) {
  for (int rpm = 60; rpm <= 100; rpm += 40) {
    PID_Handle_t p = handle();
    int target = rpm * 10 / 60;
    int output = 0;
    for (int ms = 1; ms <= 10000; ++ms) {
      int error = target * (ms < 500 ? ms : 500) / 500;
      output = PI_Controller(&p, error);
      assert(output >= 0 && output <= 1632);
      if (ms == 500 || ms == 2000 || ms == 10000)
        printf("requested_rpm=%d encoded_rpm=%d ms=%d iq_reference_sdk=%d nominal_a=%.6f\n",
               rpm, target * 60 / 10, ms, output, output / 544.0);
      if (rpm == 60 && ms == 2000) {
        if (CHECK_KP == 2325) assert(output == 31);
        else assert(output >= 250 && output <= 270);
      }
    }
  }
  PID_Handle_t p = handle();
  for (int i = 0; i < 10000; ++i) {
    int output = PI_Controller(&p, 1000);
    assert(output <= 1632 && output >= -1632);
  }
  assert(PI_Controller(&p, 1000) == 1632);
  p = handle();
  for (int i = 0; i < 10000; ++i) {
    int output = PI_Controller(&p, -1000);
    assert(output <= 1632 && output >= -1632);
  }
  assert(PI_Controller(&p, -1000) == -1632);
  return 0;
}
'''
code = code.replace('CHECK_KP', str(kp)).replace('CHECK_KI', str(ki))
print('Checking ' + ('experimental' if args.trial else 'original') + f' gains: Kp={kp}/1024 Ki={ki}/16384')
with tempfile.TemporaryDirectory(prefix='speed-pi-check-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(code)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    str(tmp / 'test.c'), '-o', str(tmp / 'test')], check=True)
    subprocess.run([str(tmp / 'test')], check=True)
print('PASS: actual SDK PI arithmetic, ramp response and +/-3 A nominal saturation; NOT physical tuning validation')
