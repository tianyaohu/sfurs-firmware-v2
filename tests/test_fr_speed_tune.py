#!/usr/bin/env python3
"""FR P2 opt-in and actual SDK PI arithmetic tests; no hardware access."""
import ctypes as ct
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
MOTOR = ROOT / 'HW/B-G431B-ESC1/motor_config'
SDK = MOTOR / 'MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any'
WRAPPER = r'''
#include <stdint.h>
#include <stdlib.h>
#define SPEED_UNIT 10
#define LOG2(x) __builtin_ctz((unsigned)(x))
#include "drive_parameters.h"
#include "pid_regulator.h"

static PID_Handle_t handle(void) {
  PID_Handle_t p = {
    .hDefKpGain = PID_SPEED_KP_DEFAULT, .hDefKiGain = PID_SPEED_KI_DEFAULT,
    .wUpperIntegralLimit = 1632 * SP_KIDIV,
    .wLowerIntegralLimit = -1632 * SP_KIDIV,
    .hUpperOutputLimit = 1632, .hLowerOutputLimit = -1632,
    .hKpDivisor = SP_KPDIV, .hKiDivisor = SP_KIDIV,
    .hKpDivisorPOW2 = SP_KPDIV_LOG, .hKiDivisorPOW2 = SP_KIDIV_LOG
  };
  PID_HandleInit(&p);
  return p;
}
void parameters(int32_t *out) {
  PID_Handle_t p=handle();
  const int32_t values[]={p.hKpGain,p.hKiGain,p.hKpDivisor,p.hKpDivisorPOW2,
    p.hKiDivisor,p.hKiDivisorPOW2,p.wUpperIntegralLimit,p.wLowerIntegralLimit,
    p.hUpperOutputLimit,p.hLowerOutputLimit,ESC_TEST_HALL_STALL_MS,
    ESC_TEST_TIMEOUT_MS,ESC_TEST_TARGET_NET_EDGES,ESC_TEST_RPM_MIN,
    ESC_TEST_RPM_MAX,ESC_TEST_MAX_MOTOR_RPM,
    (int32_t)(ESC_TEST_REFERENCE_A*1000),(int32_t)(ESC_TEST_SAMPLED_STOP_A*1000),
    ESC_TEST_AUTO_CAL_ON_ARM,SPEED_LOOP_FREQUENCY_HZ};
  for(unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i) out[i]=values[i];
}
int proportional(int error) {
  PID_Handle_t p=handle(); p.hKiGain=0; return PI_Controller(&p,error);
}
int ramp_then_hold(int units) {
  PID_Handle_t p=handle(); int result=0;
  for(int t=0;t<2000;++t) {
    int error=t<500 ? units*t/500 : units;
    result=PI_Controller(&p,error);
    if(result>1632 || result< -1632) abort();
  }
  return result;
}
int saturation_and_restart(void) {
  PID_Handle_t p=handle();
  for(int sign=-1;sign<=1;sign+=2) {
    for(int t=0;t<5000;++t) {
      int result=PI_Controller(&p,sign*1000);
      if(result>1632 || result< -1632 ||
         p.wIntegralTerm>p.wUpperIntegralLimit ||
         p.wIntegralTerm<p.wLowerIntegralLimit) return 0;
    }
  }
  PID_HandleInit(&p);
  return p.wIntegralTerm==0 && PI_Controller(&p,0)==0;
}
'''


class TuneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='fr-p2-test-', dir='/tmp/opencode')
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.directory = pathlib.Path(cls.tmp.name)
        (cls.directory / 'mc_type.h').write_text(
            '#include <stdint.h>\n#include <limits.h>\n#include <stdbool.h>\n'
            '#define __weak __attribute__((weak))\n#define MC_NULL ((void*)0)\n')
        (cls.directory / 'wrapper.c').write_text(WRAPPER)

    def build(self, wheel='NE', candidate=False, misra=False, extra=(), success=True):
        name=f'{wheel}-{candidate}-{misra}-{len(extra)}-{len(list(self.directory.glob("*.so")))}'
        output=self.directory / (name+'.so')
        flags=['-DMOTOR_'+wheel, '-DESC_LOW_CURRENT_TEST',
               '-DESC_REVERSE_SPEED_DIAGNOSTIC', '-DESC_BOOT_OFFSET_APP']
        if candidate: flags.append('-DESC_NE_LOW_SPEED_P2')
        if misra: flags.append('-DFULL_MISRA_C_COMPLIANCY_PID_REGULATOR')
        command=['cc','-std=gnu11','-Wall','-Wextra','-Werror','-shared','-fPIC',
                 '-I',str(self.directory),'-I',str(MOTOR/'Inc'),'-I',str(SDK/'Inc'),
                 *flags,*extra,str(self.directory/'wrapper.c'),
                 str(SDK/'Src/pid_regulator.c'),'-o',str(output)]
        result=subprocess.run(command,capture_output=True,text=True)
        if not success:
            self.assertNotEqual(result.returncode,0)
            return
        self.assertEqual(result.returncode,0,result.stderr)
        return ct.CDLL(str(output))

    def parameters(self, lib):
        data=(ct.c_int32*20)(); lib.parameters(data); return list(data)

    def test_baseline_all_four_unchanged_and_candidate_single_variable(self):
        baseline=self.parameters(self.build())
        for wheel in ('SE','SW','NW'):
            self.assertEqual(self.parameters(self.build(wheel)),baseline)
        candidate=self.parameters(self.build(candidate=True))
        self.assertEqual([(i,a,b) for i,(a,b) in enumerate(zip(baseline,candidate)) if a!=b],
                         [(2,1024,512),(3,10,9)])
        self.assertEqual(baseline[:2],[-18600,-72])
        self.assertEqual(baseline[4:],[16384,14,26738688,-26738688,1632,-1632,
                                    2000,30000,480,-900,900,900,3000,3000,1,1000])

    def test_real_pi_both_division_modes_signed_errors_ramp_and_saturation(self):
        for misra in (False,True):
            baseline=self.build(misra=misra)
            candidate=self.build(candidate=True,misra=misra)
            for error in (-8,-4,0,4,8):
                a,b=baseline.proportional(error),candidate.proportional(error)
                self.assertLessEqual(abs(b-2*a),1)
                self.assertTrue(error==0 or b*error<0) # preserve baseline gain sign
                self.assertGreaterEqual(abs(candidate.ramp_then_hold(error)),
                                        abs(baseline.ramp_then_hold(error)))
            self.assertEqual(candidate.saturation_and_restart(),1)
            self.assertEqual(baseline.saturation_and_restart(),1)

    def test_candidate_rejects_other_wheels_and_conflicting_profiles(self):
        for wheel in ('SE','SW','NW'):
            self.build(wheel,candidate=True,success=False)
        for macro in ('ESC_RPM_DIAGNOSTIC','ESC_HALF_AMP_DIAGNOSTIC',
                      'ESC_TORQUE_COUNT_DIAGNOSTIC','MOTOR_SE'):
            self.build(candidate=True,extra=('-D'+macro,),success=False)
        for undef in ('ESC_LOW_CURRENT_TEST','ESC_REVERSE_SPEED_DIAGNOSTIC',
                      'ESC_BOOT_OFFSET_APP','MOTOR_NE'):
            self.build(candidate=True,extra=('-U'+undef,),success=False)

    def test_cli_rejects_bad_combinations_before_building(self):
        for wheel,options in [('SE',['--reverse-speed-diagnostic','--app-offset']),
                              ('NE',['--app-offset']),
                              ('NE',['--reverse-speed-diagnostic'])]:
            result=subprocess.run(['python3',str(MOTOR/'build_cli.py'),'--motor',wheel,
                                   '--ne-low-speed-p2','--output',str(self.directory/'forbidden'),
                                   *options],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('--ne-low-speed-p2 requires',result.stderr)
        self.assertFalse((self.directory/'forbidden').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
