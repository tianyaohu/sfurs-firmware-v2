#!/usr/bin/env python3
"""Ki4 configuration and real SDK PI arithmetic; no hardware or plant model."""
import ctypes as ct
import subprocess
import unittest

import test_fr_speed_tune as support

TRACE_WRAPPER = r'''
/* Synthetic integer ramp, not the STC/state-machine ramp or a motor model. */
void trace(int units, int milliseconds, int32_t *out) {
  PID_Handle_t p=handle(); int result=0;
  for(int t=0;t<milliseconds;++t) {
    int error=t<500 ? units*t/500 : units;
    result=PI_Controller(&p,error);
    if(result>1632 || result< -1632 ||
       p.wIntegralTerm>p.wUpperIntegralLimit ||
       p.wIntegralTerm<p.wLowerIntegralLimit) abort();
  }
  out[0]=result; out[1]=p.wIntegralTerm;
}
int clear_after_saturation(void) {
  PID_Handle_t p=handle();
  for(int t=0;t<5000;++t) (void)PI_Controller(&p,40);
  PID_SetIntegralTerm(&p,0); /* STC_Clear uses this API in speed mode. */
  return p.wIntegralTerm==0 && PI_Controller(&p,0)==0;
}
int reversal_after_saturation(void) {
  PID_Handle_t p=handle();
  for(int t=0;t<5000;++t) (void)PI_Controller(&p,40);
  for(int t=0;t<2000;++t) {
    int value=PI_Controller(&p,-40);
    if(value>1632 || value< -1632) abort();
    if(value>0) return t+1;
  }
  return -1;
}
'''


class Ki4Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        support.TuneTests.setUpClass.__func__(cls)
        (cls.directory / 'wrapper.c').write_text(support.WRAPPER + TRACE_WRAPPER)

    build = support.TuneTests.build
    parameters = support.TuneTests.parameters

    def candidate(self, wheel='NE', misra=False):
        return self.build(wheel, misra=misra, extra=('-DESC_RPM2_KI4',))

    def trace(self, lib, units, milliseconds):
        out = (ct.c_int32 * 2)()
        lib.trace(units, milliseconds, out)
        return list(out)

    def test_all_four_only_ki_changes(self):
        for wheel in ('NE', 'SE', 'SW', 'NW'):
            stock = self.parameters(self.build(wheel))
            candidate = self.parameters(self.candidate(wheel))
            self.assertEqual([(i, a, b) for i, (a, b) in
                              enumerate(zip(stock, candidate)) if a != b],
                             [(1, -72, -288)])
            self.assertEqual(candidate[:6], [-18600, -288, 1024, 10, 16384, 14])
            self.assertEqual(candidate[6:], [26738688, -26738688, 1632, -1632,
                                             2000, 30000, 480, -900, 900, 900,
                                             3000, 3000, 1, 1000])

    def test_signed_ramps_both_sdk_arithmetic_modes(self):
        for misra in (False, True):
            stock, candidate = self.build(misra=misra), self.candidate(misra=misra)
            for units in (4, 8, 19, 21, 39, 42, 150):
                for sign in (-1, 1):
                    for milliseconds in (500, 1000, 1500, 2000):
                        result, integral = self.trace(candidate, sign*units, milliseconds)
                        self.assertLess(result*sign, 0)
                        self.assertLessEqual(abs(result), 1632)
                        self.assertGreaterEqual(abs(result), abs(self.trace(
                            stock, sign*units, milliseconds)[0]))
                        self.assertLessEqual(abs(integral), 1632*16384)
                    self.assertEqual(stock.proportional(sign*units),
                                     candidate.proportional(sign*units))
                for milliseconds in (500, 1000, 1500):
                    positive = self.trace(candidate, units, milliseconds)[0]
                    negative = self.trace(candidate, -units, milliseconds)[0]
                    self.assertLessEqual(abs(positive+negative), 2)
            # At 1 m/s, ideal stationary demand rises without reaching the
            # command limit in one second (not proof of hardware headroom).
            for units in (39, 42):
                current = abs(self.trace(candidate, units, 1000)[0])/544
                self.assertGreater(current, 2.2)
                self.assertLess(current, 2.5)

    def test_saturation_clear_and_error_reversal(self):
        for misra in (False, True):
            lib = self.candidate(misra=misra)
            self.assertEqual(lib.saturation_and_restart(), 1)
            self.assertEqual(lib.clear_after_saturation(), 1)
            self.assertTrue(0 < lib.reversal_after_saturation() < 2000)
            for sign in (-1, 1):
                self.assertEqual(abs(self.trace(lib, sign*42, 5000)[0]), 1632)

    def test_compile_time_invalid_configurations(self):
        for macro in ('ESC_LOW_CURRENT_TEST', 'ESC_REVERSE_SPEED_DIAGNOSTIC',
                      'ESC_BOOT_OFFSET_APP', 'MOTOR_NE'):
            self.build(extra=('-DESC_RPM2_KI4', '-U'+macro), success=False)
        for macro in ('ESC_RPM_DIAGNOSTIC', 'ESC_HALF_AMP_DIAGNOSTIC',
                      'ESC_TORQUE_COUNT_DIAGNOSTIC', 'ESC_NE_LOW_SPEED_P2',
                      'ESC_TUNE_GAIN_SET', 'MOTOR_SE'):
            self.build(extra=('-DESC_RPM2_KI4', '-D'+macro), success=False)

    def test_missing_low_current_flag_fails_in_header_itself(self):
        # Do not rely on the test wrapper referencing otherwise undefined
        # profile macros: a real build must reject the bad option on its own.
        result = subprocess.run(['cc', '-std=gnu11', '-x', 'c', '-fsyntax-only',
                                 '-I'+str(support.MOTOR/'Inc'), '-DMOTOR_NE',
                                 '-DESC_RPM2_KI4', '-DESC_REVERSE_SPEED_DIAGNOSTIC',
                                 '-DESC_BOOT_OFFSET_APP', '-'], input=
                                '#include <stdint.h>\n#include "drive_parameters.h"\n',
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('ESC_RPM2_KI4 requires', result.stderr)

    def test_cli_rejects_before_creating_output(self):
        for wheel in ('NE', 'SE', 'SW', 'NW'):
            for flags in ([], ['--app-offset'], ['--reverse-speed-diagnostic'],
                          ['--app-offset', '--reverse-speed-diagnostic', '--ne-low-speed-p2'],
                          ['--app-offset', '--reverse-speed-diagnostic', '--rpm2-tune']):
                output = self.directory / 'forbidden'
                result = subprocess.run(['python3', str(support.MOTOR/'build_cli.py'),
                                         '--motor', wheel, '--rpm2-ki4', '--output',
                                         str(output), *flags], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn('--rpm2-ki4', result.stderr)
                self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
