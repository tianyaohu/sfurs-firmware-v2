#!/usr/bin/env python3
"""Compile the real startup-reference block for every wheel/profile.

Guards must boot without a queued drive command; ordinary SDK builds retain
their default reference. Full drive-start behavior is covered by guard tests.
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent
source = (root / 'Src/mc_tasks_foc.c').read_text()
start = source.index('#if !ESC_TEST_GUARDED_MOTION')
end = source.index('#endif', start) + len('#endif')
block = source[start:end]
harness = '''
#include <assert.h>
#include "test_profile.h"
#if !ESC_TEST_GUARDED_MOTION
static int Mci[1], storage, *pSTC[1] = {&storage};
static int STC_GetMecSpeedRefUnitDefault(int *s) { (void)s; return 60; }
static int queued;
static void MCI_ExecSpeedRamp(int *m, int speed, int duration) {
    (void)m; assert(speed == 60 && duration == 0); ++queued;
}
#else
static int queued;
#endif
#define M1 0
int main(void) {
''' + block + '''
    assert(queued == !ESC_TEST_GUARDED_MOTION);
    return 0;
}
'''
profiles = [None, 'ESC_HALF_AMP_DIAGNOSTIC', 'ESC_RPM_DIAGNOSTIC',
            'ESC_REVERSE_SPEED_DIAGNOSTIC', 'ESC_TORQUE_COUNT_DIAGNOSTIC']
with tempfile.TemporaryDirectory(prefix='boot-reference-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    for wheel in ('NE', 'SE', 'SW', 'NW'):
        for profile in profiles:
            flags = ['-DMOTOR_' + wheel] + (['-D' + profile] if profile else [])
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            *flags, '-I' + str(root / 'Inc'),
                            str(tmp / 'test.c'), '-o', str(tmp / 'test')], check=True)
            subprocess.run([str(tmp / 'test')], check=True)
print('PASS: all four wheels boot guarded profiles without a queued drive; stock reference retained')
