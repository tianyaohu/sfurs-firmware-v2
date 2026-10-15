#!/usr/bin/env python3
"""Compile and run the real portable helpers and vendored session, without HAL."""
import pathlib
import subprocess
import tempfile

boot = pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='esc-boot-tests-', dir='/tmp/opencode') as out:
    exe = pathlib.Path(out) / 'test_boot_helpers'
    subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    '-I' + str(boot), str(boot / 'tests/test_boot_helpers.c'),
                    str(boot / 'can_fwu_protocol.c'), str(boot / 'can_fwu_boot.c'),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
