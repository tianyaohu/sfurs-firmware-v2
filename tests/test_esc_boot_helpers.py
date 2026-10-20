#!/usr/bin/env python3
"""Run the ESC's real shared RX, mailbox and metadata code under sanitizers."""
import os
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='esc-boot-helpers-') as tmp:
    exe = pathlib.Path(tmp) / 'test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-g',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    '-I', str(root / 'src/shared/inc'),
                    str(root / 'tests/esc_boot_helpers.c'),
                    str(root / 'src/shared/src/can_fwu_protocol.c'),
                    str(root / 'src/shared/src/can_fwu_boot.c'), '-o', str(exe)], check=True)
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0')
    subprocess.run([str(exe)], check=True, env=env)
