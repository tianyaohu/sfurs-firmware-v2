#!/usr/bin/env python3
"""Run every host test (no hardware). Exit status is nonzero on any failure."""
import os
import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLCHAIN = "/tmp/opencode/cm7-toolchain/root/usr/bin"
if not shutil.which("arm-none-eabi-gcc") and os.path.isdir(TOOLCHAIN):
    os.environ["PATH"] = TOOLCHAIN + os.pathsep + os.environ["PATH"]
tests = sorted((ROOT / "tests").glob("test_*.py"))
tests.append(ROOT / "HW/B-G431B-ESC1/bootloader/tests/run_tests.py")
failed = []
for test in tests:
    result = subprocess.run([sys.executable, str(test)], capture_output=True, text=True, cwd=ROOT)
    status = "PASS" if result.returncode == 0 else "FAIL"
    print(f"{status}  {test.relative_to(ROOT)}", flush=True)
    if result.returncode:
        failed.append(test)
        print((result.stdout + result.stderr)[-3000:])
print(f"{len(tests) - len(failed)}/{len(tests)} passed")
sys.exit(1 if failed else 0)
