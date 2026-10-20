#!/usr/bin/env python3
"""Run every host test (no hardware). Exit status is nonzero on any failure.

Python suites run directly; the sensor-module gtest suites are configured,
built and run with CMake under build/gtest/ (googletest is downloaded on the
first run, so that run needs network).
"""
import os
import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLCHAIN = "/tmp/opencode/cm7-toolchain/root/usr/bin"
if not shutil.which("arm-none-eabi-gcc") and os.path.isdir(TOOLCHAIN):
    os.environ["PATH"] = TOOLCHAIN + os.pathsep + os.environ["PATH"]
GTEST_SUITES = [
    "src/sensors/stm32h7/CM7/optical_flow/tests",
    "src/sensors/stm32h7/CM7/IMU/tests",
    "src/sensors/stm32h7/CM7/wheel_encoder/tests",
]


def python_suite(test):
    return [[sys.executable, str(test)]]


def gtest_suite(rel):
    build = ROOT / "build/gtest" / rel.replace("/", "_")
    generator = ["-G", "Ninja"] if shutil.which("ninja") else []
    return [["cmake", "-S", str(ROOT / rel), "-B", str(build), *generator],
            ["cmake", "--build", str(build)],
            ["ctest", "--test-dir", str(build), "--output-on-failure"]]


suites = [(test.relative_to(ROOT), python_suite(test))
          for test in sorted((ROOT / "tests").glob("test_*.py"))]
suites.append((pathlib.Path("src/esc/bootloader/tests/run_tests.py"),
               python_suite(ROOT / "src/esc/bootloader/tests/run_tests.py")))
suites += [(pathlib.Path(rel), gtest_suite(rel)) for rel in GTEST_SUITES]
failed = []
for name, commands in suites:
    output = ""
    for command in commands:
        result = subprocess.run(command, capture_output=True, text=True, cwd=ROOT)
        output += result.stdout + result.stderr
        if result.returncode:
            break
    status = "PASS" if result.returncode == 0 else "FAIL"
    print(f"{status}  {name}", flush=True)
    if result.returncode:
        failed.append(name)
        print(output[-3000:])
print(f"{len(suites) - len(failed)}/{len(suites)} passed")
sys.exit(1 if failed else 0)
