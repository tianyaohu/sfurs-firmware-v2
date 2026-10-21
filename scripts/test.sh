#!/bin/sh
# Run every host test (no hardware): the C tests under tests/ (CTest) and the
# sensor-module gtest suites. Exit status is nonzero on any failure.
#
# The gtest suites download googletest on their first configure (needs network once).
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TOOLCHAIN=/tmp/opencode/cm7-toolchain/root/usr/bin
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1 && [ -d "$TOOLCHAIN" ]; then
  PATH="$TOOLCHAIN:$PATH"
fi
GEN=""
command -v ninja >/dev/null 2>&1 && GEN="-G Ninja"

failed=0
total=0
run() { # run NAME CMD... : print PASS/FAIL, show the tail of the output on failure
  name=$1
  shift
  total=$((total + 1))
  if out=$("$@" 2>&1); then
    echo "PASS  $name"
  else
    echo "FAIL  $name"
    printf '%s\n' "$out" | tail -n 40
    failed=$((failed + 1))
  fi
}

# shellcheck disable=SC2086
run "tests/ (CTest)" sh -c "cmake -S '$ROOT/tests' -B '$ROOT/build/host' $GEN >/dev/null &&
  cmake --build '$ROOT/build/host' &&
  ctest --test-dir '$ROOT/build/host' --output-on-failure"

# shellcheck disable=SC2086
run "src/tools (build)" sh -c "cmake -S '$ROOT/src/tools' -B '$ROOT/build/tools' $GEN >/dev/null &&
  cmake --build '$ROOT/build/tools'"

for suite in src/sensors/stm32h7/CM7/optical_flow/tests \
             src/sensors/stm32h7/CM7/IMU/tests \
             src/sensors/stm32h7/CM7/wheel_encoder/tests; do
  build="$ROOT/build/gtest/$(echo "$suite" | tr / _)"
  # shellcheck disable=SC2086
  run "$suite" sh -c "cmake -S '$ROOT/$suite' -B '$build' $GEN >/dev/null &&
    cmake --build '$build' &&
    ctest --test-dir '$build' --output-on-failure"
done

echo "$((total - failed))/$total passed"
[ "$failed" -eq 0 ]
