#!/usr/bin/env bash
# Host tests (kinematics and the motor-frame codec), then cross-compile the
# vendored Nucleo and ESC images. No board, probe, or CAN interface.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
echo "host tests"
cmake -S "$root/tests" -B "$root/build/host" -G Ninja
cmake --build "$root/build/host"
ctest --test-dir "$root/build/host" --output-on-failure

echo "cross-compiling firmware images (no board)"
tool=$root/src/nucleo/gcc-arm-none-eabi.cmake
build_one() {
  echo "building $1"
  cmake -S "$2" -B "$3" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE="$tool" \
    -DCMAKE_BUILD_TYPE=Debug
  cmake --build "$3"
}
build_one "CM7 image" "$root/src/nucleo/CM7" "$root/build/nucleo-cm7"
build_one "CM4 skeleton" "$root/src/nucleo/CM4" "$root/build/nucleo-cm4"
build_one "ESC image" "$root/src/esc/motor_config" "$root/build/esc"
