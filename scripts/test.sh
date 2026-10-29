#!/usr/bin/env bash
# No host tests yet. Build the vendored Nucleo images.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
echo "no host tests yet"
tool=$root/src/nucleo/gcc-arm-none-eabi.cmake
build_one() {
  echo "building $1"
  cmake -S "$2" -B "$3" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE="$tool" \
    -DCMAKE_BUILD_TYPE=Debug
  cmake --build "$3"
}
build_one "CM7 blink" "$root/src/nucleo/CM7" "$root/build/nucleo-cm7"
build_one "CM4 skeleton" "$root/src/nucleo/CM4" "$root/build/nucleo-cm4"
