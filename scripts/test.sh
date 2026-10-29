#!/usr/bin/env bash
# No host tests yet. Cross-compile the vendored Nucleo images. No board.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
echo "no host tests yet; cross-compiling Nucleo images (no board)"
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
