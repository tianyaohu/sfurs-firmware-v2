#!/bin/sh
# Build every artifact into build/, without accessing hardware:
# esc/<W>/motor_<W>_drive_offset.{elf,bin}, esc/<W>/fwu_payload.c,
# updater/<W>/sfurs_firmware_v2_CM7.elf, drive/sfurs_firmware_v2_CM7.elf.
. "$(dirname -- "$0")/build_common.sh"
wheels='NE SE SW NW'
shift_NE= shift_SE= shift_SW= shift_NW=
drive_only=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --*=*) option=$1; shift; set -- "${option%%=*}" "${option#*=}" "$@" ;;
  esac
  case "$1" in
    --wheels)
      shift; wheels=
      while [ "$#" -gt 0 ]; do
        case "$1" in --*) break ;; esac
        wheel_valid "$1" || fail "Unknown wheel: $1"
        wheels="$wheels $1"; shift
      done
      [ -n "$wheels" ] || fail '--wheels requires at least one wheel' ;;
    --shift)
      shift
      while [ "$#" -gt 0 ]; do
        case "$1" in --*) break ;; esac
        wheel=${1%%=*}; value=${1#*=}
        wheel_valid "$wheel" && shift_valid "$value" && [ "$value" != "$1" ] || fail "Invalid shift: $1"
        case "$wheel" in NE) shift_NE=$value ;; SE) shift_SE=$value ;; SW) shift_SW=$value ;; NW) shift_NW=$value ;; esac
        shift
      done ;;
    --drive-only) drive_only=1; shift ;;
    --quiet) quiet=1; shift ;;
    --dry-run) dry_run=1; shift ;;
    --help) echo 'Usage: build.sh [--wheels NE SE SW NW] [--shift NE=-60 ...] [--drive-only] [--quiet] [--dry-run]'; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done
require_toolchain
cd "$repo"
out=$repo/build
cmake_firmware() {
  build_dir=$1; shift
  run cmake -S "$repo/src/nucleo/CM7" -B "$build_dir" -G Ninja \
    "-DCMAKE_TOOLCHAIN_FILE=$repo/src/nucleo/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug "$@"
  run cmake --build "$build_dir"
}
if [ "$drive_only" = 0 ]; then
  run cmake -S "$repo/src/tools/fwu_image" -B "$out/tools/fwu_image" -G Ninja -DCMAKE_BUILD_TYPE=Release
  run cmake --build "$out/tools/fwu_image"
  for wheel in $wheels; do
    esc=$out/esc/$wheel
    case "$wheel" in NE) value=$shift_NE ;; SE) value=$shift_SE ;; SW) value=$shift_SW ;; NW) value=$shift_NW ;; esac
    set -- sh "$repo/scripts/esc_build.sh" --motor "$wheel" --app-offset --output "$esc"
    if [ -n "$value" ]; then set -- "$@" --hall-shift "$value"; fi
    if [ "$dry_run" = 1 ]; then
      # Execute the child's dry run so its actual compiler flags remain visible.
      "$@" --dry-run
    else
      run "$@"
    fi
    run "$out/tools/fwu_image/fwu_image" --bin "$esc/motor_${wheel}_drive_offset.bin" --output "$esc/fwu_payload.c"
    cmake_firmware "$out/updater/$wheel" "-DFWU_IMAGE_FILE=$esc/fwu_payload.c"
  done
fi
cmake_firmware "$out/drive" -UFWU_IMAGE_FILE -DFWU_IMAGE_FILE=
if [ "$dry_run" = 0 ]; then
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT HUP INT TERM
  hash_artifact() {
    path=$1
    case "$path" in
      *.elf) run arm-none-eabi-objcopy -O binary "$path" "$tmp/image.bin"; data=$tmp/image.bin ;;
      *) data=$path ;;
    esac
    hash=$(sha256sum "$data")
    printf '%.16s %s\n' "$hash" "${path#"$repo/"}"
  }
  if [ "$drive_only" = 0 ]; then
    for wheel in $wheels; do
      hash_artifact "$out/esc/$wheel/motor_${wheel}_drive_offset.bin"
      hash_artifact "$out/updater/$wheel/sfurs_firmware_v2_CM7.elf"
    done
  fi
  hash_artifact "$out/drive/sfurs_firmware_v2_CM7.elf"
fi
