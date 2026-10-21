#!/bin/sh
set -eu

cc=arm-none-eabi-gcc
objcopy=arm-none-eabi-objcopy
host_cc=${CC:-cc}
output=
embed=
while [ "$#" -gt 0 ]; do
  case "$1" in
    --cc) cc=$2 ;;
    --objcopy) objcopy=$2 ;;
    --host-cc) host_cc=$2 ;;
    --output) output=$2 ;;
    --image-embed) embed=$2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
  shift 2
done
[ -n "$output" ] || { echo 'Required: --output DIRECTORY' >&2; exit 1; }
if ! command -v "$cc" >/dev/null 2>&1 || ! command -v "$objcopy" >/dev/null 2>&1; then
  echo 'SKIP: ARM GCC and objcopy are required for the blink payload'
  exit 77
fi
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo=$(CDPATH= cd -- "$here/../.." && pwd)
mkdir -p "$output"
"$cc" -mcpu=cortex-m4 -mthumb -Os -std=c11 -ffreestanding -nostdlib \
  -fno-unwind-tables -fno-asynchronous-unwind-tables -Wall -Wextra -Werror \
  -I "$repo/src/shared/inc" -T "$here/app.ld" "$here/main.c" "$here/can_boot.c" \
  -o "$output/esc_blink.elf"
"$objcopy" -O binary "$output/esc_blink.elf" "$output/esc_blink.bin"
if [ -z "$embed" ]; then
  embed=$output/image_embed
  "$host_cc" -std=c11 -O2 -Wall -Wextra -Werror -I "$here/.." \
    -I "$repo/src/tools/lib" "$here/image_embed.c" "$repo/src/tools/lib/crc32.c" \
    -o "$embed"
fi
"$embed" "$output/esc_blink.bin" "$output/fwu_blink_image.c"
