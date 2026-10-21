#!/bin/sh
set -eu
repo=$1
harness=$2
mode=$3
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1 &&
    [ -d /tmp/opencode/cm7-toolchain/root/usr/bin ]; then
  PATH=/tmp/opencode/cm7-toolchain/root/usr/bin:$PATH
  export PATH
fi
for tool in arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-objdump; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "SKIP: $tool unavailable"
    exit 77
  fi
done
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
sh "$repo/tests/esc_blink/build.sh" --output "$tmp"
arm-none-eabi-objdump -h "$tmp/esc_blink.elf" > "$tmp/sections"
awk '$2 == ".isr_vector" && $4 == "08004000" { found = 1 } END { exit !found }' "$tmp/sections"
if [ "$mode" = test_elf_round_trip ]; then
  "$harness" "$mode" "$tmp/esc_blink.elf" "$tmp/esc_blink.bin"
else
  "$harness" "$mode" "$tmp/esc_blink.bin" "$tmp/fwu_blink_image.c"
fi
