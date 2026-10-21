#!/bin/sh
set -eu
repo=$1
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
contains() {
  awk -v wanted="$2" 'index($0, wanted) { found = 1 } END { exit !found }' "$1"
}
reject() {
  if "$@" > "$tmp/reject.log" 2>&1; then
    echo "Unexpected success: $*" >&2
    exit 1
  fi
}
for wheel in NE SE SW NW; do
  sh "$repo/scripts/esc_build.sh" --motor "$wheel" --output "$tmp/ESC $wheel" \
    --app-offset --hall-shift -60 --dry-run > "$tmp/esc.log"
  contains "$tmp/esc.log" "[-DMOTOR_$wheel]"
  contains "$tmp/esc.log" '[-DESC_BOOT_OFFSET_APP]'
  contains "$tmp/esc.log" '[-DHALL_PHASE_SHIFT_OVERRIDE=-60]'
  contains "$tmp/esc.log" "motor_${wheel}_drive_offset.bin"
  contains "$tmp/esc.log" 'STM32G431CBUX_FLASH_APP.ld'
  expected=$(awk '!/^#/ && NF { n++ } END { print n }' "$repo/scripts/esc_sources.txt")
  actual=$(awk 'index($0, " [-c] ") { n++ } END { print n }' "$tmp/esc.log")
  [ "$actual" = "$expected" ]
  sh "$repo/scripts/boot_build.sh" --motor "$wheel" --output "$tmp/boot $wheel" \
    --diagnostic --dry-run > "$tmp/boot.log"
  contains "$tmp/boot.log" "[-DMOTOR_$wheel]"
  contains "$tmp/boot.log" '[-DBOOT_BLINK_ONLY]'
  contains "$tmp/boot.log" '[-DBOOT_DIAGNOSTIC]'
  contains "$tmp/boot.log" "boot_${wheel}_diagnostic.bin"
done
sh "$repo/scripts/esc_build.sh" --motor=NE --hall-shift=0008 \
  --output "$tmp/decimal" --dry-run > "$tmp/esc.log"
contains "$tmp/esc.log" '[-DHALL_PHASE_SHIFT_OVERRIDE=8]'
for bad in -360 360 3.5 '' nonsense; do
  reject sh "$repo/scripts/esc_build.sh" --motor NE --output "$tmp/reject" --hall-shift "$bad" --dry-run
done
reject sh "$repo/scripts/esc_build.sh" --motor XX --output "$tmp/reject" --dry-run
reject sh "$repo/scripts/boot_build.sh" --motor XX --output "$tmp/reject" --dry-run
reject sh "$repo/scripts/build.sh" --wheels XX --dry-run
sh "$repo/scripts/build.sh" --wheels NE SE SW NW --shift NE=-60 SE=-64 SW=-59 NW=-62 \
  --dry-run > "$tmp/all.log"
for wheel in NE SE SW NW; do
  contains "$tmp/all.log" "build/esc/$wheel/fwu_payload.c"
  contains "$tmp/all.log" "build/updater/$wheel"
done
for degrees in -60 -64 -59 -62; do
  contains "$tmp/all.log" "[-DHALL_PHASE_SHIFT_OVERRIDE=$degrees]"
done
contains "$tmp/all.log" '[-UFWU_IMAGE_FILE] [-DFWU_IMAGE_FILE=]'
sh "$repo/scripts/build.sh" --drive-only --dry-run > "$tmp/drive.log"
contains "$tmp/drive.log" 'build/drive'
if contains "$tmp/drive.log" 'build/esc/'; then
  echo 'Drive-only build unexpectedly built an ESC' >&2
  exit 1
fi
