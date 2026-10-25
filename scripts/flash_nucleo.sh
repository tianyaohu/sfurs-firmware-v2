#!/bin/sh
# Back up the Nucleo's CM7 flash bank, then program a drive/updater ELF with OpenOCD and verify it.
# HARDWARE WRITE: read system-info/hardware.md first. Touches bank 1 (0x08000000) only, never CM4 or
# option bytes. Works wherever OpenOCD runs (lab PC or the robot Pi, which has no STM32CubeProgrammer).
. "$(dirname -- "$0")/build_common.sh"
elf=$repo/build/drive/sfurs_firmware_v2_CM7.elf
backups=$HOME/nucleo-backups
serial=002200364142500620353451
while [ "$#" -gt 0 ]; do
  case "$1" in
    --elf) [ "$#" -ge 2 ] || fail '--elf needs a path'; elf=$2; shift 2 ;;
    --backups) [ "$#" -ge 2 ] || fail '--backups needs a directory'; backups=$2; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    --help) echo 'Usage: flash_nucleo.sh [--elf F] [--backups DIR] [--dry-run]'; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done
[ "$dry_run" = 1 ] || [ -f "$elf" ] || fail "ELF not found: $elf"
command -v openocd >/dev/null 2>&1 || [ "$dry_run" = 1 ] || fail 'openocd not found (apt install openocd)'
# Same probe setup as src/tools/lib/ocd.c: pick the Nucleo by its full serial, no reset lines.
ocd() {
  run openocd -f interface/stlink.cfg -c "adapter serial $serial" -c 'gdb_port disabled' \
    -c 'telnet_port disabled' -c 'tcl_port disabled' -f target/stm32h7x.cfg -c 'reset_config none' "$@"
}
backup=$backups/nucleo-cm7-$(date -u +%Y%m%dT%H%M%SZ).bin
run mkdir -p "$backups"
ocd -c init -c "dump_image $backup 0x08000000 0x100000" -c shutdown
if [ "$dry_run" = 0 ]; then
  [ "$(wc -c < "$backup")" -eq 1048576 ] || fail "backup incomplete: $backup"
  echo "backup $backup"
fi
ocd -c "program $elf verify reset exit"
