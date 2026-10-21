#!/bin/sh
# The linker script enforces the bootloader's 16 KiB flash region.
. "$(dirname -- "$0")/build_common.sh"
motor= output=
blink=0 diagnostic=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --*=*) option=$1; shift; set -- "${option%%=*}" "${option#*=}" "$@" ;;
  esac
  case "$1" in
    --motor|--output)
      [ "$#" -ge 2 ] || fail "Missing value: $1"
      case "$1" in --motor) motor=$2 ;; --output) output=$2 ;; esac
      shift 2 ;;
    --blink-only) blink=1; shift ;;
    --diagnostic) diagnostic=1; shift ;;
    --dry-run) dry_run=1; shift ;;
    --help) echo 'Usage: boot_build.sh --motor NE|SE|SW|NW --output DIR [--blink-only] [--diagnostic] [--dry-run]'; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done
wheel_valid "$motor" || fail 'Required: --motor NE|SE|SW|NW'
[ -n "$output" ] || fail 'Required: --output DIR'
require_toolchain
mkdir -p "$output"
output=$(CDPATH= cd -- "$output" && pwd)
here=$repo/src/esc/bootloader
mc=$repo/src/esc/motor_config
shared=$repo/src/shared
compile() {
  set -- -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -Os -g3 -std=gnu11 -ffunction-sections -fdata-sections -Wall -Wextra \
    -DUSE_HAL_DRIVER -DSTM32G431xx "-DMOTOR_$motor" "-I$here" "-I$shared/inc" "-I$mc/Inc" \
    "-I$mc/Drivers/STM32G4xx_HAL_Driver/Inc" "-I$mc/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy" \
    "-I$mc/Drivers/CMSIS/Device/ST/STM32G4xx/Include" "-I$mc/Drivers/CMSIS/Include"
  if [ "$blink" = 1 ]; then set -- "$@" -DBOOT_BLINK_ONLY; fi
  if [ "$diagnostic" = 1 ]; then set -- "$@" -DBOOT_BLINK_ONLY -DBOOT_DIAGNOSTIC; fi
  run arm-none-eabi-gcc "$@" -c "$source" -o "$object"
}
set --
for source in "$here/boot_main.c" "$here/boot_it.c" \
    "$shared/src/can_fwu_protocol.c" "$shared/src/can_fwu_boot.c" \
    "$mc/Src/system_stm32g4xx.c" "$mc/STM32CubeIDE/Application/Startup/startup_stm32g431cbux.s"; do
  object=$output/$(basename "${source%.*}").o
  compile
  set -- "$@" "$object"
done
for module in stm32g4xx_hal stm32g4xx_hal_cortex stm32g4xx_hal_rcc stm32g4xx_hal_rcc_ex \
    stm32g4xx_hal_gpio stm32g4xx_hal_fdcan stm32g4xx_hal_flash stm32g4xx_hal_flash_ex \
    stm32g4xx_hal_pwr stm32g4xx_hal_pwr_ex; do
  source=$mc/Drivers/STM32G4xx_HAL_Driver/Src/$module.c
  object=$output/$module.o
  compile
  set -- "$@" "$object"
done
tag=boot_$motor
if [ "$blink" = 1 ]; then tag=${tag}_blink; fi
if [ "$diagnostic" = 1 ]; then tag=${tag}_diagnostic; fi
elf=$output/$tag.elf
run arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  "-T$here/boot.ld" --specs=nano.specs --specs=nosys.specs \
  -Wl,--gc-sections -Wl,--print-memory-usage "-Wl,-Map=$output/boot.map" "$@" \
  -Wl,--start-group -lc -lm -Wl,--end-group -o "$elf"
run arm-none-eabi-objcopy -O binary "$elf" "$output/$tag.bin"
if [ "$dry_run" = 0 ]; then
  size=$(wc -c < "$output/$tag.bin")
  [ "$size" -le 16384 ] || fail "bootloader overflow: $size B > 16K"
  echo "bootloader fits: $size / 16384 B"
fi
echo "$elf"
