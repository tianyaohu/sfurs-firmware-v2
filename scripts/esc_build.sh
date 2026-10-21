#!/bin/sh
# Build the checked-in CubeIDE source list without CubeIDE or XML tooling.
. "$(dirname -- "$0")/build_common.sh"
motor= output= hall_shift=
offset=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --*=*) option=$1; shift; set -- "${option%%=*}" "${option#*=}" "$@" ;;
  esac
  case "$1" in
    --motor|--output|--hall-shift)
      [ "$#" -ge 2 ] || fail "Missing value: $1"
      case "$1" in
        --motor) motor=$2 ;; --output) output=$2 ;; --hall-shift) shift_valid "$2" || fail '--hall-shift must be in (-360, 360) degrees'; hall_shift=$2 ;;
      esac
      shift 2 ;;
    --app-offset) offset=1; shift ;;
    --dry-run) dry_run=1; shift ;;
    --help) echo 'Usage: esc_build.sh --motor NE|SE|SW|NW --output DIR [--app-offset] [--hall-shift DEG] [--dry-run]'; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done
wheel_valid "$motor" || fail 'Required: --motor NE|SE|SW|NW'
[ -n "$output" ] || fail 'Required: --output DIR'
[ -z "$hall_shift" ] || shift_valid "$hall_shift" || fail '--hall-shift must be in (-360, 360) degrees'
if [ -n "$hall_shift" ]; then
  sign=
  case "$hall_shift" in -*) sign=-; hall_shift=${hall_shift#-} ;; +*) hall_shift=${hall_shift#+} ;; esac
  hall_shift=$(printf '%s\n' "$hall_shift" | sed 's/^0*//')
  hall_shift=$sign${hall_shift:-0}
fi
require_toolchain
mkdir -p "$output"
output=$(CDPATH= cd -- "$output" && pwd)
root=$repo/src/esc/motor_config
compile() {
  set -- -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -Ofast -g3 -std=gnu11 -ffunction-sections -fdata-sections -Wall \
    -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx "-DMOTOR_$motor"
  for inc in Inc Drivers/STM32G4xx_HAL_Driver/Inc Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
      Drivers/CMSIS/Device/ST/STM32G4xx/Include Drivers/CMSIS/Include Drivers/CMSIS/DSP/Include \
      MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Inc \
      MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Inc ../Inc ../../shared/can ../../shared/inc; do
    set -- "$@" "-I$root/$inc"
  done
  if [ "$offset" = 1 ]; then set -- "$@" -DESC_BOOT_OFFSET_APP; fi
  if [ -n "$hall_shift" ]; then set -- "$@" "-DHALL_PHASE_SHIFT_OVERRIDE=$hall_shift"; fi
  run arm-none-eabi-gcc "$@" -c "$root/$source" -o "$object"
}
set --
while IFS= read -r source || [ -n "$source" ]; do
  case "$source" in ''|\#*) continue ;; esac
  object=$output/$(printf '%s' "$source" | tr / _).o
  compile
  set -- "$@" "$object"
done < "$repo/scripts/esc_sources.txt"
suffix= ld=STM32G431CBUX_FLASH.ld
if [ "$offset" = 1 ]; then suffix=_offset; ld=STM32G431CBUX_FLASH_APP.ld; fi
elf=$output/motor_${motor}_drive${suffix}.elf
run arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  "-T$root/STM32CubeIDE/$ld" --specs=nano.specs --specs=nosys.specs \
  -Wl,--gc-sections -Wl,--print-memory-usage "-Wl,-Map=$output/motor.map" "$@" \
  -Wl,--start-group -lc -lm -Wl,--end-group -o "$elf"
run arm-none-eabi-objcopy -O binary "$elf" "${elf%.elf}.bin"
echo "$elf"
