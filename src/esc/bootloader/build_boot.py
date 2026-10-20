#!/usr/bin/env python3
"""Build the CAN bootloader for B-G431B-ESC1 without CubeIDE.

Usage: python3 build_boot.py --motor NE --output /path/to/build
Compiles boot_main.c + vendored can_fwu stack + G4 HAL subset + stock
startup file, linked at 0x08000000 (16K). Asserts the image fits.
Does not flash hardware.
"""
import argparse
import pathlib
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--motor', choices=['NE', 'SE', 'SW', 'NW'], required=True)
parser.add_argument('--output', type=pathlib.Path, required=True)
parser.add_argument('--blink-only', action='store_true',
                    help='Check build: blink forever, never jump to the app.')
parser.add_argument('--diagnostic', action='store_true',
                    help='Stay in boot; answer ENTER, reject other commands; no flash writes.')
args = parser.parse_args()

here = pathlib.Path(__file__).resolve().parent
mc = here.parent / 'motor_config'
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)

flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
compile_flags = flags + ['-Os', '-g3', '-std=gnu11', '-ffunction-sections',
                         '-fdata-sections', '-Wall', '-Wextra',
                         '-DUSE_HAL_DRIVER', '-DSTM32G431xx',
                         '-DMOTOR_' + args.motor,
                         '-I' + str(here),
                         '-I' + str(mc / 'Inc'),
                         '-I' + str(mc / 'Drivers/STM32G4xx_HAL_Driver/Inc'),
                         '-I' + str(mc / 'Drivers/STM32G4xx_HAL_Driver/Inc/Legacy'),
                         '-I' + str(mc / 'Drivers/CMSIS/Device/ST/STM32G4xx/Include'),
                         '-I' + str(mc / 'Drivers/CMSIS/Include')]
if args.blink_only:
    compile_flags.append('-DBOOT_BLINK_ONLY')
if args.diagnostic:
    compile_flags += ['-DBOOT_BLINK_ONLY', '-DBOOT_DIAGNOSTIC']
hal = mc / 'Drivers/STM32G4xx_HAL_Driver/Src'
sources = [here / 'boot_main.c',
           here / 'boot_it.c',
           here / 'can_fwu_protocol.c',
           here / 'can_fwu_boot.c',
           mc / 'Src/system_stm32g4xx.c',
           mc / 'STM32CubeIDE/Application/Startup/startup_stm32g431cbux.s']
for mod in ['stm32g4xx_hal.c', 'stm32g4xx_hal_cortex.c', 'stm32g4xx_hal_rcc.c',
            'stm32g4xx_hal_rcc_ex.c', 'stm32g4xx_hal_gpio.c',
            'stm32g4xx_hal_fdcan.c', 'stm32g4xx_hal_flash.c',
            'stm32g4xx_hal_flash_ex.c', 'stm32g4xx_hal_pwr.c',
            'stm32g4xx_hal_pwr_ex.c']:
    sources.append(hal / mod)
for source in sources:
    assert source.exists(), f'missing {source}'

objects = []
for source in sources:
    obj = out / (source.stem + '.o')
    subprocess.run(['arm-none-eabi-gcc', *compile_flags, '-c', str(source),
                    '-o', str(obj)], check=True)
    objects.append(str(obj))

tag = f'boot_{args.motor}' + ('_blink' if args.blink_only else '')
if args.diagnostic:
    tag += '_diagnostic'
elf = out / (tag + '.elf')
link = subprocess.run(['arm-none-eabi-gcc', *flags, '-T' + str(here / 'boot.ld'),
                       '--specs=nano.specs', '--specs=nosys.specs',
                       '-Wl,--gc-sections', '-Wl,--print-memory-usage',
                       '-Wl,-Map=' + str(out / 'boot.map'), *objects,
                       '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group',
                       '-o', str(elf)],
                      check=True, capture_output=True, text=True)
print(link.stdout, end='')
print(link.stderr, end='')
flash_use = None
for line in (link.stdout + link.stderr).splitlines():
    if 'FLASH' in line and 'B' in line:
        parts = line.split()
        if len(parts) >= 3 and parts[2] == 'B':
            flash_use = int(parts[1])
if flash_use is not None:
    assert flash_use <= 16 * 1024, f'bootloader overflow: {flash_use} B > 16K'
    print(f'bootloader fits: {flash_use} / 16384 B')
subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(elf),
                str(out / (tag + '.bin'))], check=True)
print(elf)
