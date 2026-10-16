#!/usr/bin/env python3
"""Build the ESC firmware with GCC, without installing CubeIDE.

Usage: python3 build_cli.py --motor NE --output /path/to/build [--app-offset]
                            [--hall-shift DEG]
Compiles the CubeIDE linked source list (which includes Src/drive_app.c).
Does not flash hardware.
"""
import argparse
import pathlib
import subprocess
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--motor', choices=['NE', 'SE', 'SW', 'NW'], required=True)
parser.add_argument('--output', type=pathlib.Path, required=True)
parser.add_argument('--app-offset', action='store_true',
                    help='Link the app at 0x08004000 (110K) under the CAN '
                         'bootloader and enable safe remote boot entry; '
                         'the bootloader sets VTOR before jumping.')
parser.add_argument('--hall-shift', type=int, metavar='DEG',
                    help='HALL_PHASE_SHIFT measured for this wheel by CAN '
                         'HALL_ALIGN (default: generated 120)')
args = parser.parse_args()
if args.hall_shift is not None and not -360 < args.hall_shift < 360:
    parser.error('--hall-shift must be in (-360, 360) degrees')
root = pathlib.Path(__file__).resolve().parent
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
includes = ['Inc', 'Drivers/STM32G4xx_HAL_Driver/Inc',
            'Drivers/STM32G4xx_HAL_Driver/Inc/Legacy',
            'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include',
            'Drivers/CMSIS/DSP/Include',
            'MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Inc',
            'MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Inc',
            '../Inc', '../../common/CAN']
compile_flags = flags + ['-Ofast', '-g3', '-std=gnu11', '-ffunction-sections',
                         '-fdata-sections', '-Wall', '-DDEBUG', '-DARM_MATH_CM4',
                         '-DUSE_HAL_DRIVER', '-DSTM32G431xx', '-DMOTOR_' + args.motor]
compile_flags += ['-I' + str(root / folder) for folder in includes]
if args.app_offset:
    compile_flags += ['-DESC_BOOT_OFFSET_APP', '-I' + str(root.parent / 'bootloader')]
if args.hall_shift is not None:
    compile_flags += [f'-DHALL_PHASE_SHIFT_OVERRIDE={args.hall_shift}']
sources = []
for link in ET.parse(root / 'STM32CubeIDE/.project').findall('.//link'):
    uri = link.findtext('locationURI', '')
    if uri.startswith('PARENT-1-PROJECT_LOC/') and uri.endswith('.c'):
        sources.append(root / uri.split('/', 1)[1])
sources += sorted((root / 'STM32CubeIDE/Application').rglob('*.c'))
sources += sorted((root / 'STM32CubeIDE/Application').rglob('*.s'))
objects = []
for source in sources:
    obj = out / (str(source.relative_to(root)).replace('/', '_') + '.o')
    subprocess.run(['arm-none-eabi-gcc', *compile_flags, '-c', str(source),
                    '-o', str(obj)], check=True)
    objects.append(str(obj))
elf = out / f"motor_{args.motor}_drive{'_offset' if args.app_offset else ''}.elf"
ld = root / ('STM32CubeIDE/STM32G431CBUX_FLASH_APP.ld' if args.app_offset
             else 'STM32CubeIDE/STM32G431CBUX_FLASH.ld')
subprocess.run(['arm-none-eabi-gcc', *flags, '-T' + str(ld),
                '--specs=nano.specs', '--specs=nosys.specs',
                '-Wl,--gc-sections', '-Wl,--print-memory-usage',
                '-Wl,-Map=' + str(out / 'motor.map'), *objects,
                '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group',
                '-o', str(elf)], check=True)
subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(elf),
                str(elf.with_suffix('.bin'))], check=True)
print(elf)
