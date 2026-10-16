#!/usr/bin/env python3
"""Build the CubeIDE linked source list with GCC, without installing CubeIDE.

Usage: python3 build_cli.py --motor NE --output /path/to/build
Does not flash hardware or modify motor parameters.
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
profiles = parser.add_mutually_exclusive_group()
profiles.add_argument('--low-current-test', action='store_true',
                     help='Build disarmed NE-only 0.25 A / 60 RPM one-shot test profile')
profiles.add_argument('--half-amp-diagnostic', action='store_true',
                      help='Build disarmed NE-only 1.5 A reference / 2000 ms torque diagnostic (legacy option name)')
profiles.add_argument('--rpm-diagnostic', action='store_true',
                      help='Build disarmed NE-only 60 RPM / experimental speed PI / 3 A / 10000 ms validation')
profiles.add_argument('--torque-count-diagnostic', action='store_true',
                      help='Build disarmed NE-only 1.5 A torque / 14400-edge (300-turn) count stop / 180 s backup')
profiles.add_argument('--reverse-speed-diagnostic', action='store_true',
                       help='Build disarmed RPM2 profile: +/-900 RPM bounds / '
                            '-100 RPM default / negated PI / 3 A / 30 s backup')
parser.add_argument('--ne-low-speed-p2', action='store_true',
                    help='FR-only RPM2 comparison: double proportional response; '
                         'requires --motor NE --reverse-speed-diagnostic --app-offset')
parser.add_argument('--rpm2-tune', action='store_true',
                    help='RPM2 + RAM-only CAN GAIN_SET (0x500|dev) for auto-tune '
                         'sweeps; requires --motor NE --reverse-speed-diagnostic '
                          '--app-offset')
parser.add_argument('--rpm2-ki4', action='store_true',
                    help='Opt-in RPM2 candidate: Ki -288/16384, original Kp and '
                         'guards; requires --reverse-speed-diagnostic --app-offset')
args = parser.parse_args()
if args.rpm2_ki4 and (not args.reverse_speed_diagnostic or not args.app_offset):
    parser.error('--rpm2-ki4 requires --reverse-speed-diagnostic --app-offset')
if args.rpm2_ki4 and (args.ne_low_speed_p2 or args.rpm2_tune):
    parser.error('--rpm2-ki4 cannot be combined with another gain experiment')
if args.ne_low_speed_p2 and (args.motor != 'NE' or not args.reverse_speed_diagnostic or not args.app_offset):
    parser.error('--ne-low-speed-p2 requires --motor NE --reverse-speed-diagnostic --app-offset')
if args.rpm2_tune and (args.motor != 'NE' or not args.reverse_speed_diagnostic or not args.app_offset):
    parser.error('--rpm2-tune requires --motor NE --reverse-speed-diagnostic --app-offset')
if args.rpm2_tune and args.ne_low_speed_p2:
    parser.error('--rpm2-tune and --ne-low-speed-p2 are mutually exclusive')
if (args.low_current_test or args.half_amp_diagnostic or args.rpm_diagnostic or args.torque_count_diagnostic or args.reverse_speed_diagnostic) and args.motor != 'NE':
    print(f'WARNING: diagnostic profile for {args.motor} (validated on NE only); '
          f'sign/gains unproven for this wheel', flush=True)
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
if args.low_current_test or args.half_amp_diagnostic or args.rpm_diagnostic or args.torque_count_diagnostic or args.reverse_speed_diagnostic:
    compile_flags += ['-DESC_LOW_CURRENT_TEST']
if args.half_amp_diagnostic:
    compile_flags += ['-DESC_HALF_AMP_DIAGNOSTIC']
if args.rpm_diagnostic:
    compile_flags += ['-DESC_RPM_DIAGNOSTIC']
if args.reverse_speed_diagnostic:
    compile_flags += ['-DESC_REVERSE_SPEED_DIAGNOSTIC']
if args.ne_low_speed_p2:
    compile_flags += ['-DESC_NE_LOW_SPEED_P2']
if args.rpm2_tune:
    compile_flags += ['-DESC_TUNE_GAIN_SET']
if args.rpm2_ki4:
    compile_flags += ['-DESC_RPM2_KI4']
if args.torque_count_diagnostic:
    compile_flags += ['-DESC_TORQUE_COUNT_DIAGNOSTIC']
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
elf = out / ('motor_' + args.motor +
              ('_torque_count_diagnostic.elf' if args.torque_count_diagnostic else
               '_reverse_speed_diagnostic.elf' if args.reverse_speed_diagnostic else
               '_rpm_diagnostic.elf' if args.rpm_diagnostic else
               '_half_amp_diagnostic.elf' if args.half_amp_diagnostic else
               '_low_current_test.elf' if args.low_current_test else '_telemetry.elf'))
if args.app_offset:
    elf = out / (elf.stem + '_offset.elf')
if args.ne_low_speed_p2:
    elf = out / (elf.stem + '_ne_low_speed_p2.elf')
if args.rpm2_tune:
    elf = out / (elf.stem + '_rpm2_tune.elf')
if args.rpm2_ki4:
    elf = out / (elf.stem + '_rpm2_ki4.elf')
ld = root / ('STM32CubeIDE/STM32G431CBUX_FLASH_APP.ld' if args.app_offset
             else 'STM32CubeIDE/STM32G431CBUX_FLASH.ld')
subprocess.run(['arm-none-eabi-gcc', *flags, '-T' + str(ld),
                '--specs=nano.specs', '--specs=nosys.specs',
                '-Wl,--gc-sections', '-Wl,--print-memory-usage',
                '-Wl,-Map=' + str(out / 'motor.map'), *objects,
                '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group',
                '-o', str(elf)], check=True)
print(elf)
