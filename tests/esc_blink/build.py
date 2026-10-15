#!/usr/bin/env python3
"""Build the G431 blink payload and generate its Nucleo C embedding."""
import argparse
import pathlib
import struct
import subprocess
import zlib

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', default='arm-none-eabi-gcc')
parser.add_argument('--objcopy', default='arm-none-eabi-objcopy')
parser.add_argument('--output', type=pathlib.Path, required=True)
args = parser.parse_args()
here = pathlib.Path(__file__).resolve().parent
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
elf = out / 'esc_blink.elf'
binary = out / 'esc_blink.bin'
subprocess.run([args.cc, '-mcpu=cortex-m4', '-mthumb', '-Os', '-std=c11',
                '-ffreestanding', '-nostdlib', '-fno-unwind-tables',
                '-fno-asynchronous-unwind-tables', '-Wall', '-Wextra', '-Werror',
                '-T', str(here / 'app.ld'), str(here / 'main.c'),
                '-o', str(elf)], check=True)
subprocess.run([args.objcopy, '-O', 'binary', str(elf), str(binary)], check=True)
data = binary.read_bytes()
sp, pc = struct.unpack_from('<II', data)
assert sp == 0x20008000 and pc & 1 and 0x08004000 <= (pc & ~1) < 0x08004000 + len(data)
assert 8 <= len(data) <= 110 * 1024
lines = ['/* Generated motor-disabled ESC blink payload. */',
         '#include "fwu_test_image.h"',
         f'const uint32_t fwu_test_image_size = {len(data)}U;',
         f'const uint32_t fwu_test_image_crc = 0x{zlib.crc32(data):08X}U;',
         'const uint8_t fwu_test_image[] = {']
for i in range(0, len(data), 12):
    lines.append('  ' + ', '.join(f'0x{x:02X}' for x in data[i:i+12]) + ',')
lines.append('};')
(out / 'fwu_blink_image.c').write_text('\n'.join(lines) + '\n')
print(f'Blink payload: {len(data)} bytes, CRC32={zlib.crc32(data):08X}')
