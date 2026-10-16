#!/usr/bin/env python3
"""Host-test the motor app image generator (scripts/fwu_image.py).

No hardware, no ARM toolchain required (the --elf path self-skips without
objcopy). Covers: oversized image rejected, bad entry SP rejected, CRC
round-trip against Common/Src/can_fwu_protocol.c, generated C parses back
to identical bytes/size/CRC, and vector agreement with can_fwu_vectors_valid.
"""
import pathlib
import re
import shutil
import struct
import subprocess
import tempfile
import zlib

root = pathlib.Path(__file__).resolve().parents[1]
gen = root / 'scripts' / 'fwu_image.py'

APP_MAX = 110 * 1024
APP_BASE = 0x08004000
SRAM_TOP = 0x20007FF0


def make_image(size=256, sp=SRAM_TOP, reset=APP_BASE + 9):
    assert size >= 8
    body = bytes((i * 13 + 7) & 0xFF for i in range(size - 8))
    return struct.pack('<II', sp, reset) + body


def run_gen(args):
    return subprocess.run(['python3', str(gen)] + args,
                          capture_output=True, text=True)


def parse_c(text):
    """Extract (size, crc, bytes) from generated C; must mirror the header."""
    header = (root / 'CM7' / 'Core' / 'Inc' / 'fwu_test_image.h').read_text()
    for sym in ('fwu_test_image_size', 'fwu_test_image_crc', 'fwu_test_image'):
        assert sym in header, f'header missing {sym}'
    assert '#include "fwu_test_image.h"' in text
    size = int(re.search(r'fwu_test_image_size = (\d+)U', text).group(1))
    crc = int(re.search(r'fwu_test_image_crc = 0x([0-9A-F]+)U', text).group(1), 16)
    body = text.split('const uint8_t fwu_test_image[] = {', 1)[1]
    encoded = bytes(int(x.strip(), 16) for x in body.split('};', 1)[0].split(',')
                    if x.strip())
    return size, crc, encoded


def vectors_valid(sp, reset, size):
    """Python mirror of can_fwu_vectors_valid (validity header)."""
    pc = reset & ~1
    return (size >= 8 and size <= APP_MAX
            and 0x20000000 < sp <= SRAM_TOP and sp % 8 == 0
            and reset & 1 and APP_BASE <= pc < APP_BASE + size)


with tempfile.TemporaryDirectory(prefix='fwu-motor-image-') as tmp:
    tmp = pathlib.Path(tmp)

    # 1. Oversized image rejected; exact slot size accepted, undersize refused.
    big = tmp / 'big.bin'
    big.write_bytes(make_image(APP_MAX + 1))  # valid vectors, one byte over slot
    r = run_gen(['--bin', str(big), '--output', str(tmp / 'big.c')])
    assert r.returncode != 0 and 'outside' in r.stderr, r.stderr
    full = tmp / 'full.bin'
    full.write_bytes(make_image(APP_MAX))
    r = run_gen(['--bin', str(full), '--output', str(tmp / 'full.c')])
    assert r.returncode == 0, r.stderr
    tiny = tmp / 'tiny.bin'
    tiny.write_bytes(b'\x00' * 7)
    r = run_gen(['--bin', str(tiny), '--output', str(tmp / 'tiny.c')])
    assert r.returncode != 0, r.stderr

    # 2. Bad entry SP rejected (bounds, mailbox top, alignment, flash alias).
    for bad_sp in (0x20000000, 0x20008000, SRAM_TOP - 1,
                   SRAM_TOP + 8, 0x08004000, 0x00000000):
        bad = tmp / 'badsp.bin'
        bad.write_bytes(make_image(64, sp=bad_sp))
        r = run_gen(['--bin', str(bad), '--output', str(tmp / 'badsp.c')])
        assert r.returncode != 0, f'SP 0x{bad_sp:08X} accepted'
    for good_sp in (SRAM_TOP, 0x20004000):  # mailbox top and mid-SRAM
        good = tmp / 'goodsp.bin'
        good.write_bytes(make_image(64, sp=good_sp))
        r = run_gen(['--bin', str(good),
                     '--output', str(tmp / 'goodsp.c')])
        assert r.returncode == 0, f'SP 0x{good_sp:08X} rejected: {r.stderr}'

    # Bad reset vectors rejected: even (ARM), below base, past image end.
    for bad_reset in (APP_BASE + 8, APP_BASE - 1 | 1, APP_BASE + 64 + 1, 0x08000001):
        bad = tmp / 'badpc.bin'
        bad.write_bytes(make_image(64, reset=bad_reset))
        r = run_gen(['--bin', str(bad), '--output', str(tmp / 'badpc.c')])
        assert r.returncode != 0, f'reset 0x{bad_reset:08X} accepted'

    # 3. CRC round-trip against the real can_fwu_crc32 (compiled from Common).
    blobs = {'vector': b'123456789',
             'motorlike': make_image(1024),
             'allff': bytes([0xFF]) * 100}
    harness = r'''
#include "can_fwu_protocol.h"
#include "can_fwu_validity.h"
#include <stdio.h>
#include <stdint.h>
int main(int argc, char **argv) {
  if (argc == 4) { /* vectors check: sp reset size */
    uint32_t sp, reset, size;
    sscanf(argv[1], "%x", &sp);
    sscanf(argv[2], "%x", &reset);
    sscanf(argv[3], "%x", &size);
    printf("%d\n", can_fwu_vectors_valid(sp, reset, size));
    return 0;
  }
  uint8_t buf[1 << 20];
  size_t n = fread(buf, 1, sizeof(buf), stdin);
  printf("%08X\n", can_fwu_crc32(buf, n));
  return 0;
}
'''
    (tmp / 'crc.c').write_text(harness)
    exe = tmp / 'crc'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'Common/Inc'),
                    str(tmp / 'crc.c'),
                    str(root / 'Common/Src/can_fwu_protocol.c'),
                    '-o', str(exe)], check=True)
    out = subprocess.run([str(exe)], input=blobs['vector'],
                         capture_output=True, check=True).stdout.decode()
    assert out.strip() == 'CBF43926', out  # shared IEEE test vector
    for name, blob in blobs.items():
        out = subprocess.run([str(exe)], input=blob,
                             capture_output=True, check=True).stdout.decode()
        assert out.strip() == f'{zlib.crc32(blob) & 0xFFFFFFFF:08X}', name

    # 4. Generated C parses back to identical bytes/size/CRC.
    src = tmp / 'motor.bin'
    src.write_bytes(make_image(4096))
    r = run_gen(['--bin', str(src), '--output', str(tmp / 'fwu_test_image.c')])
    assert r.returncode == 0, r.stderr
    size, crc, encoded = parse_c((tmp / 'fwu_test_image.c').read_text())
    assert size == len(src.read_bytes()) == 4096
    assert crc == zlib.crc32(src.read_bytes()) & 0xFFFFFFFF
    assert encoded == src.read_bytes()

    # 5. Python validation agrees with can_fwu_vectors_valid on every case.
    cases = [(SRAM_TOP, APP_BASE + 9, 64), (SRAM_TOP, APP_BASE + 9, 8),
             (SRAM_TOP, APP_BASE + 9, APP_MAX), (0x20000000, APP_BASE + 9, 64),
             (0x20008000, APP_BASE + 9, 64), (SRAM_TOP - 1, APP_BASE + 9, 64),
             (SRAM_TOP, APP_BASE + 8, 64), (SRAM_TOP, 0x08000001, 64),
             (SRAM_TOP, APP_BASE + 65, 64), (SRAM_TOP, APP_BASE + 9, 7),
             (SRAM_TOP, APP_BASE + 9, APP_MAX + 1)]
    for sp, reset, size in cases:
        out = subprocess.run([str(exe), f'{sp:X}', f'{reset:X}', f'{size:X}'],
                             capture_output=True, check=True).stdout.decode()
        assert (out.strip() == '1') == vectors_valid(sp, reset, size), \
            f'divergence at sp=0x{sp:X} reset=0x{reset:X} size={size}'

    # 6. --elf path: reuse the blink offset build when the ARM tools exist.
    if shutil.which('arm-none-eabi-gcc') and shutil.which('arm-none-eabi-objcopy'):
        out = tmp / 'elfbuild'
        subprocess.run(['python3', str(root / 'tests/esc_blink/build.py'),
                        '--output', str(out)], check=True)
        r = run_gen(['--elf', str(out / 'esc_blink.elf'),
                     '--output', str(tmp / 'from_elf.c')])
        assert r.returncode == 0, r.stderr
        size, crc, encoded = parse_c((tmp / 'from_elf.c').read_text())
        assert encoded == (out / 'esc_blink.bin').read_bytes()
        assert size == len(encoded) and crc == zlib.crc32(encoded) & 0xFFFFFFFF
    else:
        print('SKIP: --elf round-trip (no arm-none-eabi toolchain)')

print('PASS: motor image generator rejects bad slot/SP/vectors, '
      'CRC matches can_fwu_crc32, C parses back byte-identical')
