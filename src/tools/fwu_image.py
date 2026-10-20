#!/usr/bin/env python3
"""Convert a G431 offset application into the Nucleo embedded FWU payload.

Reads a motor application linked at 0x08004000 either as an offset ELF
(``--elf``, converted with ``arm-none-eabi-objcopy``) or as a raw binary
(``--bin``), validates it against the same contract the bootloader enforces,
and writes ``CM7/Core/Src/fwu_test_image.c`` with matching size/CRC values
per ``CM7/Core/Inc/fwu_test_image.h``.

Validation mirrors ``can_fwu_vectors_valid`` (``Common/Inc/can_fwu_validity.h``)
and the ENTER size gate (``Common/Src/can_fwu_boot.c``):

* 8 bytes <= image size <= CAN_FWU_APP_MAX (110 KiB app slot)
* entry stack pointer in SRAM: 0x20000000 < SP <= ESC_BOOT_MAILBOX_ADDR,
  8-byte aligned
* entry reset vector odd (Thumb) with the target PC inside the app image
* CRC32 is IEEE (zlib), the same function ``can_fwu_crc32`` computes;
  ``can_fwu_crc32("123456789") == 0xCBF43926`` is the shared test vector.

Usage:
  scripts/fwu_image.py --elf motor_app.elf [--output CM7/Core/Src/fwu_test_image.c]
  scripts/fwu_image.py --bin motor_app.bin --output /tmp/fwu_test_image.c
"""
import argparse
import pathlib
import re
import struct
import subprocess
import sys
import tempfile
import zlib


def _repo_root():
    return pathlib.Path(__file__).resolve().parents[1]


def _define_int(path, name, default):
    """Parse a simple ``#define NAME <int>`` (hex or decimal, or N*1024)."""
    try:
        text = pathlib.Path(path).read_text()
    except OSError:
        return default
    m = re.search(r'#define\s+' + name + r'\s+\(?([0-9a-fA-FxUu\s\*]+)\)?', text)
    if not m:
        return default
    expr = m.group(1).replace('U', '').replace('u', '').strip()
    try:
        return eval(expr, {'__builtins__': {}}, {})  # noqa: S307 - header constants only
    except Exception:
        return default


def load_layout(root):
    """Read slot limits from the canonical headers (fall back to docs)."""
    common = root / 'Common' / 'Inc'
    return {
        'app_max': _define_int(common / 'can_fwu_protocol.h',
                               'CAN_FWU_APP_MAX', 110 * 1024),
        'app_min': _define_int(common / 'can_fwu_protocol.h',
                               'CAN_FWU_APP_MIN', 8),
        'app_base': _define_int(common / 'can_fwu_validity.h',
                                'FWU_APP_BASE', 0x08004000),
        'sram_top': _define_int(common / 'boot_entry.h',
                                'ESC_BOOT_MAILBOX_ADDR', 0x20007FF0),
        'sram_base': 0x20000000,
    }


def validate_vectors(data, layout):
    """Return (sp, reset) or raise ValueError; mirrors can_fwu_vectors_valid."""
    size = len(data)
    if not layout['app_min'] <= size <= layout['app_max']:
        raise ValueError(
            f'image size {size} outside [{layout["app_min"]}, '
            f'{layout["app_max"]}] (app slot is {layout["app_max"] // 1024} KiB)')
    sp, reset = struct.unpack_from('<II', data)
    if not (layout['sram_base'] < sp <= layout['sram_top']):
        raise ValueError(f'entry SP 0x{sp:08X} outside SRAM range '
                         f'(0x{layout["sram_base"]:08X}, 0x{layout["sram_top"]:08X}]')
    if sp % 8 != 0:
        raise ValueError(f'entry SP 0x{sp:08X} is not 8-byte aligned')
    pc = reset & ~1
    if not (reset & 1):
        raise ValueError(f'reset vector 0x{reset:08X} lacks the Thumb bit')
    base = layout['app_base']
    if not (base <= pc < base + size):
        raise ValueError(f'reset target 0x{pc:08X} outside app image '
                         f'[0x{base:08X}, 0x{base + size:08X})')
    return sp, reset


def elf_to_bin(elf, objcopy, objdump, layout):
    """Convert an offset ELF to raw bytes; refuse non-app link addresses."""
    with tempfile.TemporaryDirectory(prefix='fwu-image-') as tmp:
        tmp = pathlib.Path(tmp)
        binary = tmp / 'app.bin'
        subprocess.run([objcopy, '-O', 'binary', str(elf), str(binary)],
                       check=True, capture_output=True, text=True)
        try:
            sections = subprocess.run(
                [objdump, '-h', str(elf)], check=True,
                capture_output=True, text=True).stdout
        except (OSError, subprocess.CalledProcessError):
            sections = ''
        if sections:
            vector_row = next((line for line in sections.splitlines()
                               if '.isr_vector' in line), None)
            if vector_row is None:
                raise ValueError('ELF has no .isr_vector section; '
                                 'link the app at 0x08004000 with app.ld-style layout')
            if '08004000' not in vector_row:
                raise ValueError(f'.isr_vector not at 0x08004000: {vector_row.strip()}')
        return binary.read_bytes()


def render_c(data, crc, source_name):
    lines = [f'/* Generated from {source_name} by scripts/fwu_image.py. Do not edit. */',
             '#include "fwu_test_image.h"',
             f'const uint32_t fwu_test_image_size = {len(data)}U;',
             f'const uint32_t fwu_test_image_crc = 0x{crc:08X}U;',
             'const uint8_t fwu_test_image[] = {']
    for i in range(0, len(data), 12):
        lines.append('  ' + ', '.join(f'0x{x:02X}' for x in data[i:i + 12]) + ',')
    lines.append('};')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--elf', type=pathlib.Path,
                       help='offset ELF linked at 0x08004000 (motor app build)')
    group.add_argument('--bin', type=pathlib.Path,
                       help='raw app binary starting at the vector table')
    parser.add_argument('--output', type=pathlib.Path,
                        help='generated C file (default: CM7/Core/Src/fwu_test_image.c)')
    parser.add_argument('--objcopy', default='arm-none-eabi-objcopy')
    parser.add_argument('--objdump', default='arm-none-eabi-objdump')
    args = parser.parse_args(argv)
    root = _repo_root()
    layout = load_layout(root)
    if args.elf is not None:
        data = elf_to_bin(args.elf.resolve(), args.objcopy,
                          args.objdump, layout)
        source_name = args.elf.name
    else:
        data = args.bin.read_bytes()
        source_name = args.bin.name
    validate_vectors(data, layout)
    crc = zlib.crc32(data) & 0xFFFFFFFF
    output = args.output.resolve() if args.output else \
        root / 'CM7' / 'Core' / 'Src' / 'fwu_test_image.c'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_c(data, crc, source_name))
    print(f'FWU image: {len(data)} bytes, CRC32={crc:08X} -> {output}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, subprocess.CalledProcessError, OSError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(1)
