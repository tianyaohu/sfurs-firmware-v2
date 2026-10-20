#!/usr/bin/env python3
"""Build and validate the actual blink artifact. Requires the ARM toolchain on PATH."""
import pathlib
import struct
import subprocess
import tempfile
import zlib

root = pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='fwu-artifact-') as tmp:
    out = pathlib.Path(tmp)
    subprocess.run(['python3', str(root / 'tests/esc_blink/build.py'),
                    '--output', str(out)], check=True)
    data = (out / 'esc_blink.bin').read_bytes()
    sp, reset = struct.unpack_from('<II', data)
    assert 8 <= len(data) <= 110 * 1024
    assert sp > 0x20000000 and sp <= 0x20008000 and sp % 8 == 0
    assert reset & 1 and 0x08004000 <= (reset & ~1) < 0x08004000 + len(data)
    generated = (out / 'fwu_blink_image.c').read_text()
    assert f'fwu_test_image_size = {len(data)}U' in generated
    assert f'fwu_test_image_crc = 0x{zlib.crc32(data):08X}U' in generated
    # Reassemble the C byte array: embedding must not omit, duplicate, or
    # reorder bytes relative to the independently built binary.
    body = generated.split('const uint8_t fwu_test_image[] = {', 1)[1]
    encoded = bytes(int(x.strip(), 16) for x in body.split('};', 1)[0].split(',')
                    if x.strip())
    assert encoded == data
    sections = subprocess.check_output(['arm-none-eabi-objdump', '-h',
                                         str(out / 'esc_blink.elf')], text=True)
    vector_row = next(line for line in sections.splitlines() if '.isr_vector' in line)
    assert vector_row.split()[3] == '08004000'
print('PASS: real blink ELF vectors, image size/CRC, and generated embedding')
