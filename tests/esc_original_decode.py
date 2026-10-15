"""Decode offsets verified in original ESC disassembly, not a newer SDK struct."""
import math
import re
import struct


def parse_words(text):
    text = re.sub(r'\x1b\[[0-9;]*m', '', text)
    words = {}
    for match in re.finditer(r'^0x([0-9A-Fa-f]+) : ([0-9A-Fa-f ]+)$', text, re.M):
        address = int(match[1], 16)
        for i, value in enumerate(match[2].split()):
            words[address + 4*i] = int(value, 16)
    return words


def signed_half(words, address):
    value = (words[address & ~3] >> (8 * (address & 3))) & 0xffff
    return value - 65536 if value & 0x8000 else value


def ram_pointer(value, size=4):
    if value & 3 or not 0x20000000 <= value <= 0x20008000 - size:
        raise ValueError(f'Invalid original ESC SRAM pointer: {value:#x}')
    return value


def decode_foc(words, foc, speed, current_scale):
    """Getters at 0x08002188..0x0800221C establish FOC offsets 0,8,12,18,22.

    MCI_GetAvrgMecSpeed getter: MCI+28 -> STC; STC+20 -> SPD; SPD+12 signed.
    Report SDK raw values; scaling is firmware-reported, not physical validation.
    """
    result = {
        'ia_sdk': signed_half(words, foc), 'ib_sdk': signed_half(words, foc + 2),
        'iq_sdk': signed_half(words, foc + 8), 'id_sdk': signed_half(words, foc + 10),
        'iq_ref_sdk': signed_half(words, foc + 12),
        'id_ref_sdk': signed_half(words, foc + 14),
        'vq_sdk': signed_half(words, foc + 18), 'vd_sdk': signed_half(words, foc + 20),
        'v_alpha_sdk': signed_half(words, foc + 22),
        'v_beta_sdk': signed_half(words, foc + 24),
        'measured_speed_sdk': signed_half(words, speed + 12),
        'hall_pb6_pb8_raw': (words[0x48000410] >> 6) & 7,
    }
    result['measured_motor_rpm'] = result['measured_speed_sdk'] * 6
    if not math.isfinite(current_scale) or not 0 < current_scale < 1:
        raise ValueError('Unrecognized original firmware current scale')
    for field in ('ia', 'ib', 'iq', 'id', 'iq_ref', 'id_ref'):
        result[field + '_a_firmware_scaled'] = result[field + '_sdk'] * current_scale
    return result


def scale_float(word):
    return struct.unpack('<f', struct.pack('<I', word))[0]
