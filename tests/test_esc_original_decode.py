#!/usr/bin/env python3
"""Offline tests for original-image memory decoding; no hardware access."""
from esc_original_decode import decode_foc, parse_words, ram_pointer, scale_float

foc, speed = 0x20000944, 0x20001000
mem = {foc + i: 0 for i in range(0, 32, 4)}
mem.update({speed + i: 0 for i in range(0, 16, 4)})
mem.update({foc: 0xffff0002, foc + 8: 0xfffe0003,
            foc + 12: 0xfffc0005, foc + 16: 0x00060000,
            foc + 20: 0x0008fff9, foc + 24: 9,
            speed + 12: 0xfffb, 0x48000410: 5 << 6})
values = decode_foc(mem, foc, speed, 1/544)
assert values['ia_sdk'] == 2 and values['ib_sdk'] == -1
assert values['iq_sdk'] == 3 and values['id_sdk'] == -2
assert values['iq_ref_sdk'] == 5 and values['id_ref_sdk'] == -4
assert values['vq_sdk'] == 6 and values['vd_sdk'] == -7
assert values['v_alpha_sdk'] == 8 and values['v_beta_sdk'] == 9
assert values['measured_motor_rpm'] == -30 and values['hall_pb6_pb8_raw'] == 5
assert values['iq_ref_a_firmware_scaled'] == 5/544
assert scale_float(0x3f800000) == 1.0
assert parse_words('\x1b[0m0x20000000 : FFFF0001 00000002\n') == {
    0x20000000: 0xffff0001, 0x20000004: 2}
for invalid in (0, 0x08000000, 0x20000001, 0x20008000):
    try:
        ram_pointer(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid pointer accepted')
for invalid in (float('nan'), float('inf'), 0, -1, 1):
    try:
        decode_foc(mem, foc, speed, invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid current scale accepted')
print('PASS: original FOC/speed signed offsets, raw Hall pins, scale and pointer validation')
