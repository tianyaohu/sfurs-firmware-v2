#!/usr/bin/env python3
"""Actual C CAN decoder -> RAM ABI -> actual TUI decoding, without hardware."""
import ctypes as ct
import importlib.util
import pathlib
import struct
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('keyboard_drive', ROOT / 'scripts/keyboard_drive.py')
kd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kd)


class DecoderToTUITests(unittest.TestCase):
    def setUp(self):
        self.words = (ct.c_uint32 * 52).in_dll(self.lib, 'ik_drive_telem')
        self.tick = ct.c_uint32.in_dll(self.lib, 'ik_drive_telem_now')
        ct.memset(ct.addressof(self.words), 0, ct.sizeof(self.words))
        self.tick.value = 1000
        outer = self

        class Target:
            def read_memory_block32(self, address, size):
                offset = (address - 0x24000000) // 4
                return list(outer.words[offset:offset + size])

            def read32(self, address):
                if address == 0x24001000:
                    return outer.tick.value
                return outer.words[(address - 0x24000000) // 4]

            def write32(self, *args):
                raise AssertionError('telemetry reads must not write')

        self.drive = kd.Drive(Target(), {'telem': 0x24000000, 'telem_now': 0x24001000})

    def receive(self, identifier, data, tick):
        payload = (ct.c_uint8 * 8).from_buffer_copy(data)
        return self.lib.ik_telemetry_rx(identifier, payload, tick)

    def test_all_wheels_signed_fields_current_scaling_and_identity(self):
        for i, selector in enumerate((1, 2, 4, 8)):
            self.assertEqual(self.receive(0x280 | selector,
                             struct.pack('<hBBI', -120 + i, 6, 5, 123456 + i), 980), 1)
            self.assertEqual(self.receive(0x380 | selector,
                             struct.pack('<hhHH', 544, -272, 0xB800, 1000 + i), 990), 1)
        rows = self.drive.poll_telem(10)['rows']
        self.assertEqual([r['selector'] for r in rows], [1, 2, 4, 8])
        self.assertEqual([r['wheel'] for r in rows], ['FR', 'BR', 'BL', 'FL'])
        for i, row in enumerate(rows):
            self.assertEqual(row['rpm'], -120 + i)
            self.assertEqual(row['hall_changes'], 123456 + i)
            self.assertEqual(row['iq_counts'], 544)
            self.assertEqual(row['id_counts'], -272)
            self.assertEqual((row['speed_age_ms'], row['current_age_ms']), (20, 10))
            self.assertTrue(row['current_valid'])
            self.assertEqual((row['iq_a'], row['id_a']), (1.0, -0.5))

    def test_current_fault_invalidates_amps_but_preserves_raw(self):
        self.receive(0x281, struct.pack('<hBBI', 0, 6, 5, 10), 1000)
        self.receive(0x381, struct.pack('<hhHH', 1632, 0, 0xB804, 1632), 1000)
        row = self.drive.poll_telem(10)['rows'][0]
        self.assertEqual(row['flags']['faults'], 4)
        self.assertIsNone(row['iq_a'])
        self.assertEqual(row['iq_counts'], 1632)

    def test_wrong_wheel_selector_is_not_a_reply(self):
        self.assertEqual(self.receive(0x283, bytes(8), 1000), 0)
        for row in self.drive.poll_telem(10)['rows']:
            self.assertIsNone(row['rpm'])
            self.assertIsNone(row['current_age_ms'])
            self.assertIsNone(row['iq_a'])

    def test_independent_group_freshness_after_hal_wrap(self):
        self.tick.value = 20
        self.receive(0x281, struct.pack('<hBBI', 30, 6, 5, 1), 0xFFFFFFF0)
        self.receive(0x381, struct.pack('<hhHH', 544, 0, 0xB800, 544), 0xFFFFFC00)
        row = self.drive.poll_telem(10)['rows'][0]
        self.assertEqual(row['speed_age_ms'], 36)
        self.assertEqual(row['current_age_ms'], 1044)
        self.assertTrue(row['speed_fresh'])
        self.assertFalse(row['current_fresh'])
        self.assertIsNone(row['iq_a'])


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='ik-telem-tui-', dir='/tmp/opencode') as tmp:
        library = pathlib.Path(tmp) / 'telemetry.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        '-I', str(ROOT / 'CM7/Core/Inc'),
                        str(ROOT / 'CM7/Core/Src/ik_telemetry.c'), '-o', str(library)], check=True)
        DecoderToTUITests.lib = ct.CDLL(str(library))
        DecoderToTUITests.lib.ik_telemetry_rx.argtypes = [ct.c_uint32,
                                                       ct.POINTER(ct.c_uint8), ct.c_uint32]
        DecoderToTUITests.lib.ik_telemetry_rx.restype = ct.c_int
        unittest.main()
