#!/usr/bin/env python3
"""Real C telemetry decoder -> RAM row words -> real Python decode_row."""
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import ik_link  # noqa: E402
import keyboard_drive  # noqa: E402

HARNESS = r'''
#include "ik_telemetry.h"
#include "can_datatypes.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
  /* poll schedule: 17 slots, 3 status rounds, 1 diag round, 1 ident for the next wheel */
  uint32_t seen[4 * 17]; unsigned n = 0;
  for (uint32_t t = 5; n < 4 * 17; ++t) { uint32_t id = ik_telemetry_poll_due(t); if (id) seen[n++] = id; }
  for (unsigned c = 0; c < 4; ++c) {
    for (unsigned i = 0; i < 16; ++i)
      assert(seen[17 * c + i] == ((i >= 12 ? DIAG_READ : STATUS_READ) | (1U << (i % 4))));
    assert(seen[17 * c + 16] == (IDENT_READ | (1U << c)));
  }
  assert(ik_telemetry_poll_due(1000) && !ik_telemetry_poll_due(1004));
  /* capture frames: by sample index and bank, info bank, bad tags ignored */
  const uint8_t c0[8] = {0x0A, 0x00, 0x20, 0x02, 0xFF, 0xFF, 0x10, 0x27}; /* s10 bank0: 544, -1, 10000 */
  const uint8_t c1[8] = {0x0A, 0x10, 0x9C, 0xFF, 0x60, 0x06, 0x00, 0x00}; /* s10 bank1: -100, 1632, 0 */
  const uint8_t ci[8] = {0x00, 0x30, 0x02, 0x00, 0xD4, 0x04, 0x00, 0x02}; /* done, 12.36 V, 512 */
  const uint8_t cbad[8] = {0x00, 0x22, 1, 2, 3, 4, 5, 6};                 /* bank 2 */
  assert(ik_telemetry_rx(CAPTURE_REPLY | MOTOR_SE_CAN_ID, c0, 1));
  assert(ik_telemetry_rx(CAPTURE_REPLY | MOTOR_SE_CAN_ID, c1, 1));
  assert(ik_telemetry_rx(CAPTURE_REPLY | MOTOR_SE_CAN_ID, ci, 1));
  assert(ik_telemetry_rx(CAPTURE_REPLY | MOTOR_SE_CAN_ID, cbad, 1));
  assert(!ik_telemetry_rx(CAPTURE_REPLY | 3U, c0, 1));
  assert(ik_drive_capture.frames == 3 && ik_drive_capture.source == MOTOR_SE_CAN_ID);
  assert(ik_drive_capture.status == ESC_CAPTURE_DONE && ik_drive_capture.vbus_centivolts == 1236);
  assert(ik_drive_capture.samples == 512);
  const int16_t want[6] = {544, -1, 10000, -100, 1632, 0};
  for (unsigned k = 0; k < 6; ++k) assert(ik_drive_capture.sample[10][k] == want[k]);

  const uint8_t status[8] = {0x38, 0xff, 0xd2, 0x04, 0x14, 0x05, 6, 0x31};   /* -200, 1234, 1300 */
  const uint8_t diag[8] = {0xd4, 0x04, 0x40, 0x00, 0xb3, 0xff, 1, 42};       /* 12.36 V, -77 */
  assert(ik_telemetry_rx(STATUS_REPLY | MOTOR_SW_CAN_ID, status, 77));
  assert(ik_telemetry_rx(DIAG_REPLY | MOTOR_SW_CAN_ID, diag, 78));
  const uint8_t ident[8] = {0xEF, 0xBE, 0xAD, 0xDE, 0x18, 0xB1, 0x00, 0x00}; /* crc, 45336 */
  assert(ik_telemetry_rx(IDENT_REPLY | MOTOR_SW_CAN_ID, ident, 79));
  assert(!ik_telemetry_rx(STATUS_REPLY | 3U, status, 79));        /* not one wheel */
  assert(!ik_telemetry_rx(SPEED_WRITE | MOTOR_SW_CAN_ID, status, 79));
  const volatile uint32_t *w = (const volatile uint32_t *)&ik_drive_telem[2];
  for (unsigned i = 0; i < IK_TELEMETRY_WORDS; ++i) printf("%u ", (unsigned)w[i]);
  return 0;
}
'''


class TelemetryTest(unittest.TestCase):
    def test_c_decoder_to_python_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = pathlib.Path(tmp)
            (tmp / "t.c").write_text(HARNESS)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "CM7/Core/Inc"), "-I", str(ROOT / "HW/common/CAN"),
                            str(tmp / "t.c"), str(ROOT / "CM7/Core/Src/ik_telemetry.c"),
                            "-o", str(tmp / "t")], check=True)
            out = subprocess.run([str(tmp / "t")], check=True, capture_output=True, text=True).stdout
        words = [int(x) for x in out.split()]
        self.assertEqual(len(words), len(ik_link.ROW_FIELDS))
        row = ik_link.decode_row(words)
        self.assertEqual(row["generation"], 6)
        self.assertEqual((row["rpm"], row["iq_counts"], row["iq_ref_counts"]), (-200, 1234, 1300))
        self.assertAlmostEqual(row["iq_a"], 1234 / 544)
        self.assertEqual(row["mc_state"], "RUN")
        self.assertEqual((row["stop_reason"], row["active"], row["derated"], row["aligning"],
                          row["aligned"]), ("stop", True, True, False, False))
        self.assertEqual((row["status_tick"], row["status_count"]), (77, 1))
        self.assertAlmostEqual(row["vbus_v"], 12.36)
        self.assertEqual(row["fault_names"], ["overcurrent"])
        self.assertEqual((row["hall_shift_deg"], row["align"], row["i2t_percent"]), (-77, "ok", 42))
        self.assertEqual((row["diag_tick"], row["diag_count"]), (78, 1))
        self.assertEqual((row["app_crc"], row["app_size"], row["ident_tick"], row["ident_count"]),
                         (0xDEADBEEF, 45336, 79, 1))


class HostHelpersTest(unittest.TestCase):
    def test_capture_words(self):
        self.assertEqual(ik_link.capture_start_word(3000), 0xC5000000 | 3000)
        self.assertEqual(ik_link.capture_read_word(1, 64, 32), 0x3D000000 | (1 << 16) | (32 << 10) | 64)
        self.assertEqual(ik_link.capture_read_word(3, 0, 1), 0x3D000000 | (3 << 16) | (1 << 10))
        for bad in ((2, 0, 1), (0, 512, 1), (0, 0, 33), (0, 0, 0)):
            with self.assertRaises(ValueError):
                ik_link.capture_read_word(*bad)
        with self.assertRaises(ValueError):
            ik_link.capture_start_word(7000)

    def test_config_frame(self):
        self.assertEqual(ik_link.config_frame(4, "shift", -30), (0x504, 3 | (0xFFE2 << 16)))
        self.assertEqual(ik_link.config_frame(1, "kp", 4096), (0x501, 1 | (4096 << 16)))
        with self.assertRaises(ValueError):
            ik_link.config_frame(1, "kp", 40000)
        with self.assertRaises(ValueError):
            ik_link.config_frame(1, "gain", 1)

    def test_keys_combine_and_expire(self):
        keys = keyboard_drive.Keys(hold=0.6)
        keys.press("w", 0.0)
        keys.press("d", 0.1)
        keys.press("q", 0.2)
        self.assertEqual(keys.vector(0.3, 0.5, 2.0), (500, 500, 2000))
        keys.press("s", 0.4)  # same axis: latest wins
        self.assertEqual(keys.vector(0.5, 0.5, 2.0), (500, -500, 2000))
        self.assertEqual(keys.vector(0.85, 0.5, 2.0), (0, -500, 0))  # d and q expired
        keys.press(" ", 0.9)
        self.assertEqual(keys.vector(0.9, 0.5, 2.0), (0, 0, 0))

    def test_row_layout_matches_header(self):
        header = (ROOT / "CM7/Core/Inc/ik_telemetry.h").read_text()
        self.assertIn(f"#define IK_TELEMETRY_WORDS {len(ik_link.ROW_FIELDS)}U", header)
        body = header[header.index("struct ik_telemetry_wheel {"):header.index("};")]
        order = [name.strip(" ;\n") for line in body.splitlines()[1:]
                 for name in line.split(None, 1)[1].split(",")] if body else []
        self.assertEqual(tuple(order), ik_link.ROW_FIELDS)


if __name__ == "__main__":
    unittest.main()
