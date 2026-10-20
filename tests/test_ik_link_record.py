#!/usr/bin/env python3
"""Host encoding of the ESC speed-loop recorder and startup-loop CONFIG keys,
checked against can_datatypes.h so the two sides cannot drift."""
import pathlib
import re
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/tools"))
import esc_tool  # noqa: E402
import move_test  # noqa: E402
import ik_link  # noqa: E402

HEADER = (ROOT / "src/shared/can/can_datatypes.h").read_text()


def c_value(name):
    return int(re.search(rf"\b{name}\b\s*=?\s*\(?(0x[0-9A-Fa-f]+|\d+)U?", HEADER)[1], 0)


class Record(unittest.TestCase):
    def test_constants_match_header(self):
        self.assertEqual(ik_link.RECORD_START_OP, c_value("ESC_RECORD_START"))
        self.assertEqual(ik_link.RECORD_SAMPLES, c_value("ESC_RECORD_SAMPLES"))
        self.assertEqual(ik_link.RECORD_INFO_SAMPLE, c_value("ESC_RECORD_INFO_SAMPLE"))
        self.assertEqual(ik_link.RECORD_READ_CH67, 1 << 18)
        self.assertIn("ESC_RECORD_READ_CH67 (1U << 18)", HEADER)
        for key, number in ik_link.CONFIG_KEYS.items():
            if key.startswith("dsc_"):
                self.assertEqual(c_value("ESC_CFG_" + key.upper()), number, key)

    def test_words(self):
        self.assertEqual(ik_link.record_start_word(50, 1000), 0x5E << 24 | 1000 << 9 | 50)
        self.assertEqual(ik_link.record_read_word(0, 32, 32, ch67=True),
                         0x3D << 24 | 1 << 18 | 32 << 10 | 32)
        self.assertEqual(ik_link.record_read_word(1, 0, 1), 0x3D << 24 | 1 << 16 | 1 << 10)
        for bad in ((384, 0), (-1, 0), (0, 4096)):
            with self.assertRaises(ValueError):
                ik_link.record_start_word(*bad)
        with self.assertRaises(ValueError):
            ik_link.record_read_word(1, 0, 1, ch67=True)
        with self.assertRaises(ValueError):
            ik_link.record_read_word(2, 0, 1)

    def test_unpack(self):
        packed = 2 | 2 << 2 | 0x40 | 0x80 | 3 << 8 | 5 << 12 | 1 << 15
        r = ik_link.unpack_record(dict(ref_rpm=51, rpm=40, p=272, i=0, ff=326, iq_ref=598,
                                       iq=-544, packed=packed - 65536 if packed > 32767 else packed))
        self.assertEqual((r["state"], r["reason"], r["capped"], r["dsc"]), ("running", "edge", True, True))
        self.assertEqual((r["edges_mod16"], r["captures_mod8"], r["loop"]), (3, 5, True))
        self.assertAlmostEqual(r["p_a"], 0.5)
        self.assertAlmostEqual(r["iq_a"], -1.0)

    def test_defaults_cover_every_dsc_key(self):
        d = esc_tool.firmware_defaults()
        self.assertEqual({k for k in ik_link.CONFIG_KEYS if k.startswith("dsc_")},
                         {k for k in d if k.startswith("dsc_")})
        self.assertEqual(d["dsc_enable"], 0)


class Summary(unittest.TestCase):
    @staticmethod
    def row(state, reason, iq=0.0, rpm=0):
        return dict(state=state, reason=reason, iq_ref_a=iq, rpm=rpm)

    def test_observed_handoff(self):
        rows = [self.row("stopped", "stop")] * 5 + [self.row("starting", "armed", 1.0 + k / 10)
                                                     for k in range(10)]
        rows += [self.row("running", "edge", 0.6, 40), self.row("running", "edge", 0.7, 60)]
        s = move_test.record_summary(dict(samples=len(rows), trigger_index=5), rows)
        self.assertEqual((s["handoff_ms"], s["handoff_iq_a"], s["peak_rpm"]), (10, 1.9, 60))

    def test_delayed_window_has_no_handoff(self):
        rows = [self.row("running", "edge", 0.6, 50)] * 20
        s = move_test.record_summary(dict(samples=20, trigger_index=0), rows)
        self.assertIsNone(s["handoff_ms"])
        self.assertIsNone(s["handoff_iq_a"])


if __name__ == "__main__":
    unittest.main()
