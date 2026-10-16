#!/usr/bin/env python3
"""Host-only NE monitor tests. No probe discovery or pyOCD imports."""

from contextlib import redirect_stdout, redirect_stderr
import ast
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import monitor_esc_power as monitor


def manifest():
    return {"wheel": "NE", "probe": monitor.NE_PROBE,
            "source_commit": monitor.EXPECTED_SOURCE_COMMIT,
            "after_sha256": monitor.EXPECTED_BACKUP_SHA256,
            "verified_app_readback": True, "app_size": monitor.APP_SIZE,
            "app_crc32": f"{monitor.APP_CRC:08X}",
            "metadata": [f"{word:08X}" for word in monitor.METADATA],
            "app_elf": "/unused/fake.elf"}


def fake_symbols():
    # Deliberately NOT the archived addresses, and reverse the standalone order.
    addresses = {"Mci": 0x20000100, "PIDSpeedHandle_M1": 0x20000180,
                 "SpeednTorqCtrlM1": 0x20000220, "BusVoltageSensor_M1": 0x20000260,
                 "VbusRegConv_M1": 0x20000280, "scaleParams_M1": 0x200002A0}
    standalone = [name for name in monitor.OBJECT_FIELDS if name not in addresses]
    addresses.update({name: 0x20001000 + index * 8
                      for index, name in enumerate(reversed(standalone))})
    return {name: (addresses[name], size)
            for name, (size, _) in monitor.OBJECT_FIELDS.items()}


def raw_values():
    return {"vbus_conversion_factor": 34, "vbus_average_raw": 20000,
            "vbus_adc_raw": 19000, "vbus_fault": 0,
            "vbus_undervoltage_threshold_raw": 15420,
            "mci_state": 6, "current_faults": 0, "past_faults": 4,
            "current_scale_a_per_count": monitor.CURRENT_SCALE,
            "pid_upper_limit_counts": 1632, "pid_lower_limit_counts": -1632,
            "torque_positive_limit_counts": 1632, "torque_negative_limit_counts": -1632,
            "iq_reference_counts": -1632, "id_reference_counts": -2,
            "measured_iq_counts": -544, "measured_id_counts": 272,
            "current_valid": 1, "calibration_result": 2, "calibration_valid": 1,
            "stop_reason": 9, "measured_rpm": -90, "peak_l1_counts": 816,
            "active": 1, "armed": 0}


class FakeTarget:
    """Every write/control operation is an immediate test failure."""
    def __init__(self):
        self.memory = dict.fromkeys(range(monitor.RAM_BASE, monitor.RAM_END), 0)
        self.reads, self.writes = [], []

    def put(self, address, data):
        self.memory.update({address + offset: value for offset, value in enumerate(data)})

    def read_memory_block8(self, address, size):
        self.reads.append((address, size))
        return [self.memory[address + offset] for offset in range(size)]

    def forbidden(self, *args, **kwargs):
        self.writes.append((args, kwargs))
        raise AssertionError("target write/control operation is forbidden")

    write_memory = write_memory_block8 = write_memory_block32 = forbidden
    write32 = write16 = write8 = reset = reset_and_halt = halt = resume = forbidden


def populate_ram(target, symbols=None, values=None):
    symbols, values = symbols or fake_symbols(), values or raw_values()
    for name, (_, fields) in monitor.OBJECT_FIELDS.items():
        for key, offset, fmt in fields:
            target.put(symbols[name][0] + offset, struct.pack("<" + fmt, values[key]))
    return symbols


class ImageTests(unittest.TestCase):
    def test_explicit_candidate_profile_restores_baseline_and_prepared_is_not_verified(self):
        baseline = (monitor.APP_CRC, monitor.EXPECTED_ELF_SHA256)
        with monitor.audited_profile('ne-low-speed-p2'):
            prepared = manifest()
            prepared['verified_app_readback'] = False
            monitor.validate_manifest(prepared)
            self.assertEqual(monitor.APP_CRC, 0x43ACF5E2)
            self.assertEqual(monitor.METADATA[3], 0x43ACF5E2)
        self.assertEqual((monitor.APP_CRC, monitor.EXPECTED_ELF_SHA256), baseline)
        with self.assertRaises(ValueError):
            monitor.validate_manifest(prepared)

    def test_candidate_requires_explicit_manifest_before_probe_access(self):
        for profile in ('ne-low-speed-p2', 'ne-900-p2', 'ne-900-stock', 'ne-900-ki4'):
            with redirect_stderr(io.StringIO()), patch.object(monitor, 'create_session') as connect:
                with self.assertRaises(SystemExit) as error:
                    monitor.main(['--profile', profile])
                self.assertEqual(error.exception.code, 2)
                connect.assert_not_called()

    def test_900_candidate_size_identity_and_live_gate_when_available(self):
        path = Path('/tmp/opencode/caps-5ms-900rpm-owned-20261015/NE/candidate.json')
        if not path.exists() or not monitor.DEFAULT_MANIFEST.exists():
            self.skipTest('local audited cap artifacts not available')
        baseline = monitor.load_image(monitor.DEFAULT_MANIFEST)
        with monitor.audited_profile('ne-900-p2'):
            candidate = monitor.load_image(path)
            self.assertEqual(monitor.APP_SIZE, 40868)
            self.assertEqual(len(candidate.app), 40868)
            self.assertEqual(monitor.APP_CRC, 0x3725CEE8)
            self.assertFalse(candidate.manifest['verified_app_readback'])
            self.assertFalse(candidate.manifest['runtime_gain_set_linked'])
            self.assertEqual(candidate.symbols, baseline.symbols)
            target = FakeTarget()
            target.put(monitor.APP_BASE, baseline.app + b'\xff' * 8)
            target.put(monitor.METADATA_ADDRESS, struct.pack('<8I', *monitor.METADATA))
            target.put(monitor.VTOR_ADDRESS, struct.pack('<I', monitor.APP_BASE))
            with self.assertRaisesRegex(ValueError, 'live full application'):
                monitor.verify_live(target, candidate)
            self.assertFalse(any(monitor.RAM_BASE <= a < monitor.RAM_END for a, _ in target.reads))
            self.assertEqual(target.writes, [])
            target.put(monitor.APP_BASE, candidate.app)
            self.assertTrue(monitor.verify_live(target, candidate)['full_app_equal'])
        self.assertEqual(monitor.APP_SIZE, 40860)
        self.assertEqual(monitor.AUDIT_PROFILE_NAME, 'baseline')

    def test_900_stock_restore_two_bytes_and_live_gate_when_available(self):
        path = Path('/tmp/opencode/fr-900-stock-20261015/app/candidate.json')
        p2_path = Path('/tmp/opencode/caps-5ms-900rpm-owned-20261015/NE/candidate.json')
        if not path.exists() or not p2_path.exists():
            self.skipTest('local audited restoration artifacts not available')
        with monitor.audited_profile('ne-900-p2'):
            p2 = monitor.load_image(p2_path)
        with monitor.audited_profile('ne-900-stock'):
            stock = monitor.load_image(path)
            self.assertFalse(stock.manifest['verified_app_readback'])
            self.assertFalse(stock.manifest['runtime_gain_set_linked'])
            self.assertEqual(stock.symbols, p2.symbols)
            differences = [(i,a,b) for i,(a,b) in enumerate(zip(p2.app,stock.app)) if a!=b]
            self.assertEqual(differences, [(0x9D61,2,4),(0x9D64,9,10)])
            target = FakeTarget()
            target.put(monitor.APP_BASE, p2.app)
            target.put(monitor.METADATA_ADDRESS, struct.pack('<8I', *monitor.METADATA))
            target.put(monitor.VTOR_ADDRESS, struct.pack('<I', monitor.APP_BASE))
            with self.assertRaisesRegex(ValueError, 'live full application'):
                monitor.verify_live(target, stock)
            self.assertFalse(any(monitor.RAM_BASE <= a < monitor.RAM_END for a,_ in target.reads))
            target.put(monitor.APP_BASE, stock.app)
            self.assertTrue(monitor.verify_live(target, stock)['full_app_equal'])
            self.assertEqual(target.writes, [])
        self.assertEqual(monitor.AUDIT_PROFILE_NAME, 'baseline')

    def test_ki4_two_bytes_and_live_identity_gate_when_available(self):
        path = Path('/tmp/opencode/rpm2-ki4-review-20261016/NE/ki4/monitor-prepared.json')
        stock_path = Path('/home/fh/sfurs_firmware/esc-backups/fr-stock900-restore-20261015T231437Z/NE-after.json')
        if not path.exists() or not stock_path.exists():
            self.skipTest('local audited Ki4 artifacts not available')
        with monitor.audited_profile('ne-900-stock'):
            stock = monitor.load_image(stock_path)
        with monitor.audited_profile('ne-900-ki4'):
            candidate = monitor.load_image(path)
            self.assertFalse(candidate.manifest['verified_app_readback'])
            self.assertFalse(candidate.manifest['installed'])
            self.assertEqual(candidate.symbols, stock.symbols)
            self.assertEqual([(i,a,b) for i,(a,b) in enumerate(zip(stock.app,candidate.app))
                              if a != b], [(0x9D4A,184,224),(0x9D4B,255,254)])
            target = FakeTarget()
            target.put(monitor.APP_BASE, stock.app)
            target.put(monitor.METADATA_ADDRESS, struct.pack('<8I', *monitor.METADATA))
            target.put(monitor.VTOR_ADDRESS, struct.pack('<I', monitor.APP_BASE))
            with self.assertRaisesRegex(ValueError, 'live full application'):
                monitor.verify_live(target, candidate)
            self.assertFalse(any(monitor.RAM_BASE <= a < monitor.RAM_END for a,_ in target.reads))
            target.put(monitor.APP_BASE, candidate.app)
            self.assertTrue(monitor.verify_live(target, candidate)['full_app_equal'])
            for address in (monitor.METADATA_ADDRESS+12, monitor.VTOR_ADDRESS):
                original = bytes(target.read_memory_block8(address,4))
                target.put(address, b'\x00'*4)
                with self.assertRaises(ValueError):
                    monitor.verify_live(target, candidate)
                target.put(address, original)
            self.assertEqual(target.writes, [])
        self.assertEqual(monitor.AUDIT_PROFILE_NAME, 'baseline')

    def test_tune_profile_merges_and_restores_gain_symbols_and_app_size(self):
        baseline_size = monitor.APP_SIZE
        baseline_keys = set(monitor.OBJECT_FIELDS)
        self.assertNotIn("test_speed_kp_gain", monitor.OBJECT_FIELDS)
        with monitor.audited_profile('rpm2-tune'):
            self.assertEqual(monitor.APP_SIZE, 41068)
            self.assertEqual(monitor.APP_CRC, 0x9976A657)
            for name in ("test_speed_kp_gain", "test_speed_ki_gain",
                         "test_gain_set_result", "test_gain_set_count"):
                self.assertIn(name, monitor.OBJECT_FIELDS)
            monitor.validate_symbols(fake_symbols())
            # Fail-closed until the first-flash backup hash is pinned.
            prepared = manifest()
            prepared['verified_app_readback'] = False
            prepared['after_sha256'] = 'ab' * 32
            with self.assertRaises(ValueError):
                monitor.validate_manifest(prepared)
        self.assertEqual(monitor.APP_SIZE, baseline_size)
        self.assertEqual(set(monitor.OBJECT_FIELDS), baseline_keys)
        monitor.validate_symbols(fake_symbols())

    def test_prepared_candidate_live_gate_and_two_byte_difference_when_available(self):
        path = Path('/tmp/opencode/fr-tune-p2-20261015/app/candidate.json')
        if not path.exists() or not monitor.DEFAULT_MANIFEST.exists():
            self.skipTest('local audited artifacts not available')
        baseline = monitor.load_image(monitor.DEFAULT_MANIFEST)
        with monitor.audited_profile('ne-low-speed-p2'):
            candidate = monitor.load_image(path)
            self.assertFalse(candidate.manifest['verified_app_readback'])
            self.assertEqual(baseline.symbols, candidate.symbols)
            differences = [(i, a, b) for i, (a, b) in enumerate(zip(baseline.app, candidate.app))
                           if a != b]
            self.assertEqual(differences, [(0x9D59, 4, 2), (0x9D5C, 10, 9)])
            target = FakeTarget()
            target.put(monitor.APP_BASE, baseline.app)
            target.put(monitor.METADATA_ADDRESS, struct.pack('<8I', *monitor.METADATA))
            target.put(monitor.VTOR_ADDRESS, struct.pack('<I', monitor.APP_BASE))
            with self.assertRaisesRegex(ValueError, 'live full application'):
                monitor.verify_live(target, candidate)
            self.assertFalse(any(monitor.RAM_BASE <= a < monitor.RAM_END for a, _ in target.reads))
            self.assertEqual(target.writes, [])
            target.put(monitor.APP_BASE, candidate.app)
            self.assertTrue(monitor.verify_live(target, candidate)['full_app_equal'])

    def test_clip_prefix_and_data_physical_lma(self):
        # Pretend first segment includes ELF/boot prefix, second is .data's LMA,
        # not its runtime SRAM VMA. Outside-window segments must be ignored.
        segments = [(96, b"BOOTabcdefgh"), (108, b"ijkl"),
                    (monitor.RAM_BASE, b"wrong SRAM VMA")]
        self.assertEqual(monitor.reconstruct_app(segments, 100, 12), b"abcdefghijkl")

    def test_gaps_and_conflicting_overlaps_rejected(self):
        with self.assertRaisesRegex(ValueError, "gap"):
            monitor.reconstruct_app([(100, b"ab"), (103, b"d")], 100, 4)
        with self.assertRaisesRegex(ValueError, "overlap"):
            monitor.reconstruct_app([(100, b"abcd"), (102, b"XY")], 100, 4)
        self.assertEqual(monitor.reconstruct_app([(100, b"abcd"), (102, b"cd")], 100, 4),
                         b"abcd")

    def test_crc_size_and_sha_gate(self):
        with self.assertRaisesRegex(ValueError, "size/CRC/SHA"):
            monitor.validate_app(b"not the installed image")
        image = bytes(monitor.APP_SIZE)
        with patch.object(monitor, "EXPECTED_APP_SHA256", hashlib.sha256(image).hexdigest()):
            with self.assertRaisesRegex(ValueError, "CRC"):
                monitor.validate_app(image)

    def test_manifest_exact_identity(self):
        monitor.validate_manifest(manifest())
        with self.assertRaisesRegex(ValueError, "JSON object"):
            monitor.validate_manifest([])
        for key, wrong in [("wheel", "SE"), ("probe", "other"), ("app_size", True),
                           ("app_crc32", "00000000"), ("source_commit", "unknown"),
                           ("after_sha256", "unknown"), ("verified_app_readback", False),
                           ("metadata", ["00000000"] * 8), ("app_elf", "")]:
            with self.subTest(key=key):
                bad = manifest()
                bad[key] = wrong
                with self.assertRaises(ValueError):
                    monitor.validate_manifest(bad)

    def test_wrong_elf_hash_rejected_before_parsing_or_pyocd(self):
        with tempfile.TemporaryDirectory(dir="/tmp/opencode") as directory:
            directory = Path(directory)
            elf = directory / "unknown.elf"
            elf.write_bytes(b"not an ELF and not the expected hash")
            data = manifest()
            data["app_elf"] = str(elf)
            sidecar = directory / "manifest.json"
            sidecar.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "ELF SHA-256"):
                monitor.load_image(sidecar)
            self.assertFalse(any(name.startswith("pyocd") for name in sys.modules))

    def test_no_symbol_zero_fallback_wrong_size_or_bounds(self):
        monitor.validate_symbols(fake_symbols())
        for bad in [None, (0, 4), (monitor.RAM_END - 2, 4), (0x20001000, 8)]:
            symbols = fake_symbols()
            if bad is None:
                del symbols["test_drive_iq"]
            else:
                symbols["test_drive_iq"] = bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                monitor.plan_blocks(symbols)
        self.assertNotIn("test_profile_reference_limit_ma", monitor.OBJECT_FIELDS)

    def test_archived_elf_when_available(self):
        if not monitor.DEFAULT_MANIFEST.exists():
            self.skipTest("local audited archive unavailable")
        try:
            import elftools  # Optional offline archive check, never pyOCD.
        except ImportError:
            self.skipTest("pyelftools unavailable")
        image = monitor.load_image(monitor.DEFAULT_MANIFEST)
        self.assertEqual(image.elf_sha256, monitor.EXPECTED_ELF_SHA256)
        self.assertEqual(len(image.app), 40860)
        self.assertEqual(zlib.crc32(image.app), 0xB93E5A8A)
        self.assertEqual(image.symbols["BusVoltageSensor_M1"], (0x20000290, 18))
        self.assertEqual(image.symbols["test_drive_iq_reference"], (0x2000095C, 4))
        self.assertEqual(len(monitor.plan_blocks(image.symbols)), 4)


class LiveIdentityTests(unittest.TestCase):
    def setUp(self):
        # Host-generated stand-in image; change only test expectations, not code
        # under test. Production constants stay pinned to the audited archive.
        self.app = bytes(index % 251 for index in range(monitor.APP_SIZE))
        crc = zlib.crc32(self.app)
        metadata = list(monitor.METADATA)
        metadata[3] = crc
        for name, value in [("APP_CRC", crc), ("METADATA", tuple(metadata)),
                            ("EXPECTED_APP_SHA256", hashlib.sha256(self.app).hexdigest())]:
            context = patch.object(monitor, name, value)
            context.start()
            self.addCleanup(context.stop)
        self.target = FakeTarget()
        self.target.put(monitor.APP_BASE, self.app)
        self.target.put(monitor.METADATA_ADDRESS, struct.pack("<8I", *metadata))
        self.target.put(monitor.VTOR_ADDRESS, struct.pack("<I", monitor.APP_BASE))
        self.symbols = populate_ram(self.target)
        self.image = monitor.AuditedImage("fake.json", manifest(), "manifest-hash",
                                          "fake.elf", monitor.EXPECTED_ELF_SHA256,
                                          self.app, self.symbols)

    def assert_no_ram_or_writes(self):
        self.assertFalse(any(monitor.RAM_BASE <= address < monitor.RAM_END
                             for address, _ in self.target.reads))
        self.assertEqual(self.target.writes, [])

    def test_full_flash_verification_before_ram(self):
        identity = monitor.verify_live(self.target, self.image)
        self.assertTrue(identity["full_app_equal"])
        self.assertEqual(sum(size for address, size in self.target.reads
                             if monitor.APP_BASE <= address < monitor.APP_BASE + monitor.APP_SIZE),
                         monitor.APP_SIZE)
        self.assert_no_ram_or_writes()

    def test_wrong_live_flash_even_with_correct_metadata(self):
        self.target.put(monitor.APP_BASE + 1234, b"\xff")
        with self.assertRaisesRegex(ValueError, "full application"):
            monitor.monitor(self.target, self.image, io.StringIO(), 1, 0.05)
        self.assert_no_ram_or_writes()

    def test_each_wrong_metadata_word_blocks_ram(self):
        for index in range(8):
            with self.subTest(word=index):
                self.target.reads.clear()
                words = list(monitor.METADATA)
                words[index] ^= 1
                self.target.put(monitor.METADATA_ADDRESS, struct.pack("<8I", *words))
                with self.assertRaisesRegex(ValueError, "metadata"):
                    monitor.monitor(self.target, self.image, io.StringIO(), 1, 0.05)
                self.assert_no_ram_or_writes()

    def test_wrong_vtor_blocks_ram(self):
        self.target.put(monitor.VTOR_ADDRESS, struct.pack("<I", 0x08000000))
        with self.assertRaisesRegex(ValueError, "VTOR"):
            monitor.monitor(self.target, self.image, io.StringIO(), 1, 0.05)
        self.assert_no_ram_or_writes()

    def test_header_sample_provenance_and_interrupt_no_cleanup_commands(self):
        output = io.StringIO()
        with patch.object(monitor, "print_status", side_effect=KeyboardInterrupt):
            monitor.monitor(self.target, self.image, output, 60, 0.05)
        header, sample, end = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(header["type"], "header")
        self.assertEqual(header["probe"], monitor.NE_PROBE)
        self.assertEqual(header["elf_sha256"], monitor.EXPECTED_ELF_SHA256)
        self.assertEqual(header["initial_scaling_limits"]["pid_lower_limit_counts"], -1632)
        self.assertIn("iq_reference_counts", header["field_blocks"])
        self.assertTrue(any("transients" in note for note in header["caveats"]))
        self.assertTrue(any("stale" in note for note in header["caveats"]))
        self.assertEqual(sample["type"], "sample")
        self.assertEqual(end["reason"], "interrupted")
        self.assertEqual(end["samples"], 1)
        self.assertEqual(self.target.writes, [])


class SamplingTests(unittest.TestCase):
    def setUp(self):
        self.target = FakeTarget()
        self.symbols = populate_ram(self.target)
        self.sampler = monitor.Sampler(self.target, self.symbols)

    def test_signed_fields_voltage_formula_raw_and_block_timestamps(self):
        sample = self.sampler.sample(12)
        raw, derived = sample["raw"], sample["derived"]
        self.assertEqual(raw["measured_iq_counts"], -544)
        self.assertEqual(raw["iq_reference_counts"], -1632)
        self.assertEqual(raw["measured_rpm"], -90)
        self.assertEqual(raw["pid_lower_limit_counts"], -1632)
        self.assertEqual(raw["torque_negative_limit_counts"], -1632)
        self.assertAlmostEqual(derived["vbus_average_sdk_v"], 20000 * 34 / 65536)
        self.assertAlmostEqual(derived["vbus_adc_sdk_v"], 19000 * 34 / 65536)
        self.assertAlmostEqual(derived["measured_iq_sdk_a"], -544 * monitor.CURRENT_SCALE)
        self.assertEqual(derived["stop_reason_name"], "stall")
        self.assertTrue(derived["iq_reference_near_pid_limit"])
        self.assertFalse(sample["atomic"])
        self.assertLess(len(sample["blocks"]), len(self.symbols))
        for block in sample["blocks"]:
            self.assertGreaterEqual(block["monotonic_ns_end"], block["monotonic_ns_start"])
            self.assertIn("+00:00", block["utc_start"])
            self.assertIn("+00:00", block["utc_end"])
            self.assertEqual(len(bytes.fromhex(block["data_hex"])), block["size"])
        self.assertEqual(self.target.writes, [])

    def test_all_validity_gates_null_measured_but_preserve_raw(self):
        for key, value in [("current_valid", 0), ("calibration_valid", 0),
                           ("calibration_result", 1), ("calibration_result", 0),
                           ("mci_state", 0), ("current_faults", 4)]:
            with self.subTest(key=key, value=value):
                values = raw_values()
                values[key] = value
                populate_ram(self.target, self.symbols, values)
                sample = self.sampler.sample(0)
                self.assertEqual(sample["raw"]["measured_iq_counts"], -544)
                self.assertIsNone(sample["derived"]["measured_iq_sdk_a"])
                self.assertIsNone(sample["derived"]["measured_id_sdk_a"])
                self.assertTrue(sample["derived"]["reference_may_be_stale"])
                self.assertFalse(sample["derived"]["reference_valid"])
                self.assertIsNone(sample["derived"]["iq_reference_near_pid_limit"])
                self.assertLess(sample["derived"]["iq_reference_sdk_a"], 0)

    def test_limit_proximity_is_sign_sensitive_and_gated(self):
        values = raw_values()
        for reference, expected in [(100, False), (1632, True), (-100, False), (-1632, True)]:
            values["iq_reference_counts"] = reference
            self.assertEqual(monitor.derive(values)["iq_reference_near_pid_limit"], expected)
        values["pid_upper_limit_counts"] = 0
        self.assertIsNone(monitor.derive(values)["iq_reference_near_pid_limit"])

    def test_bad_scale_or_conversion_preserves_bytes_without_interpretation(self):
        for key, value in [("current_scale_a_per_count", math.nan),
                           ("current_scale_a_per_count", math.inf),
                           ("current_scale_a_per_count", -1.0),
                           ("current_scale_a_per_count", 0.001),
                           ("vbus_conversion_factor", 0)]:
            values = raw_values()
            values[key] = value
            populate_ram(self.target, self.symbols, values)
            record = self.sampler.sample(0)
            self.assertEqual(record["type"], "invalid_sample")
            self.assertNotIn("derived", record)
            self.assertTrue(record["blocks"][0]["data_hex"])
            output = io.StringIO()
            monitor.write_record(output, record)
            self.assertNotIn("NaN", output.getvalue())
            self.assertNotIn("Infinity", output.getvalue())
            self.assertEqual(json.loads(output.getvalue())["raw"]["measured_iq_counts"], -544)

    def test_invalid_adc_and_unknown_stop_reason(self):
        values = raw_values()
        values.update(vbus_adc_raw=0xFFFF, stop_reason=123)
        derived = monitor.derive(values)
        self.assertIsNone(derived["vbus_adc_sdk_v"])
        self.assertEqual(derived["stop_reason_name"], "unknown")


class InterfaceTests(unittest.TestCase):
    def test_output_exclusive_create_never_overwrites(self):
        with tempfile.TemporaryDirectory(dir="/tmp/opencode") as directory:
            path = Path(directory) / "capture.jsonl"
            actual, output = monitor.open_output(path)
            with output:
                output.write("existing evidence\n")
            self.assertEqual(actual, path)
            with self.assertRaises(FileExistsError):
                monitor.open_output(path)
            self.assertEqual(path.read_text(), "existing evidence\n")

    def test_default_output_path_utc_pid(self):
        with patch.object(Path, "open", return_value=io.StringIO()) as opened:
            path, output = monitor.open_output()
        self.assertEqual(path.parent, Path("/tmp/opencode"))
        self.assertRegex(path.name, r"^esc-power-\d{8}T\d{6}\.\d{6}Z-\d+\.jsonl$")
        opened.assert_called_once_with("x", encoding="utf-8")
        output.close()

    def test_exact_probe_only_and_safe_session_options_without_pyocd(self):
        probe = SimpleNamespace(unique_id=monitor.NE_PROBE)
        calls = []
        discovery = []
        def find_probes(**kwargs):
            discovery.append(kwargs)
            return [probe]
        helper = SimpleNamespace(get_all_connected_probes=find_probes)
        def session_class(selected, **kwargs):
            calls.append((selected, kwargs))
            return "fake session"
        self.assertEqual(monitor.create_session(helper, session_class), "fake session")
        self.assertEqual(discovery, [{"blocking": False, "unique_id": monitor.NE_PROBE}])
        self.assertIs(calls[0][0], probe)
        options = calls[0][1]["options"]
        self.assertEqual(options["target_override"], "stm32g431cbux")
        self.assertEqual(options["connect_mode"], "attach")
        for key in ["resume_on_disconnect", "auto_unlock", "pack.debug_sequences.enable",
                    "cache.enable_memory", "cache.enable_register", "enable_swv"]:
            self.assertIs(options[key], False)
        self.assertTrue(options["no_config"])
        self.assertEqual(options["user_script"], "/dev/null")
        self.assertFalse(calls[0][1]["auto_open"])
        for probes in [[], [SimpleNamespace(unique_id="other")],
                       [SimpleNamespace(unique_id=monitor.NE_PROBE + "extra")], [probe, probe]]:
            helper.get_all_connected_probes = lambda **kwargs: probes
            with self.assertRaisesRegex(ValueError, "exact-serial"):
                monitor.create_session(helper, session_class)
        self.assertEqual(len(calls), 1)

    def test_bad_period_or_duration_rejected_before_probe_import(self):
        for args in [["--period", "0.001"], ["--period", "2"], ["--period", "nan"],
                     ["--seconds", "0"], ["--seconds", "inf"]]:
            with self.subTest(args=args), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    monitor.main(args)
                self.assertEqual(result.exception.code, 2)

    def test_main_closes_session_after_interrupt_without_motor_cleanup(self):
        target = FakeTarget()
        counts = {"open": 0, "close": 0}
        session = SimpleNamespace(target=target)
        session.open = lambda: counts.update(open=counts["open"] + 1)
        session.close = lambda: counts.update(close=counts["close"] + 1)
        with tempfile.TemporaryDirectory(dir="/tmp/opencode") as directory:
            path = Path(directory) / "interrupt.jsonl"
            with patch.object(monitor, "load_image", return_value=object()), \
                    patch.object(monitor, "create_session", return_value=session), \
                    patch.object(monitor, "monitor", side_effect=KeyboardInterrupt), \
                    redirect_stdout(io.StringIO()):
                self.assertEqual(monitor.main(["--output", str(path)]), 0)
            self.assertEqual(json.loads(path.read_text())["reason"], "interrupted")
        self.assertEqual(counts, {"open": 1, "close": 1})
        self.assertEqual(target.writes, [])

    def test_existing_output_rejected_before_probe_discovery(self):
        with tempfile.TemporaryDirectory(dir="/tmp/opencode") as directory:
            path = Path(directory) / "existing.jsonl"
            path.write_text("preserve evidence")
            with patch.object(monitor, "load_image", return_value=object()), \
                    patch.object(monitor, "create_session") as create, \
                    redirect_stderr(io.StringIO()):
                self.assertEqual(monitor.main(["--output", str(path)]), 1)
                create.assert_not_called()
            self.assertEqual(path.read_text(), "preserve evidence")

    def test_no_target_write_control_or_flash_calls_in_logger(self):
        tree = ast.parse(Path(monitor.__file__).read_text())
        forbidden = {"write_memory", "write_memory_block8", "write_memory_block32",
                     "write32", "write16", "write8", "reset", "reset_and_halt",
                     "halt", "resume", "program", "erase", "flash"}
        calls = {node.func.attr for node in ast.walk(tree)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        self.assertFalse(calls & forbidden)

    def test_no_pyocd_imported(self):
        self.assertFalse(any(name.startswith("pyocd") for name in sys.modules))


if __name__ == "__main__":
    unittest.main(verbosity=2)
