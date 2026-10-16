#!/usr/bin/env python3
"""Hardware-free IK TUI/transport tests; no pyOCD or compiler dependency."""
import contextlib
import importlib.util
import io
import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("keyboard_drive", ROOT / "scripts/keyboard_drive.py")
kd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kd)
ADDRS = {name: 0x24000000 + index * 256 for index, name in enumerate(kd.FIELDS)}
FLASH = [(0x08000000, bytes(range(256)) * 9), (0x08018000, b"data initializer")]


class FakeTarget:
    def __init__(self, immediate=True):
        self.mem = {address: 0 for address in ADDRS.values()}
        self.mem[ADDRS["abi"]] = kd.ABI
        self.flash = {address + offset: byte for address, data in FLASH
                      for offset, byte in enumerate(data)}
        self.writes, self.commands, self.flash_reads = [], [], []
        self.events = []
        self.immediate = immediate
        self.current = None
        self.block_reads = []
        self.torn_reads = {}

    def read32(self, address):
        self.events.append(("read", address))
        return self.mem.get(address, 0)

    def read_memory_block8(self, address, length):
        self.flash_reads.append((address, length))
        self.events.append(("flash", address, length))
        return [self.flash.get(address + offset, 0) for offset in range(length)]

    def read_memory_block32(self, address, length):
        self.block_reads.append((address, length))
        self.events.append(("block32", address, length))
        words = [self.mem.get(address + offset * 4, 0) for offset in range(length)]
        if self.torn_reads.get(address, 0):
            self.torn_reads[address] -= 1
            self.mem[address] = words[0] + 2
        return words

    def flush(self):
        self.events.append(("flush",))

    def write32(self, address, value):
        self.writes.append((address, value))
        self.events.append(("write", address, value))
        self.mem[address] = value
        if address == ADDRS["sequence"]:
            request = self.mem[ADDRS["request"]]
            vector = tuple(kd.signed32(self.mem[ADDRS[name]])
                           for name in ("vx_mmps", "vy_mmps", "omega_mradps"))
            self.commands.append((request, value, vector))
            self.current = request, value
            self.mem[ADDRS["state"]] = kd.STOPPING if request == kd.STOP else kd.BUSY
            if self.immediate:
                self.complete()

    def complete(self, result=kd.SENT):
        request, sequence = self.current
        self.mem[ADDRS["state"]] = kd.RUNNING if request == kd.START else kd.IDLE
        self.mem[ADDRS["result"]] = result
        self.mem[ADDRS["completed"]] = sequence
        self.mem[ADDRS["tx_count"]] += 3
        self.mem[ADDRS["ack_count"]] += 3 if result == kd.SENT else 0
        # Deliberately leave request unchanged: firmware NEVER clears it.
        self.current = None


def verified(immediate=True):
    target = FakeTarget(immediate)
    drive = kd.Drive(target, ADDRS)
    with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
        drive.verify("selected.elf")
    return target, drive


def requests(target):
    return [command[0] for command in target.commands]


def telemetry_row(target, index=0, **changes):
    row = dict(zip(kd.TELEM_FIELDS, (2, -123, 6, 5, 77, 900, -1088, 544,
                                   0xF800, 2000, 950, 10, 20)))
    row.update(changes)
    base = ADDRS["telem"] + index * 52
    for offset, name in enumerate(kd.TELEM_FIELDS):
        target.mem[base + offset * 4] = row[name] & 0xFFFFFFFF
    target.mem[ADDRS["telem_now"]] = 1000
    return base


class Segment(dict):
    def __init__(self, address, data, *, vaddr=None, kind="PT_LOAD", size=None):
        super().__init__(p_type=kind, p_paddr=address,
                         p_vaddr=address if vaddr is None else vaddr,
                         p_filesz=len(data) if size is None else size)
        self.payload = data

    def data(self):
        return self.payload


class ResolverTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir="/tmp/opencode")
        self.addCleanup(self.temp.cleanup)
        self.elf = pathlib.Path(self.temp.name) / "explicit.elf"
        self.elf.touch()
        self.output = "\n".join(f"{address:08x} B ik_drive_{name}" for name, address in ADDRS.items())

    def resolve(self, output):
        with mock.patch.object(kd.shutil, "which", return_value="/usr/bin/nm"), \
                mock.patch.object(kd.subprocess, "run", return_value=mock.Mock(stdout=output)) as run:
            result = kd.resolve_addrs(self.elf)
        run.assert_called_once_with(["/usr/bin/nm", "--defined-only", str(self.elf)],
                                    check=True, capture_output=True, text=True, timeout=30)
        return result

    def test_all_symbols_resolved_from_explicit_elf(self):
        self.assertEqual(self.resolve(self.output + "\n20000000 B wheel_bench_request"), ADDRS)

    def test_missing_symbol_fails_instead_of_using_fallback(self):
        with self.assertRaisesRegex(RuntimeError, "missing: ik_drive_fault"):
            self.resolve("\n".join(line for line in self.output.splitlines()
                                    if not line.endswith("ik_drive_fault")))

    def test_both_abi2_telemetry_symbols_are_required(self):
        for name in ("telem", "telem_now"):
            with self.subTest(name=name), self.assertRaisesRegex(RuntimeError, "missing: ik_drive_" + name):
                self.resolve("\n".join(line for line in self.output.splitlines()
                                       if not line.endswith("ik_drive_" + name)))

    def test_missing_elf_fails_before_nm(self):
        with mock.patch.object(kd.subprocess, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "ELF not found"):
                kd.resolve_addrs(self.elf.parent / "missing.elf")
            run.assert_not_called()

    def test_nm_failure_propagates(self):
        with mock.patch.object(kd.shutil, "which", return_value="nm"), \
                mock.patch.object(kd.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "nm")):
            with self.assertRaises(subprocess.CalledProcessError):
                kd.resolve_addrs(self.elf)

    def test_duplicate_symbol_fails(self):
        with self.assertRaisesRegex(RuntimeError, "duplicate"):
            self.resolve(self.output + "\n24010000 B ik_drive_request")

    def test_toolchain_nm_used_only_when_available(self):
        original = pathlib.Path.is_file
        def is_file(path):
            return str(path).endswith("/arm-none-eabi-nm") or original(path)
        with mock.patch.object(kd.shutil, "which", return_value=None), \
                mock.patch.object(pathlib.Path, "is_file", is_file), \
                mock.patch.object(kd.subprocess, "run", return_value=mock.Mock(stdout=self.output)) as run:
            self.assertEqual(kd.resolve_addrs(self.elf), ADDRS)
            self.assertEqual(run.call_args.args[0][0],
                             "/tmp/opencode/cm7-toolchain/root/usr/bin/arm-none-eabi-nm")


class IdentityTests(unittest.TestCase):
    def test_all_flash_bytes_compared_in_chunks_before_abi(self):
        target, drive = verified()
        self.assertTrue(drive.verified)
        expected = [(address + offset, min(1024, len(data) - offset))
                    for address, data in FLASH for offset in range(0, len(data), 1024)]
        self.assertEqual(target.flash_reads, expected)
        self.assertEqual(target.events[-1], ("read", ADDRS["abi"]))
        self.assertEqual(target.writes, [])

    def test_flash_segments_use_physical_address_including_data_lma(self):
        segments = [Segment(0x08000000, b"code"),
                    Segment(0x08018000, b"data", vaddr=0x24000000),
                    Segment(0x24000100, b"ram"), Segment(0x08019000, b"", size=0),
                    Segment(0x08020000, b"metadata", kind="PT_NOTE")]
        elf = mock.Mock()
        elf.iter_segments.return_value = iter(segments)
        self.assertEqual(kd.flash_segments(elf), [(0x08000000, b"code"), (0x08018000, b"data")])

    def test_empty_truncated_and_out_of_range_segments_fail(self):
        for segments in ([], [Segment(0x08000000, b"short", size=8)],
                         [Segment(0x081FFFFF, b"too long")]):
            with self.subTest(segments=segments):
                elf = mock.Mock()
                elf.iter_segments.return_value = iter(segments)
                with self.assertRaises(RuntimeError):
                    kd.flash_segments(elf)

    def test_wrong_flash_including_last_data_byte_causes_zero_writes(self):
        for address in (FLASH[0][0], FLASH[0][0] + len(FLASH[0][1]) - 1,
                        FLASH[1][0] + len(FLASH[1][1]) - 1):
            with self.subTest(address=address):
                target = FakeTarget()
                target.flash[address] ^= 1
                drive = kd.Drive(target, ADDRS)
                body = mock.Mock()
                with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                    with self.assertRaisesRegex(RuntimeError, "reflash"):
                        kd.operate(drive, "wrong.elf", body)
                body.assert_not_called()
                self.assertFalse(drive.verified)
                self.assertEqual(target.writes, [])

    def test_wrong_abi_causes_zero_writes_even_on_exit(self):
        for abi in (0x494B0001, kd.ABI + 1):
            with self.subTest(abi=abi):
                target = FakeTarget()
                target.mem[ADDRS["abi"]] = abi
                drive = kd.Drive(target, ADDRS)
                with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                    with self.assertRaisesRegex(RuntimeError, "ABI mismatch"):
                        kd.operate(drive, "wrong.elf", mock.Mock())
                self.assertEqual(target.writes, [])

    def test_unverified_transport_refuses_even_stop(self):
        target = FakeTarget()
        with self.assertRaisesRegex(RuntimeError, "writes forbidden"):
            kd.Drive(target, ADDRS).submit(kd.STOP)
        self.assertEqual(target.writes, [])


class TransportTests(unittest.TestCase):
    def test_signed_inputs_request_then_sequence_last_commit(self):
        target, drive = verified()
        sequence = drive.submit(kd.START, (-200, 150, -500))
        self.assertEqual(target.writes, [(ADDRS["vx_mmps"], 0xFFFFFF38),
                                         (ADDRS["vy_mmps"], 150),
                                         (ADDRS["omega_mradps"], 0xFFFFFE0C),
                                         (ADDRS["request"], kd.START),
                                         (ADDRS["sequence"], 1)])
        self.assertEqual(target.commands, [(kd.START, sequence, (-200, 150, -500))])
        commit = target.events.index(("write", ADDRS["sequence"], 1))
        self.assertEqual(target.events[commit - 1], ("flush",))
        self.assertEqual(drive.poll()["completed"], sequence)

    def test_rejects_float_and_out_of_range_input_without_writes(self):
        target, drive = verified()
        for vector in ((0.2, 0, 0), (1 << 31, 0, 0), (0, -(1 << 31) - 1, 0)):
            with self.assertRaises(ValueError):
                drive.submit(kd.START, vector)
        self.assertEqual(target.writes, [])

    def test_only_stop_can_supersede_pending_transaction(self):
        target, drive = verified(immediate=False)
        first = drive.submit(kd.PREPARE)
        for command in (kd.PREPARE, kd.START):
            with self.assertRaisesRegex(RuntimeError, "only STOP"):
                drive.submit(command)
        second = drive.submit(kd.STOP)
        self.assertNotEqual(first, second)
        self.assertEqual(requests(target), [kd.PREPARE, kd.STOP])
        self.assertEqual(target.writes[-2:], [(ADDRS["request"], kd.STOP),
                                             (ADDRS["sequence"], second)])
        target.complete()
        self.assertEqual(drive.poll()["completed"], second)

    def test_nonzero_request_does_not_block_completed_mailbox(self):
        target, drive = verified()
        drive.submit(kd.PREPARE)
        self.assertEqual(target.mem[ADDRS["request"]], kd.PREPARE)
        drive.submit(kd.START, (200, 0, 0))
        self.assertEqual(requests(target), [kd.PREPARE, kd.START])

    def test_sequence_wrap_skips_zero(self):
        target, drive = verified()
        target.mem[ADDRS["sequence"]] = 0xFFFFFFFF
        self.assertEqual(drive.submit(kd.STOP), 1)

    def test_all_four_rpm_outputs_signed_and_counters_visible(self):
        target, drive = verified()
        for index, value in enumerate((123, -456, -(1 << 31), (1 << 31) - 1)):
            target.mem[ADDRS["rpm"] + 4 * index] = value & 0xFFFFFFFF
        status = drive.poll()
        self.assertEqual(status["rpm"], [123, -456, -(1 << 31), (1 << 31) - 1])
        for name in ("state", "result", "tx_count", "ack_count", "fault", "completed", "sequence"):
            self.assertIn(name, status)


class TelemetryTests(unittest.TestCase):
    def test_unchanged_ram_ages_out_even_when_firmware_clock_freezes(self):
        target, drive = verified()
        telemetry_row(target)
        self.assertTrue(drive.poll_telem(0)['rows'][0]['current_valid'])
        for now in (1, 10):
            row = drive.poll_telem(now)['rows'][0]
            self.assertGreaterEqual(row['speed_age_ms'], 100 + now * 1000)
            self.assertFalse(row['current_valid'])
        # A resuming clock alone is not fresh CAN evidence either.
        target.mem[ADDRS['telem_now']] = 1100
        self.assertFalse(drive.poll_telem(11)['rows'][0]['current_valid'])
        telemetry_row(target, speed_tick=1100, current_tick=1100,
                      speed_count=11, current_count=21)
        target.mem[ADDRS['telem_now']] = 1100
        self.assertTrue(drive.poll_telem(12)['rows'][0]['current_valid'])

    def setUp(self):
        self.target, self.drive = verified()

    def row(self, **changes):
        telemetry_row(self.target, **changes)
        return self.drive.poll_telem(0)["rows"][0]

    def test_four_mapping_block_layout_signed_decode_and_flags(self):
        self.assertEqual(kd.WHEELS, (("FR", "NE", 1), ("BR", "SE", 2),
                                     ("BL", "SW", 4), ("FL", "NW", 8)))
        self.assertEqual(kd.TELEM_FIELDS,
                         ("generation", "rpm", "state", "hall", "hall_changes", "speed_tick",
                          "iq_counts", "id_counts", "flags", "peak_l1_counts", "current_tick",
                          "speed_count", "current_count"))
        for index in range(4):
            telemetry_row(self.target, index, rpm=-100 - index, iq_counts=-544 * (index + 1))
        result = self.drive.poll_telem(10)
        self.assertEqual(self.target.block_reads, [(ADDRS["telem"] + i * 52, 13) for i in range(4)])
        for index, (row, identity) in enumerate(zip(result["rows"], kd.WHEELS)):
            self.assertEqual((row["wheel"], row["compass"], row["selector"]), identity)
            self.assertEqual(row["rpm"], -100 - index)
            self.assertEqual(row["raw"]["iq_counts"], -544 * (index + 1))
            self.assertEqual(row["iq_a"], -(index + 1))
            self.assertEqual(row["id_a"], 1)
            self.assertEqual(row["peak_l1_counts"], 2000)
            self.assertTrue(all(row["flags"][name] for name in kd.FLAG_BITS))
            self.assertEqual(row["flags"]["faults"], 0)
            block = self.target.events.index(("block32", ADDRS["telem"] + index * 52, 13))
            self.assertEqual(self.target.events[block + 1], ("read", ADDRS["telem"] + index * 52))
        self.assertEqual(self.target.writes, [])

    def test_stable_retry_succeeds_on_third_attempt(self):
        base = telemetry_row(self.target)
        self.target.torn_reads[base] = 2
        row = self.drive.poll_telem(0)["rows"][0]
        self.assertTrue(row["available"])
        self.assertEqual(row["raw"]["generation"], 6)
        self.assertEqual(self.target.block_reads.count((base, 13)), 3)

    def test_final_torn_or_odd_is_unavailable_not_previous_row(self):
        for odd in (False, True):
            with self.subTest(odd=odd):
                target, drive = verified()
                base = telemetry_row(target)
                self.assertEqual(drive.poll_telem(0)["rows"][0]["rpm"], -123)
                if odd:
                    target.mem[base] = 3
                else:
                    target.torn_reads[base] = 3
                row = drive.poll_telem(0.11)["rows"][0]
                self.assertFalse(row["available"])
                self.assertIsNone(row["raw"])
                self.assertIsNone(row["rpm"])
                self.assertIsNone(row["iq_a"])
                self.assertEqual(target.block_reads.count((base, 13)), 4)

    def test_zero_reply_counts_are_none_not_measured_zero(self):
        row = self.row(speed_count=0, current_count=0, rpm=0, iq_counts=0, id_counts=0)
        for name in ("rpm", "state", "hall", "hall_changes", "iq_counts", "id_counts",
                     "peak_l1_counts", "speed_age_ms", "current_age_ms", "flags", "iq_a", "id_a"):
            self.assertIsNone(row[name], name)
        self.assertEqual(row["raw"]["rpm"], 0)
        self.assertFalse(row["current_valid"])
        text = "\n".join(kd.telemetry_lines(row, 0))
        self.assertIn("MEASURED=--", text)
        self.assertIn("NO REPLY", text)
        self.assertIn("INVALID", text)

    def test_independent_reply_ages_and_500ms_boundary(self):
        row = self.row(speed_tick=500, current_tick=499)
        self.assertEqual(row["speed_age_ms"], 500)
        self.assertEqual(row["current_age_ms"], 501)
        self.assertTrue(row["speed_fresh"])
        self.assertFalse(row["current_fresh"])
        self.assertIsNone(row["iq_a"])
        self.assertEqual(row["iq_counts"], -1088)
        telemetry_row(self.target, speed_tick=499, current_tick=500,
                      speed_count=11, current_count=21)
        row = self.drive.poll_telem(0.11)["rows"][0]
        self.assertFalse(row["speed_fresh"])
        self.assertTrue(row["current_fresh"])
        self.assertIsNone(row["iq_a"])
        telemetry_row(self.target, speed_tick=500, current_tick=500,
                      speed_count=12, current_count=22)
        self.assertTrue(self.drive.poll_telem(0.22)["rows"][0]["current_valid"])

    def test_reply_groups_are_independent_even_before_first_reply(self):
        row = self.row(speed_count=0)
        self.assertIsNone(row["rpm"])
        self.assertIsNone(row["speed_age_ms"])
        self.assertEqual(row["iq_counts"], -1088)
        self.assertTrue(row["current_fresh"])
        self.assertIsNone(row["iq_a"])
        telemetry_row(self.target, current_count=0)
        row = self.drive.poll_telem(1)["rows"][0]
        self.assertEqual(row["rpm"], -123)
        self.assertTrue(row["speed_fresh"])
        self.assertIsNone(row["iq_counts"])
        self.assertIsNone(row["current_age_ms"])
        self.assertIsNone(row["iq_a"])

    def test_wrap_safe_hal_ages(self):
        telemetry_row(self.target, speed_tick=0xFFFFFFF0, current_tick=0xFFFFFFE0)
        self.target.mem[ADDRS["telem_now"]] = 0x10
        row = self.drive.poll_telem(2)["rows"][0]
        self.assertEqual(row["speed_age_ms"], 32)
        self.assertEqual(row["current_age_ms"], 48)
        self.assertTrue(row["current_valid"])

    def test_expensive_rows_cached_but_monotonic_ages_advance(self):
        telemetry_row(self.target, speed_tick=510, current_tick=520)
        first = self.drive.poll_telem(10)
        second = self.drive.poll_telem(10.08)
        self.assertEqual(len(self.target.block_reads), 4)
        self.assertEqual(first["snapshot_monotonic"], 10)
        self.assertAlmostEqual(second["rows"][0]["speed_age_ms"], 570)
        self.assertAlmostEqual(second["rows"][0]["current_age_ms"], 560)
        self.assertFalse(second["rows"][0]["current_valid"])
        # Reading cache must not mutate the prior sample used by logging.
        self.assertEqual(first["rows"][0]["speed_age_ms"], 490)
        self.drive.poll_telem(10.11)
        self.assertEqual(len(self.target.block_reads), 8)

    def test_read_errors_cannot_leave_fresh_prior_values(self):
        telemetry_row(self.target)
        self.drive.poll_telem(0)
        with mock.patch.object(self.target, "read_memory_block32", side_effect=OSError("probe")):
            row = self.drive.poll_telem(0.11)["rows"][0]
        self.assertFalse(row["available"])
        self.assertEqual(row["read_error"], "probe")
        self.assertIsNone(row["rpm"])
        with mock.patch.object(self.drive, "read", side_effect=OSError("tick unavailable")):
            rows = self.drive.poll_telem(0.22)["rows"]
        self.assertTrue(all(not row["available"] for row in rows))

    def test_strict_current_gate_each_required_flag_state_fault_and_group(self):
        invalid = [dict(state=state) for state in (0, 5, 7, 10)]
        invalid += [dict(flags=0xF800 & ~kd.FLAG_BITS[name]) for name in ("profile", "cal", "active")]
        invalid += [dict(flags=0xF800 | fault) for fault in (1, 2, 0x400, 0x7FF)]
        invalid += [dict(speed_count=0), dict(current_count=0), dict(speed_tick=0), dict(current_tick=0)]
        for changes in invalid:
            with self.subTest(changes=changes):
                telemetry_row(self.target, **changes)
                words = self.target.read_memory_block32(ADDRS["telem"], 13)
                row = kd.decode_telem(words, kd.WHEELS[0], 1000)
                self.assertFalse(row["current_valid"])
                self.assertIsNone(row["iq_a"])
                self.assertIsNone(row["id_a"])
                self.assertEqual(row["raw"]["iq_counts"], -1088)
                self.assertTrue(row["invalid_reasons"])
        # Armed/everrun are displayed, not additional undocumented conversion gates.
        row = self.row(flags=0x9800)
        self.assertTrue(row["current_valid"])
        self.assertFalse(row["flags"]["armed"])
        self.assertFalse(row["flags"]["everrun"])

    def test_fast_poll_does_not_read_telemetry(self):
        self.drive.poll()
        self.assertEqual(self.target.block_reads, [])

    def test_signed_32bit_extremes_and_real_zero_with_fresh_reply(self):
        row = self.row(rpm=-(1 << 31), iq_counts=(1 << 31) - 1, id_counts=-(1 << 31))
        self.assertEqual(row["rpm"], -(1 << 31))
        self.assertEqual(row["iq_a"], ((1 << 31) - 1) / 544)
        self.assertEqual(row["id_a"], -(1 << 31) / 544)
        telemetry_row(self.target, rpm=0, iq_counts=0, id_counts=0,
                      speed_count=11, current_count=21)
        row = self.drive.poll_telem(1)["rows"][0]
        self.assertEqual(row["rpm"], 0)
        self.assertEqual(row["iq_a"], 0.0)


class KeyboardTests(unittest.TestCase):
    def setUp(self):
        self.target, self.drive = verified()
        self.controller = kd.Controller(self.drive)

    def tick(self, keys, now):
        return self.controller.tick(list(keys), now)

    def test_each_key_is_single_axis_and_q_is_positive_yaw_not_quit(self):
        # Physical body frame from rover_kinematics.h: x right, y forward.
        vectors = {"w": (0, 200, 0), "s": (0, -200, 0), "a": (-200, 0, 0),
                   "d": (200, 0, 0), "q": (0, 0, 500), "e": (0, 0, -500)}
        for key, vector in vectors.items():
            with self.subTest(key=key):
                target, drive = verified()
                controller = kd.Controller(drive)
                self.assertTrue(controller.tick([key], 0))
                self.assertEqual(target.commands, [(kd.START, 1, vector)])

    def test_same_key_repeat_refreshes_timeout_without_repeat_start(self):
        for now in (0, 0.2, 0.4, 0.7, 1.0):
            self.tick("w", now)
        self.assertEqual(requests(self.target), [kd.START])
        self.tick("", 1.4)
        self.assertEqual(requests(self.target), [kd.START])

    def test_direction_change_stops_and_buffered_repeats_cannot_restart(self):
        self.tick("w", 0)
        self.tick("d", 0.2)
        for now in (0.4, 0.8, 1.1, 2.0):
            self.tick("dddd", now)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP])
        self.assertEqual(self.controller.requested, (0, 0, 0))
        self.tick("", 2.5)
        self.assertTrue(self.controller.blocked)
        self.tick("", 2.7)
        self.assertFalse(self.controller.blocked)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP])
        self.tick("d", 2.8)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP, kd.START])

    def test_timeout_stops_even_if_new_repeat_arrives_after_deadline(self):
        self.tick("w", 0)
        self.tick("w", 0.61)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP])
        self.assertTrue(self.controller.blocked)

    def test_release_timeout_stops_and_does_not_restart_without_new_key(self):
        self.tick("w", 0)
        self.tick("", 0.61)
        self.tick("", 1.3)
        self.tick("", 2.0)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP])

    def test_space_stops_and_inhibits_repeats(self):
        self.tick("w", 0)
        self.tick(" ww", 0.2)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP])
        self.assertTrue(self.controller.blocked)

    def test_prepare_only_arms_never_starts_or_reports_verified_readiness(self):
        self.tick("p", 0)
        self.tick("p", 0.2)
        self.tick("", 0.7)
        self.tick("", 10)
        self.assertEqual(requests(self.target), [kd.PREPARE])
        self.assertIn("Readiness NOT verified", self.controller.message)
        self.assertIsNone(self.controller.active_key)
        self.tick("w", 10.1)
        self.assertEqual(requests(self.target), [kd.PREPARE, kd.START])

    def test_pending_prepare_is_nonblocking_and_motion_is_not_queued(self):
        target, drive = verified(immediate=False)
        controller = kd.Controller(drive)
        controller.tick(["p"], 0)
        controller.tick(["w"], 0.1)
        controller.tick(["w"], 0.2)
        self.assertEqual(requests(target), [kd.PREPARE])
        target.complete()
        controller.tick(["w"], 0.4)
        self.assertEqual(requests(target), [kd.PREPARE])
        controller.tick([], 1.1)
        controller.tick(["w"], 1.2)
        self.assertEqual(requests(target), [kd.PREPARE, kd.START])

    def test_start_is_nonblocking_and_stop_supersedes_pending_start(self):
        target, drive = verified(immediate=False)
        controller = kd.Controller(drive)
        controller.tick(["w"], 0)
        controller.tick(["w"], 0.1)
        controller.tick(["d"], 0.2)
        self.assertEqual(requests(target), [kd.START, kd.STOP])
        target.complete()
        controller.tick(["d"], 1.0)
        self.assertEqual(requests(target), [kd.START, kd.STOP])

    def test_no_restart_until_stopping_completes_and_idle(self):
        self.tick("w", 0)
        self.target.immediate = False
        self.tick("d", 0.2)
        self.tick("", 0.9)
        self.assertTrue(self.controller.blocked)
        self.tick("d", 1.0)
        self.target.complete()
        self.tick("", 1.7)
        self.assertFalse(self.controller.blocked)
        self.tick("d", 1.8)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP, kd.START])

    def test_multiple_keys_are_not_combined_into_diagonal(self):
        self.tick("wd", 0)
        self.assertEqual(requests(self.target), [kd.START, kd.STOP])
        self.assertEqual(self.target.commands[0][2], (0, 200, 0))

    def test_completed_nonzero_request_does_not_trigger_startup_stop(self):
        self.drive.submit(kd.PREPARE)
        self.tick("", 0)
        self.assertEqual(requests(self.target), [kd.PREPARE])

    def test_existing_pending_transaction_gets_stop_only_on_startup(self):
        self.target.immediate = False
        self.drive.submit(kd.PREPARE)
        self.tick("w", 0)
        self.assertEqual(requests(self.target), [kd.PREPARE, kd.STOP])

    def test_rejection_fault_or_transaction_timeout_exits_without_retry(self):
        for result in (kd.REJECTED, kd.TX_FAULT, kd.BAD_TARGET):
            with self.subTest(result=result):
                target, drive = verified(immediate=False)
                controller = kd.Controller(drive)
                controller.tick(["w"], 0)
                target.complete(result)
                with self.assertRaisesRegex(RuntimeError, "no automatic retry"):
                    controller.tick(["w"], 0.1)
                self.assertEqual(requests(target), [kd.START])
        target, drive = verified(immediate=False)
        controller = kd.Controller(drive)
        controller.tick(["p"], 0)
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            controller.tick([], 5.1)
        self.assertEqual(requests(target), [kd.PREPARE])

    def test_fault_or_unexpected_host_stop_never_rearms(self):
        self.tick("w", 0)
        self.tick("w", 0.1)
        self.target.mem[ADDRS["state"]] = kd.IDLE
        with self.assertRaisesRegex(RuntimeError, "no automatic restart"):
            self.tick("w", 0.2)
        self.assertEqual(requests(self.target), [kd.START])
        self.target.mem[ADDRS["state"]] = kd.FAULT
        with self.assertRaisesRegex(RuntimeError, "FAULT"):
            self.tick("w", 0.3)

    def test_escape_and_control_c_quit_without_processing_buffered_motion(self):
        for key in ("\x1b", "\x03"):
            self.assertFalse(self.tick("w" + key, 0))
        self.assertEqual(requests(self.target), [])


class CleanupTests(unittest.TestCase):
    def test_failed_cleanup_outcomes_are_recorded_without_retry(self):
        target, drive = verified(immediate=False)
        events = []
        drive.logger = mock.Mock()
        drive.logger.write.side_effect = lambda kind, **data: events.append((kind, data))
        now = [0.0]
        def sleep(seconds):
            now[0] += seconds
        kd.finish_stop(drive, report=lambda _: None, timeout=0.2,
                       clock=lambda: now[0], sleep=sleep)
        self.assertEqual(requests(target), [kd.STOP])
        failure = [data for kind, data in events if kind == 'cleanup_stop_failed']
        self.assertEqual(failure[0]['reason'], 'timeout')
        self.assertEqual(failure[0]['sequence'], 1)
        events.clear()
        with mock.patch.object(drive, 'submit', side_effect=OSError('disconnected')):
            kd.finish_stop(drive, report=lambda _: None)
        failure = [data for kind, data in events if kind == 'cleanup_stop_failed']
        self.assertEqual(failure[0]['reason'], 'transport_or_interrupt')
        self.assertIn('disconnected', failure[0]['error'])

    def test_stop_on_normal_exit_exception_and_keyboard_interrupt(self):
        for failure in (None, ValueError("UI failure"), KeyboardInterrupt()):
            with self.subTest(failure=failure):
                target = FakeTarget()
                drive = kd.Drive(target, ADDRS)
                reports = []
                def body(transport):
                    transport.submit(kd.START, (200, 0, 0))
                    if failure is not None:
                        raise failure
                with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                    if failure is None:
                        kd.operate(drive, "selected.elf", body, reports.append)
                    else:
                        with self.assertRaises(type(failure)):
                            kd.operate(drive, "selected.elf", body, reports.append)
                self.assertEqual(requests(target), [kd.START, kd.STOP])
                self.assertIn("CAN TX ACK only", reports[-1])
                self.assertIn("physical stop NOT verified", reports[-1])

    def test_cleanup_supersedes_pending_operation(self):
        target, drive = verified(immediate=False)
        drive.submit(kd.START, (200, 0, 0))
        def sleep(_):
            target.complete()
        reports = []
        kd.finish_stop(drive, reports.append, sleep=sleep)
        self.assertEqual(requests(target), [kd.START, kd.STOP])
        self.assertIn("transaction completed", reports[-1])

    def test_cleanup_waits_for_delayed_stop_completion_not_state_alone(self):
        target, drive = verified(immediate=False)
        now = [0.0]
        reports = []
        def sleep(seconds):
            now[0] += seconds
            if now[0] >= 0.8 and target.current:
                target.complete()
        kd.finish_stop(drive, reports.append, clock=lambda: now[0], sleep=sleep)
        self.assertGreaterEqual(now[0], 0.8)
        self.assertLess(now[0], 2)
        self.assertEqual(requests(target), [kd.STOP])
        self.assertIn("CAN TX ACK only", reports[-1])

    def test_cleanup_timeout_and_transport_error_do_not_retry(self):
        target, drive = verified(immediate=False)
        reports = []
        now = [0.0]
        def sleep(seconds):
            now[0] += seconds
        kd.finish_stop(drive, reports.append, timeout=2, clock=lambda: now[0], sleep=sleep)
        self.assertEqual(requests(target), [kd.STOP])
        self.assertIn("timed out", reports[-1])
        self.assertLess(now[0], 2.1)
        with mock.patch.object(drive, "submit", side_effect=OSError("disconnected")) as submit:
            kd.finish_stop(drive, reports.append)
            submit.assert_called_once_with(kd.STOP)
        self.assertIn("No retry", reports[-1])


class LogAndMonitorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir="/tmp/opencode")
        self.addCleanup(self.temp.cleanup)
        self.elf = pathlib.Path(self.temp.name) / "selected.elf"
        self.elf.write_bytes(b"explicit ELF identity")
        self.path = pathlib.Path(self.temp.name) / "events.jsonl"

    def records(self, path=None):
        return [json.loads(line) for line in (path or self.path).read_text().splitlines()]

    def run_ui(self, drive, frames, monitor_only=False):
        import types
        now = [0.0]
        class Screen:
            index = 0
            def nodelay(self, enabled):
                pass
            def getch(self):
                return ord(frames[self.index].pop(0)) if frames[self.index] else -1
            def erase(self):
                pass
            def getmaxyx(self):
                return 40, 180
            def addnstr(self, *args):
                pass
            def refresh(self):
                self.index += 1
                now[0] += 0.11
        with mock.patch.dict(sys.modules, {"curses": types.SimpleNamespace(flushinp=lambda: None)}), \
                mock.patch.object(kd.time, "monotonic", side_effect=lambda: now[0]), \
                mock.patch.object(kd.time, "sleep"):
            kd.run_ui(Screen(), drive, 0.2, 0.5, monitor_only)

    def test_jsonl_header_sha_units_sample_keys_submissions_and_outcomes(self):
        target, drive = verified()
        for index in range(4):
            telemetry_row(target, index)
        with kd.JsonlLog(self.path, self.elf) as logger:
            drive.logger = logger
            with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                kd.operate(drive, self.elf,
                           lambda d: self.run_ui(d, [[], ["w"], [], [" "], ["\x1b"]]),
                           report=lambda text: None)
        self.assertTrue(logger.stream.closed)
        records = self.records()
        header = records[0]
        self.assertEqual(header["event"], "session")
        self.assertEqual(header["elf_path"], str(self.elf))
        self.assertEqual(header["elf_sha256"], hashlib.sha256(self.elf.read_bytes()).hexdigest())
        self.assertEqual(header["abi"], 0x494B0002)
        self.assertEqual(header["abi_version"], 2)
        self.assertEqual(header["units"]["counts_per_amp"], 544)
        self.assertIn("provisional", " ".join(header["notes"]))
        self.assertIn("offsets not independently validated", " ".join(header["notes"]))
        samples = [r for r in records if r["event"] == "sample"]
        self.assertEqual(len(samples), 4)
        self.assertEqual(samples[0]["requested_twist"], [0, 0, 0])
        self.assertEqual(samples[1]["requested_twist"], [0, 200, 0])
        self.assertEqual(samples[3]["requested_twist"], [0, 0, 0])
        self.assertEqual(len(samples[0]["telemetry"]["rows"]), 4)
        self.assertEqual(samples[0]["telemetry"]["rows"][0]["raw"]["iq_counts"], -1088)
        self.assertTrue(samples[0]["telemetry"]["rows"][0]["current_valid"])
        self.assertIn("rpm", samples[0]["host_status"])
        self.assertIn("mailbox_twist", samples[0]["host_status"])
        self.assertEqual([r["keys"] for r in records if r["event"] == "keys"], [["w"], [" "], ["\x1b"]])
        for event in ("mailbox_submission", "mailbox_committed", "mailbox_outcome"):
            self.assertEqual(len([r for r in records if r["event"] == event]), 3)
        for record in records:
            self.assertIsInstance(record["monotonic"], (float, int))
            self.assertIn("+00:00", record["utc"])
        self.assertEqual(requests(target), [kd.START, kd.STOP, kd.STOP])

    def test_log_is_exclusive_never_truncates_and_default_is_fresh(self):
        self.path.write_text("user evidence\n")
        with self.assertRaises(FileExistsError):
            kd.JsonlLog(self.path, self.elf)
        self.assertEqual(self.path.read_text(), "user evidence\n")
        with kd.JsonlLog(None, self.elf) as first, kd.JsonlLog(None, self.elf) as second:
            self.addCleanup(first.path.unlink)
            self.addCleanup(second.path.unlink)
            self.assertNotEqual(first.path, second.path)
            self.assertEqual(first.path.parent, pathlib.Path("/tmp/opencode"))
            self.assertRegex(first.path.name, r"^ik-telemetry-\d{8}T\d{6}\.\d+Z-\d+\.jsonl$")

    def test_log_closes_on_body_exception(self):
        with self.assertRaisesRegex(ValueError, "failure"):
            with kd.JsonlLog(self.path, self.elf) as logger:
                raise ValueError("failure")
        self.assertTrue(logger.stream.closed)

    def test_failed_logging_stops_then_exits_not_drives_without_log(self):
        target, drive = verified()
        reports = []
        with kd.JsonlLog(self.path, self.elf) as logger:
            drive.logger = logger
            with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                def body(d):
                    d.submit(kd.START, (0, 200, 0))
                    logger.stream.close()  # Persistent I/O failure, including cleanup events.
                    d.event("sample", requested_twist=(0, 200, 0))
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, "logging failed"):
                        kd.operate(drive, self.elf, body, reports.append)
        self.assertEqual(requests(target), [kd.START, kd.STOP])
        self.assertIn("transaction completed", reports[-1])

    def test_failed_submission_log_blocks_start_but_not_cleanup_stop(self):
        target, drive = verified()
        drive.logger = mock.Mock()
        drive.logger.write.side_effect = OSError("disk full")
        # Verification logging failure itself also enters safe STOP cleanup.
        with mock.patch.object(kd, "load_flash_segments", return_value=FLASH), \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(OSError):
                kd.operate(drive, self.elf, lambda d: d.submit(kd.START), report=lambda text: None)
        self.assertEqual(requests(target), [kd.STOP])
        target, drive = verified()
        drive.logger = mock.Mock()
        drive.logger.write.side_effect = OSError("disk full")
        with self.assertRaises(OSError):
            drive.submit(kd.START)
        self.assertEqual(target.writes, [])

    def test_readonly_never_prepares_starts_or_stops_even_busy_fault_or_exit(self):
        for state in (kd.IDLE, kd.BUSY, kd.RUNNING, kd.STOPPING, kd.FAULT):
            with self.subTest(state=state):
                target, drive = verified()
                target.mem[ADDRS["state"]] = state
                target.mem[ADDRS["sequence"]] = 7
                with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                    kd.operate(drive, self.elf,
                               lambda d: self.run_ui(d, [["w", "p", " "], [], ["\x1b"]], True),
                               monitor_only=True)
                self.assertEqual(target.writes, [])
                for command in (kd.PREPARE, kd.START, kd.STOP):
                    with self.assertRaisesRegex(RuntimeError, "read-only"):
                        drive.submit(command)

    def test_readonly_zero_motion_readback_logged_without_fake_telemetry_zeros(self):
        target, drive = verified()
        with kd.JsonlLog(self.path, self.elf, monitor_only=True) as logger:
            drive.logger = logger
            with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                kd.operate(drive, self.elf,
                           lambda d: self.run_ui(d, [[], ["w", "p", " "], ["\x1b"]], True),
                           monitor_only=True)
        records = self.records()
        self.assertTrue(records[0]["monitor_only"])
        samples = [r for r in records if r["event"] == "sample"]
        self.assertEqual(len(samples), 2)
        self.assertIsNone(samples[0]["requested_twist"])
        self.assertEqual(samples[0]["host_status"]["rpm"], [0, 0, 0, 0])
        for row in samples[0]["telemetry"]["rows"]:
            self.assertIsNone(row["rpm"])
            self.assertIsNone(row["iq_a"])
        self.assertFalse(any(r["event"].startswith("mailbox_") for r in records))
        self.assertEqual(target.writes, [])

    def test_readonly_exception_and_keyboard_interrupt_have_zero_cleanup_writes(self):
        for failure in (ValueError("failure"), KeyboardInterrupt()):
            target, drive = verified()
            def body(d):
                raise failure
            with mock.patch.object(kd, "load_flash_segments", return_value=FLASH):
                with self.assertRaises(type(failure)):
                    kd.operate(drive, self.elf, body, monitor_only=True)
            kd.finish_stop(drive, report=lambda text: None)
            self.assertEqual(target.writes, [])


class CliTests(unittest.TestCase):
    def test_default_elf_selects_verified_high_cap_installation(self):
        self.assertEqual(kd.DEFAULT_ELF,
                         ROOT / 'CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf')

    def test_float_ranges_allow_fractional_values_but_not_nan_or_inf(self):
        for low, high in ((0.05, 5.0), (0.1, 1.0)):
            parse = kd.bounded_float(low, high)
            self.assertEqual(parse(str(low)), low)
            self.assertEqual(parse(str(high)), high)
            for value in ("nan", "inf", "-inf", str(low - 0.001), str(high + 0.001)):
                with self.assertRaises(kd.argparse.ArgumentTypeError):
                    parse(value)
        target, drive = verified()
        kd.Controller(drive, speed=0.125, yaw=0.375).tick(["s"], 0)
        self.assertEqual(target.commands[0][2], (0, -125, 0))

    def test_cli_rejects_invalid_speed_before_probe_import(self):
        for args in (("--speed", "0.01"), ("--speed", "5.001"),
                     ("--speed", "nan"), ("--speed", "inf"), ("--yaw", "1.1")):
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    kd.main(list(args))
            self.assertEqual(error.exception.code, 2)

    def test_cli_accepts_tournament_speeds_before_hardware_access(self):
        self.assertEqual(kd.MAX_LINEAR_SPEED, 5.0)
        for speed in (0.05, 0.3, 1, 3, 5):
            with mock.patch.object(kd, "resolve_addrs", side_effect=RuntimeError("parsed offline")):
                with self.assertRaisesRegex(RuntimeError, "parsed offline"):
                    kd.main(["--speed", str(speed)])

    def test_high_speed_keys_publish_signed_mmps_and_keep_yaw_independent(self):
        vectors={"w": (0,5000,0), "s": (0,-5000,0), "a": (-5000,0,0),
                 "d": (5000,0,0), "q": (0,0,500), "e": (0,0,-500)}
        for key, expected in vectors.items():
            target, drive = verified()
            kd.Controller(drive, speed=5).tick([key], 0)
            self.assertEqual(target.commands[0][2], expected)
            self.assertEqual(requests(target), [kd.START])

    def test_help_exposes_log_and_readonly_options_without_probe_import(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), self.assertRaises(SystemExit) as result:
            kd.main(["--help"])
        self.assertEqual(result.exception.code, 0)
        self.assertIn("--log", output.getvalue())
        self.assertIn("--monitor-only", output.getvalue())
        self.assertIn("no prepare/start/STOP", output.getvalue())

    def test_import_is_hardware_dependency_free(self):
        self.assertNotIn("pyocd", sys.modules)


if __name__ == "__main__":
    unittest.main(verbosity=2)
