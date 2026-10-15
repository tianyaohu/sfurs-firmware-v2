#!/usr/bin/env python3
"""Hardware-free IK TUI/transport tests; no pyOCD or compiler dependency."""
import contextlib
import importlib.util
import io
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
ADDRS = {name: 0x24000000 + index * 32 for index, name in enumerate(kd.FIELDS)}
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

    def read32(self, address):
        self.events.append(("read", address))
        return self.mem.get(address, 0)

    def read_memory_block8(self, address, length):
        self.flash_reads.append((address, length))
        self.events.append(("flash", address, length))
        return [self.flash.get(address + offset, 0) for offset in range(length)]

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
        target = FakeTarget()
        target.mem[ADDRS["abi"]] = kd.ABI + 1
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


class CliTests(unittest.TestCase):
    def test_float_ranges_allow_fractional_values_but_not_nan_or_inf(self):
        for low, high in ((0.05, 0.3), (0.1, 1.0)):
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
        for args in (("--speed", "0.01"), ("--yaw", "1.1")):
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    kd.main(list(args))
            self.assertEqual(error.exception.code, 2)

    def test_import_is_hardware_dependency_free(self):
        self.assertNotIn("pyocd", sys.modules)


if __name__ == "__main__":
    unittest.main(verbosity=2)
