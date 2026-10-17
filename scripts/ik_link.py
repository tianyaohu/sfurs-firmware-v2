"""SWD link to the Nucleo streaming IK image (ABI 0x494B0004).

Resolves the ik_drive_* mailbox and telemetry rows from the flashed ELF,
writes setpoints/heartbeat, sends one-shot raw ESC frames, and decodes the
per-wheel telemetry. Pure helpers are importable without pyOCD.
"""
import pathlib
import re
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_ELF = ROOT / "build/drive/sfurs_firmware_v2_CM7.elf"  # scripts/build_all.py
DEFAULT_SN = "002200364142500620353451"
TARGET = "stm32h755zitx"
ABI = 0x494B0004
WHEELS = (("FR", "NE", 1), ("BR", "SE", 2), ("BL", "SW", 4), ("FL", "NW", 8))
STATES = {0: "STOPPED", 1: "DRIVING", 2: "STOPPING"}
MC_STATES = {0: "IDLE", 4: "START", 6: "RUN", 8: "STOP", 10: "FAULT_NOW", 11: "FAULT_OVER",
             12: "WAIT_STOP", 16: "CHARGE_BOOT_CAP", 17: "OFFSET_CALIB"}
STOP_REASONS = {0: "-", 1: "stop", 2: "timeout", 3: "fault", 4: "align", 5: "capture"}
ALIGN_RESULTS = {0: "-", 1: "ok", 2: "REVERSED", 3: "NO_EDGES", 4: "aborted", 5: "running"}
FAULT_BITS = {0x01: "duration", 0x02: "overvolt", 0x04: "undervolt", 0x08: "overtemp",
              0x10: "startup", 0x20: "speed_fdbk", 0x40: "overcurrent", 0x80: "sw_error"}
COUNTS_PER_AMP = 544
# can_datatypes.h
SPEED_WRITE, STATUS_READ, DIAG_READ, HALL_ALIGN, CONFIG_SET, MOTOR_STOP = (
    0x080, 0x100, 0x300, 0x480, 0x500, 0x780)
IDENT_READ, CAPTURE = 0x180, 0x200
CAPTURE_START_OP, CAPTURE_READ_OP, CAPTURE_INFO_BANK = 0xC5, 0x3D, 3
CAPTURE_SAMPLES, CAPTURE_PRE, CAPTURE_HOLD_MA, CAPTURE_READ_MAX = 512, 64, 1000, 32
CAPTURE_STATUS = {0: "none", 1: "running", 2: "done", 3: "aborted", 4: "rejected"}
CAPTURE_HEADER = ("frames", "source", "status", "vbus_centivolts", "samples")
CAPTURE_CHANNELS = ("id", "iq", "vd", "vq", "id_ref", "angle")
HALL_ALIGN_MAGIC = 0x4E4C4148
CONFIG_KEYS = {"kp": 1, "ki": 2, "shift": 3, "iqmax_ma": 4, "current_kp": 5, "current_ki": 6,
               "ff_static_ma": 7, "ff_kinetic_ma": 8}

SCALARS = ("abi", "vx_mmps", "vy_mmps", "omega_mradps", "enable", "heartbeat",
           "accel_rpmps", "raw_id", "raw_data", "raw_seq", "state", "raw_done",
           "tx_count", "tx_fail", "bad_input", "telem_now")
ARRAYS = ("target_rpm", "rpm")
ROW_FIELDS = ("generation", "rpm", "iq_counts", "iq_ref_counts", "state", "flags",
              "status_tick", "status_count", "vbus_centivolts", "faults", "hall_shift_deg",
              "align_result", "i2t_percent", "diag_tick", "diag_count",
              "app_crc", "app_size", "ident_tick", "ident_count")
SIGNED = {"rpm", "iq_counts", "iq_ref_counts", "hall_shift_deg", "vx_mmps", "vy_mmps",
          "omega_mradps", "accel_rpmps"}


def signed32(value):
    return value - (1 << 32) if value & 0x80000000 else value


def resolve_addrs(elf):
    """Map every ABI symbol to its address using the selected ELF."""
    elf = pathlib.Path(elf)
    if not elf.is_file():
        raise RuntimeError(f"IK ELF not found: {elf}")
    nm = shutil.which("arm-none-eabi-nm") or "/tmp/opencode/cm7-toolchain/root/usr/bin/arm-none-eabi-nm"
    out = subprocess.run([nm, "--defined-only", str(elf)], check=True,
                         capture_output=True, text=True, timeout=30).stdout
    addrs = {}
    for line in out.splitlines():
        m = re.fullmatch(r"([0-9a-fA-F]+)\s+\w\s+ik_drive_(\w+)", line.strip())
        if m:
            addrs[m[2]] = int(m[1], 16)
    missing = set(SCALARS + ARRAYS + ("telem", "capture")) - addrs.keys()
    if missing:
        raise RuntimeError("Not a streaming IK ELF; missing ik_drive_" + ", ik_drive_".join(sorted(missing)))
    return addrs


def decode_row(words):
    row = dict(zip(ROW_FIELDS, words))
    for name in SIGNED & row.keys():
        row[name] = signed32(row[name])
    row["iq_a"] = row["iq_counts"] / COUNTS_PER_AMP
    row["iq_ref_a"] = row["iq_ref_counts"] / COUNTS_PER_AMP
    row["vbus_v"] = row["vbus_centivolts"] / 100
    row["stop_reason"] = STOP_REASONS.get(row["flags"] & 0x0F, str(row["flags"] & 0x0F))
    row["active"] = bool(row["flags"] & 0x10)
    row["derated"] = bool(row["flags"] & 0x20)
    row["aligning"] = bool(row["flags"] & 0x40)
    row["aligned"] = bool(row["flags"] & 0x80)
    row["mc_state"] = MC_STATES.get(row["state"], str(row["state"]))
    row["fault_names"] = [n for bit, n in FAULT_BITS.items() if row["faults"] & bit]
    row["align"] = ALIGN_RESULTS.get(row["align_result"], str(row["align_result"]))
    return row


def capture_start_word(step_ma):
    if not 0 <= step_ma <= 6000:
        raise ValueError("step must be 0..6000 mA (IQMAX)")
    return (CAPTURE_START_OP << 24) | step_ma


def capture_read_word(bank, first, count):
    if bank not in (0, 1, CAPTURE_INFO_BANK) or not 0 <= first < CAPTURE_SAMPLES or \
            not 1 <= count <= CAPTURE_READ_MAX:
        raise ValueError("bad capture read")
    return (CAPTURE_READ_OP << 24) | (bank << 16) | (count << 10) | first


def config_frame(selector, key, value):
    if key not in CONFIG_KEYS:
        raise ValueError(f"unknown key {key}; one of {sorted(CONFIG_KEYS)}")
    if not -32768 <= value <= 32767:
        raise ValueError("value must fit int16")
    return CONFIG_SET | selector, CONFIG_KEYS[key] | ((value & 0xFFFF) << 16)


class Link:
    def __init__(self, target, addrs):
        self.target, self.addrs = target, addrs
        self.heartbeat = target.read32(addrs["heartbeat"])

    @classmethod
    def open(cls, sn=DEFAULT_SN, elf=DEFAULT_ELF):
        """Attach without halting or resetting. Returns (session, link)."""
        from pyocd.core.helpers import ConnectHelper
        addrs = resolve_addrs(elf)
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=sn, connect_mode="attach", target_override=TARGET, blocking=False)
        if session is None:
            raise RuntimeError(f"Nucleo probe {sn} not found")
        session.open()
        link = cls(session.target, addrs)
        abi = link.read("abi")
        if abi != ABI:
            session.close()
            raise RuntimeError(f"IK ABI 0x{abi:08x} != 0x{ABI:08x}: flash {elf} first")
        return session, link

    def read(self, name):
        value = self.target.read32(self.addrs[name])
        return signed32(value) if name in SIGNED else value

    def write(self, name, value):
        self.target.write32(self.addrs[name], value & 0xFFFFFFFF)

    def drive(self, vx_mmps, vy_mmps, omega_mradps, enable=True):
        """Publish one setpoint and bump the heartbeat (call at >= 10 Hz)."""
        self.write("vx_mmps", vx_mmps)
        self.write("vy_mmps", vy_mmps)
        self.write("omega_mradps", omega_mradps)
        self.write("enable", 1 if enable else 0)
        self.heartbeat = (self.heartbeat + 1) & 0xFFFFFFFF
        self.write("heartbeat", self.heartbeat)

    def status(self):
        s = {name: self.read(name) for name in ("state", "tx_count", "tx_fail", "bad_input",
                                                 "raw_done", "telem_now")}
        s["state_name"] = STATES.get(s["state"], str(s["state"]))
        for name in ARRAYS:
            s[name] = [signed32(w) for w in self.target.read_memory_block32(self.addrs[name], 4)]
        return s

    def rows(self):
        """Four decoded telemetry rows (None if torn three times)."""
        rows = []
        for i in range(4):
            base = self.addrs["telem"] + i * 4 * len(ROW_FIELDS)
            row = None
            for _ in range(3):
                words = self.target.read_memory_block32(base, len(ROW_FIELDS))
                if words[0] == self.target.read32(base) and not words[0] & 1:
                    row = decode_row(words)
                    break
            rows.append(row)
        return rows

    def capture_header(self):
        return dict(zip(CAPTURE_HEADER, self.target.read_memory_block32(
            self.addrs["capture"], len(CAPTURE_HEADER))))

    def _wait_frames(self, count, timeout):
        import time
        deadline = time.monotonic() + timeout
        while self.capture_header()["frames"] < count:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.005)
        return True

    def capture(self, selector, step_ma, timeout=3.0):
        """Run one current-loop step test on one wheel (IDLE, battery on) and
        read it back. Returns (info, samples): samples is a list of dicts
        with CAPTURE_CHANNELS in counts / s16V."""
        import struct
        import time
        self.send_raw(CAPTURE | selector, capture_start_word(step_ma))
        deadline = time.monotonic() + timeout
        while True:
            time.sleep(0.05)
            before = self.capture_header()["frames"]
            self.send_raw(CAPTURE | selector, capture_read_word(CAPTURE_INFO_BANK, 0, 1))
            if self._wait_frames(before + 1, 0.3):
                info = self.capture_header()
                if info["status"] != 1:
                    break
            if time.monotonic() > deadline:
                raise RuntimeError("capture did not finish")
        info["status_name"] = CAPTURE_STATUS.get(info["status"], str(info["status"]))
        if info["status"] != 2:
            raise RuntimeError(f"capture {info['status_name']}")
        for bank in (0, 1):
            for first in range(0, CAPTURE_SAMPLES, CAPTURE_READ_MAX):
                for _attempt in range(3):  # CAN frames are not retransmitted
                    before = self.capture_header()["frames"]
                    self.send_raw(CAPTURE | selector, capture_read_word(bank, first, CAPTURE_READ_MAX))
                    if self._wait_frames(before + CAPTURE_READ_MAX, 0.5):
                        break
                else:
                    raise RuntimeError(f"capture readout bank {bank} @{first} incomplete")
        raw = bytes(self.target.read_memory_block8(
            self.addrs["capture"] + 4 * len(CAPTURE_HEADER), CAPTURE_SAMPLES * 12))
        samples = [dict(zip(CAPTURE_CHANNELS, struct.unpack_from("<6h", raw, 12 * i)))
                   for i in range(CAPTURE_SAMPLES)]
        return info, samples

    def send_raw(self, can_id, word, timeout=1.0):
        """One 4-byte frame through the Nucleo; only accepted while STOPPED."""
        import time
        if self.read("state") != 0:
            raise RuntimeError("drive not stopped; raw frames are refused")
        seq = (self.read("raw_seq") + 1) & 0xFFFFFFFF
        self.write("raw_id", can_id)
        self.write("raw_data", word)
        self.write("raw_seq", seq)
        deadline = time.monotonic() + timeout
        while self.read("raw_done") != seq:
            if time.monotonic() > deadline:
                raise RuntimeError(f"raw frame 0x{can_id:03x} not sent")
            time.sleep(0.005)
