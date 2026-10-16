#!/usr/bin/env python3
"""Read-only NE power/current logger for ONE audited, installed ESC image.

Before live use obtain an explicit ownership handoff for NE probe
066AFF525266555067075126 and confirm target/power with the operator. A free
process list is not permission. The Nucleo TUI may retain its DIFFERENT probe;
this tool never owns/commands CAN. Do not share the ESC probe with another tool.

No flash, motor RAM writes, CAL, fault clear, STOP, halt, reset, or resume,
including on exit. pyOCD attach still initializes debugger infrastructure, so
this is not a promise of electrically passive or zero-debug-register access.
Disconnect does not restore debug registers or command the motor to stop.

Live dependencies: pyelftools, pyocd, installed STM32G4 CMSIS device pack
(offline `python3 -m pyocd list --targets` lists stm32g431cbux here).
--self-test is host-only and needs neither pyOCD nor hardware.
"""

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import unittest
import zlib

sys.dont_write_bytecode = True

DEFAULT_MANIFEST = Path(
    "/home/fh/sfurs_firmware/esc-backups/ne-fwu-rollout-20261014/NE-fixed.json")
EXPECTED_ELF_SHA256 = "a84a0dc7c40bcaad55b7e58f33639c8d4fdff539d28f005c4e5b76df821d9b95"
EXPECTED_APP_SHA256 = "4625cf66572fb97c5137239b8b13dc5fdd7f6ced334853affe13c3a85bab74dc"
EXPECTED_BACKUP_SHA256 = "4b684c516b3081d6035de7d8aa565bea4896b745f5f1a0e530bf25048c1bddd6"
EXPECTED_SOURCE_COMMIT = "a328121ec6eaf29b36febc4d5bc85b88f017290e"
NE_PROBE = "066AFF525266555067075126"
TARGET = "stm32g431cbux"
APP_BASE, APP_SIZE, APP_CRC = 0x08004000, 40860, 0xB93E5A8A
METADATA_ADDRESS, VTOR_ADDRESS = 0x0801F800, 0xE000ED08
METADATA = (0x56414C49, 0xA9BEB3B6, 0x9F9C, APP_CRC, APP_BASE, 2, 0, 0)
RAM_BASE, RAM_END = 0x20000000, 0x20008000
CURRENT_SCALE = 0.001836396986618638
AUDIT_PROFILE_NAME = "baseline"
ALLOW_PREPARED_MANIFEST = False
# Extra RAM objects linked only into the rpm2-tune (ESC_TUNE_GAIN_SET) image:
# live GAIN_SET acceptance/applied-numerator readback. Profile-scoped so the
# baseline/P2 symbol requirements stay exactly as audited.
TUNE_OBJECT_FIELDS = {
    "test_speed_kp_gain": (4, [("speed_kp_gain", 0, "i")]),
    "test_speed_ki_gain": (4, [("speed_ki_gain", 0, "i")]),
    "test_gain_set_result": (4, [("gain_set_result", 0, "I")]),
    "test_gain_set_count": (4, [("gain_set_count", 0, "I")]),
}
# Baseline/P2/900-P2 share the audited SDK field layout and diagnostic addresses.
# The original P2 app differs from baseline only in two PID initializer bytes;
# the separate 900-P2 app changes speed bounds/guard and has its own size/hash.
# Explicit selection is required; unknown/rebuilt ELF hashes still fail closed.
AUDITED_PROFILES = {
    "baseline": {
        "EXPECTED_ELF_SHA256": EXPECTED_ELF_SHA256,
        "EXPECTED_APP_SHA256": EXPECTED_APP_SHA256,
        "EXPECTED_BACKUP_SHA256": EXPECTED_BACKUP_SHA256,
        "EXPECTED_SOURCE_COMMIT": EXPECTED_SOURCE_COMMIT,
        "APP_CRC": APP_CRC, "APP_SIZE": APP_SIZE, "METADATA": METADATA,
        "ALLOW_PREPARED_MANIFEST": False, "AUDIT_PROFILE_NAME": "baseline",
    },
    "ne-low-speed-p2": {
        "EXPECTED_ELF_SHA256": "ef107830180d33feb6ccf7ee9d1ba29b9443205624652c5397ca22405399d2a0",
        "EXPECTED_APP_SHA256": "9beab39c5b6828c4a5f401c2173424b412865a81eacc38e8457017808a49f050",
        "EXPECTED_BACKUP_SHA256": "e204a38aba09e4cbe22c79c5274a7ca1bb23e7c6905d22509b1279be0a7e8fd2",
        "EXPECTED_SOURCE_COMMIT": "572d4d8dc88a6c06aa645b26a0d7113a5cdf7d10",
        "APP_CRC": 0x43ACF5E2, "APP_SIZE": APP_SIZE,
        "METADATA": (0x56414C49, 0xA9BEB3B6, APP_SIZE, 0x43ACF5E2, APP_BASE, 2, 0, 0),
        "ALLOW_PREPARED_MANIFEST": True, "AUDIT_PROFILE_NAME": "ne-low-speed-p2",
    },
    # Offline cap-only candidate, not an installation claim. Full live app,
    # metadata and VTOR checks remain mandatory before any RAM sampling.
    "ne-900-p2": {
        "EXPECTED_ELF_SHA256": "8f0522236a9f6676bdf1617e0b39f343185b9680aa842050e20d7e53e8885dcc",
        "EXPECTED_APP_SHA256": "333b0772e0d39679ed9ca35b8830c71b62cb7b5d345fecc19af54a6125516ec4",
        "EXPECTED_BACKUP_SHA256": "42c91675757048cc07570e14932b5cfdebcc19aa9c5832087f59970c8ac7a1b5",
        "EXPECTED_SOURCE_COMMIT": "572d4d8dc88a6c06aa645b26a0d7113a5cdf7d10",
        "APP_CRC": 0x3725CEE8, "APP_SIZE": 40868,
        "METADATA": (0x56414C49, 0xA9BEB3B6, 40868, 0x3725CEE8, APP_BASE, 2, 0, 0),
        "ALLOW_PREPARED_MANIFEST": True, "AUDIT_PROFILE_NAME": "ne-900-p2",
    },
    # Original RPM2 gains restored, 900 RPM bounds retained. Explicit image
    # identity and full live verification are required; not a tuning verdict.
    "ne-900-stock": {
        "EXPECTED_ELF_SHA256": "b131f86e2d3154735823ba227011f777ffcd4ceab166d6c0f95fea7e18effaa7",
        "EXPECTED_APP_SHA256": "0bb917ba9807af0aede7226b14a08d20c4c51b9161d0d150e3d821b5c234edd8",
        "EXPECTED_BACKUP_SHA256": "df1053892346557720d8c143e2876f9672429c9c1b0c5dcda03a8919fbb42e6a",
        "EXPECTED_SOURCE_COMMIT": "572d4d8dc88a6c06aa645b26a0d7113a5cdf7d10",
        "APP_CRC": 0xCDB76180, "APP_SIZE": 40868,
        "METADATA": (0x56414C49, 0xA9BEB3B6, 40868, 0xCDB76180, APP_BASE, 2, 0, 0),
        "ALLOW_PREPARED_MANIFEST": True, "AUDIT_PROFILE_NAME": "ne-900-stock",
    },
    # Ki-only candidate, audited against stock: exactly the two Ki initializer
    # bytes differ. Expected full-flash hash is a prediction until readback;
    # prepared manifests never bypass full LIVE app/metadata/VTOR checks.
    "ne-900-ki4": {
        "EXPECTED_ELF_SHA256": "dfab08bdcdb127dc649de673c24eecd42b1846595d9cc7a74ca0cb1daaeb376c",
        "EXPECTED_APP_SHA256": "e77392a7f03732ffcc8970196a91e928fc9da92a3200264c7e0dc4b535204557",
        "EXPECTED_BACKUP_SHA256": "adb72c13d3594b8cefd886dbf7c90d33da49812a7d6019e2085db88e987f8c9c",
        "EXPECTED_SOURCE_COMMIT": "572d4d8dc88a6c06aa645b26a0d7113a5cdf7d10",
        "APP_CRC": 0x75A5B016, "APP_SIZE": 40868,
        "METADATA": (0x56414C49, 0xA9BEB3B6, 40868, 0x75A5B016, APP_BASE, 2, 0, 0),
        "ALLOW_PREPARED_MANIFEST": True, "AUDIT_PROFILE_NAME": "ne-900-ki4",
    },
    # RPM2 + runtime CAN GAIN_SET (auto-tune sweep image). NOTE: this tree's
    # RPM2 profile carries the +-900 RPM bounds, so even a plain rebuild differs
    # from the a328121 baseline app. EXPECTED_BACKUP_SHA256 is pinned after the
    # first verified flash; until then every real manifest fails closed.
    "rpm2-tune": {
        "EXPECTED_ELF_SHA256": "5af6e6a978a59f0aa7e66c0b6ea880e7f3b20589b163bd2887ef22eb0c13a594",
        "EXPECTED_APP_SHA256": "da1e3475e3c7bcd70845d9300fa256b232025ff71c55b75ad7e67a374579de07",
        "EXPECTED_BACKUP_SHA256": "PENDING-FIRST-FLASH",
        "EXPECTED_SOURCE_COMMIT": "572d4d8dc88a6c06aa645b26a0d7113a5cdf7d10",
        "APP_CRC": 0x9976A657, "APP_SIZE": 41068,
        "METADATA": (0x56414C49, 0xA9BEB3B6, 41068, 0x9976A657, APP_BASE, 2, 0, 0),
        "ALLOW_PREPARED_MANIFEST": True, "AUDIT_PROFILE_NAME": "rpm2-tune",
        "EXTRA_OBJECT_FIELDS": TUNE_OBJECT_FIELDS,
    },
}


@contextmanager
def audited_profile(name):
    """Select one frozen identity for this single-threaded CLI invocation."""
    selected = AUDITED_PROFILES[name]
    extra = selected.get("EXTRA_OBJECT_FIELDS", {})
    plain = {key: value for key, value in selected.items()
             if key != "EXTRA_OBJECT_FIELDS"}
    saved = {key: globals()[key] for key in plain}
    saved_fields = {key: OBJECT_FIELDS[key] for key in extra if key in OBJECT_FIELDS}
    globals().update(plain)
    OBJECT_FIELDS.update(extra)
    try:
        yield
    finally:
        globals().update(saved)
        for key in extra:
            if key in saved_fields:
                OBJECT_FIELDS[key] = saved_fields[key]
            else:
                OBJECT_FIELDS.pop(key, None)


STOP_REASONS = {0: "none", 1: "timeout", 2: "requested", 3: "start_rejected",
                4: "current", 5: "speed", 6: "fault", 7: "hall_invalid",
                8: "count", 9: "stall"}
CAVEATS = [
    "No cross-field or block read is atomic; each block has UTC/monotonic bounds.",
    "Vbus average uses a 1/256 IIR update; host sampling misses fast transients.",
    "Vbus ADC is pre-IIR, with no hardware sample timestamp or freshness guarantee.",
    "ADC-derived volts are SDK-scaled, not independently calibrated physical volts.",
    "Iq/Id amperes are SDK-equivalent dq values, not battery current or phase peaks.",
    "Iq reference is the actual, already-limited FOC reference, not unconstrained demand.",
    "References can be stale outside RUN; limit proximity is not a diagnosis.",
    "Current-valid/calibration/state are sampled separately and can race transitions.",
    "test_profile_reference_limit_ma was linker-discarded; never read DWARF address 0.",
]

# Frozen field offsets are valid ONLY after EXPECTED_ELF_SHA256 passes.
# Audited SDK getters: bus_voltage_sensor.c:79 and mc_interface.c:790,1332,1382.
# All symbol addresses and object sizes must still come from .symtab.
OBJECT_FIELDS = {
    "BusVoltageSensor_M1": (18, [
        ("vbus_conversion_factor", 2, "H"), ("vbus_average_raw", 6, "H"),
        ("vbus_fault", 8, "H"), ("vbus_undervoltage_threshold_raw", 16, "H")]),
    "VbusRegConv_M1": (16, [("vbus_adc_raw", 10, "H")]),
    "Mci": (40, [("mci_state", 1, "B"), ("current_faults", 2, "H"),
                 ("past_faults", 4, "H")]),
    "scaleParams_M1": (16, [("current_scale_a_per_count", 4, "f")]),
    "PIDSpeedHandle_M1": (44, [("pid_upper_limit_counts", 20, "h"),
                              ("pid_lower_limit_counts", 22, "h")]),
    "SpeednTorqCtrlM1": (56, [("torque_positive_limit_counts", 42, "H"),
                             ("torque_negative_limit_counts", 44, "h")]),
}
for _symbol, _key, _format in [
    ("test_drive_iq_reference", "iq_reference_counts", "i"),
    ("test_drive_id_reference", "id_reference_counts", "i"),
    ("test_drive_iq", "measured_iq_counts", "i"),
    ("test_drive_id", "measured_id_counts", "i"),
    ("test_drive_current_valid", "current_valid", "I"),
    ("test_calibration_result", "calibration_result", "I"),
    ("test_calibration_valid", "calibration_valid", "I"),
    ("test_drive_stop_reason", "stop_reason", "I"),
    ("test_drive_measured_rpm", "measured_rpm", "i"),
    ("test_drive_peak_l1", "peak_l1_counts", "I"),
    ("test_drive_active", "active", "I"),
    ("test_drive_armed", "armed", "I"),
]:
    OBJECT_FIELDS[_symbol] = (4, [(_key, 0, _format)])

SESSION_OPTIONS = {
    "target_override": TARGET, "connect_mode": "attach",
    "resume_on_disconnect": False, "auto_unlock": False,
    "no_config": True, "user_script": "/dev/null",
    "pack.debug_sequences.enable": False, "enable_swv": False,
    "cache.enable_memory": False, "cache.enable_register": False,
    "vector_catch": "none", "frequency": 1000000,
}


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def validate_manifest(manifest):
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be a JSON object")
    expected = {"wheel": "NE", "probe": NE_PROBE, "app_size": APP_SIZE,
                "source_commit": EXPECTED_SOURCE_COMMIT,
                "after_sha256": EXPECTED_BACKUP_SHA256}
    for key, value in expected.items():
        if manifest.get(key) != value or type(manifest.get(key)) is not type(value):
            raise ValueError(f"manifest {key} does not match audited NE image")
    verified = manifest.get("verified_app_readback")
    if verified is not True and not (ALLOW_PREPARED_MANIFEST and verified is False):
        raise ValueError("manifest verified_app_readback does not match audited policy")
    # A prepared candidate manifest does NOT assert it was programmed. verify_live
    # still compares the entire live application, metadata and VTOR before sampling.
    try:
        crc = int(manifest["app_crc32"], 16)
        metadata = tuple(int(word, 16) for word in manifest["metadata"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid manifest CRC/metadata") from exc
    if crc != APP_CRC or metadata != METADATA:
        raise ValueError("manifest CRC/metadata does not match audited NE image")
    if not isinstance(manifest.get("app_elf"), str) or not manifest["app_elf"]:
        raise ValueError("manifest must identify app_elf")


def reconstruct_app(segments, base=APP_BASE, size=APP_SIZE):
    """Clip PT_LOAD (physical LMA, bytes) pairs to app, including .data LMA.

    Refuse uncovered bytes instead of silently manufacturing erased padding.
    In this ELF the first LOAD includes a prefix below the application vector.
    """
    image, covered = bytearray(size), bytearray(size)
    for address, data in segments:
        lo, hi = max(address, base), min(address + len(data), base + size)
        if hi <= lo:
            continue
        start, end = lo - base, hi - base
        chunk = data[lo - address:hi - address]
        for i, value in enumerate(chunk, start):
            if covered[i] and image[i] != value:
                raise ValueError("conflicting ELF LOAD overlap")
        image[start:end], covered[start:end] = chunk, b"\x01" * (end - start)
    if not all(covered):
        raise ValueError("ELF LOAD segments leave an application gap")
    return bytes(image)


def validate_app(image):
    if (len(image) != APP_SIZE or zlib.crc32(image) != APP_CRC or
            hashlib.sha256(image).hexdigest() != EXPECTED_APP_SHA256):
        raise ValueError("reconstructed ELF application size/CRC/SHA mismatch")


def validate_symbols(symbols):
    for name, (size, _) in OBJECT_FIELDS.items():
        if name not in symbols:
            raise ValueError(f"missing linked object symbol: {name}")
        address, actual_size = symbols[name]
        if actual_size != size or not RAM_BASE <= address < address + size <= RAM_END:
            raise ValueError(f"invalid size/RAM bounds for {name}")
    spans = sorted((a, a + s, n) for n, (a, s) in symbols.items()
                   if n in OBJECT_FIELDS)
    if any(left[1] > right[0] for left, right in zip(spans, spans[1:])):
        raise ValueError("overlapping diagnostic object symbols")


@dataclass
class AuditedImage:
    manifest_path: str
    manifest: dict
    manifest_sha256: str
    elf_path: str
    elf_sha256: str
    app: bytes
    symbols: dict


def load_image(manifest_path, elf_override=None):
    manifest_path = Path(manifest_path).resolve()
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    validate_manifest(manifest)
    elf_path = Path(elf_override or manifest["app_elf"]).resolve()
    elf_bytes = elf_path.read_bytes()
    elf_sha = hashlib.sha256(elf_bytes).hexdigest()
    if elf_sha != EXPECTED_ELF_SHA256:
        raise ValueError("ELF SHA-256 mismatch; refusing frozen field layout")
    from elftools.elf.elffile import ELFFile
    elf = ELFFile(io.BytesIO(elf_bytes))
    if elf.elfclass != 32 or not elf.little_endian or elf["e_machine"] != "EM_ARM":
        raise ValueError("expected little-endian ARM32 ELF")
    segments = [(segment["p_paddr"], segment.data())
                for segment in elf.iter_segments() if segment["p_type"] == "PT_LOAD"]
    # Function defaults are bound at import, not when audited_profile changes
    # APP_SIZE. Pass the selected profile's size explicitly (900-P2 grew 8 B).
    app = reconstruct_app(segments, base=APP_BASE, size=APP_SIZE)
    validate_app(app)
    table = elf.get_section_by_name(".symtab")
    if table is None:
        raise ValueError("ELF has no .symtab")
    symbols = {}
    for name in OBJECT_FIELDS:
        matches = table.get_symbol_by_name(name) or []
        if (len(matches) != 1 or matches[0]["st_info"]["type"] != "STT_OBJECT"
                or not isinstance(matches[0]["st_shndx"], int)):
            raise ValueError(f"missing/ambiguous/non-object linked symbol: {name}")
        symbol = matches[0]
        symbols[name] = (symbol["st_value"], symbol["st_size"])
    validate_symbols(symbols)
    return AuditedImage(str(manifest_path), manifest,
                        hashlib.sha256(manifest_bytes).hexdigest(),
                        str(elf_path), elf_sha, app, symbols)


def read_bytes(target, address, size):
    data = bytes(target.read_memory_block8(address, size))
    if len(data) != size:
        raise ValueError(f"short target read at 0x{address:08x}")
    return data


def verify_live(target, image):
    """Only flash/system reads; no diagnostic RAM reads until all gates pass."""
    validate_app(image.app)
    live = b"".join(read_bytes(target, APP_BASE + offset, min(1024, APP_SIZE - offset))
                    for offset in range(0, APP_SIZE, 1024))
    if live != image.app:
        raise ValueError("live full application differs from audited ELF; no RAM sampling")
    metadata = struct.unpack("<8I", read_bytes(target, METADATA_ADDRESS, 32))
    if metadata != METADATA:
        raise ValueError("live validity metadata mismatch; no RAM sampling")
    vtor = struct.unpack("<I", read_bytes(target, VTOR_ADDRESS, 4))[0]
    if vtor != APP_BASE:
        raise ValueError("live VTOR is not the audited application; no RAM sampling")
    return {"full_app_equal": True, "app_base": APP_BASE, "app_size": len(live),
            "app_crc32": f"{zlib.crc32(live):08X}",
            "app_sha256": hashlib.sha256(live).hexdigest(),
            "metadata_words": list(metadata), "vtor": vtor}


def plan_blocks(symbols):
    validate_symbols(symbols)
    blocks = []
    for name in sorted(OBJECT_FIELDS, key=lambda n: symbols[n][0]):
        address, size = symbols[name]
        if (blocks and address - (blocks[-1]["address"] + blocks[-1]["size"]) <= 64
                and address + size - blocks[-1]["address"] <= 512):
            block = blocks[-1]
            block["size"] = address + size - block["address"]
            block["symbols"].append(name)
        else:
            blocks.append({"address": address, "size": size, "symbols": [name]})
    return blocks


def derive(raw):
    scale = raw["current_scale_a_per_count"]
    if (not math.isfinite(scale) or scale <= 0 or
            not math.isclose(scale, CURRENT_SCALE, rel_tol=1e-6)):
        raise ValueError("unexpected/non-finite SDK current scaling in live RAM")
    conv = raw["vbus_conversion_factor"]
    if conv != 34:
        raise ValueError("unexpected Vbus ConversionFactor in live RAM")
    valid = bool(raw["current_valid"] and raw["calibration_valid"] and
                 raw["calibration_result"] == 2 and raw["mci_state"] == 6 and
                 raw["current_faults"] == 0)
    upper, lower = raw["pid_upper_limit_counts"], raw["pid_lower_limit_counts"]
    limits_valid = upper > 0 and lower < 0
    iqref = raw["iq_reference_counts"]
    near_limit = (iqref >= 0.98 * upper if iqref >= 0 else iqref <= 0.98 * lower)
    return {
        "vbus_average_sdk_v": raw["vbus_average_raw"] * conv / 65536.0,
        "vbus_adc_sdk_v": (raw["vbus_adc_raw"] * conv / 65536.0
                           if raw["vbus_adc_raw"] != 0xFFFF else None),
        "vbus_undervoltage_threshold_sdk_v":
            raw["vbus_undervoltage_threshold_raw"] * conv / 65536.0,
        "measured_current_valid": valid,
        "measured_iq_sdk_a": raw["measured_iq_counts"] * scale if valid else None,
        "measured_id_sdk_a": raw["measured_id_counts"] * scale if valid else None,
        "iq_reference_sdk_a": iqref * scale,
        "id_reference_sdk_a": raw["id_reference_counts"] * scale,
        "reference_valid": valid, "reference_may_be_stale": not valid,
        "iq_reference_near_pid_limit": bool(near_limit) if valid and limits_valid else None,
        "limit_proximity_fraction": 0.98,
        "pid_limits_valid": limits_valid,
        "stop_reason_name": STOP_REASONS.get(raw["stop_reason"], "unknown"),
    }


class Sampler:
    def __init__(self, target, symbols):
        self.target, self.symbols = target, symbols
        self.blocks = plan_blocks(symbols)
        self.field_blocks = {key: index for index, block in enumerate(self.blocks)
                             for name in block["symbols"]
                             for key, _, _ in OBJECT_FIELDS[name][1]}

    def sample(self, sequence):
        raw, snapshots = {}, []
        for block in self.blocks:
            snapshot = dict(block)
            snapshot["utc_start"] = utc_now()
            snapshot["monotonic_ns_start"] = time.monotonic_ns()
            data = read_bytes(self.target, block["address"], block["size"])
            snapshot["monotonic_ns_end"] = time.monotonic_ns()
            snapshot["utc_end"] = utc_now()
            snapshot["data_hex"] = data.hex()
            snapshots.append(snapshot)
            for name in block["symbols"]:
                for key, offset, fmt in OBJECT_FIELDS[name][1]:
                    raw[key] = struct.unpack_from(
                        "<" + fmt, data, self.symbols[name][0] - block["address"] + offset)[0]
        # Validate interpretation before emitting derived values; raw bytes remain
        # available on an interpretation failure in the enclosing error record.
        record = {"type": "sample", "sequence": sequence, "atomic": False,
                  "blocks": snapshots, "raw": raw}
        try:
            record["derived"] = derive(raw)
        except ValueError as exc:
            record["type"], record["error"] = "invalid_sample", str(exc)
        return record


def open_output(path=None):
    if path is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        path = Path("/tmp/opencode") / f"esc-power-{stamp}-{os.getpid()}.jsonl"
    path = Path(path)
    return path, path.open("x", encoding="utf-8")


def write_record(output, record):
    # Invalid float scales must not produce non-standard JSON NaN/Infinity.
    def finite(value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: finite(item) for key, item in value.items()}
        if isinstance(value, list):
            return [finite(item) for item in value]
        return value
    output.write(json.dumps(finite(record), allow_nan=False) + "\n")
    output.flush()


def print_status(record):
    raw, derived = record["raw"], record["derived"]
    iq = derived["measured_iq_sdk_a"]
    iq_text = "invalid" if iq is None else f"{iq:+.3f}A"
    print(f"{record['blocks'][-1]['utc_end']} "
          f"Vbus={derived['vbus_average_sdk_v']:.2f}V "
          f"rpm={raw['measured_rpm']:+d} "
          f"Iqref={derived['iq_reference_sdk_a']:+.3f}A Iq={iq_text} "
          f"stop={derived['stop_reason_name']} state={raw['mci_state']} "
          f"RUN={int(raw['mci_state'] == 6)} "
          f"CAL={raw['calibration_result']}/{raw['calibration_valid']} "
          f"fault={raw['current_faults']:04x}/{raw['past_faults']:04x} "
          f"ref_valid={int(derived['reference_valid'])}", flush=True)


def monitor(target, image, output, seconds, period):
    identity = verify_live(target, image)
    sampler = Sampler(target, image.symbols)
    first = sampler.sample(0)
    header = {"type": "header", "schema": 1, "utc": utc_now(),
              "audited_profile": AUDIT_PROFILE_NAME,
              "manifest_path": image.manifest_path, "manifest": image.manifest,
              "manifest_sha256": image.manifest_sha256, "elf_path": image.elf_path,
              "elf_sha256": image.elf_sha256, "probe": NE_PROBE, "target": TARGET,
              "identity": identity, "session_options": SESSION_OPTIONS,
              "seconds": seconds, "period_seconds": period,
              "symbols": image.symbols, "field_blocks": sampler.field_blocks,
              "caveats": CAVEATS, "expected_profile_reference_ma": 3000,
              "profile_limit_source": "audited macros/linked controller limits; no profile RAM symbol",
              "initial_scaling_limits": {key: value for key, value in first["raw"].items()
                                         if "limit" in key or "scale" in key or "conversion" in key}}
    write_record(output, header)
    start = time.monotonic()
    sequence, record = 0, first
    try:
        while True:
            write_record(output, record)
            if record["type"] != "sample":
                raise ValueError(record["error"])
            sequence += 1
            print_status(record)
            now = time.monotonic()
            if now - start >= seconds:
                break
            # Skip missed deadlines rather than burst-polling to catch up.
            deadline = start + (math.floor((now - start) / period) + 1) * period
            time.sleep(min(deadline, start + seconds) - now)
            if time.monotonic() - start >= seconds:
                break
            record = sampler.sample(sequence)
    except KeyboardInterrupt:
        write_record(output, {"type": "end", "utc": utc_now(), "reason": "interrupted",
                              "samples": sequence})
        return
    write_record(output, {"type": "end", "utc": utc_now(), "reason": "duration",
                          "samples": sequence})


def create_session(connect_helper=None, session_class=None):
    if connect_helper is None or session_class is None:
        from pyocd.core.helpers import ConnectHelper
        from pyocd.core.session import Session
        connect_helper, session_class = ConnectHelper, Session
    probes = connect_helper.get_all_connected_probes(blocking=False, unique_id=NE_PROBE)
    exact = [probe for probe in probes if probe.unique_id == NE_PROBE]
    if len(exact) != 1:
        raise ValueError("expected exactly one exact-serial NE probe; refusing other probes")
    # /dev/null suppresses auto-discovered user scripts; no_config alone does not.
    return session_class(exact[0], auto_open=False, options=dict(SESSION_OPTIONS))


def self_test():
    path = Path(__file__).resolve().parents[1] / "tests/test_monitor_esc_power.py"
    spec = importlib.util.spec_from_file_location("esc_power_self_tests", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(module))
    return 0 if result.wasSuccessful() else 1


def run_monitor(args):
    try:
        image = load_image(args.manifest, args.elf)
        path, output = open_output(args.output)
        with output:
            print(f"JSONL: {path}\nNE probe ownership must be explicitly handed off before use.")
            session = None
            try:
                session = create_session()
                session.open()
                monitor(session.target, image, output, args.seconds, args.period)
            except KeyboardInterrupt:
                write_record(output, {"type": "end", "utc": utc_now(), "reason": "interrupted"})
            except Exception as exc:
                write_record(output, {"type": "error", "utc": utc_now(), "error": str(exc)})
                raise
            finally:
                if session is not None:
                    session.close()  # No motor cleanup requests; no resume on disconnect.
    except KeyboardInterrupt:
        print("ESC monitor interrupted; no motor cleanup commands issued.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ESC monitor aborted: {exc}", file=sys.stderr)
        return 1
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--profile", choices=tuple(AUDITED_PROFILES), default="baseline",
                        help="explicit frozen image identity (default: baseline)")
    parser.add_argument("--elf", type=Path, help="override path, never the exact ELF identity")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--period", type=float, default=0.05, help="seconds, 0.02..1")
    parser.add_argument("--output", type=Path, help="exclusive-create JSONL (default /tmp/opencode)")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    if not math.isfinite(args.period) or not 0.02 <= args.period <= 1:
        parser.error("--period must be finite and within 0.02..1 seconds")
    if args.profile != "baseline" and args.manifest == DEFAULT_MANIFEST:
        parser.error("candidate profile requires its explicit --manifest path")
    with audited_profile(args.profile):
        return run_monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
