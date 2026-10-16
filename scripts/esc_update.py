#!/usr/bin/env python3
"""Install the built ESC application on ONE wheel over CAN (bootloader kept).

  scripts/esc_update.py NE --battery-off [--keep-updater]

Needs: motor battery OFF, that wheel's ESC ST-LINK USB and the Nucleo USB
connected, no other tool holding either probe. Artifacts from build_all.py.

1. Full 128 KiB backup of the ESC to esc-backups/update-<W>-<UTC>/.
2. If the installed app is one this tool installed before, check from RAM
   that Vbus < 8 V and the motor is idle.
3. Flash the Nucleo updater image for this wheel, verify its payload.
4. Reset the ESC to open its bootloader window; one CAN transfer, no retry.
5. Read back: bootloader unchanged, app bytes, metadata, VTOR, 1 kHz hook alive.
6. Flash the Nucleo drive image (unless --keep-updater).
Writes session.json (with app_elf) next to the backup. Sends no motion.
"""
import argparse
import datetime
import hashlib
import json
import pathlib
import re
import struct
import subprocess
import sys
import time
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build"
BACKUPS = pathlib.Path("/home/fh/sfurs_firmware/esc-backups")
CLI = pathlib.Path.home() / "STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
NUCLEO_SN = "002200364142500620353451"
# wheel: (CAN select bit, ESC ST-LINK serial)
WHEELS = {"NE": (1, "066AFF525266555067075126"), "SE": (2, "0667FF525266555067073424"),
          "SW": (4, "066FFF525266555067073058"), "NW": (8, "066AFF515266555067023255")}
APP_BASE, APP_OFFSET, META_OFFSET = 0x08004000, 0x4000, 0x1F800
FWU_DONE, FWU_FAULT = 6, 7
sha = lambda data: hashlib.sha256(data).hexdigest()


def symbols(elf, names):
    from elftools.elf.elffile import ELFFile
    with open(elf, "rb") as f:
        tab = ELFFile(f).get_section_by_name(".symtab")
        out = {}
        for name in names:
            found = tab.get_symbol_by_name(name)
            if not found:
                raise RuntimeError(f"{elf.name}: missing symbol {name}")
            out[name] = (found[0]["st_value"], found[0]["st_size"])
        return out


def nucleo_session():
    from pyocd.core.helpers import ConnectHelper
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=NUCLEO_SN, connect_mode="attach", target_override="stm32h755zitx", blocking=False)
    if session is None:
        raise RuntimeError("Nucleo probe not found")
    return session


def esc_session(sn):
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    probes = [p for p in ConnectHelper.get_all_connected_probes(blocking=False) if p.unique_id == sn]
    if len(probes) != 1:
        raise RuntimeError(f"ESC probe {sn} not connected")
    return Session(probes[0], auto_open=False, options={
        "target_override": "stm32g431cbux", "connect_mode": "attach",
        "resume_on_disconnect": False, "auto_unlock": False, "no_config": True,
        "user_script": "/dev/null", "pack.debug_sequences.enable": False, "enable_swv": False,
        "cache.enable_memory": False, "cache.enable_register": False,
        "vector_catch": "none", "frequency": 1000000})


def read_flash(target):
    return b"".join(bytes(target.read_memory_block8(0x08000000 + off, 1024))
                    for off in range(0, 0x20000, 1024))


def program_nucleo(elf, log):
    r = subprocess.run([str(CLI), "-c", "port=SWD", f"sn={NUCLEO_SN}", "-w", str(elf), "-v", "-rst"],
                       capture_output=True, text=True)
    log.write_text(r.stdout + r.stderr)
    text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", r.stdout + r.stderr)
    if r.returncode != 0 or "Download verified successfully" not in text:
        raise RuntimeError(f"Nucleo programming failed, see {log}")


def installed_app_elf(flash_sha):
    """ELF of an app this tool installed before, matched by full-flash hash."""
    for record in sorted(BACKUPS.glob("*/session.json"), reverse=True):
        d = json.loads(record.read_text())
        if d.get("complete") and d.get("after_sha256") == flash_sha and d.get("app_elf"):
            if pathlib.Path(d["app_elf"]).is_file():
                return pathlib.Path(d["app_elf"])
    return None


def check_idle_unpowered(target, elf):
    s = symbols(elf, ("BusVoltageSensor_M1", "Mci"))
    sensor = bytes(target.read_memory_block8(*s["BusVoltageSensor_M1"]))
    vbus = struct.unpack_from("<H", sensor, 6)[0] * struct.unpack_from("<H", sensor, 2)[0] / 65536
    state = bytes(target.read_memory_block8(*s["Mci"]))[1]
    if vbus >= 8:
        raise RuntimeError(f"Vbus {vbus:.1f} V: battery is ON; switch it off")
    if state not in (0, 11):
        raise RuntimeError(f"motor state {state}, not idle")
    return dict(vbus_v=round(vbus, 2), mc_state=state)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("wheel", choices=WHEELS)
    parser.add_argument("--battery-off", action="store_true", required=True,
                        help="operator confirms the motor battery is OFF")
    parser.add_argument("--keep-updater", action="store_true",
                        help="leave the updater on the Nucleo (to update another wheel next)")
    args = parser.parse_args()
    w = args.wheel
    selector, esc_sn = WHEELS[w]
    app_elf = BUILD / f"esc/{w}/motor_{w}_drive_offset.elf"
    app = app_elf.with_suffix(".bin").read_bytes()
    crc = zlib.crc32(app) & 0xFFFFFFFF
    updater = BUILD / f"updater/{w}/sfurs_firmware_v2_CM7.elf"
    drive = BUILD / "drive/sfurs_firmware_v2_CM7.elf"
    nu = symbols(updater, ("nucleo_image_kind", "fwu_host", "can_fwu_request", "can_fwu_dev",
                           "can_fwu_result", "fwu_test_image", "fwu_test_image_size"))
    addr = {k: v[0] for k, v in nu.items()}
    app_syms = symbols(app_elf, ("drive_app",))

    out = BACKUPS / f"update-{w}-{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}"
    out.mkdir(parents=True)
    session = dict(wheel=w, selector=selector, esc_probe=esc_sn, app_elf=str(app_elf),
                   app_sha256=sha(app), app_crc32=f"{crc:08X}", app_bytes=len(app),
                   battery_off_confirmed=True, complete=False)
    save = lambda: (out / "session.json").write_text(json.dumps(session, indent=2))
    save()

    es = esc_session(esc_sn)
    es.open()
    try:
        before = read_flash(es.target)
        (out / "before.bin").write_bytes(before)
        session["before_sha256"] = sha(before)
        save()
        known = installed_app_elf(sha(before))
        if known:
            session["before_state"] = check_idle_unpowered(es.target, known)
        else:
            session["before_state"] = "unknown app; relying on operator battery-off confirmation"
    finally:
        es.close()
    save()
    print(f"{w}: backed up to {out}  ({session['before_state']})", flush=True)

    program_nucleo(updater, out / "nucleo-updater.log")
    es = esc_session(esc_sn)
    with nucleo_session() as ns:
        t = ns.target
        if t.read32(addr["nucleo_image_kind"]) != 2:
            raise RuntimeError("Nucleo is not running the updater image")
        if t.read32(addr["fwu_test_image_size"]) != len(app) or \
           bytes(t.read_memory_block8(addr["fwu_test_image"], len(app))) != app:
            raise RuntimeError("updater payload does not match the built ESC app")
        es.open()
        try:
            t.write32(addr["can_fwu_dev"], selector)
            t.write32(addr["can_fwu_request"], 1)
            from pyocd.core.target import Target
            es.target.reset(Target.ResetType.SYSRESETREQ)  # opens the ESC bootloader's CAN window
            deadline, last = time.monotonic() + 60, None
            while True:
                state, result = t.read32(addr["fwu_host"]), t.read32(addr["can_fwu_result"])
                if (state, result) != last:
                    print(f"  updater state {state} result {result}", flush=True)
                    last = (state, result)
                if state == FWU_DONE and result == 3:
                    break
                if state == FWU_FAULT and result == 4:
                    raise RuntimeError("transfer failed; ESC keeps its bootloader, rerun to retry")
                if time.monotonic() > deadline:
                    t.reset(Target.ResetType.SYSRESETREQ)  # drop the pending request
                    raise RuntimeError("no completion within 60 s")
                time.sleep(0.1)
        finally:
            es.close()

    es = esc_session(esc_sn)
    es.open()
    try:
        time.sleep(4)  # bootloader window closes, new app starts
        after = read_flash(es.target)
        (out / "after.bin").write_bytes(after)
        session["after_sha256"] = sha(after)
        checks = {
            "bootloader unchanged": after[:APP_OFFSET] == before[:APP_OFFSET],
            "app bytes": after[APP_OFFSET:APP_OFFSET + len(app)] == app,
            "metadata size+crc": {len(app), crc} <= set(struct.unpack_from("<8I", after, META_OFFSET)),
            "VTOR at app": es.target.read32(0xE000ED08) == APP_BASE,
        }
        tick_addr = app_syms["drive_app"][0]
        t0 = es.target.read32(tick_addr)
        time.sleep(0.2)
        checks["1 kHz hook running"] = es.target.read32(tick_addr) > t0
    finally:
        es.close()
    session["checks"] = checks
    save()
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise RuntimeError(f"verification failed: {failed}; backup in {out}")
    print(f"{w}: installed and verified ({', '.join(checks)})", flush=True)

    if not args.keep_updater:
        program_nucleo(drive, out / "nucleo-drive.log")
        session["nucleo_after"] = dict(elf=str(drive), sha256=sha(drive.read_bytes()))
    session["complete"] = True
    save()
    print(f"{w}: done. Nucleo runs the {'updater' if args.keep_updater else 'drive'} image.")


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        sys.exit(f"ERROR: {exc}")
