#!/usr/bin/env python3
"""Install the built ESC application on ONE wheel over CAN (bootloader kept).

  scripts/esc_update.py NE --can              no ESC USB; battery ON allowed
  scripts/esc_update.py NE --battery-off      ESC ST-LINK attached, battery OFF

--can needs only the Nucleo USB. Through the drive image it checks that all
four ESCs answer, are IDLE (no fault) and report Vbus; then the updater image
sends the CAN boot-entry request (the app accepts it only when idle), streams
the app, and the bootloader verifies its CRC before it writes the validity
record and jumps. Back on the drive image, the wheel's IDENT reply must give
the built image's CRC32 and size, and all four must be IDLE again. No flash
backup (the bootloader is never touched). If a transfer fails the ESC stays
in its bootloader listening; rerun with --can --target-in-boot.

--battery-off needs: motor battery OFF, that wheel's ESC ST-LINK USB and the
Nucleo USB connected, no other tool holding either probe:

1. Full 128 KiB backup of the ESC to esc-backups/update-<W>-<UTC>/.
2. Identify the installed app by its flash bytes (build/esc/*, or the ELFs in
   esc-backups/known-apps/) and read from its RAM that it is running, Vbus is
   below 8 V and the motor is not driving; refuse otherwise. Checked again
   just before the transfer.
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
import shutil
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


def flash_matches(elf, flash):
    """True if every loaded section of the ELF equals the flash bytes at its load
    address (segments also cover header padding, so compare sections)."""
    from elftools.elf.elffile import ELFFile
    with open(elf, "rb") as f:
        e = ELFFile(f)
        loads = [s for s in e.iter_segments() if s["p_type"] == "PT_LOAD" and s["p_filesz"]]
        checked = 0
        for sec in e.iter_sections():
            if not sec["sh_flags"] & 2 or sec["sh_type"] == "SHT_NOBITS" or not sec["sh_size"]:
                continue  # not SHF_ALLOC, or no file bytes
            seg = next((s for s in loads if s["p_offset"] <= sec["sh_offset"] and
                        sec["sh_offset"] + sec["sh_size"] <= s["p_offset"] + s["p_filesz"]), None)
            if seg is None:
                continue
            lma = seg["p_paddr"] + sec["sh_offset"] - seg["p_offset"] - 0x08000000
            if not 0 <= lma <= len(flash) - sec["sh_size"] or flash[lma:lma + sec["sh_size"]] != sec.data():
                return False
            checked += 1
        return checked > 0


def installed_app_elf(flash):
    candidates = sorted(BUILD.glob("esc/*/motor_*_drive_offset.elf")) + \
        sorted((BACKUPS / "known-apps").glob("*.elf"))
    return next((elf for elf in candidates if flash_matches(elf, flash)), None)


def check_battery_off(target, elf):
    """Read the running app's own Vbus measurement (MCSDK) over SWD."""
    s = symbols(elf, ("BusVoltageSensor_M1", "Mci", "uwTick"))
    t0 = target.read32(s["uwTick"][0])
    time.sleep(0.05)
    if target.read32(s["uwTick"][0]) == t0:
        raise RuntimeError("ESC app is not running, so its Vbus reading is stale")
    sensor = bytes(target.read_memory_block8(*s["BusVoltageSensor_M1"]))
    vbus = struct.unpack_from("<H", sensor, 6)[0] * struct.unpack_from("<H", sensor, 2)[0] / 65536
    state = bytes(target.read_memory_block8(*s["Mci"]))[1]
    if vbus >= 8:
        raise RuntimeError(f"Vbus {vbus:.1f} V: battery is ON; switch it off")
    if state not in (0, 10, 11):  # IDLE, FAULT_NOW (USB undervolt), FAULT_OVER
        raise RuntimeError(f"motor state {state}, not idle")
    return dict(vbus_v=round(vbus, 2), mc_state=state, app_elf=elf.name)


def bus_snapshot(link, settle=0.4):
    """Live telemetry rows (None for a wheel whose status count didn't move)."""
    before = link.rows()
    time.sleep(settle)
    after = link.rows()
    return [r if r and (not r0 or r["status_count"] != r0["status_count"]) else None
            for r0, r in zip(before, after)]


def wait_ident(link, index, timeout=2.0):
    """A fresh IDENT reply for one wheel (polled once per ~85 ms per wheel)."""
    start = link.rows()[index]
    count = start["ident_count"] if start else 0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        row = link.rows()[index]
        if row and row["ident_count"] != count:
            return row["app_crc"], row["app_size"]
        time.sleep(0.05)
    return None


def check_bus(rows, names, skip=None):
    """All wheels answering and IDLE; returns {wheel: Vbus}."""
    problems, vbus = [], {}
    for i, (name, row) in enumerate(zip(names, rows)):
        if i == skip:
            continue
        if row is None:
            problems.append(f"{name} no reply")
        elif row["state"] != 0 or row["active"] or row["aligning"]:
            problems.append(f"{name} {row['mc_state']}{' active' if row['active'] else ''}")
        else:
            vbus[name] = round(row["vbus_v"], 2)
    if problems:
        raise RuntimeError("bus not ready: " + ", ".join(problems) +
                           " (esc_tool.py stop clears a cleared fault)")
    return vbus


def can_install(args, w, selector, app, crc, app_elf, updater, drive, out, session, save):
    sys.path.insert(0, str(ROOT / "scripts"))
    import ik_link
    index = [sel for _n, _e, sel in ik_link.WHEELS].index(selector)
    names = [name for name, _e, _s in ik_link.WHEELS]
    nu = symbols(updater, ("nucleo_image_kind", "fwu_host", "can_fwu_request", "can_fwu_dev",
                           "can_fwu_result", "can_fwu_enter_boot", "can_fwu_boot_result",
                           "fwu_test_image", "fwu_test_image_size"))
    addr = {k: v[0] for k, v in nu.items()}

    def open_drive():
        try:
            return ik_link.Link.open(NUCLEO_SN, drive)
        except RuntimeError:  # not the drive image (or an older ABI)
            program_nucleo(drive, out / "nucleo-drive-before.log")
            time.sleep(1.0)
            return ik_link.Link.open(NUCLEO_SN, drive)

    lsession, link = open_drive()
    try:
        rows = bus_snapshot(link)
        skip = index if args.target_in_boot else None
        session["before_vbus"] = check_bus(rows, names, skip)
        if args.target_in_boot:
            if rows[index] is not None:
                raise RuntimeError(f"{w} answers as an app; drop --target-in-boot")
            session["before_ident"] = "target in bootloader"
        else:
            ident = wait_ident(link, index)
            session["before_ident"] = (dict(crc=f"{ident[0]:08X}", size=ident[1]) if ident
                                       else "no IDENT reply (app predates it)")
    finally:
        lsession.close()
    save()
    print(f"{w}: bus ready, Vbus {session['before_vbus']}, installed {session['before_ident']}",
          flush=True)

    program_nucleo(updater, out / "nucleo-updater.log")
    with nucleo_session() as ns:
        t = ns.target
        if t.read32(addr["nucleo_image_kind"]) != 2:
            raise RuntimeError("Nucleo is not running the updater image")
        if t.read32(addr["fwu_test_image_size"]) != len(app) or \
           bytes(t.read_memory_block8(addr["fwu_test_image"], len(app))) != app:
            raise RuntimeError("updater payload does not match the built ESC app")
        t.write32(addr["can_fwu_dev"], selector)
        if not args.target_in_boot:
            t.write32(addr["can_fwu_enter_boot"], 1)
            time.sleep(0.2)
            boot = t.read32(addr["can_fwu_boot_result"])
            if boot != 1:
                raise RuntimeError(f"boot-entry request not sent (result {boot})")
            time.sleep(0.3)  # the app resets into its bootloader, which then listens
        t.write32(addr["can_fwu_request"], 1)
        from pyocd.core.target import Target
        deadline, last = time.monotonic() + 60, None
        while True:
            state, result = t.read32(addr["fwu_host"]), t.read32(addr["can_fwu_result"])
            if (state, result) != last:
                print(f"  updater state {state} result {result}", flush=True)
                last = (state, result)
            if state == FWU_DONE and result == 3:
                break
            if state == FWU_FAULT and result == 4:
                raise RuntimeError(f"transfer failed; {w} stays in its bootloader. "
                                   "Rerun with --can --target-in-boot")
            if time.monotonic() > deadline:
                t.reset(Target.ResetType.SYSRESETREQ)  # drop the pending request
                raise RuntimeError(f"no completion within 60 s; if {w} stopped answering it is "
                                   "in its bootloader: rerun with --can --target-in-boot")
            time.sleep(0.1)

    program_nucleo(drive, out / "nucleo-drive.log")
    session["nucleo_after"] = dict(elf=str(drive), sha256=sha(drive.read_bytes()))
    time.sleep(1.0)
    lsession, link = ik_link.Link.open(NUCLEO_SN, drive)
    try:
        ident = wait_ident(link, index)
        rows = bus_snapshot(link)
        session["after_vbus"] = check_bus(rows, names)
    finally:
        lsession.close()
    checks = {"IDENT reply": ident is not None,
              "CRC32 matches build": bool(ident) and ident[0] == crc,
              "size matches build": bool(ident) and ident[1] == len(app),
              "all four IDLE": True}
    session["after_ident"] = dict(crc=f"{ident[0]:08X}", size=ident[1]) if ident else None
    session["checks"] = checks
    save()
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise RuntimeError(f"verification failed: {failed}; see {out}")
    print(f"{w}: installed and verified over CAN ({', '.join(checks)})", flush=True)
    known_dir = BACKUPS / "known-apps"
    known_dir.mkdir(exist_ok=True)
    shutil.copy2(app_elf, known_dir / f"motor_{w}_drive_{sha(app)[:12]}.elf")
    session["complete"] = True
    save()
    print(f"{w}: done. Nucleo runs the drive image.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("wheel", choices=WHEELS)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--can", action="store_true",
                      help="CAN only, no ESC USB: all four ESCs IDLE, Vbus measured")
    mode.add_argument("--battery-off", action="store_true",
                      help="SWD path: operator confirms the motor battery is OFF")
    parser.add_argument("--target-in-boot", action="store_true",
                        help="--can recovery: the wheel is already in its bootloader")
    parser.add_argument("--keep-updater", action="store_true",
                        help="SWD path: leave the updater on the Nucleo")
    args = parser.parse_args()
    if args.target_in_boot and not args.can:
        parser.error("--target-in-boot needs --can")
    w = args.wheel
    if args.can:
        selector = WHEELS[w][0]
        app_elf = BUILD / f"esc/{w}/motor_{w}_drive_offset.elf"
        app = app_elf.with_suffix(".bin").read_bytes()
        out = BACKUPS / f"can-update-{w}-{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}"
        out.mkdir(parents=True)
        session = dict(wheel=w, selector=selector, mode="can", app_elf=str(app_elf),
                       app_sha256=sha(app), app_crc32=f"{zlib.crc32(app):08X}",
                       app_bytes=len(app), complete=False)
        save = lambda: (out / "session.json").write_text(json.dumps(session, indent=2))
        save()
        can_install(args, w, selector, app, zlib.crc32(app) & 0xFFFFFFFF, app_elf,
                    BUILD / f"updater/{w}/sfurs_firmware_v2_CM7.elf",
                    BUILD / "drive/sfurs_firmware_v2_CM7.elf", out, session, save)
        return
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
                   battery_off_confirmed=args.battery_off, complete=False)
    save = lambda: (out / "session.json").write_text(json.dumps(session, indent=2))
    save()

    es = esc_session(esc_sn)
    es.open()
    try:
        before = read_flash(es.target)
        (out / "before.bin").write_bytes(before)
        session["before_sha256"] = sha(before)
        save()
        known = installed_app_elf(before)
        if known:
            session["before_state"] = check_battery_off(es.target, known)
        elif args.battery_off:
            session["before_state"] = "unidentified app; operator confirmed battery OFF"
        else:
            raise RuntimeError("installed app is unidentified, so Vbus can't be read; "
                               f"confirm the battery is OFF and rerun with --battery-off (backup in {out})")
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
            if known:  # battery may have been switched on while the Nucleo was flashed
                check_battery_off(es.target, known)
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
    # Keep the installed ELF: the next run identifies this app (and reads its
    # Vbus) after build/ has been rebuilt.
    known_dir = BACKUPS / "known-apps"
    known_dir.mkdir(exist_ok=True)
    shutil.copy2(app_elf, known_dir / f"motor_{w}_drive_{sha(app)[:12]}.elf")

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
