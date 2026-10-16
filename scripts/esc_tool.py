#!/usr/bin/env python3
"""ESC setup through the Nucleo streaming IK image (drive must be stopped).

  esc_tool.py status                    print all four wheels once
  esc_tool.py align FR|BR|BL|FL|all     measure + apply Hall phase shift (RAM)
  esc_tool.py set WHEEL KEY VALUE       KEY: kp, ki, shift (deg), iqmax_ma (RAM)
  esc_tool.py stop                      broadcast STOP; also clears a fault whose
                                        cause is gone (FAULT_OVER -> IDLE)

align turns the wheel about 1/4 revolution forward and back at 2 A: battery
ON, wheels off the ground. Results are lost on ESC reset; bake them into the
per-wheel build with build_cli.py --hall-shift.
"""
import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import ik_link  # noqa: E402

NAMES = {name: (i, sel) for i, (name, _esc, sel) in enumerate(ik_link.WHEELS)}
NAMES.update({esc: (i, sel) for i, (_name, esc, sel) in enumerate(ik_link.WHEELS)})


def wheels(arg):
    if arg.lower() == "all":
        return [(name, i, sel) for i, (name, _e, sel) in enumerate(ik_link.WHEELS)]
    key = arg.upper()
    if key not in NAMES:
        raise SystemExit(f"unknown wheel {arg}; use FR BR BL FL (or NE SE SW NW) or all")
    return [(key, *NAMES[key])]


def fresh_row(link, index, field, after, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        row = link.rows()[index]
        if row and row[field] != after:
            return row
        time.sleep(0.02)
    raise RuntimeError("no telemetry reply from this ESC (powered? CAN connected?)")


def show(rows):
    for (name, esc, _), r in zip(ik_link.WHEELS, rows):
        if r is None or not r["status_count"]:
            print(f"{name}/{esc}: no reply")
            continue
        print(f"{name}/{esc}: {r['mc_state']:<10s} rpm {r['rpm']:5d}  Iq {r['iq_a']:5.2f} A  "
              f"Vbus {r['vbus_v']:5.2f} V  faults {','.join(r['fault_names']) or '-'}  "
              f"hall shift {r['hall_shift_deg']} deg ({'known' if r['aligned'] else 'NOT ALIGNED'})  "
              f"align {r['align']}  stop {r['stop_reason']}")


def align(link, name, index, selector):
    before = link.rows()[index]
    count = before["diag_count"] if before else 0
    link.send_raw(ik_link.HALL_ALIGN | selector, ik_link.HALL_ALIGN_MAGIC)
    print(f"{name}: aligning...", flush=True)
    time.sleep(0.3)
    deadline = time.monotonic() + 8.0
    row = fresh_row(link, index, "diag_count", count)
    while row["align_result"] == 5 or row["aligning"]:
        if time.monotonic() > deadline:
            raise RuntimeError(f"{name}: alignment did not finish")
        time.sleep(0.1)
        row = link.rows()[index]
    print(f"{name}: result {row['align']}, hall shift {row['hall_shift_deg']} deg "
          f"(was {before['hall_shift_deg'] if before else '?'})")
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", type=pathlib.Path, default=ik_link.DEFAULT_ELF)
    parser.add_argument("--sn", default=ik_link.DEFAULT_SN)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("stop")
    p = sub.add_parser("align")
    p.add_argument("wheel")
    p = sub.add_parser("set")
    p.add_argument("wheel")
    p.add_argument("key", choices=sorted(ik_link.CONFIG_KEYS))
    p.add_argument("value", type=int)
    args = parser.parse_args(argv)
    session, link = ik_link.Link.open(args.sn, args.elf)
    try:
        if args.command == "status":
            time.sleep(0.2)
            show(link.rows())
        elif args.command == "stop":
            link.send_raw(ik_link.MOTOR_STOP | 0x0F, 0)
        elif args.command == "align":
            results = [align(link, name, i, sel) for name, i, sel in wheels(args.wheel)]
            print("build flags: " + "  ".join(
                f"{name}:--hall-shift {r['hall_shift_deg']}"
                for (name, _i, _s), r in zip(wheels(args.wheel), results) if r["align_result"] == 1))
        elif args.command == "set":
            for name, _i, sel in wheels(args.wheel):
                link.send_raw(*ik_link.config_frame(sel, args.key, args.value))
                print(f"{name}: {args.key} = {args.value} (RAM)")
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
