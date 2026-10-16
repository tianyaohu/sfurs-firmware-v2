#!/usr/bin/env python3
"""ESC setup through the Nucleo streaming IK image (drive must be stopped).

  esc_tool.py status                    print all four wheels once
  esc_tool.py align FR|BR|BL|FL|all     measure + apply Hall phase shift (RAM)
  esc_tool.py set WHEEL KEY VALUE       KEY: kp, ki (speed PI), current_kp,
                                        current_ki (Id+Iq PI), shift (deg),
                                        iqmax_ma (RAM)
  esc_tool.py capture WHEEL [--step-ma 3000] [--out f.json]
                                        current-loop step test: R, L, step
                                        response, PI gains for 0.5/0.8/1 kHz
  esc_tool.py spin WHEEL --rpm 60 [--seconds 3] [--kp N --ki N] [--log f.jsonl]
                                        one wheel at a speed setpoint, telemetry
                                        logged; speed-loop check (motion!)
  esc_tool.py stop                      broadcast STOP; also clears a fault whose
                                        cause is gone (FAULT_OVER -> IDLE)

align turns the wheel about 1/4 revolution forward and back at 2 A: battery
ON, wheels off the ground. Results are lost on ESC reset; bake them into the
per-wheel build with build_cli.py --hall-shift. capture holds the rotor with
d-axis current (1 A, then the step): it may click into position but makes no
torque; battery ON. spin streams SPEED_WRITE to one wheel only; --kp/--ki
are set in RAM first and the firmware defaults (drive_parameters.h) restored
afterwards; STOP on exit, and the ESC stops itself 0.25 s after the stream.
"""
import argparse
import json
import pathlib
import re
import statistics
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import capture_analysis  # noqa: E402
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


def show(before, rows):
    for (name, esc, _), r0, r in zip(ik_link.WHEELS, before, rows):
        if r is None or not r["status_count"] or (r0 and r0["status_count"] == r["status_count"]):
            print(f"{name}/{esc}: no reply")  # rows keep the last values; only a new count is live
            continue
        app = f"{r['app_crc']:08X}/{r['app_size']}" if r["ident_count"] else "?"
        print(f"{name}/{esc}: {r['mc_state']:<10s} rpm {r['rpm']:5d}  Iq {r['iq_a']:5.2f} A  "
              f"Vbus {r['vbus_v']:5.2f} V  faults {','.join(r['fault_names']) or '-'}  "
              f"hall shift {r['hall_shift_deg']} deg ({'known' if r['aligned'] else 'NOT ALIGNED'})  "
              f"align {r['align']}  stop {r['stop_reason']}  app {app}")


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


def capture(link, name, selector, args):
    info, samples = link.capture(selector, args.step_ma)
    a = capture_analysis.analyze(info, samples)
    if args.out:
        args.out.write_text(json.dumps(dict(wheel=name, step_ma=args.step_ma, info=info,
                                            analysis=a, samples=samples)))
    rise = f"{a['rise_us']:.0f} us" if a["rise_us"] is not None else "-"
    settle = f"{a['settle_us']:.0f} us" if a["settle_us"] is not None else "not settled"
    print(f"{name}: Vbus {a['vbus_v']:.2f} V, Id {a['hold_a']:.2f} -> {a['step_a']:.2f} A")
    print(f"  R {a['r_ohm'] * 1e3:.0f} mOhm  L {a['l_h'] * 1e6:.0f} uH  tau {a['tau_ms']:.2f} ms  "
          f"PWM delay {a['delay_periods']} periods")
    print(f"  step: rise 10-90% {rise}, overshoot {a['overshoot_pct']:.1f}%, "
          f"settle {settle} (band {a['settle_band_a'] * 1e3:.0f} mA), "
          f"steady error {a['steady_error_a'] * 1e3:.0f} mA, noise {a['noise_a'] * 1e3:.0f} mA rms, "
          f"{a['disturbance_samples']} disturbed samples later")
    for fc in (500, 800, 1000):
        kp, ki = capture_analysis.pi_gains(a["r_ohm"], a["l_h"], a["vbus_v"], fc_hz=fc)
        print(f"  for {fc} Hz: current_kp {kp} current_ki {ki}")
    return a


def speed_defaults():
    text = (pathlib.Path(__file__).resolve().parents[1] /
            "HW/B-G431B-ESC1/motor_config/Inc/drive_parameters.h").read_text()
    return tuple(int(re.search(rf"#define PID_SPEED_{k}_DEFAULT\s+(\d+)", text)[1]) for k in ("KP", "KI"))


def spin(link, name, index, selector, args):
    """Speed step on one wheel; returns the steady-state summary."""
    if abs(args.rpm) > 300 or not 0 < args.seconds <= 10:
        raise SystemExit("spin: |rpm| <= 300 and 0 < seconds <= 10")
    kp0, ki0 = speed_defaults()
    gains = args.kp is not None or args.ki is not None
    kp = kp0 if args.kp is None else args.kp
    ki = ki0 if args.ki is None else args.ki
    rows = []
    try:
        if gains:
            link.send_raw(*ik_link.config_frame(selector, "kp", kp))
            link.send_raw(*ik_link.config_frame(selector, "ki", ki))
        word = ((args.ramp_ms & 0xFFFF) << 16) | (args.rpm & 0xFFFF)
        t0, last_cmd, last_count = time.monotonic(), -1.0, None
        while (now := time.monotonic() - t0) < args.seconds + 0.6:
            if now < args.seconds and now - last_cmd >= 0.05:
                link.send_raw(ik_link.SPEED_WRITE | selector, word)
                last_cmd = now
            r = link.rows()[index]
            if r and r["status_count"] != last_count:  # one entry per fresh reply
                last_count = r["status_count"]
                rows.append(dict(t=round(now, 4), cmd=args.rpm if now < args.seconds else 0,
                                 **{k: r[k] for k in ("rpm", "iq_a", "iq_ref_a", "mc_state",
                                                      "fault_names", "stop_reason")}))
            time.sleep(0.005)
    finally:
        for _ in range(3):
            try:
                link.send_raw(ik_link.MOTOR_STOP | 0x0F, 0)
            except RuntimeError:
                pass
        if gains:
            time.sleep(0.3)  # CONFIG_SET is accepted only once the wheel is IDLE
            link.send_raw(*ik_link.config_frame(selector, "kp", kp0))
            link.send_raw(*ik_link.config_frame(selector, "ki", ki0))
        if args.log:
            args.log.write_text("".join(json.dumps(r) + "\n" for r in rows))
    steady = [r for r in rows if args.ramp_ms / 1000 + 0.5 < r["t"] < args.seconds]
    if not steady:
        print(f"{name}: no telemetry during the run")
        return None
    rpm = [r["rpm"] for r in steady]
    ref = [r["iq_ref_a"] for r in steady]
    s = dict(wheel=name, cmd=args.rpm, kp=kp, ki=ki, rpm_mean=statistics.mean(rpm),
             rpm_sd=statistics.pstdev(rpm), wrong_sign=sum(1 for v in rpm if v * args.rpm < 0),
             samples=len(rpm), iq_ref_mean=statistics.mean(ref), iq_ref_peak=max(abs(x) for x in ref),
             faults=sorted({f for r in rows for f in r["fault_names"]}))
    print(f"{name}: cmd {args.rpm} rpm (kp {kp} ki {ki}): mean {s['rpm_mean']:.1f} sd {s['rpm_sd']:.1f} "
          f"wrong-sign {s['wrong_sign']}/{s['samples']} | Iq ref mean {s['iq_ref_mean']:.2f} A "
          f"peak {s['iq_ref_peak']:.2f} A | faults {','.join(s['faults']) or '-'}")
    return s


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
    p = sub.add_parser("capture")
    p.add_argument("wheel")
    p.add_argument("--step-ma", type=int, default=3000)
    p.add_argument("--out", type=pathlib.Path)
    p = sub.add_parser("spin")
    p.add_argument("wheel")
    p.add_argument("--rpm", type=int, required=True)
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--ramp-ms", type=int, default=300)
    p.add_argument("--kp", type=int)
    p.add_argument("--ki", type=int)
    p.add_argument("--log", type=pathlib.Path)
    args = parser.parse_args(argv)
    session, link = ik_link.Link.open(args.sn, args.elf)
    try:
        if args.command == "status":
            before = link.rows()
            time.sleep(0.3)
            show(before, link.rows())
        elif args.command == "stop":
            link.send_raw(ik_link.MOTOR_STOP | 0x0F, 0)
        elif args.command == "align":
            results = [align(link, name, i, sel) for name, i, sel in wheels(args.wheel)]
            print("build flags: " + "  ".join(
                f"{name}:--hall-shift {r['hall_shift_deg']}"
                for (name, _i, _s), r in zip(wheels(args.wheel), results) if r["align_result"] == 1))
        elif args.command == "capture":
            for name, _i, sel in wheels(args.wheel):
                capture(link, name, sel, args)
        elif args.command == "spin":
            targets = wheels(args.wheel)
            if len(targets) != 1:
                raise SystemExit("spin: one wheel at a time")
            spin(link, *targets[0], args)
        elif args.command == "set":
            for name, _i, sel in wheels(args.wheel):
                link.send_raw(*ik_link.config_frame(sel, args.key, args.value))
                print(f"{name}: {args.key} = {args.value} (RAM)")
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
