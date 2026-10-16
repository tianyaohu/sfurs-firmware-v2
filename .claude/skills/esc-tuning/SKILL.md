---
name: esc-tuning
description: Tune, install and diagnose the B-G431B-ESC1 wheel motor controllers on the SFURS omni robot — Hall alignment, current-loop capture (R, L, PI gains), speed-loop step tests, baking gains, and installing ESC firmware over CAN with esc_update.py --can. Use this whenever the user mentions motors buzzing, wheels not turning or weak torque, oscillation, PI/gain tuning, Kp/Ki, Hall shift or alignment, motor constants, current loop, speed loop, reflashing or updating an ESC, or "the tune is off", even if they don't name a tool.
---

# ESC tuning and install

Four B-G431B-ESC1 (MCSDK FOC, Hall sensors, 8 pole pairs) on one CAN bus,
driven through the Nucleo drive image. Everything here goes over CAN from
the Nucleo's USB; no ESC USB is needed once an ESC runs the current app.
Read `docs/HARDWARE.md` and the Hardware section of `AGENTS.md` first.

Tools (all in `scripts/`, Nucleo must run the drive image, one tool at a time):

| Command | What it does | Moves? |
|---|---|---|
| `esc_tool.py status` | live state, Vbus, faults, Hall shift, app CRC per wheel | no |
| `esc_tool.py stop` | broadcast STOP; clears a FAULT_OVER whose cause is gone | no |
| `esc_tool.py align <W\|all>` | measure Hall phase shift (RAM) | ~¼ turn each way |
| `esc_tool.py capture <W>` | 1 A → 3 A d-axis current step, 16 kHz, R/L/step metrics | clicks, no torque |
| `esc_tool.py set <W\|all> KEY N` | RAM gain: `kp ki` (speed), `current_kp current_ki` | no |
| `esc_tool.py spin <W> --rpm N` | one-wheel speed step, logged; optional `--kp --ki` | yes |
| `esc_update.py <W> --can` | install `build/esc/<W>` over CAN, verify by IDENT CRC | no |
| `build_all.py --shift NE=.. SE=.. SW=.. NW=..` | build ESC apps with baked shifts | — |

Wheels: FR=NE (sel 1), BR=SE (2), BL=SW (4), FL=NW (8).

## Safety, and why

- **Know the battery state; measure it.** `esc_tool.py status` shows each
  ESC's Vbus: ~12 V is the 3S pack, ~4.6 V is USB only, "no reply" is dark.
  Only a row whose reply count moved is live — the Nucleo keeps old values.
- **Wheels off the ground** for align, capture and the first spin after any
  firmware change; a wrong shift or sign drives the robot off the bench.
  You can't measure this, so it needs the user's word.
- **Motion only when the user asks** (align, spin, keyboard_drive). capture
  makes no torque but the rotor can click into place.
- **USB before battery latches undervolt** (FAULT_OVER on every wheel).
  `esc_tool.py stop` clears it; a cold start (USB out, battery off, on) avoids it.
- Flashing over SWD (`esc_update.py --battery-off`) needs the battery OFF;
  over CAN (`--can`) the battery may be ON — the script refuses unless all
  four ESCs answer IDLE with Vbus, and the bootloader checks the CRC.
- Leave everything stopped and say what state the hardware is in.

## Order: inside out

Tune the inner loop before the one wrapped around it. A speed loop tuned
over a bad current loop or a wrong commutation angle hides the real fault
(flipping gain signs once masked a 180° Hall error — see `docs/LESSONS.md`).

### 0. Firmware and state

`esc_tool.py status`: all four IDLE, no faults, `hall shift ... (known)`,
an `app` CRC. Idle Iq of ±25 A is normal: the current offsets are only
calibrated when the motor starts. If the app isn't the build you expect:
`build_all.py --shift ...` then `esc_update.py <W> --can` per wheel
(about a minute each). A failed transfer leaves the ESC in its bootloader;
rerun with `--can --target-in-boot`. First install on a board, or an app
that can't take the boot request: `--battery-off` with that ESC's USB.

### 1. Commutation angle (Hall shift)

`esc_tool.py align all` (battery on, wheels raised). Expect `ok` and
similar shifts on all four (this robot: −59…−64°). `REVERSED` = Hall order
vs phase order wrong (wiring), `NO_EDGES` = rotor stuck or Halls dead.
Shifts are RAM-only until baked: record them in `docs/HARDWARE.md` and
build with `--shift`.

### 2. Current loop

`esc_tool.py capture <W> --step-ma 3000 --out cap.json` for each wheel.

Read it like this:
- **R, L** should agree across the four motors (here: dq R 0.30–0.32 Ω,
  L 135–150 µH). One outlier usually means a disturbance landed in the
  transient — capture again before believing it.
- **Step**: rise 10–90 % ~200–250 µs, overshoot a few %, settled < 1 ms,
  zero steady error. Overshoot is noise-sensitive (±0.05 A rms); trust
  repeated captures, not one number.
- **Disturbed samples later**: isolated false readings the loop reacts to.
  ~4 per capture is the known baseline here (phase U only; open lead in
  `docs/DRIVE.md`). To attribute a spike to a phase, use the capture's fixed
  angle 0: Iα = Id, Iβ = Iq, and MCSDK's Clarke gives ΔIq = −ΔId/√3 when
  only Ia is wrong. Judge a fix by counting spikes over several captures per
  wheel before and after (one capture proves nothing; counts vary 1–10).
- The printed `current_kp/current_ki` for 0.5/0.8/1 kHz come from
  Kp = L·ωc, Ki = R·ωc. With the measured PWM delay 0, 1 kHz is fine; with
  1–2 periods of delay, stay ≤ 0.8 kHz (overshoot grows fast).

Try gains live: `esc_tool.py set all current_kp N`, `... current_ki N`,
capture again, compare. Bake good values in `PID_TORQUE_*`/`PID_FLUX_*`
(`HW/B-G431B-ESC1/motor_config/Inc/drive_parameters.h`). RAM values are
lost on ESC reset, which is also how you undo an experiment.

### 3. Speed loop — in the air, then on the ground

`esc_tool.py spin <W> --rpm 60 --seconds 3 [--kp N --ki N] --log s.jsonl`,
then 30, 120 and −60 RPM. Gains are in counts (544/A) per RPM:
Kp/256 counts per RPM of error, Ki/16384 counts per RPM per ms.
Always translate to amps before judging (`docs/LESSONS.md`).

Signatures:
- **Limit cycle**: rpm swinging ±300 at a 60 RPM command, Iq ref pinned
  at ±6 A, loud buzz, wheel barely turns → gain far too high for the Hall
  speed estimate's delay. Seen in free air with 4096/4424.
- **Hall scatter**: sd ≈ 20–25 RPM and a few wrong-sign samples at 30–60
  RPM with every gain → the estimate (48 edges/rev), not the loop.
- **Too weak on the ground**: spins freely in the air, stalls loaded,
  Iq ref stays well under 1 A for seconds → Kp/Ki too low. Free-air
  stability says little about the ground: the robot's inertia is part of
  the plant. Current defaults 1024/553 (4 counts/RPM, ~1.9 A/s at 30 RPM
  error) are a free-air choice and must be checked on the ground.

Retune on the ground with the user driving slowly (`keyboard_drive.py
--speed 0.3`) and `esc_tool.py set all kp/ki` between runs; log with
`--log`. Bake into `PID_SPEED_*_DEFAULT`.

### 4. Bake and record

1. Update the parameter headers (with a comment saying what was measured).
2. `python3 tests/run_all.py`, `scripts/build_all.py --shift ...` (no warnings).
3. `esc_update.py <W> --can` for each wheel; check `status` shows the new
   CRCs and `(known)` shifts.
4. Capture/spin once more to confirm the baked build behaves like RAM did.
5. Evidence goes in the commit message (numbers, wheels, battery voltage);
   docs describe the current state only; one line in `docs/LESSONS.md` for
   anything that should change what the next person does.
