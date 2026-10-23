# Plan: tuning and calibration (measured model, one source of truth)

Serves the [PRD section "Tuning and calibration"](../prd/index.md). Saved 2026-10-22, before work starts.
Runs after [phase1-esc-command-v2](phase1-esc-command-v2.md) H5 retest; its outputs feed H5 step 3 and H6.

## Context

Measured today: pole pairs (8), Hall shifts (-60/-64/-59/-62 deg, baked), dq R 0.30-0.32 Ohm and L 135-150 uH
(`esc_tool capture`), mat breakaway and rolling current per wheel (A, not N), H4a speed-loop behaviour.
Not measured: Ke, Kt, rotor+wheel inertia J, bearing friction, Hall edge map, floor friction in body units, robot
yaw inertia. `pmsm_motor_parameters.h` still holds the Workbench placeholder (0.22 Ohm, 0.11 mH, 3.0 V/kRPM); the
Nucleo friction ff uses a placeholder 1/Kt of 10 A/N·m.

Tunables are spread over `drive_parameters.h` (27 defines), `motor_speed.h` (11), `motor_cmd.h` (3), `ik_drive.h` (9),
`rover_kinematics.h` (2, incl. the 900 RPM wheel cap), `drive_ops.h` (3), `scripts/build.sh --shift` and eleven
`--ff-*` tool flags. `src/tools/esc_defaults.cmake` already copies six ESC defines into a tool header: the pattern
to extend, not replace.

Reference: TIGERs 2026 (`/home/fh/sfurs_firmware/reference/README.md`, ideas only, non-commercial licence). Their
drive motor (Nanotec DF45L024048: 8 pp, R 0.325 Ohm, L 130 uH, Ke 0.0225 V·s/rad) matches ours closely. They identify
Ke from two steady voltage points, J and b from a current step, body yaw friction and inertia from a gyro sweep and a
coast-down, and store Km = 1.5·Ke. We improve on that where noted (**[+]**).

Facts the plan relies on:
- MCSDK dq is amplitude-invariant: Vq = R·Iq + L·dIq/dt + ωe·λ, torque = 1.5·pp·λ·Iq. With Ke_dq = pp·λ
  (V per mechanical rad/s), Kt = 1.5·Ke_dq. Assumes sinusoidal back-EMF, hence the static cross-check in stage 2.
- The 1 kHz recorder (`can_datatypes.h:88-103`, 384 samples) logs ref, estimate, P, I, ff, Iq ref, Iq and a
  packed word; no voltage. Only the 16 kHz current capture logs Vd/Vq, and `capture_analysis.c` already converts
  them to volts.
- `friction_map` velocity segments gave 0 accepted points on the mat in vx (stick-slip at 0.1-0.8 m/s, sd 20-60 RPM).
  One wheel alone breaks free at ~1.4 A; all four together need 4-6 A each.

## Decisions to confirm (my recommendation first)

1. **Storage**: `calibration/robot.cal` (per robot) plus `calibration/floor-<name>.cal` (per surface), plain
   `key_unit = value  # source date` text. CMake generates the ESC, Nucleo and tool headers from `robot.cal`
   (extending `esc_defaults.cmake`); tools read a floor file at run time with `--floor FILE`. Alternative: keep
   headers as the source and generate the rest from them (less change, but measured data then lives in C headers).
2. **Tool flags**: the eleven `--ff-*` flags go; `--floor FILE` plus `--set key=value` for one-off overrides
   (one way to do each thing). Alternative: keep the flags as overrides of the file.
3. **Identification lives in `esc_tool ident`** (motor, raised) and `friction_map` (floor); `tune_ground` merges into
   `friction_map` and is deleted.
4. **Hall edge map** (stage 4) is done in this plan, not deferred: the startup metric and the speed observer
   (ideas log #4) both need it.

## Stages

Each stage: own commits; `scripts/test.sh` and `scripts/build.sh` (0 warnings); docs in the same commit; fitting
code host-tested on analytic signals (the fit's maths, not an emulated motor); per-wheel code tested on all four
wheels. Hardware: ask first, measure Vbus, wheels raised for the first run after a firmware change, record in
`hardware-test-results/`.

### 0. One source of truth for tunables (host only; this is the queued limits refactor)
1. Inventory every tunable and limit (files above, plus tool defaults and literals found by grep) into a table: name,
   unit, value, owner (ESC/Nucleo/tool), source (measured/datasheet/placeholder), runtime-settable or compile-time.
2. Create `calibration/robot.cal` and the generator; ESC, Nucleo and tools include the generated headers; delete the
   copies. The `rover_ik` 900 RPM wheel cap becomes a robot value (runtime-settable on the Nucleo mailbox).
3. `calibration/floor-mat.cal` from the 2026-10-22 mat numbers, marked placeholder where 1/Kt is involved.
- Done: a host test parses `robot.cal` and checks each generated header against it; grep finds no tunable literal
  outside `calibration/`; ESC and Nucleo images byte-identical to before when the values are unchanged.

### 1. Recorder channel set with voltages (ESC + tools, host first)
1. A channel-set field in the recorder START word: set 0 = today's speed-loop channels (default, unchanged);
   set 1 = electrical: applied ref, estimate, Iq, Id, Vq, Vd, Vbus, packed word. Same 384 x 8 buffer.
2. Trigger option "now" (no reference edge needed), for current-mode and coast-down tests.
3. `drive_record.c` decodes set 1 to volts and amperes using the capture's Vq scaling.
- Done: host tests for both sets and the trigger on all four wheels; raised, `esc_tool spin` at 300 RPM with set 1
  shows Vq ≈ R·Iq + Ke·ω with the placeholder Ke within 30 %.

### 2. Ke and Kt (raised)
1. `esc_tool ident <W|all> --ke`: speed mode at ±150, 300, 450, 600, 750, 900 RPM, 1 s settle, 0.3 s recorded each.
   Fit (Vq - R·Iq) = Ke_dq·ω + d·sign(ω) by least squares. **[+]** TIGERs average two points; six signed points
   let d absorb the dead-time voltage error and give a residual to judge the fit.
2. Kt = 1.5·Ke_dq. Static cross-check **[+]**: robot raised, a known mass hangs from a string wound on the wheel (radius
   from `robot.cal`), current mode, find the Iq that just lifts it and the Iq at which it just lowers. Kt =
   m·g·r / mean(Iq); half the difference is bearing Coulomb friction. One wheel is enough to validate the 1.5 factor.
3. Write Ke, Kt per wheel into `robot.cal`; replace the 1/Kt placeholder; update `pmsm_motor_parameters.h` values
   through the generator (first grep where the generated MCSDK code uses its BEMF constant; Hall FOC may not).
- Done: Ke repeat within 3 % over 3 runs per wheel; static Kt within 10 % of 1.5·Ke; four wheels within 10 % of each
  other (a larger spread means a motor or a measurement problem: stop and look).

### 3. Inertia, viscous and Coulomb friction of the raised wheel
1. Current step: speed mode to 200 RPM, switch to current mode at the held Iq + 0.5 A, record (set 1, trigger now).
   J = Kt·ΔIq / (dω/dt) from a line fit over the first 50 ms. **[+]** TIGERs time the 63 % point of the full
   response, which mixes J with the friction model; the initial slope does not.
2. Coast-down: from 600 RPM, output off, record ω(t). With J known, fit dω/dt = -(c + b·ω)/J for c and b.
   If the 384 ms window ends early, chain two recordings (second with a delay) or use 50 Hz telemetry.
3. Sanity bands per value against the four-wheel median (warn ±30 %, refuse ±50 %), as TIGERs do against nominals.
- Done: J repeat within 10 %; b and c per wheel stored; free-air friction current c/Kt agrees with the measured
  ~0.1 A at 120 RPM within 50 %.

### 4. Hall edge map (raised)
1. `esc_tool ident <W|all> --hall-map`: steady 120 and 240 RPM, record set 0 (edge count + Hall state per ms) for
   several turns; edge angles from edge timing x speed, 48 per wheel, averaged over turns, stored in `robot.cal`.
2. `startup_metrics` uses the map instead of uniform 7.5 deg sectors.
- Done: map repeats within 0.5 deg between the two speeds; host test of the map fit on a synthetic uneven edge pattern.

### 5. Speed-loop gains from the model
1. Ground effective inertia per axis: J_wheel + the robot mass reflected through the IK matrix (from `robot.cal`
   geometry and mass), so the gain choice uses the loaded plant, not free air.
2. `esc_tool ident` prints Kp/Ki for a chosen bandwidth, limited by the speed estimate's delay (6-edge average at
   the gate speed), the way `capture` prints current-loop gains.
3. Compare with today's 1024/553 on the H4a spin and mat start sets.
- Done: model gains no worse than 1024/553 on overshoot and breakaway spread (H4a metrics); the chosen pair stored
  in `robot.cal` with its derivation.

### 6. Floor friction model (mat)
1. `friction_map` fits in body units: per-wheel force = Kt·Iq / r through the IK matrix to body force and torque,
   giving Coulomb x, y, w and viscous terms directly (no hand conversion from mA).
2. Steady segments: drive with the breakaway boost and the current floor file's ff on, legs up to 1.6 m (2 m x 2 m
   mat), speeds 0.5-1.5 m/s, so the wheels roll steadily instead of stick-slipping at 0.1-0.3 m/s.
3. Breakaway in force space: ramp a body force along each of 8 directions on all four wheels together, stop at two
   wheels moving (same rule as the boost); per wheel a max-RPM guard zeroes only that wheel (the FL abort of
   2026-10-22 ended the whole run). Gives breakaway x/y/w and per-wheel scale.
4. `tune_ground` merged in and deleted; `friction_map --write floor-<name>.cal` writes the result with source and date.
- Done: fit residual RMS < 20 %; Coulomb repeats within 15 % over 3 runs; a vx and a vy `move_test` with the new
  file is no worse than the hand-tuned starting point recorded 2026-10-22.

### 7. Robot mass and yaw inertia
1. Mass: weigh the robot (user), store in `robot.cal`.
2. Yaw inertia: TIGERs-style coast-down from a spin on the mat: current ff off, output off at 4 rad/s, yaw rate
   from wheel odometry while the left-null residual stays small (no slip), fit with the floor model's w terms.
   Use the IMU instead if one is fitted (open question).
- Done: inertia repeat within 15 %; value stored. Used later for acceleration feed-forward.

### 8. Venue workflow and validation
1. `architecture/drive.md` "Calibration" section: raised motor ident (stages 2-4) once per robot or motor change;
   floor (stage 6) per surface. Timed on the mat.
2. H6 of phase 1 with `--floor calibration/floor-mat.cal`.
- Done: raised ident of four wheels < 5 min; floor recalibration < 15 min with no rebuild; H6 recorded.

## Hardware sessions

| Session | Stages | Setup |
|---|---|---|
| A | 1-4 | raised, 3S/4S, one string + known mass for the static Kt check |
| B | 5 (spin part), 6, 7 | mat 2 m x 2 m, operator ready to stop |
| C | 5 (mat part), 8 | mat, H6 |

## Out of scope

Kinematics geometry fit (phase 2, needs the camera rig fixes in `open-questions/`); online surface-adaptive
friction (ideas log #5); current-sense phase-U glitch (open lead in `architecture/drive.md`).

## Risks

- Vq at low speed is dominated by dead-time error: fit only at ≥150 RPM and keep the sign term.
- The 1.5 factor depends on MCSDK's scaling: the static torque test is the guard.
- Raised J excludes the robot; ground gains need the reflected-mass step (stage 5), not raised J alone.
- Floor friction varies with battery-related speed and mat wear: repeat spreads, not single runs, decide pass.
