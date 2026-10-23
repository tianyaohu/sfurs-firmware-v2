# Product requirements

What the code must do, by phase. How it does it lives in
[ARCHITECTURE.md](../architecture/index.md) and [DRIVE.md](../architecture/drive.md). A phase is done
when its acceptance line is met and recorded in [test-results/](../hardware-test-results/).

## Phase 1: reliable wheel drive

- Each wheel follows a speed command from standstill under load, without
  buzzing, stalls or faults.
- ESCs can be re-flashed over CAN without a probe.
- Acceptance: the robot drives on the ground in all four translation
  directions and both spins from the keyboard.

## Phase 2: accurate open-loop motion

- Kinematics table (wheel angles, radii, contact positions) fitted to
  measured motion, not hand measurements.
- Even breakaway across wheels so starts do not yaw.
- Acceptance: 1 m in any of 8 directions ends within ±10 cm; spin error
  recorded. Measured with the camera rig (own repository; join its poses to the tool logs by `CLOCK_MONOTONIC` time).

## Tuning and calibration (serves phases 1 and 2)

Every number the controllers use is measured on this robot or marked as a
placeholder, and a tool can re-measure it without editing code. Plan:
[phase2-tuning-calibration](../technical-development-plans/phase2-tuning-calibration.md).

- **One source of truth.** Each tunable (gains, limits, motor constants,
  Hall shifts, friction, geometry) is defined once with its unit, its source
  (measured, datasheet or placeholder) and the date; firmware and tool
  defaults are generated from it. Robot data and floor data are separate,
  so a new floor does not touch the robot's values.
- **Motor model per wheel**, measured with the wheels raised: R, L, back-EMF
  constant Ke, torque constant Kt (cross-checked by a static torque test, not
  only derived from Ke), rotor and wheel inertia J, bearing viscous and
  Coulomb friction, Hall shift and Hall edge map.
- **Gains from the model.** Current-loop gains from R and L (exists today);
  speed-loop gains from J, Kt and the speed estimate's delay, then checked on
  hardware.
- **Floor model per surface**, measured on that floor: Coulomb and viscous
  friction per body axis, breakaway level per axis, per-wheel scale; robot
  mass and yaw inertia.
- **Trustworthy results.** Each identification repeats within a stated
  spread and is checked against nominal values: outside ±30 % it warns,
  outside ±50 % it refuses to store the value.
- **Safe and quick.** Every run is guarded (speed, current, Vbus, time) and
  stops cleanly on Ctrl-C. All four wheels raised: under 5 min. A new floor
  at a venue: under 15 min, with no rebuild or reflash.
- Acceptance: no placeholder left in the control path; repeat agreement
  R ±5 %, Ke ±3 %, J ±10 %, floor Coulomb ±15 %; the final startup gate (H6)
  passes with friction from the measured floor model; the floor
  recalibration timed on the mat.

## Phase 3: closed-loop motion (only if phase 2 is not enough)

- Heading hold from the IMU; position/velocity estimation (sensor fusion).
- Distance-move command (today only velocity is commanded).
- Acceptance: same 1 m ±10 cm, repeated over a square path.
