# Product requirements

What the code must do, by phase. How it does it lives in
[ARCHITECTURE.md](ARCHITECTURE.md) and [DRIVE.md](DRIVE.md). A phase is done
when its acceptance line is met and recorded in [test-results/](test-results/).

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
  recorded. Measured with the camera rig (`docs/rig/`).

## Phase 3: closed-loop motion (only if phase 2 is not enough)

- Heading hold from the IMU; position/velocity estimation (sensor fusion).
- Distance-move command (today only velocity is commanded).
- Acceptance: same 1 m ±10 cm, repeated over a square path.
