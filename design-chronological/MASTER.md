# Master design log

Append only.

## 2026-10-29  Phase 1 kinematics is the linear wheel map  (who: cloud agent)

- Decision: the first owned module maps a body twist to four raw motor RPM
  (FR, BR, BL, FL) with the provisional wheel map: contacts hand-measured,
  front rollers ±30° from forward, rear ±35°, radius 32.75 mm, direct drive,
  left motors mirrored (`motor_sign` −1). A command past 900 RPM scales every
  wheel by the same factor. Zero is left at zero. The function is host-tested
  and not called from CM7 yet.
- Why: Phase 1 of the from-scratch tree is the math the drive will need, checked
  on the PC. The map matches the reviewed provisional geometry. One shared scale
  keeps the requested direction; clamping a single wheel would skew the twist.
- Rejected: per-wheel clamp; a speed floor; copying the previous tree's test
  harness and validation matrix into this change.
- Touches: `src/shared/inc/rover_kinematics.h`, `src/shared/src/rover_kinematics.c`,
  `tests/test_rover_kinematics.c`, `scripts/test.sh`.
