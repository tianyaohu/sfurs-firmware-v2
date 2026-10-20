# Master design log

Append only; format in [README](README.md).

## 2026-10-16  Motion accuracy is the MVP goal  (who: Frank H + claude)
- Decision: target 1 m command landing within ±10 cm in every direction;
  measure with an overhead webcam and AprilTags, then fit the kinematics table
  to the data.
- Why: the team lead asked for it; hand-measured wheel geometry is provisional.
- Rejected: hand-tuning geometry by eye (no measurement, not repeatable).
- Touches: `docs/PRD.md`, `scripts/rig/`, `Common/Src/rover_kinematics.c`.

## 2026-10-16  Hall-sensor FOC, not sensorless  (who: Frank H + claude)
- Decision: ESCs run Hall-sensor FOC.
- Why: the as-received sensorless image needs ≥1743 RPM, so at robot speeds it
  buzzes in rev-up at up to 6 A (see LESSONS).
- Touches: `HW/B-G431B-ESC1/motor_config/Src/drive_app.c`.
