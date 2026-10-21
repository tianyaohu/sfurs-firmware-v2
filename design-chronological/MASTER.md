# Master design log

Append only; format in design/README.md.

## 2026-10-16  Motion accuracy is the MVP goal  (who: Frank H + claude)
- Decision: target 1 m command landing within ±10 cm in every direction;
  measure with an overhead webcam and AprilTags, then fit the kinematics table
  to the data.
- Why: the team lead asked for it; hand-measured wheel geometry is provisional.
- Rejected: hand-tuning geometry by eye (no measurement, not repeatable).
- Touches: `prd/index.md`, `src/tools/rig/`, `src/shared/src/rover_kinematics.c`.

## 2026-10-16  Hall-sensor FOC, not sensorless  (who: Frank H + claude)
- Decision: ESCs run Hall-sensor FOC.
- Why: the as-received sensorless image needs ≥1743 RPM, so at robot speeds it
  buzzes in rev-up at up to 6 A (see LESSONS).
- Touches: `src/esc/motor_config/Src/drive_app.c`.

## 2026-10-20  Restructure: code under src/, one folder per doc concern  (who: Frank H + claude)
- Decision: all code lives in `src/{nucleo,esc,shared,sensors,tools}`; documentation
  folders mirror the team's outline (docs, prd, technical-development-plans,
  design-chronological, architecture, system-info, open-questions,
  hardware-test-results, references); unused trees go to `archive/`.
- Why: the repo mixed the CubeMX default layout with a catch-all `HW/`, shared code
  lived in two places, and the ESC bootloader kept 8 byte-identical copies of shared
  files that had to be synced by hand.
- Rejected: leaving the Nucleo project at the repo root (inconsistent with the ESC and
  sensor projects); keeping the bootloader copies behind a drift test.
- Touches: every path; ESC and bootloader images are byte-identical before and after
  (sha256 checked), host tests 22/22.

## 2026-10-20  OpenOCD replaces pyOCD for SWD access  (who: Frank H + claude)
- Decision: the PC tools read/write target memory through `src/tools/swd.py`, which
  runs OpenOCD per probe serial and uses its Tcl RPC port; pyOCD is removed.
- Why: OpenOCD is the common packaged probe tool (apt, Docker), and one wrapper
  gives every tool the same attach-only behaviour.
- Rejected: GDB-server clients per tool; keeping both back ends (one way to do each thing).
- Touches: `esc_update.py`, `ik_link.py`, `docker/Dockerfile`. Host-tested against a fake
  Tcl endpoint only; bench check with real probes still open (ESC `reset_sysreq` is a
  direct AIRCR write).
