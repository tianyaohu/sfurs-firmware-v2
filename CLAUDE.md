# SFURS firmware

Omni-wheel SSL robot: Nucleo-H755 (CM7) + four B-G431B-ESC1 wheel controllers
on CAN. Current goal: 1 m command lands within ±10 cm in every direction.

@AGENTS.md

## File structure

- Read in this order: `docs/index.md` (goal) → `prd/index.md` (what to
  build, by phase) → `architecture/index.md` (what each file does) →
  `architecture/drive.md`, `system-info/hardware.md`, `architecture/can-fwu.md`.
- `open-questions/index.md`: unanswered questions; `docs/lessons.md`: hard-won
  one-liners; `hardware-test-results/`: one file per hardware test session;
  `technical-development-plans/`: phased plans, saved before work starts, deleted when done;
  `references/`: datasheet and manual PDFs.
- `design-chronological/`: dated design log (`MASTER.md` plus one file per person).
  Append an entry whenever you make a non-obvious design choice; read it
  before reversing one. Never rewrite old entries.
- All code is under `src/`: `nucleo/` (CubeMX project), `esc/` (wheel
  controller + bootloader), `shared/` (code both use, incl. the CAN protocol),
  `sensors/` (separate sensor app), `tools/` (PC scripts). Host tests are in
  `tests/`; unused trees in `archive/`.
- Per-file roles: `architecture/index.md`. When you add a file or folder, add
  its line there in the same commit.

## Coding conventions

- Our Nucleo code is `src/nucleo/CM7/Core/Src/*.c` modules and CubeMX `USER CODE`
  sections; our ESC code is `src/esc/motor_config/Src/drive_app.c`.
  Everything else under `Drivers/` and MCSDK is generated: do not hand-edit.
- Comments explain why. Public functions get a Doxygen comment (`@brief`,
  parameters, units). `doxygen Doxyfile` builds the API docs into
  `build/doxygen/` and CI runs it.
- Physical units in names and comments (mA, RPM, mm/s, mrad/s).

## Hard rules

- Do not emulate hardware you do not fully know: fake only at a boundary whose
  behaviour is documented (HAL call, CAN frame), never a made-up plant model.
- New C logic gets a host unit test; run `scripts/test.sh` (all
  existing tests, so other firmware is regression-checked) before committing.
- Ask before touching hardware; rules in `system-info/hardware.md`.
- A hardware test is recorded in `hardware-test-results/` before the task is
  called done.
