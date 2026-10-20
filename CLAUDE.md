# SFURS firmware

Omni-wheel SSL robot: Nucleo-H755 (CM7) + four B-G431B-ESC1 wheel controllers
on CAN. Current goal: 1 m command lands within ±10 cm in every direction.

@AGENTS.md

## File structure

- Read in this order: `docs/CONTEXT.md` (goal) → `docs/PRD.md` (what to
  build, by phase) → `docs/ARCHITECTURE.md` (what each file does) →
  `docs/DRIVE.md`, `docs/HARDWARE.md`, `docs/CAN_FWU.md`.
- `docs/OPEN_QUESTIONS.md`: unanswered questions; `docs/LESSONS.md`: hard-won
  one-liners; `docs/test-results/`: one file per hardware test session;
  `docs/plans/`: phased plans, saved before work starts, deleted when done;
  `docs/reference/`: datasheet and manual PDFs.
- `docs/design/`: dated design log (`MASTER.md` plus one file per person).
  Append an entry whenever you make a non-obvious design choice; read it
  before reversing one. Never rewrite old entries.
- Source layout and per-file roles: `docs/ARCHITECTURE.md`. When you add a
  file or folder, add its line there in the same commit.

## Coding conventions

- Our Nucleo code is `CM7/Core/Src/*.c` modules and CubeMX `USER CODE`
  sections; our ESC code is `HW/B-G431B-ESC1/motor_config/Src/drive_app.c`.
  Everything else under `Drivers/` and MCSDK is generated: do not hand-edit.
- Comments explain why. Public functions get a Doxygen comment (`@brief`,
  parameters, units). `doxygen Doxyfile` builds the API docs into
  `build/doxygen/` and CI runs it.
- Physical units in names and comments (mA, RPM, mm/s, mrad/s).

## Hard rules

- Do not emulate hardware you do not fully know: fake only at a boundary whose
  behaviour is documented (HAL call, CAN frame), never a made-up plant model.
- New C logic gets a host unit test; run `python3 tests/run_all.py` (all
  existing tests, so other firmware is regression-checked) before committing.
- Ask before touching hardware; rules in `docs/HARDWARE.md`.
- A hardware test is recorded in `docs/test-results/` before the task is
  called done.
