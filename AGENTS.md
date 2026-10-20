# Working in this repo

Firmware for an omni-wheel robot: Nucleo-H755 (CM7) drive + four
B-G431B-ESC1 wheel controllers on CAN. Start with `README.md`; system details
are in `docs/DRIVE.md`, `docs/HARDWARE.md`, `docs/CAN_FWU.md`.

## Priorities

1. Working code over paperwork. Spend effort on the change and its tests,
   not on describing the session.
2. Fix causes, not symptoms. If a module is a pile of patches and flags,
   rewrite it cleanly instead of adding another layer.
3. Keep one way to do each thing. No parallel build profiles, diagnostic
   `#ifdef` variants or duplicate scripts; use runtime configuration
   (e.g. CAN `CONFIG_SET`) or a short-lived branch for experiments, and delete
   what you replaced.

## Documentation rules

- Docs describe the system **as it is now**. Edit the relevant doc in the
  same commit as the behaviour change; delete text that is no longer true.
- Do not add session logs, handoff notes, run records, per-experiment docs,
  RCA write-ups or "status" sections. Put test and hardware evidence briefly
  in the commit message or PR description.
- A hard-won fact that should change future behaviour gets one line in
  `docs/LESSONS.md`. Nothing else.
- Code comments explain *why*, not what the next line does.
- Reports to the user: results, failures and decisions needed; keep them short.

## Code and tests

- Nucleo: our code lives in `CM7/Core/Src/*.c` modules and `USER CODE`
  sections of CubeMX files. ESC: `motor_config/Src/drive_app.c`, `USER CODE`
  sections, and the parameter headers listed in `HW/B-G431B-ESC1/README.md`.
- New C logic gets a host test that compiles the real source against fakes
  (pattern: `tests/test_esc_drive_app.py`). `python3 tests/run_all.py` must
  pass before committing; `scripts/build_all.py` must build without warnings.
- Anything addressed per wheel is tested for all four wheels.

## Git

- Branching follows `CONTRIBUTING.md`: topic branches `type/topic`
  (`feature/`, `fix/`, `hw/`, `docs/`, `chore/`, `release/`) off `develop`
  (agents: `develop_ai`, below),
  MR into `develop`; `main` only from `develop`/`release/*`.
- Agents branch off `develop_ai` and merge back into it (no MR). Promotion
  `develop_ai` -> `develop` is one MR the user approves.
- Each agent works on its own topic branch and worktree; never edit another
  agent's worktree or switch its branch.
- Pushing your own topic branch is fine. Never push to `develop` or `main`;
  opening or merging an MR into them needs the user's approval.
- Stage files explicitly; commit only your own changes. No force-push,
  reset, or deletion of branches/worktrees you did not create unless asked.
- Build outputs, backups and generated payloads stay out of git.

## Hardware

Read `docs/HARDWARE.md` first. In short:

- Ask before touching hardware and say what you will do (board, probe,
  operation, power state needed). Only one agent or tool uses a probe or the
  CAN bus at a time.
- The user must confirm the battery state before each hardware step and
  again after any cable or power change; never infer it.
- Battery OFF to flash an ESC. Wheels off the ground for alignment and the
  first run after any firmware change. Motion only when the user asks for it.
- Check the full probe serial before writing. Back up an ESC's flash before
  overwriting. Never mass-erase, change option bytes or flash CM4.
- Leave the hardware stopped and probes released when you finish, and tell
  the user the power and firmware state.
