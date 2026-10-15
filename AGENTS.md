# Agent coordination and firmware safety

## Before starting work: declare an independent lane

Before editing, building artifacts for hardware, or accessing a probe:

1. Read applicable `AGENTS.md` files and relevant `.opencode/skills/` runbooks.
2. Inspect `git status`, current branch, and `git worktree list`. Preserve
   dirty/untracked files; they may belong to the user or another agent.
3. State the task, owned files, worktree path, branch/base, checks to run,
   and intended merge destination. Identify active agents and shared resources.
4. Ensure the lane is non-overlapping with other agents. If scope, ownership,
   or merge direction is unclear, ask the user before proceeding.

Each coding agent must use its **own branch and worktree**, normally based on
the agreed integration tip. Do not share a working tree/index, switch another
agent's branch, or edit its owned files. A worktree isolates files, not physical
hardware, shared buses, or external build-output directories.

Delegations must name the lane and boundaries explicitly. Parallelize only
independent work. Read-only reviewers may inspect another checkout, but cannot
silently become its editor or hardware owner.

## Integration / merge direction

- Current hardware integration lane: branch `hw-session`, worktree
  `/home/fh/sfurs_firmware/Project1/sfurs-hw-session`.
- Feature/fix lanes merge **into `hw-session` first**, after review and tests;
  the designated integrator performs those merges with a clean index.
- Agreed next integration direction: **`hw-session` → `develop`**, only when
  the user approves. Do not assume authorization to merge to `main` or push.
- A new task must confirm this direction still applies, rather than treating
  the above paths/branches as permanent policy.
- Commit only owned, reviewed changes with explicit file staging. Do not
  sweep other agents' work into a commit. No automatic branch/worktree deletion,
  force removal, reset, or remote cleanup.

## Hardware ownership is exclusive

- Before hardware access, announce the target wheel/probe, operation, power
  requirements, and owner. Confirm other agents and the TUI have released it.
  A free process list alone is not agreement with another active agent.
- Only one owner may command the shared CAN bus/Nucleo at a time. A second
  agent can do offline work; live work waits for explicit release/handoff.
  Distinct worktrees do not make simultaneous flashes or CAN commands safe.
- Confirm the attached probe serial and selected wheel before writes. Quit
  the TUI before debugger access. Resolve RAM addresses from the **flashed**
  ELF; rebuilding in another lane does not update the board.
- Motor battery **OFF for ESC flashing**, ON for calibration/motion checks.
  Obtain a fresh confirmation after cable/power changes; never guess from
  ambiguous wording such as “wheel on.” Use HOTPLUG for ESC RAM access.
- Back up unknown ESC flash before overwriting. Do not alter option bytes,
  mass-erase, flash CM4, or command wheel motion without authorization.
- Release probe sessions and report power/target state and pending requests
  at handoff. Do not leave FWU retry loops or queued motion unattended.

## Evidence and regression rules

- CAN TX ACK and host ARMED/RUNNING are **not** target acceptance or motion.
  Distinguish installed image, calibration, arming, actual rotation, and faults.
- Calibration is RAM-only and resets invalidate prior CAL results. Auto-CAL
  completion must never automatically RUN or write offsets to flash.
- Shared per-wheel changes need all-four-wheel tests and layer-by-layer
  addressing review. Do not generalize success on one wheel to the others.
- Separate measured findings from hypotheses. Log recurring incidents and
  remaining defects in `docs/` and cross-link the relevant skill.
- Record exact build configuration, payload identity, tests, and hardware
  evidence. Keep generated wheel payloads/build outputs out of source commits;
  keep backup binaries outside Git with provenance sidecars.
- Current open issues: target-ready TUI handshake and manual CAN-CAL byte
  order; see `docs/NE_SE_POST_ROLLOUT_RCA.md`. A rollout/documentation commit
  must not claim these defects are fixed.
