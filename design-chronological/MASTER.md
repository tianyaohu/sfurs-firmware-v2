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

## 2026-10-20  No Python in the repo: C tools, CTest, camera rig out  (who: Frank H + claude, lanes delegated to Codex)
- Decision: PC tools are C programs in `src/tools/` over `lib/ocd.c` (OpenOCD Tcl client) and the firmware's own
  headers; host tests are CMake/CTest (`tests/`); build and test drivers are shell (`scripts/`). The overhead
  camera rig (OpenCV/AprilTag) left to its own repository (history: branch `rig-split`).
- Why: one toolchain (gcc/CMake) for firmware, tests and tools; the CAN/mailbox constants come from
  `can_datatypes.h` instead of being copied, so drift is a compile error; no Python/uv/pyocd to install.
- Consequence: `move_test` and `tune_ground` no longer run the camera in-process. They drive, guard and log
  with `CLOCK_MONOTONIC` stamps and per-move windows; judging a move against camera poses is done on the rig
  side by joining on time. The live "path stays on the mat" and "tag seen still" preflights are gone until the rig
  repo provides them.
- Rejected: Rust/Go tools (new toolchain, cannot include the firmware headers); shell-only tools (numerics and
  ELF handling too awkward); keeping a Python rig in this repo (the aim was zero Python).
- Verified: ESC binaries from `scripts/build.sh` are byte-identical to the former Python build; numeric ports match
  recorded Python outputs (`tests/golden/`) at 1e-9; every ported test was mutation-checked. Not verified on
  hardware: everything that drives or programs a board (only read-only OpenOCD attach to the Nucleo was run).
- Touches: all of `src/tools/`, `tests/`, `scripts/`, CI, Dockerfile, docs.

## 2026-10-21  Motor command protocol v1 and a staged rewrite of the ESC command + speed layer  (who: Frank H + claude)
- Decision: replace `SPEED_WRITE`/`MOTOR_STOP` with one fixed 8-byte CONTROL frame (version, mode speed/current/disable,
  setpoint, feed-forward mA, sequence, ESC-issued session) and a reply to every frame (outcome, reason, lifecycle);
  one shared codec (`motor_protocol.c`); an explicit ESC lifecycle (DISARMED, READY, STARTING, RUNNING, HOLD, FAULTED)
  with ARM, 100 ms watchdog that disarms, and ordered lifecycle frames; later a P + feed-forward speed loop replacing
  the startup state machine, zero-current HOLD instead of output-off parking. Design artifacts A-D in
  `technical-development-plans/esc-command-v2/`, approved by Frank H on this date.
- Why: the old path grew by patching (three byte layouts, three feed-forwards, three speed estimators, a torn command
  read, rejected frames feeding the watchdog, no command feedback); the code goes to an outside reviewer. Startup is
  judged by per-boundary Hall tracking deadlines, because a wheel that tracks its reference perfectly fails a raw
  "first edge within 20 ms" bar about 86 % of the time (the first edge can be a whole 7.5 deg sector away).
- Rejected: keeping the 4/8-byte SPEED_WRITE (length changes meaning, no version); separate frames for speed and
  feed-forward (they can disagree); copying TIGERs' SPI struct (unverified, different transport, licence).
- Touches: `src/shared/`, `src/esc/motor_config/`, `src/nucleo/CM7/`, `src/tools/`, tests, `architecture/`.

## 2026-10-21  Stage 4a (new speed loop) before Stage 3 (config ack)  (who: Frank H + claude)
- Decision: after the Stage 2 protocol checks, build the new speed loop next and do config ack/readback after it;
  the remaining Stage 2 mat starts are skipped.
- Why: the first mat start on protocol v1 stalled on static friction: Iq followed its reference, the old PI's I term
  climbed ~7.4 A/s to ~3 A over 360 ms with the wheels creeping, then the wound-up I term overshot one wheel to
  450 RPM (hardware-test-results/2026-10-21-h2-install-raised.md). The protocol is not the limit; the loop is.
  More starts on the old loop would each end in the overspeed abort and add nothing.
- Rejected: Stage 3 first (better tuning workflow, but the ground start waits); a RAM friction ff on the old loop to
  drive today (the three-ff stack is what Stage 4a removes).
- Touches: technical-development-plans/phase1-esc-command-v2.md (order), src/esc/motor_config/.

## 2026-10-21  Stage 4a speed loop: stall ramp, slip clamp, over-speed bleed; HALL_LOST redefined  (who: Frank H + claude)
- Decision: P + I with three integrator rules (stalled ramp at 10 A/s once the reference is 4 edges ahead, slip
  clamped at 4 edges; conditional integration; halve I on a progress edge above 1.15x target), one speed estimate,
  output via MCSDK torque mode from drive_app's tick (MCSDK files back to stock), Ki kept, HALL_LOST = Hall-code fault
  not "estimate invalid after motion". Spec: technical-development-plans/esc-command-v2/E-speed-loop.md.
- Why: today's records show the stuck-then-runaway start comes from the I term winding up while the reference runs
  away from a stuck wheel; a re-stuck loaded wheel looks like an invalid estimate, so faulting on that would stop
  normal mat starts. Ki 0 cannot hold speed against kinetic friction with ff 0.
- Rejected: the DSC STARTING state machine (14 keys, hand-off hold, same 2.25x overshoot), Ki 0, chasing D's
  tracking deadlines before Stage 5 measures friction ff.
- Touches: src/esc/motor_config/, src/tools/, tests, architecture/drive.md, command-lifecycle.md.

## 2026-10-22  Build Stages 3, 5 and the review pack before the hardware gates  (who: Frank H + claude)
- Decision: skip the hardware gates for now and build the rest (Stage 3 config ack/readback, Stage 5 current mode and
  the friction/Ke fit tools, H6 tool prep, Stage 8 review pack); run H4a, H3, H5 and H6 together in one hardware session.
- Why: hardware time is available as one block (2026-10-23); host work does not need the robot.
- Risk accepted: a loop problem found at H4a may force rework in later stages; those stages touch mostly separate code
  (config path, current mode, PC tools). Friction values for the ff stay unmeasured until that session.

## 2026-10-22  Stage 3 CONFIG keys and Stage 5 measurement in current units; Ke fit deferred  (who: claude)
- Decision: CONFIG v1 key table = 2 limits (IQ_LIMIT_MA, HALL_SHIFT_DEG) and 6 tuning keys (speed Kp/Ki, current Kp/Ki,
  stall ramp, stall time); one wheel per frame; refusals in the order structure, NOT_IDLE, UNKNOWN_KEY, BAD_VALUE. The
  Nucleo stores CONFIG_REPLY in the link row (mailbox ABI 8). Current mode: the Nucleo latches ik_drive_mode per session
  and sends per-wheel CURRENT frames; the ESC clamps to the effective limit (INHIBITED(LIMITED)), no HOLD or stall monitor.
  Friction is measured as the steady q-current each wheel needs at several speeds (Iq = Ic*sign(v) + b*v per wheel),
  plus breakaway current from a current-mode ramp.
- Why: the feed-forward is sent in mA, so measuring friction in mA needs neither Kt nor the robot's mass or wheel radius;
  the body-frame model (friction_ff.c) can be fitted to these numbers later. Ke needs Vq, which no CAN channel carries
  (the recorder's 8 channels are full), so a Ke fit would mean a new telemetry path; not needed for the ff.
- Touches: src/shared (codec), src/esc (drive_app, motor_cmd), src/nucleo (ik_drive), src/tools (friction_map).

## 2026-10-22  HOLD parks after 1 s  (who: claude, decided with Frank H)
- Decision: HOLD (bridge on, Iq 0 at standstill) lasts at most 1000 ms (MOTOR_CMD_PARK_MS); then the ESC switches the
  output off and returns to READY, still armed, STATUS stop reason PARKED (8; 6 and 7 stay retired). A pause shorter than
  1 s resumes from HOLD without an MCSDK start; a longer stop restarts through READY -> STARTING.
- Why: on hardware (raised) HOLD buzzed audibly on the motors: the speed loop asked for exactly 0 A but the FOC current
  loop acted on measurement noise (Iq sd 0.03-0.07 A, peaks 0.3 A). Dropping HOLD entirely would make every start pay
  the 13-53 ms MCSDK start, which differs per wheel and breaks the 2 ms start-sync bar; parking only long stops keeps
  fast synchronized restarts within play and silence at rest. Lower current gains in HOLD were not tried.
- Touches: src/esc (motor_cmd, drive_app), src/shared/can (ESC_STOP_PARKED), src/tools (ik_link stop names).

## 2026-10-22  Friction ff: Coulomb per axis and a body-level breakaway boost  (who: claude, decided with Frank H)
- Decision: friction_ff.c takes separate Coulomb forces for x and y (force (Cx*ux, Cy*uy) along the commanded
  direction u) and breakaway levels for x, y and yaw. ik_drive.c decides breakaway for the whole robot: while the twist
  is nonzero and fewer than 2 wheels report >= 30 RPM along their targets, all four get the breakaway levels; at most
  ik_drive_ff_breakaway_ms per stuck episode; re-armed when the robot sticks again. Mailbox ABI 9;
  --ff-coulomb-xy-n is replaced by --ff-coulomb-x-n/-y-n plus --ff-breakaway-*.
- Why: on the mat the IK table is consistent (null-space residual as small as raised) but each ESC's stall ramp frees
  its wheel at a different moment and the body pivots about the wheels still stuck. A constant ff of ~1 A helped vy
  (breakaway ~2x sooner, spread ~3x tighter) while vx needed ~3 A to unstick and then overshot up to 2.2x: one level
  cannot be both, and strafing turns only 54 % of wheel force into vx (84 % for vy). Deciding breakaway from two
  wheels' reply speeds keeps the four wheels pushing together; ending on two (not all four) stops a wheel that has
  broken free from being overdriven while one is still stuck. The ESC's 50 A/s ff slew smooths the step back.
- Touches: src/shared (friction_ff), src/nucleo (ik_drive), src/tools (drive_ops, ik_link).
