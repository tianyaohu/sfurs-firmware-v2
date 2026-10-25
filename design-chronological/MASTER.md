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

## 2026-10-22  Current limit 8 A (was 6 A)  (who: claude, decided with Frank H)
- Decision: `IQMAX_A` 8 on the ESC (CONFIG range and default limit follow it); the Nucleo's ff and current-mode
  clamps 8000 mA. I2T unchanged (3.26 A nominal, 30 A²s, derate to 3 A), so it stays the thermal guard.
- Why: mat vx starts with the breakaway boost (4S, 2026-10-22) held all four wheels at the 6 A limit for ~100 ms
  with the robot not moving; breakaway came at 0.5-0.66 s and one run aborted with FL stuck at -6 A. Board sense range
  is +-60 A (3 mOhm x 9.14), so the limit is a tuning choice, not a hardware one. At a steady 8 A the I2T derates
  after ~0.56 s (6 A: ~1.2 s). The CONTROL_REPLY Iq byte saturates at 6.35 A; the recorder and STATUS do not.
- Touches: src/esc (drive_parameters.h), src/nucleo (ik_drive.h), host tests derive limits from the constants.

## 2026-10-22  Fault cause on the probe reply; Nucleo counts ESC restarts  (who: claude, decided with Frank H)
- Decision: an ARM to a FAULTED ESC is REJECTED with its latched cause before the session is checked (amends the
  lifecycle rule "a wrong-session ARM is BAD_SESSION in any state"); DISABLE while FAULTED stays APPLIED/NONE.
  The Nucleo counts per wheel a DISARMED reply in the ESC's boot session (1 + select) from a wheel that was armed,
  faulted or in another session (`ik_drive_link.restarts`, ABI 10); keyboard_drive shows and logs it and writes the
  stop error as the last log line.
- Why: tonight a FAULTED BR answered every new session's session-0 probe with BAD_SESSION, so the tools printed
  "cause not reported". And all four ESCs restarted together at least twice during keyboard drives (boot sessions
  2/3/5/9 again, the RAM 7 A limit back at 8 A); the Nucleo re-armed them silently and only "BR faulted" was
  visible. The bootloader clears the MCU reset flags, so the app cannot report a reset cause; the boot session can
  mark a restart without a protocol change.
- Touches: src/esc (motor_cmd.c), src/nucleo (ik_drive), src/tools (ik_link, esc_tool, keyboard_drive), docs.

## 2026-10-22  A stall rests and retries instead of latching FAULTED(STALL)  (who: claude, decided with Frank H)
- Decision: when the stall monitor fires (>= 80 % of the limit, no Hall progress for STALL_MS) the speed loop rests:
  Iq 0 and ff 0 for 300 ms with the output on, I/slip/timer cleared, then a fresh push from 0 (the stall ramp again).
  The wheel stays RUNNING; motion replies are INHIBITED(STALLED, new code 22); FAULT_STALL (33) is retired. Recorder
  s7 bit 6 marks the rest. MCSDK faults (over-current etc.) and HALL_LOST still latch.
- Why: on the tournament-like carpet a stuck wheel latched FAULTED for the rest of a drive and needed DISABLE +
  CLEAR_FAULT + ARM; a competition robot must keep driving on the other wheels and retry. Heat is I2T's job (it
  derates a wheel that keeps retrying to 3 A); TIGERs' motor firmware has no stall fault at all. Reverses the C4 /
  E-speed-loop "STALL -> FAULTED" decision (2026-10-21).
- Touches: src/esc (motor_speed, motor_cmd, drive_app), src/shared (motor_protocol.h, can_datatypes.h),
  src/tools (ik_link, drive_record), docs.

## 2026-10-22  Own rotor angle between Hall edges (override of MCSDK's HALL_CalcElAngle)  (who: claude, decided with Frank H)
- Decision: drive_app.c overrides MCSDK's weak HALL_CalcElAngle with hall_angle.c. Below one edge per 20 ms (~62 RPM),
  or with the speed against the last edge, the angle is the centre of the sector the Hall sensors report; at speed it
  extrapolates from the edge, holds at the next boundary for a quarter sector period more, then goes to the centre.
  No CONFIG switch: tonight's stock-MCSDK recordings are the A/B baseline.
- Why: the user asked why 6-8 A does not move the robot while 4-6 A does in current mode from rest. MCSDK runs the
  angle up to 60 deg past the last edge and returns to the sector centre only after 150 ms without an edge, so a
  wheel that creeps over an edge and sticks has its current up to 60 deg off (half the torque per amp). In all of
  tonight's mat recordings 109 of 271 high-current Hall gaps >= 40 ms ended 150-169 ms after the previous edge
  (4-7 per 10 ms bin elsewhere), 107 of them forward: stuck wheels moved when MCSDK reset its angle. The speed loop's
  stall ramp only reacted to the missing torque by raising Iq. TIGERs' Hall-only mode also uses the sector centre.
- Expected on hardware: the 150 ms gap peak disappears; vx/vy breakaway at lower current; fewer limit hits.
- Touches: src/esc (hall_angle.c/.h new, drive_app.c), tests (test_hall_angle, drive_app fake HALL fields), docs.

## 2026-10-24  Drive tools send the measured mat friction preset by default; ESC stuck/bleed rules kept  (who: claude, decided with Frank H)
- Decision: `keyboard_drive` and `move_test` write `DRIVE_FF_PRESET_*` (drive_ops.h) when no `--ff-*` option is
  given; any `--ff-*` term is a full manual spec as before; `--ff-off` turns the model off. The Nucleo still boots
  with it off. The ESC speed loop's stuck ramp and over-speed bleed stay as they are.
- Why: mat yaw at 2 rad/s (50 wheel RPM) limit-cycled: stuck ramp to 1.2-2.3 A, lurch, per-edge bleed halving I
  to ~0, stuck again every ~0.4 s (the user's 40 degree steps). TIGERs avoids this with command-side Coulomb ff and
  no such rules. With the measured ff, yaw tracked 93-95 % (83-87 % without), Iq SD 0.16 vs 0.42 A, collapsed 0-4 %
  vs 31-35 %; vy and vx 0.5 m/s went from 80 % / 63-75 % to 94-98 % / 93-97 %. A 1 kHz recording with ff showed
  the stuck ramp never fired and the bleed fired 3-5 ticks per wheel on ~0.2 A, so removing the rules would change
  little with ff on and only weaken runs without it.
- Expected on hardware: keyboard yaw without stepping; residual strafe curve remains (left/right pairs share the
  load unevenly, provisional geometry); 2 m/s slip unaffected (acceleration limit, no yaw feedback).
- Touches: src/tools/lib/drive_ops.[ch], keyboard_drive.c, tests (test_drive_stage, test_drive_cli), architecture/drive.md.
