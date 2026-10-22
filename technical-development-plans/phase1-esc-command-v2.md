# Plan: rewrite the ESC command + speed layer (staff-engineer-review quality)

Owner from 2026-10-21: Claude Opus (took over from Sonnet). Sonnet's contracts C1-C9 are kept; the changes from my
review are marked **[rev]**.

## Context
Current path: Nucleo `ik_drive.c` -> CAN FD (500 kbit/s nominal, no BRS) `SPEED_WRITE` -> ESC `drive_app.c` /
`drive_speed_ctrl.c`. Verified defects: three hand-coded byte layouts (Nucleo, ESC, tools); length-dependent 4/8-byte
frame without version or sequence; validation = length + ID; commanded ff silently ignored with DSC on
(`mc_tasks_foc.c:446`); three stacked feed-forwards and three speed estimators; torn ff/word read (`drive_app.c:157-159`
vs `:663`/`:794`); rejected frames refresh the watchdog (`:661`); two chained ramps; no command feedback. Goal: wheels
start cleanly under load; the code goes to an outside staff engineer.

## Decisions (user)
Rewrite the command + speed layer; KEEP ST FOC current loop, Hall align, capture, I2T, bootloader/FWU; DROP DSC, legacy
static/kinetic ff, 4/8-byte SPEED_WRITE. Protocol first, then loop. One fixed versioned 8-byte control frame with a
sequence. Modes speed + current. Reply to every control frame. Watchdog 100 ms. Acceleration limiting on the Nucleo.
Config acked + readback, limits split from tuning. Startup gate as redefined below. Gate speed 120 RPM peak wheel speed
(higher allowed later; robot max is 5 m/s, but `rover_ik` caps wheels at 900 RPM ~ 3 m/s today). Parking = zero-current
hold. Codex: bounded lanes; Claude keeps the core.

## Repository base
Work branch `feature/esc-command-v2` = local `develop_ai` f56eb58 (C tools over OpenOCD `lib/ocd.c`, CTest under
`tests/lanes/*.cmake`, `scripts/test.sh`, `scripts/build.sh`; no Python; camera rig in its own repo). It is checked out in
this session's worktree `.claude/worktrees/fix-underload-tuning` (the session cannot enter other worktrees; the old
`fix/underload-tuning` branch is pushed and kept as the record). `develop_ai` is 8 commits ahead of origin: pushing the
topic branch publishes them, so ask first.
**[rev]** From `fix/underload-tuning` port ONLY `friction_ff.c/.h` now (+ a CTest). Its `ik_drive.c` mailbox and the
`--body-ff` tool options are NOT ported now: Stage 2 rewrites `ik_drive.c`, so they are re-added there once, on the
new protocol (no churn).

## Startup acceptance (user definition, 2026-10-21) **[rev: replaces both 20 ms first-edge bars]**
Why: a wheel that tracks the 900 RPM/s reference perfectly turns 1.08 deg in 20 ms, while the first Hall edge can be
up to one sector (7.5 deg = 60 deg electrical / 8 pole pairs) away, so a perfect controller failed the old latency bar
~86 % of the time and the spread bar for the same reason.
1. **Startup tracking**, per active wheel: lag between the reference and the observed Hall transition at the same
   *calibrated* physical boundary. Once the ESC-applied reference crosses a boundary, the matching transition must occur
   within 20 ms, subject to a defined measurement tolerance. Evaluate up to six transitions where the speed gives enough.
   Calibration: per-wheel map of the 48 edge angles (Hall sensors are not evenly placed), measured with wheels raised at
   steady speed (edge-interval x speed), stored as tool data. Tolerance: the start position inside the first sector is
   unknown, so the absolute lag is known only within the starting sector's calibrated width; artifact D defines the band
   and the pass rule (lag growth between transitions is exact; the absolute offset carries the band).
2. **Synchronization**: command-application skew across active ESCs <= 2 ms on the Nucleo timebase (replies timestamped
   by the Nucleo; the reply carries the ESC apply tick).
3. **Overshoot**: max measured wheel speed <= 1.3 x the commanded speed, only where the speed estimate is valid.
4. **Low-speed cases**: where too few Hall edges exist, assess signed displacement and response with an independently
   validated estimator or external encoder (candidate: the AS5047P `wheel_encoder` driver in `src/sensors/`, if fitted;
   or the rig repo). Never report the Hall metric as passing when it cannot be measured.
Failures also include: protection trip, I2T derate, wrong-direction transition, FAULTED, reject/LOST.
Operating points: 120 RPM peak wheel speed per axis (vx ~ 0.72 m/s, vy ~ 0.47 m/s, wz ~ 4.8 rad/s), both signs, Nucleo
slew 900 RPM/s, on the mat, from rest; 20 starts x 6 = 120 starts at the final gate (H6). Vbus, temperature class and
initial Hall state logged per start.

## Design contracts (frozen in Stage 1 as artifacts, before firmware C)
Artifacts in `esc-command-v2/` next to this plan (they move to `architecture/` in the stage that implements them, since
`architecture/` describes the system as it is): (A) protocol; (B) lifecycle; (C) control-path ownership;
(D) `acceptance-matrix.md` (metrics above, exact pass rules, tolerances). Stage 2 starts after you approve A-D.

- **C1 Outcomes/replies.** REJECTED(reason), ACCEPTED, APPLIED (used by the controller, not "speed achieved"),
  SUPERSEDED, INHIBITED(reason). Reply: request_seq, last_applied_seq, outcome, reason, lifecycle state, first-edge bit,
  apply tick, rpm, Iq (artifact A fits these into 8 bytes; secondary fields go to DIAG). Bounded 4-entry reply ring,
  overflow drops oldest + counter. Reply latency p99 measured at H2. Nucleo deadline 40 ms; 3 misses = LOST -> Nucleo
  disables all four.
- **C2 Session + sequence.** ESC-issued nonzero session byte on boot/timeout/fault/disarm, sent while DISARMED; ARM
  (disable mode + ARM flag) must echo it; every motion frame carries it (byte 7). u8 sequence, signed 8-bit difference;
  +-128 rejected; forward window 1..16; older = STALE; duplicate = reply, no apply, no watchdog refresh; duplicate with
  different payload = CONFLICT; DISABLE honoured even when stale. Documented assumption: no CAN store-and-forward, TX FIFO
  bounded; 1/255 stale-session collision per disarm accepted.
- **C3 Ordered lifecycle.** Rx ordinal + rx timestamp per frame; DISABLE/ARM/CLEAR_FAULT in a 4-entry ordered FIFO
  (overflow -> DISABLE + DISARM); motion older than the last consumed DISABLE is discarded; pending motion expires by
  receive age.
- **C4 Lifecycle + ownership [rev: zero-current hold].**
  DISARMED --ARM--> READY (output off) --first motion--> STARTING (`MC_StartMotor1`; MCSDK owns the rotor) --RUN +
  aligned--> RUNNING (`motor_speed` owns Iq) --zero cmd + standstill 150 ms--> HOLD (output ON, speed loop off,
  Iq ref 0: cannot buzz, restarts with no MCSDK start) --motion--> RUNNING. DISABLE / timeout / fault -> output off
  (`MC_StopMotor1`); FAULTED needs DISABLE + CLEAR_FAULT then a fresh ARM. Stall monitor (|Iq| >= 80 % of the effective
  limit, |ref| >= 10 RPM, no edge along the command, 500 ms; resets on edge, sign flip, small ref, disable) ->
  FAULTED(STALL). Invalid estimate after valid motion: drop P, keep ff for the stale window
  (clamp(2 x last edge period, 20, 100) ms), then FAULTED(HALL_LOST); before the first edge est = 0.
  HOLD is a behaviour change: introduced with the new loop (Stage 4a), idle current, noise and temperature checked at H4a.
- **C5 Control law.** One estimate (int16 RPM + valid flag, chosen from recorded data in 4a). P (+ optional I, Ki 0) at
  1 kHz. ff = q-axis current predicted to cancel friction along the command; one insertion point after P, slewed, reset
  on reversal. Effective limit = min(compile max, requested limit, I2T allowance). Conditional integration on the total
  saturated request. Integrator cleared on disable/fault/reversal/mode change/gain change. int16 mA = signed q-axis
  phase current; 544 counts/A (`CURRENT_CONV_FACTOR`) verified with rounding/overflow tests. Current mode bypasses speed
  loop and ff.
- **C5b Reference reconstruction [rev].** With `ramp_ms` gone, applying each 50 Hz setpoint as a step gives a staircase
  reference and P-loop current pulses every 20 ms. The ESC instead interpolates linearly from its current reference to
  the new setpoint over one nominal command period (protocol constant 20 ms, not a frame field). This reconstructs the
  Nucleo trajectory; acceleration limiting stays only on the Nucleo. The applied (interpolated) reference is what the
  recorder logs and what the tracking metric uses.
- **C6 Atomic handoff.** One validated snapshot {mode, flags, setpoint, ff, seq, session, ordinal, rx time} published in a
  short bounded critical section; motion slot latest-wins; lifecycle via the C3 FIFO.
- **C7 One codec.** `src/shared/src/motor_protocol.c/.h` only; compiled by Nucleo, ESC and the C tools. Layout: byte 0
  version|mode, 1 flags, 2-3 int16 setpoint, 4-5 int16 ff, 6 seq, 7 session; CAN FD no BRS, DLC exactly 8; reserved
  bits must be 0. IDs `CONTROL = 8<<7`, `CONTROL_REPLY = 13<<7`; old SPEED_WRITE/STATUS_REPLY retired.
  **[rev]** The Nucleo raw-frame mailbox (`ik_drive_raw_*`, 4 bytes today) becomes 8 bytes (ABI bump) so `esc_tool`
  can send ARM/config/current-mode frames.
- **C8 Stage 2 claim.** Preserve the old control law and old ff behaviour (ff inactive with DSC on, reported
  INHIBITED(FF_UNUSED)); adapter into the old loop; intentional fault-path changes enumerated and tested; compare with
  ff off.
- **C9 Watchdog/config.** Age from the ISR receive time of the last accepted fresh motion frame; trip at >= 100 ms (3 lost
  frames tolerated, 4 trip); 150 ms only as a reviewed change. Config RAM-only, atomic, cross-key validated, IDLE =
  MCSDK IDLE + output off + lifecycle DISARMED/READY; replies report requested and effective limit. Bus load estimate
  (~16 frames per 20 ms at 500 kbit/s, ~15-20 %) recorded in artifact A and measured at H2.

## Stages (each: own commits; `scripts/test.sh` + `scripts/build.sh` 0 warnings; docs in the same commit; hardware gate
## before the next; hardware: ask first, measure Vbus, wheels raised for the first run after a firmware change, back up
## ESC flash, record in `hardware-test-results/`)
Progress (2026-10-22): Stages 0-2 done on hardware. Stages 3, 4a, 5 and the Stage 8 review pack done host-side
(design log 2026-10-22: gates moved to one hardware session): CONFIG ack/readback, new speed loop with HOLD and
stall/HALL_LOST faults, current mode, friction_map, --ff-* options, unused-code sweep, Claude + Codex reviews with all
verified findings fixed. Next: hardware session per esc-command-v2/F-hardware-session.md (H4a, H3, H5, H6).

0. **Baseline + calibration (hardware).** **[rev]** First validate the C tools that never drove a board (`esc_tool
   status/record`, `move_test`), and resolve the `status` Iq ~25 A at IDLE seen on BR/FL (2026-10-20: offset or a
   counts/A slip in the C tool). Then: ESC state, IDENT CRC vs build hashes; count edges over one marked wheel turn
   (confirms 8 pole pairs); per-wheel Hall edge map (raised, steady speed); MCSDK start latency; baseline startup runs on
   the CURRENT firmware, ff off, at the gate points: **[rev]** 5 starts per axis/sign (30) since the baseline is a
   comparison, not the gate. Host-only part: port `friction_ff` + CTest.
1. **Design freeze.** Artifacts A-D; `motor_protocol.c/.h`; independent golden-vector tests. Gate: you approve A-D.
2. **Protocol on the wire, old loop underneath (C8).** ESC `motor_rx/cmd/safety` + adapter; Nucleo codec, session/ARM,
   replies, LOST; raw mailbox 8 bytes; friction-ff mailbox and `--body-ff` re-added on the new path; delete 4/8-byte
   SPEED_WRITE and `ramp_ms` (ESC interpolation per C5b replaces it). Gate H2: rejects visible; watchdog/LOST under
   injected loss; 5 min without false trips; reply p99 and bus load measured; nominal traces vs baseline.
3. **Config ack/readback.** Gate H3: every key read back on all four wheels; refused when not IDLE.
4. **4a New speed loop, ff off, with HOLD.** DSC lessons first turned into recorded-data/scenario tests; then
   `motor_speed.c`; delete DSC, legacy ff and their keys/tests/docs. Gate H4a: correct operation, no worse than baseline,
   HOLD idle current/noise/temperature acceptable.
5. **Current mode + measurement.** Current mode; roll-out friction fit; Ke fit (C tools). Gate H5: friction map ->
   `friction_ff` defaults.
6. **Final acceptance H6.** ff on with measured values; full metric set, 120 starts.
7. **Only if H6 fails:** voltage-step start as a mode; A/B vs H6.
8. **Review pack.** Docs, design-log entries (append only), `docs/lessons.md`, Doxygen, unused-code sweep, `/code-review`
   plus a Codex second-opinion review, then the outside submission.

## Delegation **[rev]** (user: bounded lanes, Claude keeps the core)
Claude (me) writes: artifacts A-D, `motor_protocol.c/.h`, all ESC firmware (`motor_rx/cmd/safety/speed`, adapter), the
Nucleo `ik_drive.c` rewrite, integration, every review, every hardware step. Claude subagents only for broad read-only
searches (Explore), never for design.
Codex lanes (one worktree + branch each, created with plain `git worktree add <sibling dir> -b codex/<lane>
feature/esc-command-v2` from this worktree, since `git -C` is refused here; prompt from a file; no commit/push; I review
the diff, re-run `scripts/test.sh`, commit with attribution, merge into `feature/esc-command-v2`):
| Lane | Stage | Model / effort |
|---|---|---|
| `friction_ff` port + CTest (all four wheels) | 0 | `gpt-6.1-sol` medium |
| Hall edge-map + startup-tracking analysis library and CLI (synthetic-trace CTests) | 0 | `gpt-6-astra` high (metric math) |
| Independent codec golden-vector tests from artifact A (written without reading my codec) | 1 | `gpt-6.1-sol` high |
| C tools: 8-byte raw mailbox, reply parsing in `ik_link.c`, `esc_tool` ARM/CONFIG_GET, `move_test` logging | 2-3 | `gpt-6.1-sol` high |
| Roll-out friction fit and Ke fit tools | 5 | `gpt-6-astra` high |
| Second-opinion review of ESC lifecycle/speed-loop diffs before H2 and H4a | 2, 4a | `codex exec review`, `gpt-6-astra` xhigh |
Mechanical chores (renames, doc link fixes) go to `gpt-6-luna` low only if they come up. No `max`/`ultra` effort.

## Critical files
`src/shared/can/can_datatypes.h`, new `src/shared/src/motor_protocol.c`, `src/shared/src/friction_ff.c` (ported),
`src/esc/motor_config/Src/drive_app.c` (shrinks), new `motor_rx.c`/`motor_cmd.c`/`motor_safety.c`/`motor_speed.c`,
`drive_speed_ctrl.c` (deleted in 4a), `mc_tasks_foc.c`, `src/nucleo/CM7/Core/Src/ik_drive.c`, `ik_telemetry.c`, both
`main.c`, `src/tools/lib/ik_link.c`, `esc_ops.c`, `drive_ops.c`, `drive_record.c`, `esc_tool.c`, `move_test.c`; tests via
`tests/lanes/*.cmake` (new `test_motor_protocol.c`, `test_friction_ff.c`; existing `test_esc_drive_app*.c`,
`test_esc_speed_ctrl.c`, `test_ik_drive.c`, `test_ik_telemetry.c`, `test_ik_link.c`). Reuse the recorder/capture plumbing
(`drive_app.c`, `drive_record.c`), the SWD mailbox, host-test fakes at HAL/CAN boundaries only.

## Verification
Per commit: `scripts/test.sh` all pass, `scripts/build.sh` 0 warnings, `doxygen Doxyfile` clean; per-wheel code tested
for all four wheels. Per stage: its hardware gate with a `hardware-test-results/` file. End: H6 passes the acceptance set
above; then the 1 m +-10 cm check with the rig repo.

## Risks / open
- Absolute tracking lag is only known within the starting sector's width; artifact D must make the pass rule fair.
- HOLD keeps the bridge switching at rest: idle current/heat checked at H4a.
- Low-speed assessment needs an independent estimator; whether an AS5047P is fitted on the robot is unknown.
- Kt and friction unmeasured until Stage 5; going above 120 RPM / 3 m/s needs the `rover_ik` 900 RPM cap raised.
- C tools are unproven on a driving robot; Stage 0 validates them first.
- Images must be installed together at every protocol-changing stage.
