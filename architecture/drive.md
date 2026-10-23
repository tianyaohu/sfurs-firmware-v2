# Omni drive

Keyboard → PC (`build/tools/keyboard_drive`) → SWD mailbox → Nucleo
(`src/nucleo/CM7/Core/Src/ik_drive.c`) → CAN → four ESCs
(`src/esc/motor_config/Src/drive_app.c` and `motor_cmd.c`, on top of ST MCSDK 6.4.1).

## Data flow

1. The PC writes a body twist (vx, vy in mm/s, ω in mrad/s) and bumps a
   heartbeat at 50 Hz over the Nucleo's ST-LINK (OpenOCD attach through `lib/ocd.c`, no halt).
2. When driving starts, the Nucleo probes every ESC, arms it with the session the ESC
   announces, and starts motion only when all four are armed. Then every 20 ms it runs the
   inverse kinematics (`src/shared/src/rover_kinematics.c`), slews all wheels toward their
   targets with one acceleration limit (default 900 RPM/s, keeps the twist direction), and
   sends one protocol v1 CONTROL frame to each ESC ([protocol.md](protocol.md)). Every frame
   is answered; the answers land in `ik_drive_link[4]` for the tools, and the time from the
   drive tick that queued a frame to its reply goes into a per-wheel histogram
   (`ik_drive_reply_lat_hist`, 250 µs bins). For measurements the session can instead run in
   current mode (`ik_drive_mode`, latched at the session start): each wheel gets its own
   q-current setpoint (`ik_drive_current_ma`) in CURRENT frames, with the same heartbeat,
   LOST and stop handling. Mailbox ABI 9.
3. Each ESC runs MCSDK FOC with Hall sensors (16 kHz current loop) in torque mode.
   `motor_cmd.c` owns the command lifecycle (arming, session and sequence checks, watchdog,
   faults, HOLD; [command-lifecycle.md](command-lifecycle.md)); `motor_speed.c` is the 1 kHz
   speed loop (below); `drive_app.c` runs both every millisecond, hands the q-current to
   MCSDK (`MC_SetCurrentReferenceMotor1`) and replies to status/diag polls.

## CAN protocol

CAN FD frames without BRS, 500 kbit/s. Select bits: NE 1, SE 2, SW 4, NW 8.
Motion: protocol v1 ([protocol.md](protocol.md), codec `src/shared/src/motor_protocol.c`).
Service commands (`src/shared/can/can_datatypes.h`), 4-byte requests, ID = command | select bits:

| ID | Direction | Payload |
|---|---|---|
| `0x010` CONTROL / `0x410` CONTROL_REPLY | → ESC / ← ESC | protocol v1, 8 bytes ([protocol.md](protocol.md)) |
| `0x100` → `0x280` STATUS | poll/reply | i16 RPM, i16 Iq, i16 Iq ref (544 counts/A), u8 MCSDK state, u8 flags |
| `0x300` → `0x380` DIAG | poll/reply | u16 Vbus cV, u16 occurred faults, i16 Hall shift °, u8 align result, u8 I²t % |
| `0x480` HALL_ALIGN | → ESC | `"HALN"` |
| `0x600–0x6FF` | | firmware update ([CAN_FWU.md](can-fwu.md)) |

Configuration: protocol v1 CONFIG `0x020` / CONFIG_REPLY `0x420`, one wheel per frame, IDLE only, every SET
acknowledged with the stored and effective value, every key readable (GET); key table in [protocol.md](protocol.md).
`0x080` (old SPEED_WRITE), `0x500` (old CONFIG_SET) and `0x780` (old STOP) are retired and ignored.
STATUS flags: low nibble = last switch-off reason (1 DISABLE, 2 timeout, 3 fault, 4 align,
5 capture, 8 parked after 1 s of HOLD; 6 and 7 retired),
`0x10` output on, `0x20` I²t derated, `0x40` aligning, `0x80` Hall shift known.

## Protections

| Layer | Behaviour |
|---|---|
| ESC hardware/MCSDK | Over-current comparator, over-voltage 18 V, under-voltage 8 V, over-temperature 70 °C |
| ESC arming | Motion only after an ARM carrying the session the ESC announced; a timeout or a cleared fault starts a new session, so nothing sent before can move the wheel |
| ESC watchdog | No fresh accepted motion frame for 100 ms → output off, disarmed (3 lost 50 Hz frames are tolerated, 4 trip) |
| ESC I²t | Iq limit 6 A; after ~30 A²s above 3.26 A it derates to 3 A until it cools |
| ESC fault latch | Any fault (MCSDK, stall, Hall loss, start timeout) keeps the wheel off until CLEAR_FAULT, then a new ARM |
| ESC HOLD | Target 0 RPM and standstill (< 45 RPM, the Hall estimate's floor) for 150 ms → zero-current hold with the bridge on; the next non-zero target resumes without an MCSDK start. Holding zero speed on the Hall estimate only buzzed (0.2–0.6 A average, peaks 3.9 A) |
| ESC stall | ≥ 80 % of the current limit with no Hall progress along the command for 500 ms → FAULTED(STALL) |
| ESC Hall loss | Hall code 0/7 beyond the estimator's stale window, or MCSDK's speed-feedback fault → FAULTED(HALL_LOST) |
| ESC alignment gate | A non-zero target is rejected (NOT_ALIGNED) until the Hall shift is known (built in or measured) |
| ESC range check | Targets beyond 2976 RPM or 6 A are rejected, never clamped |
| Nucleo link | A wheel that misses 3 replies in a row is LOST; a LOST or FAULTED wheel stops the robot (DISABLE to all) and latches `ik_drive_fault` until the PC sets enable to 0; `esc_tool clear` clears ESC faults |
| Nucleo heartbeat | PC silent for 300 ms → decelerate, hold 200 ms, broadcast DISABLE ×3 |
| Kinematics | All wheels scaled together so none exceeds 900 RPM (~3 m/s) |

The stall fault waits 500 ms at high current, so a loaded wheel still builds torque to
breakaway first (the stall ramp below gets there in a few hundred ms).

## Hall alignment

FOC needs the electrical angle of the Hall edges (`HALL_PHASE_SHIFT`). A wrong
value reduces torque by cos(error) and at ~180° reverses it, which makes a
speed loop run away. `HALL_ALIGN` measures it on the robot: 2 A on the d-axis
at a forced angle, sweep two electrical turns forward and back (~¼ wheel turn),
compare the forced angle with the decoder's angle at every edge. Averaging both
directions cancels rotor lag. Results: `ok`, `REVERSED` (Hall order does not
match phase order: fix wiring), `NO_EDGES` (rotor stuck or Halls dead).

The result lives in RAM. Bake it in with
`scripts/build.sh --shift NE=<deg> SE=<deg> ...` and reinstall.

## Bring-up

1. `scripts/build.sh`; `build/tools/esc_update <W> --can` for each wheel
   (`--battery-off` with the ESC's USB for a first install; [CAN_FWU.md](can-fwu.md)).
2. Battery ON, wheels off the ground: `build/tools/esc_tool status`. An ESC that powered up
   on USB before the battery reports a latched under-voltage fault: `build/tools/esc_tool clear`.
   Then `build/tools/esc_tool align all`. Expect `ok`, 12 edges each way.
3. Off the ground: `build/tools/keyboard_drive --speed 0.3`; check each key
   turns the wheels the right way.
4. On the ground: `--speed 0.3`, then 0.5, 1.0.
5. Bake the measured shifts (above) so alignment survives power cycles.

## Tuning

Tune from the inside out: current loop, then speed loop.

Current loop: `build/tools/esc_tool capture <W> --step-ma 3000` (battery ON,
IDLE). The ESC holds the rotor on the d axis at 1 A, steps Id to 3 A and
records Id, Iq, Vd, Vq every FOC period (16 kHz, 512 samples); the Nucleo
relays the readout. `capture_analysis.c` fits R (from the two steady states,
cancelling dead time) and L (transient, with the PWM delay picked by fit),
reports rise time, overshoot and settling, and prints `current_kp`/`current_ki`
for 0.5, 0.8 and 1 kHz (Kp = L·ωc, Ki = R·ωc). Try them with
`esc_tool set <W> current_kp N` / `current_ki N` (RAM, both axes) and
capture again; copy good values into `PID_TORQUE_*`/`PID_FLUX_*` in
`drive_parameters.h`. With one PWM period of delay, overshoot grows quickly
above about 0.8 kHz.

Speed loop (`motor_speed.c`, spec
[E-speed-loop.md](../technical-development-plans/esc-command-v2/E-speed-loop.md)): the
reference moves linearly to each new setpoint over the 20 ms frame period; one speed
estimate (Hall edge timing for the first edges after a start, then MCSDK's 6-edge average,
decaying as 1/t when edges stop); Iq = P + I + ff, clamped to the current limit (including
the I²t derate). Kp 1024/256 = 4 counts/RPM (100 RPM error → 0.75 A), Ki 553/16384 per ms
(62 mA/s per RPM). The integrator follows three rules: while the wheel is stuck (the
reference 4 Hall edges ahead and the estimate below 45 RPM) it ramps at a fixed 10 A/s
instead of by the error, and the reference lead is capped at those 4 edges, so no wind-up
waits for the breakaway; otherwise it integrates only while the request is not saturated
(or moving back inside); and a progress edge timed above 1.15× the target halves it. The
feed-forward field from the Nucleo is added along the command only, slewed at 50 A/s.
Raised, 1024/553 holds ±60 and 120 RPM with Iq ≈ 0.1 A (4096/4424 limit-cycled at ±6 A); at
30–60 RPM the Hall estimate scatters by about ±25 RPM whatever the gains. Tune live without
reflashing: `build/tools/esc_tool set all kp 2048` (IDLE only, RAM only), then copy good
values into `drive_parameters.h`.

On the ground each wheel needs 3.5–6 A before it turns (mat 2026-10-22; free air
0.1 A) and 0.3–2.2 A to keep rolling. Left to each ESC's stall ramp the wheels break free one at a
time and the body pivots about the ones still stuck, so the Nucleo sends friction as feed-forward
from its body-frame model (`friction_ff.c`, mailbox words `ik_drive_ff_*`); the model is off by
default. Coulomb friction has its own level per axis (strafing in x needs more than driving in y).
Breakaway boost: while the twist is nonzero and fewer than 2 wheels report >= 30 RPM along their
targets, all four wheels get the breakaway levels instead, for at most `--ff-breakaway-ms` per
stuck episode; a robot that sticks again is boosted again. `move_test` and `keyboard_drive` set it
with `--ff-coulomb-x-n`, `--ff-coulomb-y-n` (N), `--ff-viscous-xy-n-per-mps` (N per m/s),
`--ff-coulomb-w-nm` (N·m), `--ff-viscous-w-nm-per-radps` (N·m per rad/s), `--ff-breakaway-x-n`,
`--ff-breakaway-y-n` (N), `--ff-breakaway-w-nm` (N·m), `--ff-breakaway-ms` (1–2000, default 300 when a
breakaway level is given), `--ff-a-per-nm` (motor A per N·m, required) and `--ff-scale FR,BR,BL,FL`
(%, 1–200); they write it only while the drive is stopped and switch it off again on exit, and
`move_test` records it under `settings.ff` in result.json. `ik_drive_ff_boost` is 1 while the frames
carry the boost. With the provisional geometry 1 N gives about 0.095 A per wheel in y and 0.15 A in
x per 10 A/N·m of `--ff-a-per-nm`. `friction_map` measures the floor friction it needs.

Motor constants in `pmsm_motor_parameters.h` are the Workbench placeholder
(Rs 0.22 Ω, Ls 0.11 mH, 3.0 V/kRPM). Measured by capture on all four motors:
dq R 0.30–0.32 Ω (includes switches and shunts), L 135–150 µH, τ ≈ 0.45 ms;
current PI 4000/4400 is matched to those. Ke is not measured yet. Every
capture also shows ~4 corrupted current samples per 512 (0.2–1 A off for one
period; the loop then reacts for ~0.3 ms). At the capture's fixed angle each
shows ΔIq ≈ −0.55·ΔId, i.e. the phase-U reading alone is wrong. Phase U is on
ADC1 with the Vbus and temperature regular conversions; that collision is the
open lead. Not CAN: moving FDCAN1 below the FOC interrupt (NVIC 0 → 3, kept
since MCSDK expects its interrupts to preempt) left the count unchanged.

## Kinematics

`rover_ik_provisional` in `src/shared/src/rover_kinematics.c`: x right, y forward,
yaw CCW positive. Per wheel: contact position, rolling direction, radius
32.75 mm (65.5 mm wheels, direct drive), motor sign (left side mirrored).
Contact positions come from hand measurements (track ~141–144 mm, wheelbase
~95 mm) and are provisional; verify with a straight-line and spin-in-place
test, then correct the table.

## Open items

- Gains, Hall shifts and geometry are not yet validated on hardware.
