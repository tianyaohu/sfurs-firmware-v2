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
   is answered; the answers land in `ik_drive_link[4]` for the tools.
3. Each ESC runs MCSDK FOC with Hall sensors: 16 kHz current loop, 1 kHz
   speed PI. `motor_cmd.c` owns the command lifecycle (arming, session and sequence checks,
   watchdog, faults, parking; [command-lifecycle.md](command-lifecycle.md)); `drive_app.c`
   executes its decisions on MCSDK and replies to status/diag polls. The ESC moves its speed
   reference to each new setpoint over the 20 ms frame period (MCSDK ramp).

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
| `0x500` CONFIG_SET | → ESC | u16 key (1 Kp, 2 Ki, 3 Hall shift °, 4 Iq max mA, 5/6 current Kp/Ki, 7/8 friction feed-forward static/kinetic mA), i16 value |
| `0x600–0x6FF` | | firmware update ([CAN_FWU.md](can-fwu.md)) |

`0x080` (old SPEED_WRITE) and `0x780` (old STOP) are retired and ignored.
STATUS flags: low nibble = last switch-off reason (1 DISABLE, 2 timeout, 3 fault, 4 align,
5 capture, 6 parked, 7 failed start),
`0x10` output on, `0x20` I²t derated, `0x40` aligning, `0x80` Hall shift known.

## Protections

| Layer | Behaviour |
|---|---|
| ESC hardware/MCSDK | Over-current comparator, over-voltage 18 V, under-voltage 8 V, over-temperature 70 °C |
| ESC arming | Motion only after an ARM carrying the session the ESC announced; a timeout or a cleared fault starts a new session, so nothing sent before can move the wheel |
| ESC watchdog | No fresh accepted motion frame for 100 ms → output off, disarmed (3 lost 50 Hz frames are tolerated, 4 trip) |
| ESC I²t | Iq limit 6 A; after ~30 A²s above 3.26 A it derates to 3 A until it cools |
| ESC fault latch | Any fault (MCSDK, failed start, start timeout) keeps the wheel off until CLEAR_FAULT, then a new ARM |
| ESC parking | Target 0 RPM and standstill (< 45 RPM, the Hall estimate's floor) for 150 ms → motor off (coasts); the next non-zero target restarts it (~30 ms). Holding zero speed on the Hall estimate only buzzed (0.2–0.6 A average, peaks 3.9 A) |
| ESC alignment gate | A non-zero target is rejected (NOT_ALIGNED) until the Hall shift is known (built in or measured) |
| ESC range check | Targets beyond 2976 RPM or 6 A are rejected, never clamped |
| Nucleo link | A wheel that misses 3 replies in a row is LOST; a LOST or FAULTED wheel stops the robot (DISABLE to all) and latches `ik_drive_fault` until the PC sets enable to 0; `esc_tool clear` clears ESC faults |
| Nucleo heartbeat | PC silent for 300 ms → decelerate, hold 200 ms, broadcast DISABLE ×3 |
| Kinematics | All wheels scaled together so none exceeds 900 RPM (~3 m/s) |

No run timeouts, revolution counts or stall stops: a loaded wheel at low speed
must be allowed to build torque.

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

Speed PI (counts per RPM of error, `SPEED_UNIT = U_RPM`): Kp 1024/256 =
4 counts/RPM (100 RPM error → 0.75 A), Ki 553/16384 per ms. Chosen with all
four wheels raised: Kp 4096 / Ki 4424 limit-cycled at ±6 A (loud buzz, no
rotation); 1024/553 holds ±60 and 120 RPM with Iq ≈ 0.1 A. At 30–60 RPM the
Hall speed estimate itself scatters by about ±25 RPM whatever the gains. On
the ground the robot's inertia is part of the plant, so retune there. Tune
live without reflashing: `build/tools/esc_tool set all kp 2048` (IDLE only,
RAM only), then copy good values into `drive_parameters.h`.

Friction feed-forward (`drive_speed_iq_ref` in `drive_app.c`, called from
`FOC_CalcCurrRef`): on the ground each wheel needs 2.2–3.2 A before it turns
(free air 0.1 A), which a PI loop only reaches by winding up its integrator —
slow starts, then overshoot to 2–3× the command. In speed mode the Iq
reference is the PI output plus sign(speed ref) × ff, with ff = static value at
standstill fading linearly to the kinetic value at 30 RPM, clamped to the
current limit (including I²t derate). Both default to 0 (`FRICTION_FF_*` in
`drive_parameters.h`); tune with `esc_tool pulse --ff-static MA
--ff-kinetic MA`, which drives the robot out and back and reports per-wheel
breakaway time. Ground ladder without feed-forward: Kp 4096 / Ki 1100 is the
best stable pair; Ki 2200 at Kp 4096 limit-cycles at the 6 A limit.

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
