# Omni drive

Keyboard → PC (`scripts/keyboard_drive.py`) → SWD mailbox → Nucleo
(`CM7/Core/Src/ik_drive.c`) → CAN → four ESCs
(`HW/B-G431B-ESC1/motor_config/Src/drive_app.c`, on top of ST MCSDK 6.4.1).

## Data flow

1. The PC writes a body twist (vx, vy in mm/s, ω in mrad/s) and bumps a
   heartbeat at 50 Hz over the Nucleo's ST-LINK (pyOCD attach, no halt).
2. Every 20 ms the Nucleo runs the inverse kinematics
   (`Common/Src/rover_kinematics.c`), slews all wheels toward their targets
   with one acceleration limit (default 900 RPM/s, keeps the twist
   direction), and sends `SPEED_WRITE` to each ESC.
3. Each ESC runs MCSDK FOC with Hall sensors: 16 kHz current loop, 1 kHz
   speed PI. `drive_app.c` starts the motor on the first command, updates
   the target while running, and replies to status/diag polls.

## CAN protocol (`HW/common/CAN/can_datatypes.h`)

CAN FD frames without BRS. ID = command | select bits (NE 1, SE 2, SW 4, NW 8).

| ID | Direction | Payload |
|---|---|---|
| `0x080` SPEED_WRITE | → ESC | i16 motor RPM, u16 ramp ms |
| `0x100` → `0x280` STATUS | poll/reply | i16 RPM, i16 Iq, i16 Iq ref (544 counts/A), u8 MCSDK state, u8 flags |
| `0x300` → `0x380` DIAG | poll/reply | u16 Vbus cV, u16 occurred faults, i16 Hall shift °, u8 align result, u8 I²t % |
| `0x480` HALL_ALIGN | → ESC | `"HALN"` |
| `0x500` CONFIG_SET | → ESC | u16 key (1 Kp, 2 Ki, 3 Hall shift °, 4 Iq max mA), i16 value |
| `0x780` STOP | → ESC | any |
| `0x600–0x6FF` | | firmware update ([CAN_FWU.md](CAN_FWU.md)) |

Flags: low nibble = last stop reason (1 STOP, 2 timeout, 3 fault, 4 align),
`0x10` active, `0x20` I²t derated, `0x40` aligning, `0x80` Hall shift known.

## Protections

| Layer | Behaviour |
|---|---|
| ESC hardware/MCSDK | Over-current comparator, over-voltage 18 V, under-voltage 8 V, over-temperature 70 °C |
| ESC watchdog | No SPEED_WRITE for 250 ms → stop |
| ESC I²t | Iq limit 6 A; after ~30 A²s above 3.26 A it derates to 3 A until it cools |
| ESC fault latch | A fault while driving keeps the wheel off until STOP or a 250 ms command gap |
| ESC alignment gate | SPEED_WRITE ignored until the Hall shift is known (built in or measured) |
| Nucleo heartbeat | PC silent for 300 ms → decelerate, hold 200 ms, broadcast STOP ×3 |
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
`scripts/build_all.py --shift NE=<deg> SE=<deg> ...` and reinstall.

## Bring-up

1. `scripts/build_all.py`; `scripts/esc_update.py <W> --can` for each wheel
   (`--battery-off` with the ESC's USB for a first install; [CAN_FWU.md](CAN_FWU.md)).
2. Battery ON, wheels off the ground: `scripts/esc_tool.py status`, then
   `scripts/esc_tool.py align all`. Expect `ok`, 12 edges each way.
3. Off the ground: `scripts/keyboard_drive.py --speed 0.3`; check each key
   turns the wheels the right way.
4. On the ground: `--speed 0.3`, then 0.5, 1.0.
5. Bake the measured shifts (above) so alignment survives power cycles.

## Tuning

Tune from the inside out: current loop, then speed loop.

Current loop: `scripts/esc_tool.py capture <W> --step-ma 3000` (battery ON,
IDLE). The ESC holds the rotor on the d axis at 1 A, steps Id to 3 A and
records Id, Iq, Vd, Vq every FOC period (16 kHz, 512 samples); the Nucleo
relays the readout. `capture_analysis.py` fits R (from the two steady states,
cancelling dead time) and L (transient, with the PWM delay picked by fit),
reports rise time, overshoot and settling, and prints `current_kp`/`current_ki`
for 0.5, 0.8 and 1 kHz (Kp = L·ωc, Ki = R·ωc). Try them with
`esc_tool.py set <W> current_kp N` / `current_ki N` (RAM, both axes) and
capture again; copy good values into `PID_TORQUE_*`/`PID_FLUX_*` in
`drive_parameters.h`. With one PWM period of delay, overshoot grows quickly
above about 0.8 kHz.

Speed PI (counts per RPM of error, `SPEED_UNIT = U_RPM`): Kp 1024/256 =
4 counts/RPM (100 RPM error → 0.75 A), Ki 553/16384 per ms. Chosen with all
four wheels raised: Kp 4096 / Ki 4424 limit-cycled at ±6 A (loud buzz, no
rotation); 1024/553 holds ±60 and 120 RPM with Iq ≈ 0.1 A. At 30–60 RPM the
Hall speed estimate itself scatters by about ±25 RPM whatever the gains. On
the ground the robot's inertia is part of the plant, so retune there. Tune
live without reflashing: `scripts/esc_tool.py set all kp 2048` (IDLE only,
RAM only), then copy good values into `drive_parameters.h`.

Motor constants in `pmsm_motor_parameters.h` are the Workbench placeholder
(Rs 0.22 Ω, Ls 0.11 mH, 3.0 V/kRPM). Measured by capture on all four motors:
dq R 0.30–0.32 Ω (includes switches and shunts), L 135–150 µH, τ ≈ 0.45 ms;
current PI 4000/4400 is matched to those. Ke is not measured yet. Every
capture also shows a few current-sense disturbances (a single ~0.5 A false
sample the loop then reacts to for ~0.3 ms); FDCAN1 runs at NVIC priority 0,
above the ADC/FOC interrupt (2), which is the first suspect.

## Kinematics

`rover_ik_provisional` in `Common/Src/rover_kinematics.c`: x right, y forward,
yaw CCW positive. Per wheel: contact position, rolling direction, radius
32.75 mm (65.5 mm wheels, direct drive), motor sign (left side mirrored).
Contact positions come from hand measurements (track ~141–144 mm, wheelbase
~95 mm) and are provisional; verify with a straight-line and spin-in-place
test, then correct the table.

## Open items

- Gains, Hall shifts and geometry are not yet validated on hardware.
