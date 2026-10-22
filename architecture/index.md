# Architecture

## System

```
PC (src/tools/) --SWD mailbox--> Nucleo-H755 CM7 --CAN FD--> 4 x B-G431B-ESC1
 keyboard / rig / tuning        inverse kinematics          Hall FOC, speed loop
```

Data flow, CAN protocol and protections: [DRIVE.md](drive.md). Boards, wheel
map, safety: [HARDWARE.md](../system-info/hardware.md). Firmware update:
[CAN_FWU.md](can-fwu.md). Motor command protocol v1 (wire format, golden vectors):
[protocol.md](protocol.md); ESC command lifecycle (states, transitions):
[command-lifecycle.md](command-lifecycle.md).

## What each file does

When you add a file or folder, add a line here in the same commit.

### Nucleo (`src/nucleo/`, `src/shared/`)

| File | Role |
|---|---|
| `src/nucleo/CM7/Core/Src/main.c` | CubeMX init; our code only in `USER CODE` sections |
| `src/nucleo/CM7/Core/Src/ik_drive.c` | 20 ms drive loop: read twist mailbox, IK, slew-limit, send protocol v1 CONTROL frames |
| `src/nucleo/CM7/Core/Src/ik_telemetry.c` | Poll ESC status/diag, keep last reply per wheel for the PC |
| `src/nucleo/CM7/Core/Src/fwu_updater.c`, `can_fwu_host.c` | Updater image: push an ESC app over CAN |
| `src/shared/src/rover_kinematics.c` | Body twist <-> wheel speeds (IK table) |
| `src/shared/src/motor_protocol.c` | Motor command protocol v1 codec (CONTROL / CONTROL_REPLY, sequence and session rules), used by the Nucleo, the ESCs and the tools |
| `src/shared/src/friction_ff.c` | Body-frame friction model: commanded twist -> per-wheel feed-forward current; `ik_drive.c` sends it when `ik_drive_ff_enable` is set (off by default) |
| `src/shared/src/can_fwu_protocol.c`, `can_fwu_boot.c` | CAN firmware-update protocol, shared host/bootloader |
| `src/nucleo/CM4/` | Unused CubeMX skeleton |
| `src/nucleo/Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c` | CubeMX-generated dual-core boot; stays beside the `.ioc` |

### ESC (`src/esc/`)

| Path | Role |
|---|---|
| `motor_config/Src/drive_app.c` | All application behaviour: start, speed target, status/diag, config, protections |
| `motor_config/Src/motor_cmd.c` | Command lifecycle (pure): ARM/session/sequence checks, watchdog, faults, replies |
| `motor_config/Src/motor_speed.c` | Speed loop (pure; drive_app runs it every 1 ms): reference interpolation, Hall speed estimate, P + I + ff with stall ramp, over-speed bleed, stall timer |
| `motor_config/Src/mc_tasks_foc.c` | Forced-angle and capture hooks in the FOC current controller (alignment, capture) |
| `motor_config/Inc/*parameters.h` | Motor and loop parameters |
| `bootloader/` | 16 KiB CAN bootloader; compiles the protocol from `src/shared` |
| `Inc/motor_controller_can.h` | `MOTOR_NE/SE/SW/NW` build macro to CAN select bit |
| everything else | ST MCSDK / CubeMX generated; do not hand-edit |

### Shared protocol

`src/shared/can/can_datatypes.h`: Nucleo <-> ESC CAN IDs and payloads.

### Tools (`src/tools/`, C; `cmake -S src/tools -B build/tools`)

Programs (one `targets/<tool>.cmake` each):

| Program | Role |
|---|---|
| `esc_update.c` | Install the ESC app over CAN or SWD, per wheel (logic in `lib/flashimg_*.c`) |
| `esc_tool.c` | ESC setup: status, align, acknowledged CONFIG set/get, capture, spin, pulse, stop, clear |
| `keyboard_drive.c` | WASD/QE drive |
| `move_test.c` | Scripted straight-line moves, logged with monotonic stamps |
| `tune_ground.c` | Ground friction identification: breakaway and kinetic current per wheel |
| `friction_map.c` | Guarded floor SPEED out-and-back friction map and CURRENT breakaway ramps; result.json |
| `sweep.c` | Oblique out-and-back pulses for friction identification |
| `startup_report.c` | Aggregate `move_test --startup-metrics` `startup.tsv` files into a startup acceptance verdict |
| `fwu_image/fwu_image.c` | ESC app to Nucleo FWU payload (own CMake project) |

Library `lib/` (every file joins library `tools`):

| File | Role |
|---|---|
| `ocd.c` | OpenOCD session: SWD read/write over its Tcl port, one process per ST-LINK serial |
| `elfsym.c`, `crc32.c`, `sha256.c` | ELF symbol lookup, CRC-32 (IEEE, = zlib), SHA-256 |
| `ik_link.c` | Nucleo mailbox: command words, current-mode mailbox helpers, CONFIG key names, telemetry and protocol-link decode, fault check, capture and recorder readout |
| `esc_ctl.c` | Protocol v1 to single wheels while the drive is stopped: ARM handshake, SPEED, DISABLE, CLEAR_FAULT (frames from `motor_protocol.c`) |
| `esc_ops.c`, `esc_defaults.c` | Breakaway timing, pulse guard and plan; ESC boot defaults from `drive_parameters.h` (generated header) |
| `ik_config.c` | Shared CONFIG v1 exchange: fresh per-wheel sequence/key matching, status and stored/effective readback |
| `friction_fit.c` | Pure steady-segment rejection, per-direction Coulomb/viscous least squares and consecutive motion detection |
| `friction_run.c` | Guarded friction acquisition: plateau samples, current ramps, telemetry freshness/Vbus/speed checks and stop cleanup |
| `pulse_run.c` | Run a plan through the Nucleo, guard it, log it, restore defaults |
| `startup_metrics.c` | Pure startup acceptance measurements from recorder data: Hall sector, deadlines, direction, overshoot, wheel synchronisation |
| `capture_analysis.c`, `tune_analysis.c` | Current-loop R/L/PI fit; friction currents, breakaway time, overshoot |
| `flashimg_*.c` | ESC update checks (Vbus, flash vs ELF, bus readiness) and install flow |
| `drive_*.c` | Keyboard control, record summary, staged drive helpers |

The overhead camera rig is a separate repository (history on branch `rig-split`). It meets
these tools only through logs: both sides stamp `CLOCK_MONOTONIC` seconds, and the tools write each
move's start/end window so poses can be joined by time.

Scripts (`scripts/`): `build.sh` (all firmware images into `build/`), `esc_build.sh`, `boot_build.sh`
(ESC app and bootloader), `test.sh` (host tests, tool build, sensor gtests).

### Tests (`tests/`)

Host tests that compile the real C source against fakes (`tests/CMakeLists.txt`, CTest; `golden/` holds recorded reference outputs for the numeric ports); `scripts/test.sh` runs
them plus the module gtests. Anything addressed per wheel is tested for all
four wheels.

### Not part of the drive image

`src/sensors/stm32h7/` (sensor app, FreeRTOS), `archive/stm32h5/` (bring-up skeleton),
`archive/rs_i2t/` (I2T limiter, unused).

## Control structure

16 kHz current loop (PI, d/q), 1 kHz speed loop (`motor_speed.c`: P + I + ff, ESC in
torque mode), Hall angle with a per-wheel shift. Friction feed-forward travels in each
CONTROL frame from the Nucleo's body-frame model (off by default). Body-level control
is open-loop velocity today; see [PRD.md](../prd/index.md) phase 3 for estimation.
