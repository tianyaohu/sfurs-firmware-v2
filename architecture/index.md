# Architecture

## System

```
PC (src/tools/) --SWD mailbox--> Nucleo-H755 CM7 --CAN FD--> 4 x B-G431B-ESC1
 keyboard / rig / tuning        inverse kinematics          Hall FOC, speed PI
```

Data flow, CAN protocol and protections: [DRIVE.md](drive.md). Boards, wheel
map, safety: [HARDWARE.md](../system-info/hardware.md). Firmware update:
[CAN_FWU.md](can-fwu.md).

## What each file does

When you add a file or folder, add a line here in the same commit.

### Nucleo (`src/nucleo/`, `src/shared/`)

| File | Role |
|---|---|
| `src/nucleo/CM7/Core/Src/main.c` | CubeMX init; our code only in `USER CODE` sections |
| `src/nucleo/CM7/Core/Src/ik_drive.c` | 20 ms drive loop: read twist mailbox, IK, slew-limit, send `SPEED_WRITE` |
| `src/nucleo/CM7/Core/Src/ik_telemetry.c` | Poll ESC status/diag, keep last reply per wheel for the PC |
| `src/nucleo/CM7/Core/Src/fwu_updater.c`, `can_fwu_host.c` | Updater image: push an ESC app over CAN |
| `src/shared/src/rover_kinematics.c` | Body twist <-> wheel speeds (IK table) |
| `src/shared/src/can_fwu_protocol.c`, `can_fwu_boot.c` | CAN firmware-update protocol, shared host/bootloader |
| `src/nucleo/CM4/` | Unused CubeMX skeleton |
| `src/nucleo/Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c` | CubeMX-generated dual-core boot; stays beside the `.ioc` |

### ESC (`src/esc/`)

| Path | Role |
|---|---|
| `motor_config/Src/drive_app.c` | All application behaviour: start, speed target, status/diag, config, protections |
| `motor_config/Src/mc_tasks_foc.c` | Hall angle hook in the FOC current controller |
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
| `esc_tool.c` | ESC setup: status, align, set, capture, spin, pulse, stop |
| `keyboard_drive.c` | WASD/QE drive |
| `move_test.c` | Scripted straight-line moves, logged with monotonic stamps |
| `tune_ground.c` | Ground friction feed-forward tuning |
| `sweep.c` | Oblique out-and-back pulses for friction identification |
| `fwu_image/fwu_image.c` | ESC app to Nucleo FWU payload (own CMake project) |

Library `lib/` (every file joins library `tools`):

| File | Role |
|---|---|
| `ocd.c` | OpenOCD session: SWD read/write over its Tcl port, one process per ST-LINK serial |
| `elfsym.c`, `crc32.c`, `sha256.c` | ELF symbol lookup, CRC-32 (IEEE, = zlib), SHA-256 |
| `ik_link.c` | Nucleo mailbox: command words, CONFIG_SET, telemetry decode, capture and recorder readout |
| `esc_ops.c`, `esc_defaults.c` | Breakaway timing, pulse guard and plan; ESC boot defaults from `drive_parameters.h` (generated header) |
| `pulse_run.c` | Run a plan through the Nucleo, guard it, log it, restore defaults |
| `capture_analysis.c`, `tune_analysis.c` | Current-loop R/L/PI fit; friction fit, overshoot, acceptance |
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

16 kHz current loop (PI, d/q), 1 kHz speed PI, Hall angle with a per-wheel
shift. Friction feed-forward is set through `CONFIG_SET`. Body-level control
is open-loop velocity today; see [PRD.md](../prd/index.md) phase 3 for estimation.
