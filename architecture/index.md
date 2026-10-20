# Architecture

## System

```
PC (scripts/) --SWD mailbox--> Nucleo-H755 CM7 --CAN FD--> 4 x B-G431B-ESC1
 keyboard / rig / tuning        inverse kinematics          Hall FOC, speed PI
```

Data flow, CAN protocol and protections: [DRIVE.md](DRIVE.md). Boards, wheel
map, safety: [HARDWARE.md](HARDWARE.md). Firmware update:
[CAN_FWU.md](CAN_FWU.md).

## What each file does

When you add a file or folder, add a line here in the same commit.

### Nucleo (`CM7/`, `Common/`)

| File | Role |
|---|---|
| `CM7/Core/Src/main.c` | CubeMX init; our code only in `USER CODE` sections |
| `CM7/Core/Src/ik_drive.c` | 20 ms drive loop: read twist mailbox, IK, slew-limit, send `SPEED_WRITE` |
| `CM7/Core/Src/ik_telemetry.c` | Poll ESC status/diag, keep last reply per wheel for the PC |
| `CM7/Core/Src/fwu_updater.c`, `can_fwu_host.c` | Updater image: push an ESC app over CAN |
| `Common/Src/rover_kinematics.c` | Body twist <-> wheel speeds (IK table) |
| `Common/Src/can_fwu_protocol.c`, `can_fwu_boot.c` | CAN firmware-update protocol, shared host/bootloader |
| `CM4/` | Unused CubeMX skeleton |

### ESC (`HW/B-G431B-ESC1/`)

| Path | Role |
|---|---|
| `motor_config/Src/drive_app.c` | All application behaviour: start, speed target, status/diag, config, protections |
| `motor_config/Src/mc_tasks_foc.c` | Hall angle hook in the FOC current controller |
| `motor_config/Inc/*parameters.h` | Motor and loop parameters |
| `motor_config/build_cli.py` | GCC build, no CubeIDE |
| `bootloader/` | 16 KiB CAN bootloader |
| `Inc/motor_controller_can.h` | `MOTOR_NE/SE/SW/NW` build macro to CAN select bit |
| everything else | ST MCSDK / CubeMX generated; do not hand-edit |

### Shared protocol

`HW/common/CAN/can_datatypes.h`: Nucleo <-> ESC CAN IDs and payloads.

### Tools (`scripts/`)

| Script | Role |
|---|---|
| `build_all.py` | Build every artifact into `build/` |
| `esc_update.py` | Install the ESC app over CAN, per wheel |
| `esc_tool.py` | ESC setup: align, status, stop, config, pulse |
| `tune_ground.py` | Ground speed-loop tuning runs |
| `keyboard_drive.py` | WASD/QE drive |
| `move_test.py` | Scripted moves |
| `ik_link.py` | SWD mailbox link library |
| `capture_analysis.py` | Current-loop capture analysis (R, L, PI gains) |
| `fwu_image.py` | ESC app to Nucleo FWU payload |
| `rig/` | Overhead camera and AprilTag tracker ([docs/rig](rig/README.md)) |

### Tests (`tests/`)

Host tests that compile the real C source against fakes; `run_all.py` runs
them plus the module gtests. Anything addressed per wheel is tested for all
four wheels.

### Not part of the drive image

`HW/STM32H7/` (sensor app, FreeRTOS), `HW/STM32H5/` (bring-up skeleton),
`rs_platform/rs_i2t/` (I2T limiter, unused).

## Control structure

16 kHz current loop (PI, d/q), 1 kHz speed PI, Hall angle with a per-wheel
shift. Friction feed-forward is set through `CONFIG_SET`. Body-level control
is open-loop velocity today; see [PRD.md](PRD.md) phase 3 for estimation.
