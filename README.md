# SFURS firmware rebuild

From-scratch rebuild of the omni-wheel firmware (Nucleo-H755 CM7 and four
B-G431B-ESC1 wheels on CAN). `develop_ai` is the read-only reference.
Owned application code is written here, in small pull requests, on top of
the vendor tree.

This rebuild is host-first. Phase 0 and the steps below run on a PC and in
GitHub Actions. No board, probe, OpenOCD session, CAN bus, or flashing is
required. Lighting LD1 on a Nucleo is a later optional hardware milestone,
and only when you ask for it.

## This phase

Phase 0 vendored the Nucleo CubeMX project: HAL, CMSIS,
`sfurs_firmware_v2.ioc`, and the generated startup for CM7 and CM4. CM7
`main` runs that init and toggles LD1 (green, PB0) in the image. That
source cross-compiles; it is not a hardware test.

Phase 1 adds omni inverse kinematics (`src/shared/inc/rover_kinematics.h`,
`src/shared/src/rover_kinematics.c`). A body twist — vx m/s to the right,
vy m/s forward, yaw rad/s CCW — maps to four motor RPM in order FR, BR,
BL, FL. The wheel map is the provisional hand measurement: radius
32.75 mm, front rollers 30° off forward, rear 35°, left motors mirrored.
A twist that would exceed 900 RPM on any wheel is scaled on all four
together. The module is host-tested and is not linked into the CM7 image
yet.

The frame codec (`src/shared/inc/motor_protocol.h`,
`src/shared/src/motor_protocol.c`, wire format in
`architecture/protocol.md`) encodes and decodes CONTROL v1, CONTROL_REPLY,
and the STATUS and DIAG polls and replies. CONTROL is 8 bytes: version and
mode (DISABLE, SPEED, or CURRENT), flags, an int16 setpoint, an int16
feed-forward, a sequence, and a session. CONTROL_REPLY is 8 bytes from one
wheel: the sequences, lifecycle and outcome, the reason byte, measured RPM,
and q-axis current in 50 mA steps. STATUS carries RPM, q-axis current in
MCSDK counts (544 per ampere), the MCSDK state, and flags. DIAG carries bus
centivolts, the fault word, Hall shift in electrical degrees, the align
result, and I²t percent. The codec enforces the CONTROL wire rules,
including which wheels a frame may name. Pure helpers pick the next session
byte and classify a sequence; they do not remember a wheel. The codec does
not apply amp or RPM limits and does not run the ESC command lifecycle.
It is host-tested and is not linked into either image.

The command-lifecycle slice (`src/shared/inc/motor_cmd.h`,
`src/shared/src/motor_cmd.c`, `architecture/command-lifecycle.md`) arms one
wheel, checks the session and the sequence window, and records an in-range
SPEED or CURRENT setpoint. With a host clock sample it disarms 100 ms after
the last fresh motion. With a host bridge sample it requests start, waits
for RUN, and latches a fault if that wait exceeds 500 ms. `CLEAR_FAULT` is
rejected unsupported. The slice does not call the bridge, and it is not
linked into either image.

The ESC vendor tree is `src/esc/motor_config`: ST MCSDK 6.4.1 and the
Cube-generated HAL for the B-G431B-ESC1 (STM32G431CBUx). `main` runs
that init and returns to an empty loop. The medium-frequency task runs
from SysTick and the TIM1 break IRQ, which is how MCSDK generated it.
The image links at `0x08000000` and cross-compiles; it is not a hardware
test. `drive_app`, `motor_speed`, and `hall_angle` are not in the tree,
and the link did not need stubs of them. `motor_cmd` is the host slice
above and is not part of this image. Hall edge counters
and the FOC angle and capture hooks those modules used are removed.
Parameters the generated code still reads stay: current-loop gains
4000/4400, speed-loop gains 1024/553, `IQMAX_A` 8, nominal current 6 A,
Hall FIFO depth 6, over-voltage 18 V, `SPEED_UNIT` `U_RPM`.

Drive, telemetry, skynet, and firmware-update modules are absent. The
CAN bootloader and the per-wheel link are a later phase. No board,
probe, OpenOCD session, CAN bus, or flashing.

STM32Cube files keep ST's license (`src/nucleo/Drivers/*/LICENSE.txt`,
`src/esc/motor_config/Drivers/*/LICENSE.txt`, and the file headers).
MCSDK sources keep ST's Ultimate Liberty license (SLA0044) in their
headers. `LICENSE` at the repo root is MIT for code we add.

## Build

`scripts/test.sh` runs the host CTests, then cross-compiles
the Nucleo images and the ESC image with `arm-none-eabi-gcc`, CMake, and
Ninja. GitHub Actions installs that toolchain on `ubuntu-latest` and runs
the script. No further compiler, pack, or MCSDK tool is required. The
runner has no robot attached.

```sh
scripts/test.sh
```

Host test binaries: `build/host/test_rover_kinematics`,
`build/host/test_motor_protocol`, `build/host/test_motor_cmd`.
CM7 output: `build/nucleo-cm7/sfurs_firmware_v2_CM7.elf`.
CM4 output: `build/nucleo-cm4/sfurs_firmware_v2_CM4.elf` (CubeMX dual-core
skeleton; the CM4 boot handshake in CM7 `main` stays disabled).
ESC output: `build/esc/motor_config.elf` (B-G431B-ESC1, flash at
`0x08000000`).

## After this

1. The rest of the owned ESC application on this vendor image: the inbox,
   HOLD and park, the speed loop, and Hall angle. CONFIG belongs with that
   work. The host slice fills a CONTROL_REPLY and names start, stop, and
   set-target, but nothing on the ESC calls it yet. This image does not
   speak CAN and does not close a speed loop.
2. Optional hardware milestones (for example LD1 on a board), only when
   you ask.
