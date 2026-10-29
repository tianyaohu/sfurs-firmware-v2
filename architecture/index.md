# Architecture

Owned files only. Vendor HAL and CMSIS under `src/nucleo/Drivers/`, and
the MCSDK 6.4.1 / Cube HAL tree under `src/esc/motor_config/`, are not
listed line by line. CM7 `main` still only clocks the chip and toggles
LD1. ESC `main` runs the generated MCSDK init and an empty loop; it does
not call the kinematics module or the frame codec.

- `src/shared/inc/rover_kinematics.h` — body twist to four wheel motor RPM.
- `src/shared/src/rover_kinematics.c` — provisional wheel map and that map.
- `src/shared/inc/motor_protocol.h` — CONTROL, CONTROL_REPLY, STATUS, and DIAG layout, plus the session and sequence helpers.
- `src/shared/src/motor_protocol.c` — encode and decode of those frames, and those helpers.
- `architecture/protocol.md` — wire format that codec implements.
- `tests/test_rover_kinematics.c` — host CTest for all four wheels.
- `tests/test_motor_protocol.c` — host CTest for the frame codec, all four wheels.
- `tests/CMakeLists.txt` — host build of the real kinematics and codec sources.
- `src/esc/motor_config/CMakeLists.txt` — host cross-compile of the vendored ESC image.
- `scripts/test.sh` — host tests, then the Nucleo CM7/CM4 and ESC cross-compile.
