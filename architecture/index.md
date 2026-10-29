# Architecture

Owned files only. Vendor HAL and CMSIS under `src/nucleo/Drivers/` are the
CubeMX tree and are not listed line by line. CM7 `main` still only clocks
the chip and toggles LD1; it does not call the kinematics module yet.

- `src/shared/inc/rover_kinematics.h` — body twist to four wheel motor RPM.
- `src/shared/src/rover_kinematics.c` — provisional wheel map and that map.
- `tests/test_rover_kinematics.c` — host CTest for all four wheels.
- `tests/CMakeLists.txt` — host build of the real kinematics source.
- `scripts/test.sh` — host tests, then the Nucleo CM7/CM4 cross-compile.
