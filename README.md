# sfurs_firmware_v2

Firmware for the SFURS omni-wheel robot: a Nucleo-H755ZI-Q (CM7) runs the
drive, four B-G431B-ESC1 boards run the wheel motors over CAN, and a PC drives
it from the keyboard over the Nucleo's ST-LINK.

## Layout

All code is under `src/`; each documentation concern has its own top-level folder.

| Path | Contents |
|---|---|
| `src/nucleo/` | Nucleo application (CubeMX project: `CM7/`, unused `CM4/`, `Drivers/`, `.ioc`, CMake). Ours: `CM7/Core/Src/{main.c USER CODE, ik_drive.c, ik_telemetry.c, fwu_updater.c, can_fwu_host.c}` |
| `src/esc/` | ESC application (`motor_config/`) and CAN bootloader ([README](src/esc/README.md)) |
| `src/shared/` | Code shared by Nucleo and ESC: kinematics, CAN-FWU protocol (`inc/`, `src/`) and the Nucleo ↔ ESC CAN protocol (`can/can_datatypes.h`) |
| `src/sensors/stm32h7/` | Sensor app for the same Nucleo (FreeRTOS; optical flow, IMU, wheel encoder modules with gtests). Separate CubeMX project, not part of the drive image |
| `src/tools/` | Build, ESC update, drive, tuning and camera-rig tools |
| `tests/` | Host tests; `tests/run_all.py` runs them plus the sensor-module gtests |
| `archive/` | Unused: `stm32h5/` sensor-board skeleton, `rs_i2t/` current-limit module. Not built or tested |
| `docs/` | [System context](docs/index.md) and [lessons](docs/lessons.md) |
| `prd/` | [Product requirements](prd/index.md), by phase |
| `technical-development-plans/` | Phased plans ([convention](technical-development-plans/README.md)) |
| `design-chronological/` | Dated design log ([format](design-chronological/README.md), [master](design-chronological/MASTER.md)) |
| `architecture/` | [System and per-file roles](architecture/index.md), [drive](architecture/drive.md), [CAN firmware update](architecture/can-fwu.md) |
| `system-info/` | [Hardware](system-info/hardware.md): boards, part numbers, wheel map, safety |
| `open-questions/` | [Open questions](open-questions/index.md) |
| `hardware-test-results/` | One file per hardware test session ([template](hardware-test-results/README.md)) |
| `references/` | Datasheets and manuals ([index](references/README.md)) |

## Quick start

```sh
src/tools/build_all.py                        # everything into build/ (no hardware)
python3 tests/run_all.py                      # host tests (no hardware)
src/tools/esc_update.py NE --can                # install ESC app over CAN, per wheel
src/tools/esc_tool.py align all                 # battery ON, wheels raised
src/tools/keyboard_drive.py --speed 0.5         # WASD move, QE rotate, X quit
```

Read [system-info/hardware.md](system-info/hardware.md) before touching hardware, and
[CONTRIBUTING.md](CONTRIBUTING.md) for branches and merge requests.
Without a host toolchain, run the same commands in the dev container
([docker/README.md](docker/README.md)); CI runs `run_all.py` and `build_all.py` in it.

## Builds

- Nucleo drive image: `cmake --preset Debug` in `src/nucleo/CM7/` (or `build_all.py`).
- Nucleo ESC-updater image: same with `-DFWU_IMAGE_FILE=<payload.c>`.
- ESC app: `src/esc/motor_config/build_cli.py` (GCC, no CubeIDE).

Generated HAL/CMSIS (`src/nucleo/Drivers/`) and MCSDK sources are committed; `build/`
directories are ignored. Keep application code in CubeMX `USER CODE` sections
or in our own files so regeneration does not erase it.
