# sfurs_firmware_v2

Firmware for the SFURS omni-wheel robot: a Nucleo-H755ZI-Q (CM7) runs the
drive, four B-G431B-ESC1 boards run the wheel motors over CAN, and a PC drives
it from the keyboard over the Nucleo's ST-LINK.

## Layout

| Path | Contents |
|---|---|
| `CM7/` | Nucleo application. Ours: `Core/Src/{main.c USER CODE, ik_drive.c, ik_telemetry.c, fwu_updater.c, can_fwu_host.c}` |
| `CM4/` | Second core, unused (CubeMX skeleton) |
| `Common/` | Code shared by Nucleo and ESC bootloader: kinematics, CAN-FWU protocol |
| `HW/B-G431B-ESC1/` | ESC application (`motor_config/`) and CAN bootloader ([README](HW/B-G431B-ESC1/README.md)) |
| `HW/common/CAN/can_datatypes.h` | Nucleo ↔ ESC CAN protocol |
| `HW/STM32H5/`, `HW/STM32H7/` | Other boards from the old firmware-v2 repo; not built here |
| `scripts/` | Build, ESC update, drive and setup tools (below) |
| `tests/` | Host tests; `tests/run_all.py` runs everything |
| `docs/` | [DRIVE](docs/DRIVE.md), [HARDWARE](docs/HARDWARE.md), [CAN_FWU](docs/CAN_FWU.md), [LESSONS](docs/LESSONS.md) |
| `sfurs_firmware_v2.ioc` | CubeMX source of truth for the Nucleo |

## Quick start

```sh
scripts/build_all.py                          # everything into build/ (no hardware)
python3 tests/run_all.py                      # host tests (no hardware)
scripts/esc_update.py NE --battery-off        # install ESC app over CAN, per wheel
scripts/esc_tool.py align all                 # battery ON, wheels raised
scripts/keyboard_drive.py --speed 0.5         # WASD move, QE rotate, X quit
```

Read [docs/HARDWARE.md](docs/HARDWARE.md) before touching hardware.

## Builds

- Nucleo drive image: `cmake --preset Debug` in `CM7/` (or `build_all.py`).
- Nucleo ESC-updater image: same with `-DFWU_IMAGE_FILE=<payload.c>`.
- ESC app: `HW/B-G431B-ESC1/motor_config/build_cli.py` (GCC, no CubeIDE).

Generated HAL/CMSIS (`Drivers/`) and MCSDK sources are committed; `build/`
directories are ignored. Keep application code in CubeMX `USER CODE` sections
or in our own files so regeneration does not erase it.
