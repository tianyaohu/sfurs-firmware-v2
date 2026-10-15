# sfurs_firmware_v2

Successor to `firmware-v2` for the NUCLEO-H755ZI-Q (STM32H755, dual-core CM7 + CM4).

Source of truth for CubeMX configuration is `sfurs_firmware_v2.ioc`.
Generated HAL/Drivers are committed; `build/` directories and `.mxproject`
are ignored.

## Layout

- `CM7/` — primary application core (wheel bench, CAN sender, ESC interface)
- `CM4/` — secondary core
- `Common/` — shared code
- `Drivers/` — CubeMX-generated HAL / CMSIS
- `tests/` — host-side Python tests (`tests/test_wheel_*.py`)

## Build

See `.opencode/skills/nucleo-h755-build/SKILL.md` for the supported
build/flash flow (CM7 / CM4, ST-LINK probe selection, STM32CubeProgrammer).

## Omni-wheel keyboard IK

See [keyboard IK setup](docs/KEYBOARD_IK.md) for the separate, opt-in CM7
IK image and keyboard TUI, and [geometry notes](docs/OMNI_KINEMATICS.md)
for provisional measurements. The normal single-wheel TUI is unchanged.
The current ESC interface requires stopping before changing direction;
continuous steering is not implemented.

## Relationship to firmware-v2

This repository is intended to eventually replace the old `firmware-v2`
repository. Until that cutover, `firmware-v2` remains the reference for
B-G431B-ESC1 motor-config work; this repo holds the new H755 bench firmware.
