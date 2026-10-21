# Hardware and tools

## Boards

- **Nucleo-H755ZI-Q** (STM32H755): only the CM7 core runs firmware; CM4 is
  unused. ST-LINK V3 on CN1. LEDs: LD1 green PB0 alive, LD2 yellow PE1 drive
  running / FWU activity, LD3 red PB14 CAN TX failure / FWU fault.
- **4 × B-G431B-ESC1** (STM32G431CB, 128 KiB flash), one per wheel, one CAN
  bus. Flash: CAN bootloader 16 KiB at `0x08000000`, app 110 KiB at
  `0x08004000`, validity page at `0x0801F800`.
- **Motors**: 8 pole pairs, Hall sensors (120°), direct drive, 65.5 mm omni
  wheels. **Battery**: 3S, ~12.4 V full.

## Wheel map

| Robot wheel | ESC build | CAN select | ESC ST-LINK serial | CAN termination | Hall shift |
|---|---|---|---|---|---|
| FR | `MOTOR_NE` | 1 | `066AFF525266555067075126` | on | -60° |
| BR | `MOTOR_SE` | 2 | `0667FF525266555067073424` | on | -64° |
| BL | `MOTOR_SW` | 4 | `066FFF525266555067073058` | off | -59° |
| FL | `MOTOR_NW` | 8 | `066AFF515266555067023255` | off | -62° |

Hall shifts measured with `esc_tool align` (2026-10-16, battery on); build
with `scripts/build.sh --shift NE=-60 SE=-64 SW=-59 NW=-62`.

Nucleo ST-LINK: `002200364142500620353451`. FR and FL serials differ in only a
few characters; check the full serial before any write.

## Tools on the lab PC

- ARM GCC 10.3 (Debian packages extracted without root) in
  `~/.local/opt/arm-gcc`, with wrappers in `~/.local/bin` that add the newlib
  headers; ninja from `pip install --user ninja`. On a fresh PC just install
  `gcc-arm-none-eabi libnewlib-arm-none-eabi cmake ninja-build`.
- `~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI`
- OpenOCD (`apt install openocd`, 0.11 or newer; `src/tools/lib/ocd.c` starts one per probe serial).
- ESC flash backups: `/home/fh/sfurs_firmware/esc-backups/` (outside git).
  `*-original-20261012.bin` are the boards as received (previous engineer's
  sensorless image, no bootloader).

## Safety rules

- **Flashing an ESC over SWD: battery OFF.** Over CAN (`esc_update --can`,
  no ESC USB): battery ON is allowed when every ESC reports IDLE with no
  fault, Vbus is measured, and the bootloader verifies the image CRC.
- Battery state is measured: ESC Vbus over CAN (`esc_tool status`) or SWD;
  < 8 V is off (USB alone ~4.6 V, the 3S pack ~12 V).
- Power an ESC from the battery before USB: USB-first latches undervolt
  (FAULT_OVER). `esc_tool stop` clears it.
- Wheels off the ground for alignment and first runs after any firmware change.
- One program per probe: close `keyboard_drive` / `esc_tool` before
  flashing or attaching another tool.
- Never mass-erase, change option bytes, or flash CM4.
- Back up an ESC's full flash before overwriting it (`esc_update` does).
- Attach to a running ESC only through `lib/ocd.c` (OpenOCD, no `init`-time reset or halt) or CLI
  `mode=HOTPLUG`; a normal connect resets it.
