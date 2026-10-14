---
name: NUCLEO H755 Build and Flash
description: Build CM7 or CM4 firmware in this project for the NUCLEO-H755ZI-Q, troubleshoot ST-LINK detection, and flash firmware with STM32CubeProgrammer when requested.
---

# Board and project

Work from the project root containing CM7, CM4, and gcc-arm-none-eabi.cmake
(three directories above this skill directory). Inspect current source and build
settings before acting; preserve user edits. Keep application additions inside
STM32CubeMX USER CODE sections.

- Board: NUCLEO-H755ZI-Q, STM32H755, Cortex-M7 and Cortex-M4.
- ST-LINK V3 USB connector: CN1. A data-capable cable is required.
- User LEDs, active high: LD1 green PB0, LD2 yellow PE1, LD3 red PB14.
- CM7 flash linker script: CM7/stm32h755xx_flash_CM7.ld.
- CM7 ELF starts at 0x08000000; use ELF addresses when programming.
- Check DUAL_CORE_BOOT_SYNC_SEQUENCE in both main.c files. Enabled CM7
  synchronization expects matching CM4 firmware. Do not change core boot option
  bytes or disable synchronization without user authorization.

# Build

1. Check cmake (>=3.22), ninja, arm-none-eabi-gcc and arm-none-eabi-g++.
2. From CM7 run `cmake --preset Debug`, then `cmake --build --preset Debug`.
   For Release use the Release preset. CM4 has its own presets; build it only
   when requested or needed with user agreement.
3. Output: CM7/build/Debug/sfurs_firmware_v2_CM7.elf (or Release).
   Building the root project uses a different output layout and may build both
   cores; prefer the per-core presets for a CM7-only request.
4. Report actual compiler/linker results and memory usage. Do not flash a stale
   ELF after a failed build.

## Tools on this Linux PC

The current locally extracted toolchain is at
`/tmp/opencode/cm7-toolchain/root/usr/bin`. If it still exists, prepend it to
PATH for both configure and build. This temporary directory may disappear;
do not assume it is a permanent installation.

System packages on Ubuntu/Debian: cmake, ninja-build, gcc-arm-none-eabi,
libnewlib-arm-none-eabi, libnewlib-dev. Ask the user to install with sudo if
needed; never request their password. An unprivileged fallback is downloading
gcc-arm-none-eabi, binutils-arm-none-eabi, libnewlib-arm-none-eabi,
libnewlib-dev and ninja-build with apt-get download into a dedicated directory
under /tmp/opencode, then extracting with dpkg-deb -x into a local root.
For these Ubuntu packages, the extracted root/usr/lib/arm-none-eabi directory
needs `include -> ../../include/newlib` and `lib -> newlib` symlinks normally
provided by installation. Inspect existing paths before creating them.

# Detect and flash

Current CLI: `$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI`.
Discover it again if this path does not exist.

1. List probes with `STM32_Programmer_CLI -l stlink` (use the full path if
   necessary). The currently known probe serial is 002200364142500620353451;
   verify rather than assuming it is the connected board.
2. If no USB device appears in lsusb, check CN1, power and the data cable.
   If libusb reports errno=13, have the user copy the install's Drivers/rules
   files into /etc/udev/rules.d, reload udev rules and reconnect the board.
3. Flash only when the user explicitly requests programming. From the project
   root, for Debug CM7:

   ```sh
   "$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI" \
     -c port=SWD sn=002200364142500620353451 \
     -w CM7/build/Debug/sfurs_firmware_v2_CM7.elf -v -rst
   ```

   Replace the serial if a different authorized board is connected. Do not
   mass-erase flash, flash CM4, or alter option bytes without authorization.
4. Confirm download, verification and reset separately using CLI output.
   Verification proves flash contents, not that the application runs. For the
   current LED test, ask the user to confirm green LD1 blinks (500 ms on/off).
