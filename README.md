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

The Nucleo CubeMX project is vendored: HAL, CMSIS, `sfurs_firmware_v2.ioc`,
and the generated startup for CM7 and CM4. CM7 `main` runs that init and
toggles LD1 (green, PB0) in the image. That source cross-compiles; it is
not a hardware test. Drive, telemetry, skynet, and firmware-update modules
are absent. The ESC / MCSDK vendor tree is the next phase, under the same
host-only rule.

STM32Cube files keep ST's license (`src/nucleo/Drivers/*/LICENSE.txt` and
the file headers). `LICENSE` at the repo root is MIT for code we add.

## Build

`scripts/test.sh` cross-compiles both images with `arm-none-eabi-gcc`,
CMake, and Ninja. GitHub Actions installs that toolchain on `ubuntu-latest`
and runs the script. The runner has no robot attached. There are no host
unit tests yet.

```sh
scripts/test.sh
```

CM7 output: `build/nucleo-cm7/sfurs_firmware_v2_CM7.elf`.
CM4 output: `build/nucleo-cm4/sfurs_firmware_v2_CM4.elf` (CubeMX dual-core
skeleton; the CM4 boot handshake in CM7 `main` stays disabled).

## After this

1. ESC / MCSDK vendor tree, cross-compiled the same way.
2. Owned modules with host tests and fakes: kinematics, then protocol
   codecs. Those pull requests do not need a Nucleo or an ESC attached.
3. Optional hardware milestones (for example LD1 on a board), only when
   you ask.
