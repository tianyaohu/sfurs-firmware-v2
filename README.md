# SFURS firmware rebuild

From-scratch rebuild of the omni-wheel firmware (Nucleo-H755 CM7 and four
B-G431B-ESC1 wheels on CAN). `develop_ai` is the read-only reference.
Owned application code is not copied from that branch; it lands later, in
small pull requests, on top of this vendor tree.

This phase vendors the Nucleo CubeMX project: HAL, CMSIS, the `.ioc`, and
the generated startup. CM7 `main` only runs that init and toggles LD1
(green, PB0, active high) every 500 ms. Drive, telemetry, skynet, and
firmware-update sources are not here. The ESC / MCSDK tree is the next phase.

STM32Cube files keep ST's license (`src/nucleo/Drivers/*/LICENSE.txt` and
the file headers). `LICENSE` at the repo root is MIT for code we add.

## Build

Needs `arm-none-eabi-gcc`, CMake, and Ninja. `scripts/test.sh` builds both
images. There are no host tests yet.

```sh
cmake -S src/nucleo/CM7 -B build/nucleo-cm7 -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=src/nucleo/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/nucleo-cm7
```

CM4 is the CubeMX dual-core skeleton. This phase runs CM7 alone (the CM4
boot handshake in `main.c` is left disabled), so the image to flash later
is `build/nucleo-cm7/sfurs_firmware_v2_CM7.elf`. Do not flash it from this
branch until a later phase says so.
