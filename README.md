# sfurs_firmware_v2

Successor to `firmware-v2` for the NUCLEO-H755ZI-Q (STM32H755, dual-core CM7 + CM4).

Source of truth for CubeMX configuration is `sfurs_firmware_v2.ioc`.
Generated HAL/Drivers are committed; `build/` directories and `.mxproject`
are ignored.

## Setup (pick one)

**A — Docker (recommended, ~5 min, no host toolchain):**

```sh
# prerequisites: docker engine + compose plugin, ST-LINK udev rules on host
UID=$(id -u) GID=$(id -g) docker compose build
docker compose run --rm dev bash
```

You land in `/work/sfurs_firmware_v2` as user `dev`, files owned by you.
Details, USB/flashing and the old-repo ESC build: see [DOCKER.md](DOCKER.md).

**B — Bare metal (Ubuntu 22.04/24.04):**

```sh
sudo apt-get update && sudo apt-get install -y build-essential cmake ninja-build \
  git python3 gcc-arm-none-eabi libnewlib-arm-none-eabi libusb-1.0-0 usbutils
sudo mkdir -p /tmp/opencode && sudo chmod 777 /tmp/opencode
```

ST-LINK access: install STM32CubeProgrammer (free, ST account) and copy its
`Drivers/rules/*` into `/etc/udev/rules.d/`, then reload udev and replug.
The bench scripts expect it at
`~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI`
(override with `--cli`).

## First run

```sh
python3 tests/test_wheel_bench.py     # host bench state-machine tests
python3 tests/test_wheel_sender.py    # host sender selection tests
cd CM7 && cmake --preset Debug && cmake --build --preset Debug
# -> CM7/build/Debug/sfurs_firmware_v2_CM7.elf
```

Flash (battery off, data cable on Nucleo, replace serial if different):

```sh
"$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI" \
  -c port=SWD sn=002200364142500620353451 \
  -w CM7/build/Debug/sfurs_firmware_v2_CM7.elf -v -rst
```

Build/flash reference (probe selection, both cores, LED check):
`.opencode/skills/nucleo-h755-build/SKILL.md`.

## Layout

- `CM7/` — primary application core (wheel bench, CAN sender, ESC interface)
- `CM4/` — secondary core
- `Common/` — shared code
- `Drivers/` — CubeMX-generated HAL / CMSIS
- `tests/` — host-side Python tests (`tests/test_wheel_*.py`)
- `docker/` + `docker-compose.yml` + `DOCKER.md` — container dev environment
- `.gitlab-ci.yml` — CI: build image, run tests + CM7 build, push to registry

## Branches

- `main` — stable base
- `motor-testing` — repeatable bench runs, NE reverse -100 RPM, keyboard jog TUI
- `docker-dev-env` — this container setup (to merge into `main`)

## Relationship to firmware-v2

This repository is intended to eventually replace the old `firmware-v2`
repository. Until that cutover, `firmware-v2` remains the reference for
B-G431B-ESC1 motor-config work; this repo holds the new H755 bench firmware.
