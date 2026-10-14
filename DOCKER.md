# Docker dev environment

One-command bench setup: ARM GCC + CMake/Ninja + host GCC + Python 3, with
ST-LINK USB passthrough. No host toolchain install needed.

## Quickstart

```sh
cd sfurs_firmware_v2
UID=$(id -u) GID=$(id -g) docker compose build
docker compose run --rm dev bash
```

Inside the container the repo is at `/work/sfurs_firmware_v2` (mounted from
the host, files stay owned by you). Output of the CM7 build:
`CM7/build/Debug/sfurs_firmware_v2_CM7.elf`.

```sh
# host-side bench tests (needs only cc + python3)
python3 tests/test_wheel_bench.py
python3 tests/test_wheel_sender.py

# CM7 firmware build (presets live in CM7/)
cd CM7 && cmake --preset Debug && cmake --build --preset Debug

If you previously built on the host, wipe the cache dir first: the CMake
cache stores absolute paths and the host ninja location, so a host-created
`CM7/build` fails inside the container (and vice versa). One-time:
`rm -rf CM7/build`, then configure fresh. (`CM7/build/` is git-ignored.)

## ESC (old `firmware-v2` repo) builds

Mount it alongside (same image, extra volume), e.g.:

```sh
docker run --rm -it -v /home/fh/sfurs_firmware:/work \
  -w /work/firmware-v2/HW/B-G431B-ESC1/motor_config sfurs-firmware-dev bash
```

then `python3 build_cli.py --motor NE --reverse-speed-diagnostic --output
/tmp/esc-build` and the `test_*_guards.py` suites.

## Flashing (needs host STM32CubeProgrammer)

CubeProgrammer is proprietary and not in the image. Mount your host install
read-only to the same path the scripts expect:

```sh
docker run --rm -it --device /dev/bus/usb \
  -v ..:/work \
  -v ~/STMicroelectronics:/home/dev/STMicroelectronics:ro \
  -w /work/sfurs_firmware_v2 sfurs-firmware-dev bash
```

Check `lsusb` shows the ST-LINK, then use the `STM32_Programmer_CLI -l stlink`
probe list and the project flash commands. Host udev rules for ST-LINK must
already exist (they do if flashing works on the host).

## Notes

- `devices: /dev/bus/usb` gives the container raw USB; no `--privileged`.
- If `docker compose run` fails on USB permissions, re-plug the probe after
  the host udev rules are installed, or add your user to `plugdev`/`dialout`.
- Image rebuild is only needed when `docker/Dockerfile` changes.
