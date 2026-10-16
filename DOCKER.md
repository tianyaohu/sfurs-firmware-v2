# Docker dev environment

ARM GCC + CMake/Ninja + host GCC + Python 3 + pyOCD, with ST-LINK USB
passthrough. No host toolchain install needed.

```sh
UID=$(id -u) GID=$(id -g) docker compose build
docker compose run --rm dev bash
# inside: the checkout is at /work/sfurs_firmware_v2
python3 tests/run_all.py
python3 scripts/build_all.py
```

`build/` CMake caches store absolute paths, so a cache made on the host fails
in the container and vice versa: `rm -rf build` when switching.

The checkout is mounted at a different path than on the host, so `git` inside
the container does not work in a linked worktree; run git on the host.

## Hardware

`scripts/` talk to the probes through pyOCD, which is in the image. Check
`lsusb` shows the ST-LINK. Host udev rules for ST-LINK must already exist (they
do if the probe works on the host); re-plug after installing them.

STM32CubeProgrammer is proprietary and not in the image. Mount a host install
read-only where `docs/HARDWARE.md` expects it:

```sh
docker compose run --rm \
  -v ~/STMicroelectronics:/home/dev/STMicroelectronics:ro dev bash
```

Image rebuild is only needed when `docker/Dockerfile` changes.
