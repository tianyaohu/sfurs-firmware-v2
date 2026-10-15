---
name: CAN FWU Update
description: Flash STM32G431 ESC application firmware over CAN-FD from the NUCLEO-H755 host — provisioning, triggering, monitoring, verifying, and recovering failed updates.
---

# Over-CAN firmware update (living doc — update as the codebase moves)

## Topology

- Host: NUCLEO-H755ZI-Q (`CM7/`, FDCAN1 `PB8/PB9`, 500 kbit/s, FD no-BRS).
- Targets: B-G431B-ESC1 (STM32G431CB, 128 KiB flash), one per wheel, shared bus.
- Protocol v2, standard IDs, 8-byte payloads: `ENTER 0x600|dev`, `ERASE 0x610|dev`,
  `DATA 0x620|dev` (seq16 + 6 bytes), `VERIFY 0x630|dev`, `JUMP 0x640|dev`,
  `ACK 0x680|dev` (`status8/command8/session16/detail32`). Remote boot entry
  `0x650|dev`. Devices: NE=1, SE=2, SW=4, NW=8.
- **v1 and v2 are wire-incompatible. Never mix versions on one bus.**

## Flash layout (G431)

| Region | Address | Size |
|---|---|---|
| Bootloader | `0x08000000` | 16 KiB |
| App | `0x08004000` | 110 KiB |
| Validity metadata | `0x0801F800` | one 2 KiB page |

Bootloader sets `VTOR=0x08004000` before jumping; the app never touches VTOR.
Validity (magic + complement, size, CRC32, base, version) commits last, only
after flash-readback CRC and vector checks. No metadata = no boot. Single
slot: reflash recovery, not rollback. SWD remains the bootloader-repair path.

## Safety rules

- Update one ESC at a time; bench must be IDLE (host refuses otherwise).
- No ARM/RUN/drive frames during a session. Bootloader never inits PWM;
  blink/test apps hold all six gate inputs LOW.
- A faulted target never jumps half-erased flash — re-ENTER recovers it.
- Full 128 KiB SWD backup before first provisioning; keep it for rollback.
- Distinguish TX-acknowledged (any node can ACK) from received-and-processed.

## A. First provisioning (SWD, one time per ESC)

```sh
# firmware-v2 side
python3 HW/B-G431B-ESC1/bootloader/build_boot.py --motor NE --output /tmp/boot
# sfurs side not needed for provisioning
CLI=~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI
$CLI -l stlink   # confirm ESC probe SN; use sn=<sn> below
$CLI -c port=SWD sn=<esc> -u 0x08000000 0x20000 esc-backup-<wheel>-<date>.bin
$CLI -c port=SWD sn=<esc> -w /tmp/boot/boot_NE.elf -v -rst
```

Without valid metadata the bootloader stays listening (red LED ~3 Hz blink);
it will not boot a stale app.

## B. Update over CAN

Prereqs: H755 firmware with embedded image (`CM7/Core/Src/fwu_test_image.c`
or generated blink image), shared bus wired + terminated, both boards powered.

```sh
# pyocd on the Nucleo probe (addresses from current ELF via arm-none-eabi-nm):
pyocd commander -u <nucleo-sn> -t stm32h755zitx \
  -c 'write32 <can_fwu_request> 1' -c quit     # arm once, bench IDLE only
# or hold blue B1 from power-up for 3 s (single-cable benches)
```

Host retries ENTER every 300 ms until answered; later failures latch and need
a fresh request. No automatic erase/data retries.

## C. Monitor

- NE red LED: window blink → fast session flicker → solid 0.5 s → off (jumped);
  solid-ON = faulted, stays for re-ENTER.
- Nucleo LD2 yellow lit while session active, LD3 red on fault.
- Live counters via debugger (no halt needed for reads):
  - NE `boot.state/rx/txok` (`can_fwu_boot` instance; state 6 = FAULT).
  - Nucleo `can_fwu_result` (0 none, 1 active, 2 rejected, 3 done, 4 failed),
    `can_fwu_rx_cnt` (ACKs passing validation).

## D. Verify (independent of host DONE)

```sh
# read back app bytes + metadata, compare to source binary + CRC32
pyocd commander -u <esc-sn> -t stm32g431cbux \
  -c 'savemem 0x08004000 <size> /tmp/readback.bin' \
  -c 'read32 0x0801F800 32' -c quit
```

Then reset and confirm VTOR `0x08004000`, PC inside the app, expected LED/
telemetry behavior. `result 3` means the JUMP reply arrived, not that the app
is healthy.

## E. Recovery

- Faulted target (solid red): just re-arm the host; ENTER re-inits from any state.
- Corrupt app: bootloader stays listening (no valid metadata); reflash over CAN.
- Broken bootloader: SWD-write `boot_<wheel>.elf`, then the app ELF.
- Full retreat: SWD-write the pre-provisioning 128 KiB backup.

## Regression gates

```sh
python3 tests/test_can_fwu.py && python3 tests/test_fwu_validity.py && \
python3 tests/test_fwu_main_timing.py && python3 tests/test_fwu_main_control.py
```

Full matrix: `docs/CAN_FWU_TEST_MATRIX.md`. Procedure: `docs/CAN_FWU_BLINK_TEST.md`.
