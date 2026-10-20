# B-G431B-ESC1 CAN bootloader

Installed once over SWD at `0x08000000`; every application update then
arrives over CAN from the Nucleo (`architecture/can-fwu.md`, `src/tools/esc_update.py`).
The protocol sources are shared with the Nucleo: `src/shared/src/can_fwu_*.c` and
`src/shared/inc/` are compiled in directly, so there are no copies to re-sync.

## Layout

| Region     | Address      | Size | Image                  |
|------------|--------------|------|------------------------|
| Bootloader | `0x08000000` | 16K  | `boot_<M>.elf` (here)  |
| App        | `0x08004000` | 110K | `motor_<M>_.._offset.elf` |
| Validity   | `0x0801F800` | 2K   | committed after VERIFY |
| Mailbox (SRAM) | `0x20007FF0` | 16 bytes | reset-retained, NOLOAD |

The app never touches `VTOR`, so the
`VTOR=0x08004000` the bootloader sets before jumping persists.
`STM32G431CBUX_FLASH_APP.ld` + `build_cli.py --app-offset` do the link.

## Behaviour

- Reset → 3 s window for ENTER (red LED `PC6` @ 2 Hz).
- No ENTER + valid app (SP in SRAM) → jump. No valid app → stay, 4 Hz.
- ENTER accepted → session owns the board until VERIFY+JUMP (jump now,
  LED solid 0.5 s) or fault (stay in boot, fast blink, never jump a
  half-erased flash).
- Per-motor build (`-DMOTOR_NE..`): CAN device id + `PC14` termination
  follow the app rule (NE/SE terminated, SW/NW not).

## Build (no hardware needed)

```sh
python3 src/esc/bootloader/build_boot.py --motor NE --output /tmp/boot
cd src/esc/motor_config
python3 build_cli.py --motor NE --app-offset --output /tmp/app
```

Both linkers exclude the mailbox from RAM and set initial MSP to
`0x20007FF0`. Normal non-offset builds retain their existing linker and
behaviour; only `--app-offset` defines `ESC_BOOT_OFFSET_APP`.

## Remote entry (offset applications only)

Send a standard-ID CAN FD data frame, BRS off, DLC exactly 8:

- ID: `0x650 | dev`, exactly one device: NE=1, SE=2, SW=4, NW=8.
- Payload bytes: `42 4f 4f 54 01 <dev> <dev XOR ff> 00`.
- NE example: ID `0x651`, bytes `42 4f 4f 54 01 01 fe 00`.

This command has no ACK and does not reuse the update ACK format. The ISR
only queues it. Admission and the main-loop reset both require MCSDK IDLE,
zero measured speed, no current fault and no running or queued drive
command or Hall alignment (`drive_app_idle()`). Queued entry suppresses subsequent CAN drive commands until main processes it.
Unsafe requests are dropped, not deferred until the motor stops.

The app publishes the mailbox and uses DSB + `NVIC_SystemReset()` in main,
with interrupts masked across the final safety check. The bootloader consumes
and clears the mailbox before HAL initialization, requires software-reset
cause and matching device/magic/complements, then stays listening without the
3-second auto-jump deadline. Explicit verified JUMP still works. A later reset
without another request restores the normal deadline.

### ABI for other applications

Copy `boot_entry.h` and reserve `0x20007FF0..0x20007FFF` as NOLOAD in the
blink linker too; RAM ends and `_estack` must be `0x20007FF0`. Do not write
this address from an old/non-offset application: its stack can occupy it.
Four little-endian 32-bit words are `0x42525131`, `~0x42525131`, `dev`, `~dev`.
Use `esc_boot_request_valid()` only after validating the CAN header, then
queue in ISR and call `esc_boot_mailbox_write()` followed by DSB and software
reset from a safe main-loop context. The command ABI version is 1, independent
of firmware-update wire/ACK versions. No change to `system_stm32g4xx.c` needed.

## Host regression tests

```sh
python3 src/esc/bootloader/tests/run_tests.py
```

Runs C helpers and the real shared session with ASan/UBSan. Covers oversized
HAL-copy buffers/session rejection, strict header checks, duplicate metadata
zero-erase/zero-program behaviour, changed-image rejection, interrupted commit
recovery, erase failure, exact device command selection and one-shot mailbox
consumption/corruption/reset cause. Metadata is invalidated before application
erase as before. Exact committed identity replays only validate flash; invalid
metadata is erased before programming identity and finally magic. Valid but
different committed metadata is never erased to accommodate a replay.

The MSP/CONTROL assembly branch, NVIC clearing, RCC/FDCAN reset and SysTick
shutdown sequence are preserved. Host tests cannot qualify SRAM retention,
real power-loss/ECC behaviour, motor safety or the application handoff on
hardware. Remote entry is not authenticated; physical CAN access is trusted.

## First install (battery OFF, ESC USB/ST-Link connected)

```sh
CLI=~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI
$CLI -c port=SWD sn=<esc-probe> -r32 0x08000000 0x20000 esc-backup-before-boot.bin
$CLI -c port=SWD sn=<esc-probe> -w /tmp/boot/boot_NE.elf -v
$CLI -c port=SWD sn=<esc-probe> -w /tmp/app/motor_NE_drive_offset.elf -v -rst
```

Expect: red LED blinks 2 Hz for 3 s, then stays in boot (no validity
metadata yet); run `src/tools/esc_update.py` to commit the app. If the LED blinks forever, the app image
is missing/invalid — perform a verified CAN update; SWD programming the app
alone does not create validity metadata. The bootloader is intact.

## Recovery

Bootloader never erases itself (pages are app-relative, capped at 55).
Worst case (power cut mid-DATA): board stays in boot blinking fast;
re-run the verified CAN update. An app ELF alone over SWD does not restore
validity metadata. A matching full-chip backup
restores the pre-bootloader state.
