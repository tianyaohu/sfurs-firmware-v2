# Over-the-CAN Firmware Update (P2–P6)

Host: `NUCLEO-H755ZI-Q` (`sfurs_firmware_v2`, CM7). Target: `B-G431B-ESC1`
(`STM32G431CB`, 128K flash) in `firmware-v2/.../motor_config` (port pending).

## Flash layout contract

| Region     | Address          | Size | Content                          |
|------------|------------------|------|----------------------------------|
| Bootloader | `0x08000000`     | 16K  | CAN-FWU target (`can_fwu_boot`)  |
| App        | `0x08004000`     | 112K | `motor_config`, `VTOR` += 0x4000 |

G4 pages are 2K; `CAN_FWU_PAGE_SIZE/APP_MAX/APP_OFFSET` enforce this.
Single source: `Common/Inc/can_fwu_version.h`.

## Wire protocol (P2)

500 kbit/s, `FD_NO_BRS`, 8-byte payloads. FWU owns `0x600..0x6FF`;
low nibble = wheel device bits (`NE=1, SE=2, SW=4, NW=8`).
No collision with drive (`0x080/0x100/0x180/0x280/0x300/0x380/0x400`)
or `STOP 0x780`.

| ID (host→target) | Payload (LE)              |
|------------------|---------------------------|
| `0x600\|dev` ENTER  | magic `CFWU`, img_size    |
| `0x610\|dev` ERASE  | start_page, num_pages     |
| `0x620\|dev` DATA   | seq u16 + 6 chunk bytes   |
| `0x630\|dev` VERIFY | crc32, size               |
| `0x640\|dev` JUMP   | magic, 0                  |
| `0x680\|dev` ACK    | status, detail (seq/crc)  |

Status: `0 OK, 1 REJECT, 2 BAD_SEQ, 3 CRC_MISMATCH, 4 FLASH_ERR, 5 BAD_SIZE`.
CRC32-IEEE; `can_fwu_crc32("123456789") == 0xCBF43926` is the test vector.
Host ACK budget 100 ms per frame (matches bench); target idles out after 2 s.

## Safety (P5) — read before any powered run

1. Bench must be `BENCH_IDLE`; `RUNNING/ARMED` refuse. Target ENTER
   refuses unless its motor reports stopped.
2. Transfers start only from an explicit debugger request
   (`can_fwu_request`); nothing is periodic or automatic.
3. `STOP 0x78F x3` first; battery discipline per `WHEEL_BENCH.md`.
4. Any timeout/corruption latches FAULT; session restarts at ENTER.
5. SWD stays the recovery path: full 128K backups live in `esc-backups/`.
   Never mass-erase, never touch option bytes without sign-off.

## G4 port checklist (follows this branch)

- [ ] New `motor_config` linker: `FLASH ORIGIN 0x08004000 LENGTH 112K`,
      `VECT_TAB_OFFSET 0x4000`, early `SCB->VTOR` relocate.
- [ ] Bootloader project at `0x08000000` (~16K): FDCAN1 `PA11/PB9 AF9`
      500k `FD_NO_BRS`, `can_fwu_boot` + `HAL_FLASHEx_Erase` /
      `HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD)` backend,
      motor-stopped hook, flash-readback verify before JUMP.
- [ ] Widen the app's exact-match CAN mask filter to accept `0x600|dev`.
- [ ] Bench-test ENTER→ERASE→DATA→VERIFY→JUMP on one wheel (NE) first.

## Files

- `Common/Inc/can_fwu_protocol.h`, `Common/Src/can_fwu_protocol.c` — P2
- `Common/Inc/can_fwu_boot.h`, `Common/Src/can_fwu_boot.c` — P3 target
- `CM7/Core/Inc/can_fwu_host.h`, `CM7/Core/Src/can_fwu_host.c` — P4 host
- `CM7/Core/Src/main.c` — dormant wiring: ACK dispatch + debugger pump
- `tests/test_can_fwu.py` — CRC, 10 KiB loopback + jump, corruption,
  motor-active, seq/timeout gates
- Follow-up: USB-CDC image staging to feed `can_fwu_host_begin`.
