# CAN flash acceptance test (NE only)

## Hardware result — 2026-10-14

Successful H755 -> NE CAN transfer, followed by software reset and app boot:

- Payload: 232 bytes, CRC32 `37C9233C` (compiler-specific build artifact).
- Host: DONE/result 3, 39 DATA frames acknowledged, then VERIFY and JUMP.
- Independent SWD readback: all 232 application bytes match the source binary.
- Metadata: magic `56414C49`, complement `A9BEB3B6`, size `000000E8`,
  CRC `37C9233C`, base `08004000`, protocol version 1.
- CPU PC sampled at `080040C4`, and `080040C2` after software reset;
  VTOR `08004000`. Motor gate output bits read LOW.
- User visually confirmed the red LED blinking continuously.

SWD delivered only the bootloader to NE, not the blink application.
Cold power-cycle and intentionally interrupted transfer are NOT yet tested.

## Proven prerequisite

NE ENTER replies reach the H755 RX FIFO and pass header validation.
The earlier handshake failure was a stale enqueue timestamp: image CRC
initialization took 162-191ms, exceeding the 100ms response budget before
transmission. Refresh `HAL_GetTick()` immediately before the host pump.
The non-destructive probe now advances ENTER -> ERASE -> diagnostic REJECT.

## Current acceptance payload

`tests/esc_blink/` builds a freestanding G431 application at `0x08004000`.
CMake generates and embeds its bytes in the Nucleo build. It contains no
motor-control library; all six PWM inputs are driven LOW. PC6 toggles every
500ms forever. No CAN traffic is emitted by the application.
The former embedded motor-control image is removed from this test build.
Nucleo bench ARM/RUN/button handling is disabled by `FWU_BLINK_BENCH_ONLY`.

## Persistent validity layout

- Boot: `0x08000000`, 16KiB.
- App: `0x08004000`, **110KiB maximum**.
- Validity: `0x0801F800`, one reserved 2KiB page.

The target invalidates metadata before erasing application pages. VERIFY
checks flash-readback CRC and stack/reset vectors. It writes size/CRC/base/
protocol version first and bootable magic/complement last. Reset validates
metadata and CRC; an incomplete update stays in boot. This is single-slot
reflash recovery, NOT rollback. SWD remains necessary for boot repair.
The old SWD-installed app has no new validity record, so it will not boot
automatically under this bootloader until a verified CAN update commits one.

## One-cable sequence

1. Flash the Nucleo test build, without arming it.
2. Move USB to NE, leave Nucleo unpowered; install only the hardened boot ELF.
   Do NOT SWD-flash the blink app; its delivery must occur over CAN.
3. Leave NE battery-powered, move USB back to the Nucleo.
4. From the debugger, explicitly set `can_fwu_request=1` using addresses
   resolved from the current ELF. Bootloader remains listening because no
   valid new metadata exists, eliminating the reset-window timing race.
5. Expected: ENTER -> ERASE -> DATA -> VERIFY -> JUMP. Nucleo result 3 means
   the JUMP response was received, not proof that the app is running.
6. Visually confirm NE's red LED blinking forever. Then SWD-read the
   application bytes and metadata, compare the actual binary/CRC, and
   power-cycle NE to confirm boot-time validity and handoff.

Do not deliberately interrupt an update until the complete path works.
The host never automatically restarts an ERASE/DATA failure. Unanswered
ENTER requests may repeat while explicitly armed; errors afterward latch.
