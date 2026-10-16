---
name: CAN FWU Update
description: Flash STM32G431 ESC application firmware over CAN-FD from the NUCLEO-H755 host — provisioning, triggering, monitoring, verifying, and recovering failed updates.
---

# Over-CAN firmware update (living doc — update as the codebase moves)

## Operator handoff UI preference

The operator prefers **interactive multiple-choice or checkbox prompts** for
firmware-update handoffs. Use the question tool rather than requiring typed
phrases such as "BR connected, battery OFF" or showing a non-interactive list.

- After completing/verifying a wheel and releasing both probes, offer a
  single-choice next step: the next named wheel (physical and compass labels),
  another wheel if appropriate, or **Pause / not ready**. Recommend the next
  pending wheel; never infer a cable move from an old selection.
- Use a multi-select confirmation checklist for **motor battery OFF**, **TUI /
  logger / tuner closed**, and **shared hardware released to this session**.
  State that all required conditions must be confirmed before programming;
  unchecked conditions are unknown, not satisfied. Offer Pause separately.
- A clear unsolicited confirmation may satisfy the current handoff without
  another redundant prompt. "BR on" alone is ambiguous; an explicit separate
  "battery OFF" confirmation plus verified probe identity is required.
- Selections confirm operator actions; the agent still independently lists and
  verifies the exact attached serial, payload identity and target selector.
  Selecting a wheel never authorizes motion, option bytes or bootloader writes.
- Keep the Nucleo USB connected; move only the ESC USB between wheels. Require
  fresh target/power confirmation after each cable/power change. Do not ask for
  cable movement while a probe/session or transfer is still active.

See `wheel-motor-setup` for physical-wheel mapping and `docs/HIGH_SPEED_CAPS.md`
for the current coordinated rollout/evidence; those records are not blanket
permission to repeat programming.

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

Motor app images (vs the blink/test payload): prereqs are the offset app
build (`FLASH ORIGIN 0x08004000`, `VECT_TAB_OFFSET 0x4000`) plus the
generated `fwu_test_image.c`; see `docs/CAN_FWU.md` (G4 port checklist /
motor-app notes).

```sh
# pyocd on the Nucleo probe (addresses from current ELF via arm-none-eabi-nm):
pyocd commander -u <nucleo-sn> -t stm32h755zitx \
  -c 'write32 <can_fwu_request> 1' -c quit     # arm once, bench IDLE only
# or hold blue B1 from power-up for 3 s (single-cable benches)
```

Host retries ENTER every 300 ms until answered; later failures latch and need
a fresh request. No automatic erase/data retries.

### Battery-OFF application updates with an existing bootloader

Keep motor battery OFF and power the selected ESC from its USB connection.
The app's undervoltage fault can correctly block remote boot entry. Do not clear
that protection just to program: after all identities/backup/idle gates pass,
request one host session and issue a normal system reset on the **selected ESC
probe** to expose its existing three-second bootloader window. The installed
pyOCD API uses `Target.ResetType.SYSRESETREQ` (not `SW_SYSRESETREQ`). This is a
reset, not a bootloader write or a fault-clear command.

Validate helper APIs and required linked symbols before queuing the session.
Unused globals may be linker-discarded in update-only builds; do not invent RAM
addresses. Independently check the flashed updater and embedded raw payload,
motor producer disabled, bench IDLE, and no pending requests. Bound the host
monitoring interval; on helper failure explicitly stop/clear pending ENTER
retries rather than leaving them armed across a handoff. Do not automatically
restart a transfer that progressed beyond ENTER.

For multi-wheel rollouts, keep the Nucleo on update-only firmware between wheels;
install the matching drive image only after every selected ESC's app/metadata
is independently verified. Back up each wheel before overwriting, preserve its
bootloader, and report partial progress truthfully. Finish/release each session
before presenting the next interactive handoff.

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
