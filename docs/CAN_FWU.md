# ESC firmware update over CAN

Each ESC keeps a CAN bootloader (`HW/B-G431B-ESC1/bootloader/`, installed
once over SWD). Applications are installed over CAN from the Nucleo, which
runs a separate **updater image** carrying one ESC application.

## Procedure

```sh
scripts/build_all.py                       # ESC apps, payloads, updaters, drive image
scripts/esc_update.py NE --can             # one wheel; repeat for SE SW NW
```

`--can` needs only the Nucleo USB; the battery may be ON. Through the drive
image it checks that all four ESCs answer, are IDLE and report Vbus, records
the wheel's IDENT (CRC32 and size of the running app). The updater image
then sends the boot-entry request `0x650|sel`, which the app accepts only
when idle with no fault: it resets into its bootloader, which listens until
ENTER. One transfer; the bootloader checks the CRC at VERIFY before it
commits the validity record and jumps. Back on the drive image, the wheel's
IDENT must equal the built image and all four must be IDLE. A failed
transfer leaves that ESC in its bootloader, listening: rerun with
`--can --target-in-boot`. No flash backup on this path.

`--battery-off` is the SWD path, for an ESC whose app can't take the boot
request (first install, or a broken app). Per wheel: battery OFF, that ESC's
ST-LINK USB and the Nucleo USB connected. It backs up the full ESC flash to `esc-backups/update-<W>-<UTC>/`,
flashes the updater onto the Nucleo, resets the ESC to open the bootloader's
3 s window, transfers once (no automatic retry), verifies by readback
(bootloader unchanged, app bytes, metadata, VTOR, app running), then puts the
drive image back on the Nucleo. `--keep-updater` skips that last step.

A failed transfer leaves the bootloader intact: the ESC blinks in boot mode
and accepts a new transfer. SWD restore of a full backup is the last resort.

## Layout and protocol

| Region | Address | Size |
|---|---|---|
| Bootloader | `0x08000000` | 16 KiB |
| Application | `0x08004000` | 110 KiB |
| Validity metadata | `0x0801F800` | 2 KiB |
| Boot mailbox (SRAM) | `0x20007FF0` | 16 B |

IDs `0x600–0x6FF`, low nibble = wheel select bit, 8-byte payloads:
ENTER `0x600` (magic `CFWU`, size), ERASE `0x610`, DATA `0x620` (u16 seq +
6 bytes), VERIFY `0x630` (CRC32-IEEE, size), JUMP `0x640`, ACK `0x680`
(status: 0 OK, 1 REJECT, 2 BAD_SEQ, 3 CRC_MISMATCH, 4 FLASH_ERR, 5 BAD_SIZE).
100 ms ACK budget per frame, 2 s for erase.

The bootloader only jumps to an app with committed metadata and a matching
CRC; an app written over SWD alone does not boot. A running app reboots into
the bootloader on `0x650|dev` with payload `BOOT 01 dev ~dev 00` when idle.

## Updater image (Nucleo)

Built with `-DFWU_IMAGE_FILE=<payload.c>` (from `scripts/fwu_image.py`).
Debugger words: `can_fwu_dev`, `can_fwu_request` (write 1),
`can_fwu_result` (1 active, 2 rejected, 3 done, 4 failed). Holding B1 for 3 s
from power-up also starts one update. ENTER repeats every 300 ms until the
bootloader answers; after that any failure latches until a new request.

## Code

- `Common/{Inc,Src}/can_fwu_protocol.*`: wire format, CRC (shared).
- `Common/{Inc,Src}/can_fwu_boot.*`: target session; vendored copy in
  `HW/B-G431B-ESC1/bootloader/` (keep in sync manually).
- `CM7/Core/Src/can_fwu_host.c`: host session; `fwu_updater.c`: request logic.
- Tests: `tests/test_can_fwu.py`, `test_fwu_updater.py`, `test_fwu_*`,
  `HW/B-G431B-ESC1/bootloader/tests/run_tests.py`.
