# ESC auto-CAL / CAN FWU rollout

Work in `sfurs-hw-session` on branch `hw-session`. Nothing pushed or merged
to `develop` during this rollout. No RUN commands have been sent.

## Motor-enabled Nucleo handoff

CM7 built with `NUCLEO_MOTOR_ENABLED=ON`, flashed and verified on probe
`002200364142500620353451`; reset completed. Live readback confirmed
`wheel_bench_motor_enabled=1`, bench IDLE, request/run count 0, FWU request
and result 0. CM4 and ESC flash were not changed for this step.
Host build: 67820 B flash, 2216 B RAM. Probe released for TUI use.
All 34 wheel-bench scenarios and sender/CAL/FWU controller/timing/motor-image
host tests passed. No physical spin was performed by the agent.

Run `python3 scripts/keyboard_spin.py` from this worktree root. Keys 1–4
select NE/SE/SW/NW; `f`/`r` set direction, `+`/`-` set speed, SPACE runs,
`s` stops and `q` stops/exits. Following a reset or power cycle, the first
ARM may only acknowledge a cleared fault or perform RAM auto-CAL; release,
wait for calibration, and retry rather than assuming host RUNNING proves motion.

| Wheel | Full backup | Bootloader | CAN app + byte/metadata readback | Powered auto-CAL | Remote boot entry |
|---|---|---|---|---|---|
| NE | yes | verified | 40860 B, CRC B93E5A8A | pending → done 2/1; retry ARM accepted; disarmed, run count 0 | verified PC 0x08000900 / VTOR 0 after CAN request |
| SE | yes (`se-fwu-rollout-20261015/SE-before.bin`, sha256 2a62af73…) | verified (SWD HOTPLUG halt, 10028 B) | 40860 B, CRC 4F2A8CB1, host result 3, 6814 ACKs, metadata v2 + VTOR 0x08004000 + PC in app | done 2/1 (auto-CAL on first DARM; retry ARM accepted 0x52504D32, disarmed, runs 0; re-done after entry-test reset) | verified: host TX ok, ESC VTOR 0 → reset → VTOR 0x08004000 + app re-proven by CAL cycle (PC halt-read skipped, powered) |
| SW | yes (`sw-fwu-rollout-20261015/SW-before.bin`, sha256 3e69e092…) | verified (SWD HOTPLUG halt, 10036 B) | 40860 B, CRC DC10480E, host result 3, 6814 ACKs, metadata v2 + VTOR 0x08004000 + PC in app | done 2/1 (battery-ON undervoltage latch auto-acked by app on DARM; retry ARM accepted 0x52504D32, disarmed, runs 0; re-done after entry-test reset) | verified: host TX ok, ESC VTOR 0 → reset → VTOR 0x08004000 + app re-proven by CAL cycle (PC halt-read skipped, powered) |
| NW | yes (`nw-fwu-rollout-20261015/NW-before.bin`, sha256 0a555545…) | verified (SWD HOTPLUG halt, 10028 B) | 40852 B, CRC 837F75D2, host result 3, 6813 ACKs, metadata v2 + VTOR 0x08004000 + PC in app | done 2/1 (clean IDLE baseline, no latch; retry ARM accepted 0x52504D32, disarmed, runs 0; re-done after entry-test reset) | verified: host TX ok, ESC VTOR 0 → reset → VTOR 0x08004000 + app re-proven by CAL cycle (PC halt-read skipped, powered) |

NE backups and hash/metadata sidecars:
`/home/fh/sfurs_firmware/esc-backups/ne-fwu-rollout-20261014/`.
Current image/readback identity is in `NE-fixed.json`; earlier snapshots
are retained. Corrected images are under `/tmp/opencode/rollout-<wheel>-fixed/`.
NE boot image is under `/tmp/opencode/rollout-NE/boot/`.
Temporary artifacts must be rebuilt if that directory disappears.

## Fixes discovered during pilot

Post-rollout no-motion investigation: see
[`NE_SE_POST_ROLLOUT_RCA.md`](NE_SE_POST_ROLLOUT_RCA.md). NE CAL loss and
ARM-only recovery are verified; the TUI target-readiness handshake remains
open. SE CAN responsiveness is verified, not its current CAL or post-recovery spin.

1. CMake originally always embedded blink, ignoring generated motor C.
   Select the motor payload explicitly (absolute path recommended):

   ```sh
   python3 scripts/fwu_image.py --elf <wheel-offset.elf>
   cmake -S CM7 -B CM7/build/Debug \
     -DFWU_IMAGE_FILE="$PWD/CM7/Core/Src/fwu_test_image.c"
   cmake --build CM7/build/Debug
   ```

   Empty `FWU_IMAGE_FILE` selects the original blink payload. The cache
   remembers this choice. Verify the flashed payload size/CRC before starting
   FWU; selecting device 2/4/8 does NOT change an embedded NE app's CAN ID.

2. SDK `FOC_Init()` queued an unexecuted default speed ramp at boot, so
   `BootEntrySafe()` correctly rejected remote entry even while IDLE.
   Guarded diagnostic profiles now omit that implicit command; their explicit
   `TestDriveStart()` still sets the reference. Safety gates were not relaxed.
   `test_boot_reference.py` covers four wheels and stock/guarded profiles.

## Safety / remaining gates

- Battery OFF for flashing; ON only for CAL and no-motion entry checks.
- Quit TUI before probe access. Use HOTPLUG for ESC RAM reads/writes.
- CLI `-w32` verification can race a consumed request. Use `-nv` for the
  volatile request and independently read the result/state, never infer success.
- Auto-CAL is RAM-only, first ARM calibrates; retry ARM after completion.
  Never automatically RUN or persist offsets.
- Provisioning used `FWU_BLINK_BENCH_ONLY=1`. For the subsequent motor host
  build, configure CM7 with `-DNUCLEO_MOTOR_ENABLED=ON`. Verify the flashed
  `wheel_bench_motor_enabled` identity is 1 before TUI testing. Never run the
  TUI during FWU or share a probe with another agent.
- At operator request, the Nucleo 15 s / 1 s RUN caps are removed. Explicit
  STOP remains; the ESC timeout/count/fault/current guards are unchanged.
  TUI/probe disconnect is not a guaranteed STOP. Motion remains unverified
  by this provisioning procedure.
- NE remote entry was checked without flashing while powered. It was reset
  afterwards to restore normal boot; calibration must be renewed after reset.
- Per-wheel CAN ARM/CAL and physical motion/sign validation remain separate
  from installation. No all-wheel readiness claim until every row is verified.
