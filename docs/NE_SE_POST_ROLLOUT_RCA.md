# NE/SE no motion after auto-CAL / FWU rollout

## Status

NE's missing calibration and recovery are hardware-confirmed. The TUI's
missing target-readiness handshake is source-confirmed. SE answers CAN and
received an ARM-only preparation cycle, but its current CAL state and the
cause of its reported no-motion were not independently confirmed. Physical
spin after recovery has not been verified. The permanent TUI fix is **open**.

## Symptom and trigger

After all four ESCs were provisioned with the offset auto-CAL application and
CAN bootloader, and a motor-enabled Nucleo was installed, the operator reported
SW/NW spinning but NE/SE not spinning. Battery cycles and ESC resets occurred
during provisioning. Calibration offsets are deliberately RAM-only, so a
reset/power loss invalidates the earlier successful CAL verification.
The exact reset that cleared NE's calibration was not observed.

## Root cause / mechanism

The first ARM is not necessarily an accepted arm:

1. `TestDriveArm()` may acknowledge a cleared fault asynchronously, or start
   RAM offset measurement when CAL is missing, and return false without arming.
2. `wheel_bench_tick()` reports host ARMED after queuing CAN ARM. Bus TX ACK
   does not prove the addressed ESC accepted ARM or completed calibration.
3. `scripts/keyboard_spin.py:Bench.start()` waits only for host ARMED and then
   immediately sends RUN. It has no target CAL-ready/armed confirmation.
4. `TestDriveStart()` correctly rejects RUN if the ESC is unarmed or CAL is
   incomplete. The TUI can nevertheless display host RUNNING.

NE was demonstrably uncalibrated at diagnosis, and a CAN ARM-only cycle
restored calibration. This explains its start-readiness failure without
requiring a flash, gain, or CAN wiring change. The exact earlier operator
ARM/RUN timing was not captured; the sequencing defect is established from
the code, not a recording of those earlier attempts.

## Measured evidence

### NE

- Full flash matched the provisioned `NE-fixed.json` SHA256. App bytes matched
  the 40860 B build, CRC `B93E5A8A`; validity metadata and VTOR were correct.
- Initially: IDLE, current/past faults 0, CAL result/valid `0/0`, disarmed,
  active 0, ESC run count 0, stop reason 0. No recorded start or guard trip
  since the observed boot.
- Initial RX snapshots/counters were zero. This was **not** proof of a broken
  CAN tap: selecting NE for read-only polling produced new `0x281` replies
  to `0x101`, with fresh IDLE/0 RPM telemetry.
- CAN ARM only (no RUN) changed CAL to `2/1`, with ESC run count still 0.
- A second CAN ARM was independently observed in ESC RAM as
  `test_drive_armed=0x52504D32`. STOP then disarmed it. No reset or flash
  was needed for recovery.

### SE

- Independent Nucleo observations captured new `0x282` replies to `0x102`,
  reporting IDLE, 0 RPM, Hall state 3; counters increased during sampling.
- This proves the SE app and its CAN addressing respond. SPEED_READ does
  **not** expose CAL validity, fault bits, ARM acceptance, or ESC run count.
- An ARM-only preparation cycle was sent, followed by STOP; SE remained
  fresh IDLE/0 RPM. This is not independent proof of CAL completion.
- Host run count 6 and TX/ACK 30/30 were cross-wheel aggregate counters;
  they cannot establish six SE starts or six SE rejections.

No RUN commands were sent during these diagnostic/recovery checks.
Both host checks were left stopped/IDLE and probe sessions released.

## Has this happened before?

Yes, related incidents are already documented in
`.opencode/skills/wheel-motor-setup/SKILL.md`:

- **Ghost no-motion after cable moves:** reset/replug cleared RAM CAL.
- **Only the auto-CAL wheel moves after flashing/power cycles:** shared
  battery cycling wiped calibration on the other wheels. At that earlier
  incident only NE had auto-CAL; **all four have it now**.
- **First-after-power-on CAL transient:** a cleared undervoltage fault may
  consume an ARM for asynchronous SDK fault acknowledgment before calibration.

Those are distinct from the earlier NE/SE-only sender ARM gate, which was
fixed. The recurring underlying condition is loss of volatile calibration;
the remaining integration defect is treating sender transmission as ESC
readiness. Installing auto-CAL on every ESC did not add a TUI handshake.

## Recovery versus permanent fix

Current workaround: ARM without RUN, wait for calibration, then issue a fresh
ARM before any operator-requested RUN. Do not retune gains or reflash merely
because CAN TX was acknowledged without motion. Never persist offsets or
automatically start a wheel after CAL.

Required permanent change (not implemented by this log):

- Obtain fresh, correctly selected target CAL-valid/armed/fault telemetry for
  **all four** selectors, with stale/wrong-wheel reply rejection.
- Separate preparation from motion: CAL/ARM completion must never itself
  trigger RUN. Report CAL pending, target rejection and actual motion separately
  from host TX states; preserve explicit operator intent and STOP/cancel.
- Regression-test cold boot, power cycle, cleared-fault acknowledgment,
  pending/failed CAL, missing/wrong-wheel/stale replies, STOP during preparation,
  and repeat operation on all four wheels. Re-prove physical motion separately.

## Separate finding: manual CAN CAL byte order

Source audit found another open defect, not proven causal for these TUI
attempts: `Common/Inc/can_cal.h:can_cal_pack()` sends ASCII `43 41 4C 31`
(MSB-first), while the deployed ESC parses with little-endian
`CAN_U32_FROM_PAYLOAD()` and compares against `0x43414C31` (which would require
`31 4C 41 43`). Auto-CAL on ARM does not use this payload and worked on NE.
Add an end-to-end sender-to-real-ESC-parser test when reconciling this contract;
sender-only packing tests currently do not catch the mismatch.
