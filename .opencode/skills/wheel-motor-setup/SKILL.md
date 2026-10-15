---
name: Wheel Motor Setup
description: Commission rover wheel motors (B-G431B-ESC1) via Nucleo H755 CAN sender and keyboard TUI — backups, ESC flashing, calibration, direct/CAN validation, and debugging no-motion faults.
---

# Wheel motor setup (living doc — update as the codebase moves)

## Run duration and motor-enabled build

The operator requested removal of the Nucleo's 15 s / 1 s RUN caps.
There is now no host duration cap; release, `s`, or `q` sends STOP.
Do not rely on the host to stop a motor if the TUI/probe disconnects.
ESC timeout/count stops and fault/current guards remain unchanged: removing
the host cap does NOT remove those independent ESC limits.

Explicitly configure CM7 with `-DNUCLEO_MOTOR_ENABLED=ON` for TUI motion;
the default OFF builds update-only firmware. `wheel_bench_motor_enabled`
reports the flashed build identity (1=motor-enabled), not target readiness.
Never use the TUI concurrently with FWU or another probe owner.

## Topology

- 4x B-G431B-ESC1 (STM32G431, 128 KiB flash), one per wheel, shared CAN bus.
- Nucleo H755ZI-Q CAN sender (`CM7/Core/Src/wheel_bench.c`), driven by
  `scripts/keyboard_spin.py` TUI (pyOCD attach to the Nucleo probe).
- ESC firmware source: `HW/B-G431B-ESC1/motor_config/` in this checkout
  (built by `build_cli.py`); the old `firmware-v2` clone is reference only.
- ESC application updates go over CAN, not SWD: see the `can-fwu-update`
  skill (protocol v2, 16 KiB boot + 110 KiB app + validity page).

| Wheel | Probe serial       | CAN ID | ARM frame | Speed frame |
|-------|--------------------|--------|-----------|-------------|
| NE    | 066AFF…075126      | 1      | 0x401     | 0x081       |
| SE    | 0667FF…073424      | 2      | 0x402     | 0x082       |
| SW    | 066FFF…073058      | 4      | 0x404     | 0x084       |
| NW    | 066AFF…023255      | 8      | 0x408     | 0x088       |

- ARM payload = `RPM2` magic bytes `32 4D 50 52` (`0x52504D32` LE).
- Speed payload: `i16` RPM LE + `u16` ramp ms (`F4 01` = 500 ms).
- STOP broadcast: `0x78F`, 4 zero bytes. TUI default speed 200 (SE/SW/NW
  cap 150, NE cap 300). Session default dir: reverse (-1).
- CAN scheme: `ID = COMMAND | SELECT`, commands `SPEED_WRITE 0x080`,
  `SPEED_READ 0x100`, `TEST_ARM 0x400`, `MOTOR_STOP 0x780`.
- Rover-frame convention (sender-side): TUI `+` = rover-forward on every
  wheel. SW/NW mounts are mirrored, so the sender negates their RPM sign;
  all ESC images stay identical. Direct STLINK pulses are shaft-frame
  (NOT mirrored): SW/NW direct visuals read opposite to TUI for the same
  sign. Validate direct runs shaft-vs-shaft against NE, not rover-frame.

## Golden rules (all learned the hard way)

1. **Battery OFF to flash an ESC, ON to CAL/run.** Never normal-connect;
   always `mode=HOTPLUG` for RAM access or the target resets.
2. **One probe holder at a time.** The TUI holds the Nucleo probe; quit (`q`)
   before any CLI access. A stale TUI looks like "probe occupied".
3. **Reflash after ANY rebuild** — sender and ESC: BSS layout shifts per
   build; a stale flash mimics a dead motor exactly.
4. **CAL is RAM: every STLINK replug wipes it.** CAL last (result 2/valid 1),
   then touch nothing but the other cable (battery stays ON). Never
   "verify" CAL by reconnecting — the reconnect is what wipes it.
   Battery OFF/ON wipes CAL on ALL wheels at once: after any flash or
   power cycle, re-CAL every non-auto-CAL wheel before the TUI. Only
   wheels with auto-CAL firmware recover alone — so "only the auto-CAL
   wheel moves" means wiped CAL on the rest, not a sender/CAN fault.
   Read the CAL words first; do not redebug the bus.
5. **TX ACK proves nothing about the target.** CAN ACK is physical-layer;
   any bus node pulls the ACK slot. No-motion + good ACKs = start gate
   (CAL/ARM) or wrong/absent frame, not a dead motor.
6. **Resolve ESC RAM addresses via `nm` on the flashed ELF, never hardcode.**
   Key words (names stable, addresses not): `test_calibration_result`,
   `test_calibration_valid`, `test_direct_last_request`, `test_direct_result`,
   `test_drive_armed/active/run_count/stop_reason/measured_rpm/iq`.
   Results: 0=none 1=pending 2=done 3=rejected 4=failed.
7. **Reload the Nucleo sender after any sender change** before blaming an ESC
   (see RCA below). Keep ESC `esc-backups/` dumps + JSON sidecars before
   overwriting any unknown flash (full 128 KiB via CLI `-u ADDR SIZE FILE`).
8. **No per-wheel special cases in shared paths — audit all layers per wheel.**
   Proving wheel X works proves nothing about wheel Y: shared code with
   per-wheel gates fails per-wheel, silently (good ACKs, no motion). When
   enabling a wheel, adding a command, or touching anything addressed per
   wheel, grep every layer for existing wheel IDs and confirm each handles
   the new one: sender TX (`wheel_bench.c` ARM/RUN/STOP construction),
   sender RX/mirror gates, TUI wheel map + display gates
   (`keyboard_spin.py`), host-test coverage (`tests/test_wheel_bench.py`
   must exercise all 4 wheels for every sender behavior), ESC CAN ID +
   filter (`motor_controller_can.h`, `MOTOR_<WHEEL>_CAN_ID`). After ANY
   sender change affecting addressing, re-prove motion on ALL commissioned
   wheels, not just the one under test. A gate with no else-alarm
   (e.g. `if wheel==1||wheel==2` silently skipping) is a bug — prefer
   unconditional shared paths, pre-gated once by a validator
   (`selection_bit`).

## Per-wheel bring-up playbook

1. Cable to ESC probe. Dump flash to `esc-backups/` + JSON sidecar if unknown.
2. Battery OFF → flash half-amp (1.5 A torque) diagnostic → ON → CAL.
3. DARM→DRUN pulse, observe shaft sign (+torque → backward on NE/SE,
   forward on SW — left-side wheels are mounted mirrored, so the same
   shaft direction rolls the rover the opposite way).
4. Build reverse-speed image with correct gain sign (`MOTOR_<WHEEL>`),
   flash, CAL, direct -100 stability check (count-stop, no runaway).
5. Cable to Nucleo (battery ON). TUI wheel key, hold SPACE: motion = CAN OK.
6. Log the run in `REVERSE_SPEED_VALIDATION.md` (firmware-v2, motor-testing).
7. After any sender/TUI change: re-prove motion on ALL commissioned wheels
   (rule 8) — never just the wheel under test.

## Reading RPM

- STLINK: `test_drive_measured_rpm` (`nm` the flashed ELF for the address).
- CAN: `SPEED_READ | ID` (4-byte request) → reply `0x280|ID`, 8 bytes:
  RPM `i16` LE, state, Hall bits, Hall-change count. The sender/TUI mirror
  reports RPM/state and reply age; a host RUNNING state alone is not proof
  that the ESC accepted ARM or started.

## Known RCAs

- **NE/SE no motion after all-wheel FWU rollout:** NE flash was correct and
  CAN polling worked, but RAM CAL was `0/0`. CAN ARM-only restored `2/1`;
  a second ARM was accepted, then STOP disarmed (no RUN sent). SE answered
  fresh IDLE telemetry, but its current CAL was not independently confirmed.
  All four now have auto-CAL; the remaining TUI gap is `Bench.start()` sending
  RUN after host ARMED without target CAL/ARM confirmation. Prior RAM-CAL-loss
  RCAs below recur under this asynchronous contract. Permanent handshake fix
  remains open. Evidence, history and separate manual-CAL byte-order finding:
  `docs/NE_SE_POST_ROLLOUT_RCA.md`.
- **SE CAN silence (2026-10-14):** Nucleo sender predated SE CAN-ARM support —
  armed locally, speed frames hit an unarmed ESC, declined silently with
  healthy ACKs. Fix was reflashing the Nucleo; SE ELF disassembly had already
  proven its ID2 constants correct. Lesson: suspect the sender first.
- **SW CAN silence (2026-10-15):** SAME pattern, one wheel later. Sender
  transmitted CAN ARM only for wheels 1/2 (`if selected==1||selected==2`,
  else "arm locally"); SW speed frames (`0x084`) hit an unarmed ESC while
  the TUI showed `RUNNING` on good `tx/ack`. Direct DARM→DRUN worked, which
  isolated it to the CAN ARM path. Fix: unconditional CAN ARM for all
  selections (pre-gated by `selection_bit`), host tests extended to assert
  ARM+RUN frames for all 4 wheels, Nucleo reflashed. Lesson: the SE fix
  covered the instance, not the pattern — see rule 8.
- **NW near-miss (2026-10-15):** same gate would have dropped NW (`0x408`)
  identically; caught by auditing all layers for wheel-ID gates instead of
  waiting for hardware symptoms. NW still needs full bring-up (unknown
  flash), but the sender path is proven by host tests for wheel 8.
- **Ghost no-motion after cable moves:** CAL wiped by replug (all-zero
  gate words). Recalibrate; do not re-tune.
- **Only the auto-CAL wheel moves after flashing/power cycles (2026-10-15):**
  SW/NW bring-up cycled the battery, rebooting all ESCs and wiping RAM CAL
  everywhere. NE self-recovered (NE-only auto-CAL prototype); SE/SW/NW
  silently declined CAN ARM with healthy ACKs. SE read IDLE + faults 0 +
  CAL 0/0, fixed by manual CAL1. Lesson: after any battery cycle, re-CAL
  all non-auto-CAL wheels first. Durable fix (open): extend auto-CAL to
  all wheels instead of NE-only.
- **First-after-power-on CAL transient:** retry works; persistent reject →
  power off for minutes (rail decay), not seconds.
- **Halls:** alive = word at Hall net address changes while turning by hand.
