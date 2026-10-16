# Separate keyboard IK test image and telemetry

Worktree: `sfurs-omni-ik`, branch `fh/omni-ik`, based on `hw-session` 4fe8e8c.
This is a stand-test implementation, not yet physically validated. Geometry
and raw motor polarity remain provisional; see [OMNI_KINEMATICS.md](OMNI_KINEMATICS.md).

## What changes (and what does not)

**Installed and independently verified:** translation input cap 5 m/s,
all-four IK caps 900 RPM, RPM2 ESC bounds +/-900 RPM. All four ESCs and the
matching IK host completed programming with battery OFF. No higher-speed
motion or loaded performance is established by that verification. See
[HIGH_SPEED_CAPS.md](HIGH_SPEED_CAPS.md) for the coordinated deployment boundary.
The prior Nucleo telemetry ELF remains preserved. The TUI now defaults to the
installed `CM7/build/IK-900-reviewed` ELF. Preserve both images; use fresh output
directories for future source changes. Programming is complete, but calibration
and a separately agreed stand test must precede motion.

- Separate `scripts/keyboard_drive.py`; `keyboard_spin.py` stays unchanged.
- C IK runs on the Nucleo using wheel radius 32.75 mm, provisional tracks
  144/141 mm, wheelbase 95 mm and front/rear rolling angles 60/55 degrees.
- A small command adapter sends existing CAN ARM/SPEED/STOP frames. It uses
  raw signed IK targets directly, without another left-wheel sign flip.
- ABI2 adds all-four-wheel CAN speed/current polling and telemetry logging.
  The first-working ABI1 ELF is preserved in `CM7/build/IK`; the older ABI2 image
  remains in `CM7/build/IK-telemetry`. Current high-cap ABI2 is in
  `CM7/build/IK-900-reviewed`. See the programming/capture records below and
  [LOW_TORQUE_DIAGNOSTICS.md](LOW_TORQUE_DIAGNOSTICS.md).
- Initial IK/ABI2 telemetry did not change ESC code or gains. Later FR P2 tuning
  and the installed all-four high-speed caps are recorded separately. Following
  reported P2 jitter, FR's original RPM2 gains were restored and read back; all
  four now match. Smoothness/load validation of that configuration is pending.
- Dedicated IK build only: bench, manual CAL and FWU pumps are not run in this
  image. Normal builds retain their existing behavior. To use FWU or the old
  single-wheel TUI, restore the matching normal Nucleo image first.
- No new watchdog/lease framework. A dead PC, USB disconnect, halted Nucleo or
  CAN failure can prevent STOP delivery. Existing ESC guards still apply.

## Current interface limitation

The deployed guarded ESC's `TestDriveStart()` accepts speed only while IDLE
and armed. SPEED_WRITE does NOT update a running motor. This TUI therefore
starts one axis, stops on key-repeat timeout, and stops when changing direction.
Release movement keys and press again after stopping to choose another axis.
The adapter waits 650 ms after the final STOP acknowledgment before reporting
IDLE (existing ESC 400 ms STOP permanency plus 250 ms quiet). This is a timing
allowance, not independently measured standstill feedback.
There is no automatic restart, continuous steering, diagonals or simultaneous
keys in v1. A future running-target interface is a separate ESC change.

`p` sends ARM-only preparation for existing auto-CAL. It never RUNs. Bus ACK is
not proof of calibration or ARM acceptance: wait for calibration and existing
ESC state transitions before explicitly starting. Preparation may need a fresh
operator action after a cleared-fault acknowledgment; no blind RUN retries.

## Build (CM7 only, isolated directory)

From this worktree's root:

```sh
export PATH=/tmp/opencode/cm7-toolchain/root/usr/bin:$PATH
cmake -S CM7 -B CM7/build/IK-900-reviewed -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DNUCLEO_MOTOR_ENABLED=ON -DNUCLEO_IK_ENABLED=ON
cmake --build CM7/build/IK-900-reviewed
```

Current-source ELF: `CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf`.
The preserved older telemetry ELF in `CM7/build/IK-telemetry` must stay unchanged.
The reviewed high-cap ELF is now installed too; do not overwrite it for a future
changed-source build. Choose a fresh `-B` directory and explicit `--elf` instead.
The compiler path above is
machine-local and temporary. Do not share binary output directories with other
builders. Building is not flashing. Programming requires separate authorization;
no CM4, bootloader or option-byte writes are needed. Higher RPM commands also
require the matching ESC application update; see HIGH_SPEED_CAPS.md.

## Keyboard test (after authorized Nucleo programming)

Start with wheels off ground, clear rotating parts and a reachable battery
cutoff. Verify exact installed ELF identity and release other probe/CAN owners.
Never infer later power state from an old setup note.

```sh
python3 scripts/keyboard_drive.py --self-test
python3 scripts/keyboard_drive.py \
  --elf CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf
```

Requires pyOCD and pyelftools for live access. The TUI attaches without target
reset/halt, resolves symbols only from the selected ELF (no RAM fallbacks), and
compares file-backed flash load segments before allowing mailbox writes.

Controls:

| Key | Action |
|---|---|
| p | ARM-only preparation; no motor start |
| W / S | forward / reverse |
| A / D | left / right |
| Q / E | CCW / CW yaw (provisional physical sign) |
| Space | STOP broadcast |
| Escape / Ctrl-C | STOP, then quit |

Default translation remains 0.2 m/s; yaw is 0.5 rad/s. Current-source `--speed`
selects 0.05–5 m/s on the now-installed matching host;
`--yaw` selects 0.1–1 rad/s. No minimum wheel-RPM floor is added. Very small
targets may quantize in the ESC (SDK speed resolution is not changed here).
This extends selectable bounds, not the default start speed. Do not select the
high-cap ELF or higher references until its coordinated rollout is authorized
and verified; uniform 900 RPM saturation can reduce the requested body speed.

Terminal key-up is unavailable: release is inferred from 0.6 seconds without
key-repeat. Motion can persist through that interval; Space is the explicit
STOP. The display distinguishes requested twist, **commanded** RPM, **measured**
RPM and bus completion; no field proves physical stopping. Existing ESC timeout/count
limits may independently end motion; the TUI does not silently re-arm.

First observations: check each requested axis and STOP on the stand. Actual
ground translation/rotation must be checked separately before assuming the
provisional model agrees with the robot. Do not increase speed to compensate
for wrong direction or an ESC start rejection.

## All-four CAN telemetry (ABI2, no ESC update)

The scheduler alternates speed and current requests for selectors 1/2/4/8,
one frame every 20 ms (about 6.25 updates/s per wheel per reply kind when not
paused). ARM/SPEED bursts and STOP drains take priority; there is no catch-up
burst. Requests never overwrite the drive adapter's tracked TX mask.

The TUI reads coherent, generation-bracketed RAM rows at about 10 Hz:
command versus actual RPM, SDK state, Hall bits/change count, raw Iq/Id and
sampled L1 peak, plus current-fault/CAL/armed/active flags. Speed and current
have separate ages/counts. Missing data is `NO REPLY`, not zero; data older
than 500 ms is `STALE`. Polling pauses during STOP may naturally produce stale
rows. Converted Iq/Id are shown only with fresh RUN, CAL-valid, active,
profile-present and fault-free evidence. The scale is firmware-equivalent
544 SDK counts/A, not independently calibrated physical or battery current.

Both drive and monitor modes write an exclusive JSONL log to a fresh
`/tmp/opencode/ik-telemetry-UTC-PID.jsonl` by default. `--log PATH` selects a
new path (existing files are never overwritten). Logs include ELF identity,
timestamps, keys, mailbox submissions/outcomes, targets and raw/decoded samples.
No Vbus or Iq reference is invented from the CAN data: those fields are absent.

```sh
python3 scripts/keyboard_drive.py --monitor-only
```

Monitor-only ignores P, movement and Space and makes no mailbox writes, even
on exit. It still needs exclusive Nucleo probe ownership; do not start a second
Nucleo TUI alongside it. A separate ESC read-only logger may use the other
USB/probe once explicitly handed over. See [loaded-motion diagnostics](LOW_TORQUE_DIAGNOSTICS.md).

## Offline verification record

- `python3 tests/test_rover_kinematics.py`: 7 tests, including 100 twist
  reconstructions.
- `python3 tests/test_ik_drive.py`: 13 fake-CAN scenarios, including signed
  payloads, prepare-only, zero target, STOP override, unsupported running-update
  rejection, sequence replay suppression, fault handling and tick wraparound.
- Existing `tests/test_wheel_bench.py`: 34 scenarios pass unchanged.
- Existing `tests/test_fwu_main_control.py`: passes.
- `python3 scripts/keyboard_drive.py --self-test` and
  `python3 tests/test_keyboard_drive.py`: same 65 TUI/transport/telemetry tests pass.
  This includes host-time aging when firmware RAM/ticks freeze and recording
  failed cleanup STOP outcomes without adding retries.
  Integration review corrected WASD to the firmware's x-right/y-forward frame.
- All 15 ABI2 symbols resolve from the actual built ELF (including 4x52-byte
  telemetry rows). Flash-segment
  verification exercised against an offline image-backed target; pyOCD and
  pyelftools imports succeed on this machine. The ABI word is explicitly
  referenced in the IK loop so linker garbage collection cannot drop it.
- `tests/test_ik_telemetry.py`: nine actual-C decoder/scheduler scenarios pass
  at both -O0 and -O2.
- `tests/test_ik_telemetry_tui.py`: four actual-C decoder-to-TUI tests pass.
- `scripts/monitor_esc_power.py --self-test` and `tests/test_monitor_esc_power.py`:
  same 25 read-only NE power/current monitor tests pass, including archived ELF
  identity/layout and fake live-flash verification.
- CM7 Debug IK-telemetry build: FLASH 22,020 B, RAM 2,088 B.
- First-working IK image: FLASH 21,200 B, RAM 1,872 B; preserved byte-identical.
- Default motor-disabled/IK-disabled build: succeeds; IK symbols absent.
- No physical drive validation or programming performed by these checks.

## Nucleo programming record (2026-10-15)

Operator confirmed motor battery OFF and authorized Nucleo programming.
Detected Nucleo `002200364142500620353451` and front-right ESC
`066AFF525266555067075126`; only the Nucleo was accessed/programmed.

- CM7 image: `CM7/build/IK/sfurs_firmware_v2_CM7.elf`.
- ELF SHA256: `4dcb293e8f3171a4c3b452bc816530d83717ba629171cef7d810affe57060ca0`.
- CubeProgrammer reported download complete, download verified successfully,
  and software reset performed.
- Subsequent read-only pyOCD attach verified all flash PT_LOAD bytes against
  that ELF and the IK ABI. State IDLE, request/sequence/completed/result all 0,
  commanded RPM `[0,0,0,0]`, TX/ACK/fault all 0.
- No PREPARE, START or STOP requests issued. No ESC, CM4 or option-byte writes.
- Probe session released; last operator-reported motor battery state OFF.
  Powered stand testing has not yet been performed with this image.

## First working IK checkpoint (operator report, 2026-10-15)

Follow-up: the operator subsequently confirmed that the weakness affects all
translations and that wheels spin only freely in the air; on the ground they
buzz without driving, sometimes with mixed rotating/buzzing wheels. See
[loaded-motion diagnostics](LOW_TORQUE_DIAGNOSTICS.md). No current/voltage
diagnosis or limit change follows from this observation alone.

After programming the image above and running the separate keyboard TUI,
the operator reported that it works and the wheels spin. They also reported
insufficient wheel effort to drive the robot. This establishes the first
operator-observed working keyboard-to-IK-to-wheel-motion path, NOT successful
loaded driving, correct ground axes, validated PID tuning or measured torque.
The exact tested key/axis and whether the weak motion was observed under load
have not yet been recorded. No current, RPM or voltage capture accompanied it.

Checkpoint tag: `ik-first-working-20261015` on branch `fh/omni-ik`.
No PID, current-limit, ESC or Nucleo binary changes were made to mark it.

### Weak-motion diagnosis: next evidence, not a current-limit change

- Default forward 0.2 m/s corresponds to about 51/48/-48/-51 motor RPM.
  Default yaw 0.5 rad/s corresponds to only about 13/12/12/13 motor RPM.
  Motor/wheel gearing is still a provisional 1:1 assumption.
- A brief tap expires after 0.6 s without key-repeat, while the ESC ramp is
  0.5 s. Short taps may provide little time at the requested speed.
- RPM2 source bounds the current reference at 3 A and samples a 3 A L1
  stop threshold; these are diagnostic profile settings, not evidence of
  actual delivered torque, calibrated current or a hard instantaneous limit.
- Low speed tracking, current-reference demand, current/voltage limits,
  mechanical drag, battery sag, wheel directions and diagnostic guard trips
  remain possible causes. Do not increase limits from this observation alone.

Existing ESC CAN replies can supply, per selector 1/2/4/8:

| Request / reply | Existing payload |
|---|---|
| `0x100|selector` / `0x280|selector` | Signed RPM, motor state, Hall bits, Hall-change count |
| `0x300|selector` / `0x380|selector` | Signed Iq/Id counts, flags (faults, armed, active, calibration valid), sampled peak L1 counts |

The first-working ABI1 image drains replies without storing them and does not
poll telemetry; its displayed RPM remains a command. The new ABI2 image adds
Nucleo polling/RX and TUI display/logging using these existing packets, with
per-wheel reply age; ABI2 is programmed, but powered telemetry validation remains pending.
No ESC update is needed for those fields. Preserve raw current counts;
any A conversion must be labeled firmware-scaled, with validity conditions.
Iq reference, stop reason, detailed CAL result, and bus voltage are not in
these two CAN replies. Read them separately through the second ESC USB/debug
connection with matching image symbols, or add a separate telemetry protocol
change later. Current-reference versus measured-Iq is particularly useful for
distinguishing low torque demand from an actual current ceiling.

## ABI2 telemetry programming record (2026-10-15)

Operator confirmed battery OFF, authorized Nucleo programming, then closed
the existing keyboard TUI to release the probe. Only Nucleo
`002200364142500620353451` was detected at this step (ESC data cable not detected).

- Image: `CM7/build/IK-telemetry/sfurs_firmware_v2_CM7.elf`.
- ELF SHA256: `b889175178c4486a7f22a1ae4bd4cf6ea353436d046453bb2342a78181057e50`.
- CubeProgrammer: download complete, verified successfully, software reset performed.
- Read-only pyOCD attach independently compared all flash load bytes and ABI2.
  IDLE, zero target RPM/twist, request/sequence/completed all 0, drive TX/ACK/fault
  all 0. No PREPARE/START/STOP writes were issued by verification.
- All four speed/current seen counts were 0 with battery OFF. Readback CAN
  PSR `0x76b`, ECR `0x100080`: missing-ACK/error-passive evidence while ESCs
  are not powered; **not bus-off**. Telemetry polls are automatic read requests,
  separate from the zero drive-command counters. Fresh powered replies still
  need validation; no fault clearing/reset workaround was performed.
- ESCs, CM4 and option bytes unchanged. Probe session released. Last
  operator-reported motor battery state OFF; no powered telemetry run yet.
