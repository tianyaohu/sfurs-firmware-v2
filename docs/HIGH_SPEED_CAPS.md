# 5 m/s input / all-four 900 RPM candidate

Status: **all four ESCs and matching Nucleo CM7 IK image installed and independently
verified; higher-speed motion and loaded tuning NOT yet validated**. User explicitly requested the cap changes,
handed this worktree's ownership to this agent, and then authorized installation
with a fresh **battery OFF / TUI closed** confirmation. No CAL or motion commands
are authorized by this programming handoff.

Lane: `/home/fh/sfurs_firmware/Project1/sfurs-omni-ik`, branch `fh/omni-ik`,
HEAD `572d4d8` plus existing dirty/untracked work. Preserve first-working tag
`ik-first-working-20261015` and both installed/past ELF artifacts. Integration
direction remains feature -> `hw-session`, then `develop` only by approval;
no commit, merge or push performed here.

## Changes

| Layer | Previous | Current source |
|---|---|---|
| `scripts/keyboard_drive.py --speed` | 0.05–0.3 m/s | 0.05–5 m/s |
| `ik_drive` vx/vy mailbox bounds | +/-300 mm/s per axis | +/-5000 mm/s per axis |
| `rover_ik_provisional` limits FR/BR/BL/FL | 300/150/150/150 RPM | 900/900/900/900 RPM |
| RPM2 ESC accepted reference | +/-300 RPM | +/-900 RPM |
| RPM2 measured-speed stop | at or beyond +/-300 | **beyond +/-900** |

RPM2 command endpoints are inclusive: exact +/-900 must not immediately trip
the speed guard. Its guard now uses strict comparisons; an observed speed
beyond either endpoint still stops. Other diagnostic profiles retain their
existing at-limit semantics. No overspeed margin was added: transient overshoot
above 900 can still stop a 900 RPM request. Host tests exercise +/-901 in the
guard shim; the real SDK's next representable speed beyond 900 is 906 RPM.

Unchanged: default TUI 0.2 m/s, default yaw 0.5 rad/s and yaw limit +/-1 rad/s;
3 A reference/output/sample-stop bounds; 2 s Hall-stall guard; 480 net-Hall-edge
count stop; 30 s backup timeout; 500 ms speed ramp; CAL/ARM gates; PI numerators,
divisors and current-loop parameters. The separate old single-wheel bench/TUI
caps remain unchanged; the IK path does not pass through them.

**Gain state at the cap rollout was mixed by prior experiment:** FR/NE had P2
proportional divisor 512 (exponent 9); the other three retain 1024/10. All retain
speed coefficients -18600/-72 and integral divisor 16384. The cap-only request
did not restore FR's original gains, propagate P2 or establish smooth tuning.
The operator reported P2 jitter; ground driving with this mixed experiment is
not validated. **Subsequent separately approved FR restoration is complete:**
all four now use original RPM2 gains, 1024/10 and 16384/14. See
[FR_LOW_SPEED_P2.md](FR_LOW_SPEED_P2.md) for its readback/backup evidence.

## Speed interpretation

Geometry and direct drive remain provisional. Rounded forward raw targets:

| Request | FR | BR | BL | FL | Common scale |
|---|---:|---:|---:|---:|---:|
| 1 m/s | 253 | 239 | -239 | -253 | 1 |
| 3 m/s | 758 | 717 | -717 | -758 | 1 |
| 5 m/s | 900 | 851 | -851 | -900 | ~0.713 |

Thus 5 m/s forward is accepted as a request but scales to about **3.56 m/s** in
this geometric model, not proven ground velocity. The common scale preserves
all wheel-speed ratios, rather than clipping individual wheels. Pure sideways
5 m/s remains below 900 RPM; directional ceilings differ with the omni geometry.

480 net Hall edges are 10 shaft revolutions under the configured 8 pole pairs:
at a sustained 900 RPM the count guard can end motion after about **0.67 s**
(plus startup/ramp timing). This guard remains enabled; it is not evidence of a
new fault. Do not silently remove it to extend a run. At high references the
unchanged current limit can be reached sooner; these caps do not guarantee
acceleration, torque, battery-voltage margin or stable regulation.

## Preserved tuning work and isolation

At handoff, runtime `GAIN_SET`, `--rpm2-tune`, `scripts/auto_tune_esc.py` and its
tests already existed. They are preserved, not run against hardware. The
cap-only ESC builds **omit `--rpm2-tune` / `ESC_TUNE_GAIN_SET`**. Verify no
`TestDriveGainSet` or gain-diagnostic objects are linked into these ESC payloads.
The exclusive IK host does not run the normal wheel-bench/CAL/FWU pumps, so it
does not run the inherited tuner. Host-only tuner tests do not constitute a
review or authorization of unattended gain sweeps.

The source handoff interrupted earlier builds. Final artifacts are rebuilt in
fresh owned directories with a source snapshot/identity manifest; earlier
`CM7/build/IK-900` and `/tmp/opencode/caps-5ms-900rpm-20261015` are preliminary,
not the deployment set. Final build/evidence paths are recorded below.

## Checks and deployment boundary

Tests cover all four signed ARM/SPEED selectors, +/-5000 mm/s acceptance and
5001 rejection, 1/3/5 m/s wire targets, uniform 900 RPM saturation, unchanged yaw
bounds, STOP priority, fresh-ARM ESC range rejection, inclusive +/-900 references,
wrong-wheel rejection, overspeed, and retained CAL/current/stall/count/timeout
guards. Math tests exercise common scaling and sign/gearing once; physical
direction remains a separate check.

Before deployment: fresh battery-OFF confirmation, exclusive probe/CAN handoff,
per-wheel selection/backup, coordinated all-four ESC application rollout,
independent app/metadata/VTOR verification and matching Nucleo CM7 flash. Do not
drive higher RPM with only part of the set installed. CAL resets must be handled
after power-on; the known target-ready handshake and manual CAN-CAL byte-order
defects remain open. No bootloader/CM4/option-byte writes are needed.

See [CAN FWU runbook](../.opencode/skills/can-fwu-update/SKILL.md) and
[ESC tuning runbook](../.opencode/skills/esc-foc-tuning/SKILL.md).

## Final offline build evidence (2026-10-15)

- CM7: `CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf`, Debug,
  `NUCLEO_MOTOR_ENABLED=ON`, `NUCLEO_IK_ENABLED=ON`, FLASH 22028 B / RAM 2088 B.
  ELF SHA256 `83cecd66e9a3ead263b40607ca909caa4a74ea9847274a4af9d99e60c419ee77`.
- ESCs and full build manifest:
  `/tmp/opencode/caps-5ms-900rpm-owned-20261015/build-manifest.json`.
  This includes exact commands, ELF/app SHA256, size, CRC, expected validity
  metadata, vector validation and linked PID default checks for each wheel.
  Every `verified_app_readback` remains **false**.

| Wheel | App bytes | App CRC32 | Speed Kp divisor | Runtime gain-set linked |
|---|---:|---|---:|---|
| FR/NE | 40868 | `3725CEE8` | 512 (existing P2) | no |
| BR/SE | 40868 | `3BA3B7BB` | 1024 | no |
| BL/SW | 40868 | `8B8AE06A` | 1024 | no |
| FL/NW | 40860 | `7D9D87D5` | 1024 | no |

All ESC builds use `--reverse-speed-diagnostic --app-offset`; only NE also uses
`--ne-low-speed-p2`. App slot 110 KiB; normal RAM 8792 B / mailbox 16 B each.
All four speed gains -18600/-72, integral divisor 16384, output +/-1632 and
integral bounds +/-26738688 verified in their linked PID initializer bytes.
The reconstructed physical LOAD application matches independent objcopy output;
all initial stack/reset vectors pass the bootloader's layout checks.

The artifact directory also retains `source.patch`, untracked build-source
copies, `source-sha256.json` and per-build logs. Selected source hashes were
unchanged before/after the builds. No runtime-tuner functions/objects are linked
into these ESC apps; no normal wheel-bench producer is linked into the IK host.
The prior installed CM7 telemetry ELF still hashes to
`b889175178c4486a7f22a1ae4bd4cf6ea353436d046453bb2342a78181057e50`.

18 host-only test suites passed again after the monitor fix; full final output:
`/tmp/opencode/caps-900-owned-tests-final-20261015.log`. These include the inherited
tuner's offline tests only, not endorsement of its physical sweep procedure.
The final monitor suite has 30 passing tests, including exact new FR artifact
loading, rejection of old live flash under the new profile, and profile-size
restoration. `git diff --check` also passes.

FR logging gains a separate explicit profile `ne-900-p2`, with exact ELF/app
hashes, updated 40868-byte size and CRC/metadata. Its candidate manifest is
`/tmp/opencode/caps-5ms-900rpm-owned-20261015/NE/candidate.json`.
The expected full-flash SHA is calculated from the archived before-image and
new app/metadata; it is not a readback claim. Full LIVE app/metadata/VTOR matching
is still mandatory. The previously installed old P2 image must fail this profile;
the new FR application has now passed live verification as recorded below.
An actual-artifact test exposed and fixed Python's import-bound default image
size in the loader: it now explicitly passes the selected profile's app size.
Baseline/P2 identities remain unchanged; no reset/write/motor cleanup was added.

## CAN installation progress — FR/NE (2026-10-15)

Only Nucleo `002200364142500620353451` and FR/NE probe
`066AFF525266555067075126` were connected. Both were listed before writes;
all TUIs/loggers/tuners were closed, and this agent held exclusive probe/CAN
ownership. Battery OFF confirmed by operator; FR preflight SDK Vbus ~4.40 V.

- Fresh full pre-update backup matched the previous installed P2 image SHA256
  `e204a38aba09e4cbe22c79c5274a7ca1bb23e7c6905d22509b1279be0a7e8fd2`.
  Backups/sidecars: `/home/fh/sfurs_firmware/esc-backups/caps900-rollout-20261015T215925Z/NE-before.bin`
  and `NE-before.json`.
- Built separate update-only Nucleo images for all four prepared payloads,
  `CM7/build/FWU-900-{NE,SE,SW,NW}/sfurs_firmware_v2_CM7.elf`, Debug,
  motor/IK producers OFF. Exact commands/hashes:
  `/tmp/opencode/caps-5ms-900rpm-owned-20261015/updater-manifest.json`.
- Installed **NE updater only** on Nucleo; CLI download, verification and reset
  passed. Independent flash/payload match checked before request. An initial
  symbol-resolution check found two unused globals were linker-discarded; it
  aborted before probe connection or a request. Corrected to require only linked
  controls and verify the motor producer is absent, with no address fallbacks.
- One explicit `can_fwu_dev=1` session; normal FR `SYSRESETREQ` exposed the
  existing three-second bootloader window without clearing battery-off faults
  or bypassing the application's remote-entry guard.
- Transfer reached DONE/result 3, DATA sequence 6811 (6812 frames), VERIFY/JUMP,
  6816 ACK frames. Log: `/tmp/opencode/caps-5ms-900rpm-owned-20261015/NE/transfer.jsonl`.
- Independent full 128 KiB readback matches expected SHA256
  `42c91675757048cc07570e14932b5cfdebcc19aa9c5832087f59970c8ac7a1b5`.
  App size 40868, CRC `3725CEE8`, metadata matches and VTOR `0x08004000`.
  Bootloader and unused app flash are byte-identical to the fresh backup.
  Live P2 divisor/exponent 512/9, gains -18600/-72 and output +/-1632 unchanged.
- Post-update full backup/verified sidecar in the same backup directory:
  `NE-after.bin`, `NE-after.json`. Prepared artifact manifests intentionally
  remain readback=false; the verified sidecar is the installation record.
- FR final sample: RPM 0, active/armed 0/0, CAL 0/0 (reset), undervoltage
  faults 4/4 with battery OFF. This proves installation, not motion or tuning.
- Nucleo handoff verified DONE/result 3, `can_fwu_request=0`, motor-enabled 0,
  bench IDLE and CAL request 0. No pending FWU retry loop. It remains on the
  **update-only NE image**, not the final 5 m/s IK image, until all four updates
  are independently verified. Both probe sessions released.

FR's physical handoff was to back-right BR/SE, recorded below with a fresh
battery-OFF/TUI-closed confirmation and independent serial verification.
Do not enable motion with this partial rollout. No CM4/bootloader/option-byte
programming or commit/merge/push performed.

## CAN installation progress — BR/SE (2026-10-15)

Operator reported BR connected, battery OFF and TUI closed. Independently listed
Nucleo `002200364142500620353451` and BR/SE `0667FF525266555067073424` before
writes; attached without resetting the ESC for backup/RAM access. Fresh Vbus
~4.45 SDK V, stopped/unarmed; old app/metadata/VTOR matched its archived image.

- Full fresh backup: same rollout directory, `SE-before.bin` / `.json`, SHA256
  `77d3488452affca914a32470ee02dad6a8792e8c5068c6fbeb31c242288fac55`.
- Prior NE updater independently verified DONE/request 0 with motor producer
  disabled before replacing it. Installed the prepared **SE update-only**
  Nucleo ELF, verified download/reset, and independently matched the flashed
  updater and embedded payload before selecting device 2.
- One explicit CAN session through the normal target-reset bootloader window;
  no fault clear, CAL, ARM, RUN or automatic post-ENTER restart. DONE/result 3,
  final DATA sequence 6811 (6812 frames), 6816 ACK frames. Transfer log:
  `/tmp/opencode/caps-5ms-900rpm-owned-20261015/SE/transfer.jsonl`.
- Independent full readback SHA256
  `e11a7289d4538b3868f021a118b8f8d953a5ca87e2d114297108e30ce33b57fa`;
  app 40868 B, CRC `3BA3B7BB`, expected metadata, VTOR `0x08004000`.
  Bootloader/unused app flash unchanged; live gains -18600/-72, P divisor 1024
  (exponent 10), integral divisor 16384 and output +/-1632 unchanged.
- Full readback/verified sidecar: same rollout backup directory,
  `SE-after.bin` / `SE-after.json`. Final BR RPM 0, inactive/unarmed, CAL 0/0,
  battery-off undervoltage 4/4. No motion/performance claim.
- Nucleo handoff checked DONE/result 3, request 0, bench IDLE, motor-enabled 0,
  CAL request 0; remains on **SE update-only host**. Both probes released.

The operator requested interactive multiple-choice/checkbox handoffs. Updated
`can-fwu-update` and `wheel-motor-setup` skills accordingly. Next recommended
wheel is **back-left BL/SW**, serial `066FFF525266555067073058`: leave Nucleo
USB connected, move only ESC USB, keep battery OFF and confirm the setup in the
selection UI before programming. A Pause option must always remain available.

## CAN installation progress — BL/SW (2026-10-15)

Operator chose BL/SW in the interactive handoff and checked battery OFF, closed
TUI/tools, BL USB connected with Nucleo retained, and exclusive hardware release.
Independently listed exact SW serial `066FFF525266555067073058` before access.

- Same rollout backup directory: `SW-before.bin` / `.json`, fresh full SHA256
  `952180f2a5f28e4311e6189996146bbba0bc1f6aae3c3fa0dbca21ec9a9df03a`, matching
  its previously verified image. Stopped/unarmed; initial Vbus ~4.43 SDK V.
- Prior SE updater independently verified DONE/no pending requests; installed
  and verified the prepared SW update-only host, then selected device **4**.
  One reset-window CAN session reached DONE/result 3, final DATA sequence 6811,
  6816 ACK frames. No CAL/ARM/RUN or automatic transfer restart.
- Full independent post-update SHA256
  `485b0a2df0c6008c8e13706091b918db213d7f63bc804b2bf4eecb6ecdf79b21`;
  app 40868 B, CRC `8B8AE06A`, expected metadata and VTOR `0x08004000`.
  Bootloader and unused flash unchanged; stock speed gains/divisors and +/-1632
  current output limits verified in live PID state.
- Verified sidecar/readback: `SW-after.json` / `SW-after.bin` in the same backup
  directory. Final RPM 0, inactive/unarmed, CAL 0/0, faults 4/4 with battery OFF.
- Nucleo remains SW update-only, DONE/result 3, request 0, motor disabled,
  bench IDLE, CAL request 0. Both probes released. FL/NW is the next pending
  target; keep battery OFF and use the selection/checklist handoff.

Logs: `/tmp/opencode/caps-5ms-900rpm-owned-20261015/SW/transfer.jsonl`,
`nucleo-programmer.log` and `nucleo-handoff.json`. An external one-wheel helper
`install_remaining.py` in that artifact root performs the previously used
backup/identity/update/readback gates for SW/NW only; it never selects another
wheel automatically and rejects already-completed targets or unknown images.

## CAN installation progress — FL/NW and final CM7 (2026-10-15)

Operator selected FL/NW and checked all four safety conditions after moving the
ESC USB. Independently listed serial `066AFF515266555067023255`; Nucleo USB
remained on `002200364142500620353451`. Battery stayed OFF throughout.

- Same backup directory, `NW-before.bin` / `.json`, SHA256
  `05b03e82928d1519bfe6fdc6dca273c8932df0df8cefddd5e51f212ef3a37fe6`, matching
  the archived installed image. Initial Vbus ~4.46 SDK V, stopped/unarmed.
- Verified the previous SW updater's flash and DONE/no-pending state, installed
  the prepared NW update-only image and selected device **8**. One reset-window
  session reached DONE/result 3, final DATA sequence 6809 (6810 frames),
  6814 ACK frames. No CAL/ARM/RUN, fault clear or automatic transfer restart.
- Independent full NW readback SHA256
  `95f50457244526336bf4e204614b971219a7b003f667e34f8b79b61cb1421482`;
  app 40860 B, CRC `7D9D87D5`, correct metadata and VTOR `0x08004000`.
  Bootloader/unused app flash unchanged; stock gains and +/-1632 output limits
  verified live. `NW-after.bin` / `NW-after.json` hold the verified evidence.
- NW final state RPM 0, inactive/unarmed, CAL 0/0, battery-off undervoltage 4/4.
  Both probes released; NW updater verified DONE/request 0/motor-disabled before
  replacing it with the final host image.

After confirming all four verified sidecars/readback hashes against the reviewed
payload set, installed the exact CM7 high-cap ELF `83cecd66...419ee77` listed above.
CLI download, verification and reset each passed. Independent full physical
FLASH-segment comparison and ABI `0x494B0002` check passed. Nucleo IDLE, request /
sequence / completed all 0, commanded RPM and twist all 0, host fault 0.
No old FWU retry pump runs in this exclusive IK image. CM4, bootloaders and
option bytes were not programmed.

Evidence:

- `/tmp/opencode/caps-5ms-900rpm-owned-20261015/nucleo-final-programmer.log`
- `/tmp/opencode/caps-5ms-900rpm-owned-20261015/nucleo-final-verified.json`
- `/tmp/opencode/caps-5ms-900rpm-owned-20261015/rollout-session.json`
- `/home/fh/sfurs_firmware/esc-backups/caps900-rollout-20261015T215925Z/rollout-complete.json`

The TUI default now selects the **verified installed high-cap ELF**, not the
preserved old telemetry image. User-facing speed default stays 0.2 m/s; selecting
a larger speed is explicit. The exact-image read-only FR logger remains on
profile `ne-900-p2` when that separate probe is reconnected and handed off.

Final handoff: all four applications plus matching CM7 verified; battery last
explicitly confirmed OFF; both probes released; no queued CAL/ARM/RUN/FWU
requests. Last connected ESC USB is FL/NW, Nucleo USB retained. Current/stall/
count protections and prior mixed P2/stock gain configuration are unchanged.
No higher-speed motion, smoothing, loaded tuning or all-four ground behavior
has been tested. Obtain fresh all-wheels-clear/power confirmations before an
explicitly agreed stand test; known handshake/manual-CAL defects remain open.

The requested selection/checklist interaction is now recorded in the CAN FWU
and wheel commissioning skills and was used for the SW and NW handoffs. No
commit, merge, push or deletion of preserved artifacts performed.

Post-install checks: TUI self-test **68 passed**, monitor self-test **30 passed**,
`git diff --check` clean. A final read-only connection using the TUI's new
default ELF independently matched live CM7 flash and confirmed IDLE with zero
targets and no pending request; the probe was released afterward.

Subsequent FR-only original-gain restoration (same day) retained all these caps
and restored the identical reviewed CM7 image afterward. Current FR readback is
`/home/fh/sfurs_firmware/esc-backups/fr-stock900-restore-20261015T231437Z/NE-after.json`,
logger profile `ne-900-stock`; historical rollout FR/P2 records above remain
preserved. The mutable rollout-session record points to the newer FR evidence.
All four now share original speed gains. Higher-speed/loaded validation remains
pending; no CAL or motion was sent during either programming operation.
