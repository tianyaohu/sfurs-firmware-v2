# FR-only low-speed proportional comparison (P2)

**Latest installation (2026-10-16):** FR now carries the separately approved
Ki4 stand candidate; other three remain stock. See
[RPM2_KI4_REVIEW.md](RPM2_KI4_REVIEW.md) for verified identity and pending stand
validation. The original-gain restoration below is historical. Do not use the
stock monitor profile for the Ki4 image; no ground driving with mixed gains.

Status: **historical P2 stand experiment showed rotation but much worse jitter;
original FR gains now restored and independently verified (2026-10-15)**.
Programming was explicitly authorized with battery OFF and sent no CAL/ARM/RUN;
subsequent operator-driven stand tests used battery ON, all wheels raised.

**Current installation:** FR carries 900 RPM caps with original RPM2 gains,
matching the other wheels; restoration evidence is below. No new motion/load
validation is claimed. See [HIGH_SPEED_CAPS.md](HIGH_SPEED_CAPS.md). The original
300 RPM/P2 hashes and capture commands below are historical, not the current
flashed identity. Current FR logger profile is `ne-900-stock`, verified sidecar
`/home/fh/sfurs_firmware/esc-backups/fr-stock900-restore-20261015T231437Z/NE-after.json`.
Do not bypass an old profile's rejection of the new image.

## Why this experiment

The synchronized baseline capture in [LOW_TORQUE_DIAGNOSTICS.md](LOW_TORQUE_DIAGNOSTICS.md)
shows FR +/-29 RPM starts requesting only ~0.16–0.19 SDK A, delivering a
similar current, remaining stationary and stopping with explicit stall reason 9
at ~2.05 s. The live +/-1632-count (~3 A) limit was not reached. Higher +/-51 RPM
references sometimes start successfully. This motivates investigating low-speed
controller demand; it is not proof that gains are the only cause.

## Exactly one effective parameter change

Opt-in `ESC_NE_LOW_SPEED_P2` / builder `--ne-low-speed-p2`:

| Setting | Baseline | Candidate |
|---|---:|---:|
| Kp numerator | -18600 | -18600 |
| Kp divisor / exponent | 1024 / 10 | **512 / 9** |
| Ki numerator / divisor | -72 / 16384 | unchanged |
| Reference/output limit | +/-1632 counts (~3 A) | unchanged |
| Sampled L1 stop threshold | 1632 counts (~3 A) | unchanged |
| No-Hall stall interval | 2000 ms | unchanged |
| RPM bounds / speed guard | +/-300 / 300 | unchanged |
| Timeout / net-Hall-edge stop | 30000 ms / 480 edges | unchanged |
| CAL/ARM, current-loop gains, boot/FWU | baseline | unchanged |

Only P doubles, not total PI output. Do not use numerator -37200: that exceeds
the signed 16-bit field and can reverse its sign on narrowing. Both numeric
divider and shift exponent are changed together for the SDK's two PI variants.
Ki, integral limits, gain polarity, current limits and all stop guards remain.

At a stationary +/-29 RPM command the SDK reference truncates to +/-4 units
(+/-24 RPM): proportional demand magnitude is approximately 0.134 -> 0.267 A.
At +/-51 RPM (SDK +/-8 units), about 0.267 -> 0.534 A. Integral action is unchanged;
integer rounding and actual ramp/state timing matter. These are model estimates,
not guaranteed torque, stability or physical current.

The feature is rejected unless NE-only, guarded reverse-speed RPM2 and offset
app are selected. Conflicting profiles or other wheel selections are rejected.
The default and all other wheel configurations retain their previous parameters.

## Reproduction and artifacts

```sh
export PATH=/tmp/opencode/cm7-toolchain/root/usr/bin:$PATH
python3 HW/B-G431B-ESC1/motor_config/build_cli.py \
  --motor NE --reverse-speed-diagnostic --app-offset --ne-low-speed-p2 \
  --output /tmp/opencode/fr-tune-p2-20261015/app
```

Prepared artifacts (not source commits):

- ELF: `/tmp/opencode/fr-tune-p2-20261015/app/motor_NE_reverse_speed_diagnostic_offset_ne_low_speed_p2.elf`
- Raw application: same directory, `app.bin`.
- Manifest: same directory, `candidate.json`; `verified_app_readback` is **false**.
- Exact firmware diff and SHA256: `source.patch`, referenced by the manifest.
- App size 40860 B; CRC32 `43ACF5E2`.
- App SHA256 `9beab39c5b6828c4a5f401c2173424b412865a81eacc38e8457017808a49f050`.
- ELF SHA256 `ef107830180d33feb6ccf7ee9d1ba29b9443205624652c5397ca22405399d2a0`.

Freshly rebuilt baseline AND a default build after adding the opt-in reproduce
the installed/audited baseline app byte-for-byte (SHA256
`4625cf66572fb97c5137239b8b13dc5fdd7f6ced334853affe13c3a85bab74dc`).
Candidate differs in exactly two app bytes:

- `0x0800DD59`: 04 -> 02 (Kp divisor high byte).
- `0x0800DD5C`: 0A -> 09 (Kp shift exponent).

Machine instructions, object layout/addresses, gain numerators and output limits
are unchanged. FLASH 40860 B / normal RAM 8792 B / mailbox 16 B for both builds.

## Offline checks and review

- `python3 tests/test_fr_speed_tune.py`: four test groups pass. They compile
  actual SDK PI code with a host type shim, test both signs, both SDK
  shift/division variants, nominal ramps, saturation, and PID initialization.
  All four default wheel configs and invalid opt-in combinations are exercised.
  This is NOT an actual STC ramp/startup or motor-plant simulation.
- `python3 scripts/monitor_esc_power.py --self-test`: 28 tests pass, including
  both frozen image identities and rejecting baseline flash under the P2 profile.
- Independent review verified the two-byte diff, unchanged limits, registration
  hashes and unchanged default image. No blocking offline defects found.

## Matching telemetry and installation boundaries

The installed P2 capture uses explicit identity:

```sh
python3 scripts/monitor_esc_power.py \
  --profile ne-low-speed-p2 \
  --manifest /home/fh/sfurs_firmware/esc-backups/FR-low-speed-p2-after-can-20261015T124744Z.json \
  --seconds 60 --period 0.05
```

The baseline monitor remains default. No identity checks are bypassed: the
candidate ELF hash, reconstructed app hash, full LIVE app bytes, updated validity
metadata and VTOR must all match before diagnostic sampling. A prepared manifest
is not a programming claim. Changing source/build identity requires a new audit,
not substituting arbitrary hashes to make a mismatched image pass.

Do not blindly program this offset ELF: its first LOAD starts below the app
vector. Use the established app-update procedure with correct validity metadata,
preserving the CAN bootloader. Updating metadata CRC is mandatory. Prefer the
existing CAN-FWU application path; its Nucleo updater is separate from the IK
image. Do not mass erase or rewrite bootloader/option bytes. Preserve an independent
pre-update backup and the working baseline for recovery.

No flashing or movement is authorized by this document. After approval, first
compare short FR starts off the ground at the same +/-29 and +/-51 RPM targets,
one operator-requested attempt at a time, with matching power/IK logs. Confirm
physical rest between directions. Review breakaway, RPM oscillation/overshoot,
Iq reference/feedback and stop reason. Do not extend a buzzing run, raise the
current ceiling or disable the stall guard. Ground comparison comes only after
repeatable stand behavior. Other three wheels remain on their existing ESC apps;
an IK key may still command them, so keep all wheels clear.

## Installation evidence and handoff (2026-10-15)

Procedure: [CAN FWU runbook](../.opencode/skills/can-fwu-update/SKILL.md).
Exclusive owner: `fh/omni-ik`, worktree
`/home/fh/sfurs_firmware/Project1/sfurs-omni-ik`, commit `572d4d8` plus the
archived reviewed diff. No merge/push or integration-direction change.

- Probes independently listed: Nucleo `002200364142500620353451`, FR ESC
  `066AFF525266555067075126`. TUIs/loggers closed; no other active agents.
- Fresh full pre-update backup outside Git:
  `/home/fh/sfurs_firmware/esc-backups/FR-before-low-speed-p2-20261015T124215Z.bin`
  plus JSON sidecar. SHA256
  `4b684c516b3081d6035de7d8aa565bea4896b745f5f1a0e530bf25048c1bddd6`.
- Temporary CM7 updater: `CM7/build/FWU-FR-P2/sfurs_firmware_v2_CM7.elf`,
  Debug, `NUCLEO_MOTOR_ENABLED=OFF`, `NUCLEO_IK_ENABLED=OFF`, payload generated
  from the audited raw `app.bin` into `/tmp/opencode/fr-tune-p2-20261015/fwu_image.c`.
  Flash verified before RAM access; target `can_fwu_dev=1` only.
- Battery-off undervoltage blocks application remote boot entry. Did not clear
  faults or bypass the application guard: normal FR system reset exposed its
  existing three-second bootloader ENTER window.
- The initial helper used nonexistent pyOCD enum `SW_SYSRESETREQ` after queuing
  ENTER, so it failed before FR reset. Inspected pending host state: ENTER only,
  zero acknowledged DATA. Corrected to the installed API's
  `Target.ResetType.SYSRESETREQ` and resumed that same request, without queuing
  another session. No application erase/write occurred before the corrected reset.
- CAN transfer reached host DONE/result 3, final DATA sequence 6809 (6810 DATA
  frames), then VERIFY/JUMP. Log:
  `/tmp/opencode/fr-tune-p2-20261015/transfer-resume.jsonl`.
- Independent full 128 KiB readback matches expected candidate SHA256
  `e204a38aba09e4cbe22c79c5274a7ca1bb23e7c6905d22509b1279be0a7e8fd2`.
  Full app/metadata match; VTOR `0x08004000`. Bootloader bytes and unused app
  region unchanged. Live PID gains `-18600/-72`, P divisor/exponent `512/9`,
  output limits `+/-1632`. This proves installation, not successful startup.
- Post-update full backup and verified sidecar:
  `/home/fh/sfurs_firmware/esc-backups/FR-low-speed-p2-after-can-20261015T124744Z.bin`
  and `.json`. Also `/tmp/opencode/fr-tune-p2-20261015/installed.json`.
  Original prepared `candidate.json` intentionally retains readback=false.
- Restored byte-identical CM7 ABI2 telemetry ELF
  `CM7/build/IK-telemetry/sfurs_firmware_v2_CM7.elf`, SHA256
  `b889175178c4486a7f22a1ae4bd4cf6ea353436d046453bb2342a78181057e50`:
  CLI download/verify/reset passed; independent live flash/ABI check passed.
  Nucleo IDLE, zero targets, request/sequence/completed all 0, host fault 0.
  `/tmp/opencode/fr-tune-p2-20261015/nucleo-restored.json` records the state.
- FWU protocol, validity, main timing and main control regressions all passed
  before the transfer. Other ESCs were not selected for update; no all-four
  readback or motion claim is made. CM4 and option bytes were not programmed.

Handoff: both probe sessions released; updater image removed, no FWU retry loop
or queued drive request remains. Last operator-confirmed battery state OFF.
FR sampled RPM 0, active/armed `0/0`, CAL `0/0`, current/past undervoltage
faults `4/4` (expected with battery OFF; not a readiness claim). Confirm battery
ON and all wheels clear before separately authorized calibration/stand motion.
Known target-ready handshake and manual CAN-CAL byte-order issues remain open.

## First P2 stand capture: startup progress, jitter remains

Operator confirmed battery ON and all four wheels securely off the ground.
The read-only logger completed 3600 samples and released its FR probe:

- FR: `/tmp/opencode/fr-p2-first-stand-20261015T1250.jsonl`.
- Matching TUI: `/tmp/opencode/ik-telemetry-20261015T125215.993354Z-3667297.jsonl`.
- Per-run analysis: `/tmp/opencode/fr-p2-first-stand-analysis-20261015.json`.

Matched mean RAM-block midpoint timestamps (host monotonic ns -> seconds) to
committed TUI commands; exclude the first 0.7 s for post-ramp speed summaries.
FR SDK speed is quantized and blocks are non-atomic: isolated sign reversals
are telemetry observations, not independently proven shaft reversals.

| TUI sequence | FR target RPM | First sampled nonzero RPM after request | Post-ramp RUN RPM median | Zero-RPM fraction of post-ramp RUN samples |
|---|---:|---:|---:|---:|
| 2 | +51 | none; FR did not start | n/a | n/a |
| 4 | +51 | 0.62 s | +48 | 3% |
| 6 | -29 | 1.83 s | -18 | 27% |
| 8 | +51 | 0.57 s | +48 | 2% |
| 10 | -51 | 0.63 s | -42 | 0% |
| 12 | -29 | 2.06 s | -30 | 30% |
| 14 | +29 | 1.73 s | 0 | 35% |
| 16 | -51 | 0.20 s | -42 | 4% |
| 18 | -51 | 0.43 s | -42 | 2% |
| 20 | +51 | 0.72 s | +42 | 19% |

The first P acknowledged cleared undervoltage only. Sequence 2's ARM completed
CAL `2/1` but FR remained IDLE with no Hall progress while other wheels ran:
the known target-ready handshake gap still reproduces. Do not call that a
torque/gain failure or claim the handshake is fixed.

All three +/-29 attempts now have sampled RPM and CAN Hall progress, unlike
the earlier two baseline attempts. Pre-breakaway requested Iq after 0.7 s was
~0.30 SDK A median, ~0.31–0.32 A maximum, versus baseline ~0.16–0.19 A.
This is limited evidence of improved breakaway under these stand conditions,
not smooth/reliable speed regulation or an all-wheel fix. FR has no sampled
stall reason 9 in this capture. Stop reason 2 persists during RUN; it is a
retained prior stop reason and must not be treated as a new stop event.

The operator explicitly reports that turning **became very jittery**. FR's
last +51 attempt includes ~1.1 s of consecutive sampled zero RPM, with other
speed samples from -36 to +90 RPM. Requested/measured Iq peaks ~1.058/1.065
SDK A, median absolute tracking error ~0.024 A across valid RUN samples. No
sampled reference approached the unchanged ~3 A PID limit. RUN averaged Vbus
~11.679–11.682 V; this does not rule out fast dips or predict loaded behavior.

Unmodified BR remains at zero RPM/no Hall progress on the three sideways
attempts and transitions RUN -> STOP -> IDLE before operator STOP. This is
consistent with its stall guard but is not a direct BR stop-reason read.
Do not generalize FR startup progress to all four wheels.

Final FR sample: IDLE, RPM 0, CAL `2/1`, unarmed/inactive, faults `0/0`.
Final fresh CAN rows show all four IDLE/RPM 0; TUI cleanup sequence 23 completed,
host fault 0, zero targets. No further firmware change or ground test performed.

Operator requested a higher-speed comparison because tournament operation is
not normally at these creep speeds. A separate three-minute read-only capture
was launched at `/tmp/opencode/fr-p2-speed03-20261015.jsonl`; the name records
the initial proposed 0.3 m/s test, not proof of the actual command. Match the
new TUI log before assigning a speed. Tournament-representative target still
needs operator input; no gain/current/stall-limit change is authorized here.

That follow-up logger completed 3600 samples and released FR: every sample was
IDLE/RPM 0/inactive. It contains **no higher-speed comparison**, despite its
filename. Final CAL `2/1`, armed/active `0/0`, faults `0/0`; battery last confirmed
ON. The operator clarified tournament translation is normally 1–3 m/s. Under
the provisional geometry, unsaturated forward demands are ~239–253 RPM at
1 m/s and ~717–758 RPM at 3 m/s. Current TUI AND Nucleo input caps are 0.3 m/s;
three provisional IK wheel caps are 150 RPM, while RPM2 ESC bounds are +/-300.
Raising only the CLI bound will not enable the proposed 1 m/s comparison.
Proposed next step (awaiting approval): prepare a 1 m/s host configuration and
compare with original FR gains; no further firmware changes/programming made.

## Original-gain restoration selected (900 RPM caps retained)

After the completed all-four 900 RPM cap rollout, the operator selected
**Restore FR gains** through the multiple-choice UI. Original RPM2 here means
`-18600/1024` proportional and `-72/16384` integral, not the generated SDK's
different default speed coefficients. Other wheels already use these RPM2 gains.

Restoration candidate built offline at `/tmp/opencode/fr-900-stock-20261015/app/`:

- `motor_NE_reverse_speed_diagnostic_offset.elf`, ELF SHA256
  `b131f86e2d3154735823ba227011f777ffcd4ceab166d6c0f95fea7e18effaa7`.
- App 40868 B, CRC `CDB76180`, SHA256
  `0bb917ba9807af0aede7226b14a08d20c4c51b9161d0d150e3d821b5c234edd8`.
- Compared with installed 900/P2 app: exactly two bytes differ,
  `0x0800DD61` 02 -> 04 and `0x0800DD64` 09 -> 0A. These restore only P's
  divisor/exponent 512/9 -> 1024/10. Code, Ki, current loops/limits, +/-900 RPM,
  stall/count/timeout/ramp/CAL behavior are unchanged. Runtime GAIN_SET absent.
- Candidate manifest `candidate.json` has readback=false; expected full-flash
  SHA256 `df1053892346557720d8c143e2876f9672429c9c1b0c5dcda03a8919fbb42e6a`
  is calculated from the last verified 900/P2 flash plus new app/metadata.
- Matching update-only host prepared in `CM7/build/FWU-FR-900-stock`, with
  exact commands/hash in `/tmp/opencode/fr-900-stock-20261015/updater-manifest.json`.
  Final Nucleo IK image must be restored afterward; other ESCs need no update.
- Read-only logger explicit profile `ne-900-stock` pins this candidate's exact
  ELF/app/metadata/size. Offline live-gate test rejects current P2 flash under
  that profile; old identities remain unchanged. Monitor 31 tests, current PI
  comparison 4 groups and all-four RPM2 guard tests pass.

At preparation time **nothing has been flashed**: pending fresh FR USB,
battery-OFF, TUI/tools-closed and exclusive hardware handoff through the checklist.
Restoring original gains is a baseline change, not proof that jitter is resolved
at higher speed or under load. No auto-tune sweep, CAL or motion authorized by
this selection.

### Restoration completed (2026-10-15)

The operator subsequently confirmed FR USB connected and all checklist conditions
met, including battery OFF, TUI/tools closed and exclusive probe/CAN release.
Independently listed FR serial `066AFF525266555067075126` and Nucleo
`002200364142500620353451` before access. Fresh full FR backup matched the
installed 900/P2 SHA256 `42c91675...c7a1b5`, Vbus ~4.40 SDK V, stopped/unarmed.

- Backup/readback/verified sidecars:
  `/home/fh/sfurs_firmware/esc-backups/fr-stock900-restore-20261015T231437Z/`,
  `NE-before.bin/.json`, `NE-after.bin/.json`, `restoration-complete.json`.
- One CAN-FWU session, selector 1 through the existing reset bootloader window,
  reached DONE/result 3, final DATA sequence 6811, 6816 ACK frames. Log:
  `/tmp/opencode/fr-900-stock-20261015/transfer.jsonl`.
- Independent full 128 KiB readback matches expected SHA256
  `df1053892346557720d8c143e2876f9672429c9c1b0c5dcda03a8919fbb42e6a`.
  App 40868 B, CRC `CDB76180`, expected metadata and VTOR `0x08004000`.
  Bootloader and unused app flash unchanged. Live speed PID confirms original
  -18600/-72, proportional divisor/exponent 1024/10, integral divisor/exponent
  16384/14, output +/-1632. No runtime gain-setting enabled.
- Reinstalled the same reviewed high-cap Nucleo IK ELF SHA256
  `83cecd66e9a3ead263b40607ca909caa4a74ea9847274a4af9d99e60c419ee77`.
  CLI download/verify/reset and independent full flash/ABI2 verification passed;
  IDLE, zero targets, request/sequence/completed all 0, host fault 0.
  `/tmp/opencode/fr-900-stock-20261015/nucleo-restored.json` records that check.
- Other ESCs were not programmed. All four now share original RPM2 speed gains,
  with the 900 RPM caps and all current/stall/count protections retained.

Handoff: battery last confirmed OFF, both probes released, no pending FWU/CAL/
ARM/RUN requests. FR CAL 0/0 and undervoltage 4/4 are expected after reset with
battery OFF. No motion, loaded tune or smoothing improvement has been tested
since restoration. Next is a separately confirmed, logged raised-wheel baseline.

### Powered readiness and calibration-only preparation

Operator confirmed all wheels securely raised, battery ON, FR/Nucleo USB
connected, TUI/tools closed, exclusive hardware access and reachable cutoff.
Initial read-only check at `2026-10-15T23:27:44Z`: FR Vbus ~11.51 SDK V, restored
image/gains matched, CAL 0/0, current fault 0, past undervoltage 4, state 11.
All four fresh CAN rows showed state 11, zero RPM, unarmed/inactive, CAL unset
and no active fault bits. No commands sent during this check.

After a separate **calibrate only** approval, issued two bounded ARM-only
PREPARE transactions: first acknowledged cleared faults; fresh per-wheel replies
confirmed all IDLE. Second triggered RAM auto-CAL; fresh replies confirmed all
four IDLE/CAL-valid without RUN. A final STOP/disarm completed, followed by fresh
CAN confirmation that all four retained CAL, RPM 0 and armed/active false.
No START/SPEED, gain, flash or reset commands were issued by this procedure.

Direct read-only FR check afterward: state IDLE, RPM 0, CAL result/valid **2/1**,
armed/active 0/0, current/past faults **0/0**, Vbus ~**11.52 SDK V**.
Nucleo STOP sequence completed with zero targets/host fault. Both probes released;
battery remains ON. CAL is volatile: retain battery/USB state and do not reset.

Evidence in `/tmp/opencode/fr-900-stock-20261015/`:
`stand-readiness-20261015T232745Z.json`,
`cal-only-20261015T233325Z.jsonl`,
`cal-only-20261015T233325Z-summary.json`.
Other wheels' CAL validity is fresh CAN evidence; result 2/valid 1 was directly
read only on FR. This is supervised preparation around the known asynchronous
handshake, **not a fix to the TUI readiness gap**. At this stage there was no
post-restoration rotation or loaded validation; subsequent stand results follow.

### Original-gain stand capture: translation rotates, default yaw does not

After an operator-confirmed power change, separately approved re-preparation
found all four already CAL-valid/IDLE with no active faults. No additional ARM
or CAL was needed; STOP/disarm and fresh all-four telemetry were verified.
FR direct CAL 2/1, faults 0/0, Vbus ~11.46 SDK V. Evidence:
`/tmp/opencode/fr-900-stock-20261015/reprepare-20261015T234424Z.json` and `.jsonl`.

The operator selected the TUI-guided 1 m/s stand test. Saved TUI records contain
multiple WASD and Q/E attempts, not only the proposed single forward attempt.
Operator report: **"WASD was working good"; Q/E had no wheels turning**.
No ground/load validation or quantitative smoothness acceptance is inferred.

Evidence in `/tmp/opencode/fr-900-stock-20261015/`:

- `tui-1ms-resume.jsonl`: exact reviewed ABI2 host identity, key/mailbox events
  and four-wheel CAN telemetry (session starts 23:46:37Z).
- `stand-1ms-resume.jsonl`: separate exact-image FR monitor at nominal 20 Hz,
  started 23:46:13Z, requested 300 s. Read-only, attach/no reset.
- `stand-1ms-analysis.json`: offline per-attempt summaries from these logs.
  Windows end at the next mailbox submission, not necessarily physical stop;
  RUN samples include transients, and CAN/FR snapshots are not simultaneous.

| Input | Logged targets, FR/BR/BL/FL RPM | Evidence |
|---|---|---|
| W, vy=+1000 mm/s | +253,+239,-239,-253 | All four fresh nonzero RPM and Hall changes |
| S, vy=-1000 mm/s | -253,-239,+239,+253 | All four fresh nonzero RPM and Hall changes |
| D, vx=+1000 mm/s | -146,+167,+167,-146 | All four fresh nonzero RPM and Hall changes |
| A, vx=-1000 mm/s | +146,-167,-167,+146 | All four fresh nonzero RPM and Hall changes |
| Q, omega=+500 mrad/s | +13,+12,+12,+13 | All four entered RUN, RPM remained 0; only isolated Hall changes |
| E, omega=-500 mrad/s | -13,-12,-12,-13 | All four entered RUN, RPM remained 0, no Hall changes in attempt |

FR translation attempts sampled peak |Iq reference| ~0.37–1.05 SDK A;
median |Iq reference - Iq| ~0.022–0.033 SDK A during valid RUN samples.
FR median |Id| ~0.022–0.032 SDK A. Count stop reason 8 appeared on the longer
translation attempts. These low-rate observations do not establish current-loop
bandwidth/stability, exact Hall alignment, or loaded torque sufficiency.

FR yaw sampled peak |Iq reference| only ~0.073–0.095 SDK A. The longer Q/E
attempts reached explicit stall stop reason **9**, not the 3 A current guard.
FR active faults remained 0; sampled Vbus during attempts was ~11.454–11.456 V,
with no observed collapse at this sample rate. The keys and IK commands are
therefore reaching the targets; weak very-low-speed breakaway is supported by
both current and Hall evidence. `--speed 1` changes translation only: default
`--yaw 0.5` still requests these ~12–13 RPM values. Even the current TUI/CM7 yaw
ceiling of 1 rad/s yields only ~25/24 RPM under provisional geometry. No yaw
ceiling, PI gain, RPM floor, or protection was changed by this test/analysis.

**STOP evidence caveat:** TUI exit at 23:47:31Z recorded `cleanup_stop_failed`,
reason `transport_or_interrupt`, empty error text, sequence 35. Its last sample
was stale and showed a pending STOP. This alone did not prove physical stop or
a firmware fault. After separate operator confirmation of visibly stopped
wheels, TUI closed/exclusive release, battery ON and unchanged cables, an
independent exact-image Nucleo readback at 23:49:10Z found STOP 35 completed,
host IDLE/fault 0/zero targets. Fresh advancing CAN replies confirmed **all four
IDLE, RPM 0, inactive/disarmed, CAL-valid and no active faults**. No extra STOP,
ARM, CAL or RUN was sent during that check. Evidence:
`post-tui-stop-20261015T234910Z.json` and `.jsonl`. Nucleo probe released;
FR probe remains owned only by the bounded read-only logger until it exits.
The reason for interrupted cleanup confirmation remains unresolved; this is
not a fix to the known target-ready handshake or manual CAN-CAL byte-order bugs.
