# RPM2 Ki4 review — 2026-10-16

Status: **FR Ki4 installed and independently verified; stand motion not yet
validated**. Other three ESCs remain stock. The initial review was offline;
subsequent separately approved installation is recorded below. No CAL or RUN
during installation. No merge/push. **No ground driving with these mixed gains.**

Lane: `fh/omni-ik`, worktree `/home/fh/sfurs_firmware/Project1/sfurs-omni-ik`,
HEAD `572d4d8` plus preserved working changes. Operator reconfirmed exclusive
ownership and integration into `hw-session` first; no integration is authorized
here. Owned changes: ESC profile/builder, candidate/guard tests, tuning docs.

## Finding and bounded adjustment

[Loaded evidence](LOADED_DRIVE_BASELINE.md): a robot reported under 5 kg on smooth
hard flooring only twitched at 0.5 m/s; at 1 m/s for 1 s it crept approximately
5 mm (operator visual estimate). All four entered RUN. Peak sampled measured
|Iq| at 1 m/s was 1.51–1.66 SDK A, with poor speed tracking. Nucleo-only ground
capture did not include Iq reference, Vbus or explicit stop reason. Consequently
insufficient demand is a plausible hypothesis, **not an isolated root cause**;
power, current tracking, commutation and mechanics remain possible contributors.

The former Kp-doubling experiment produced operator-reported severe stand jitter;
retain original Kp. Change integral gain alone via opt-in
`ESC_RPM2_KI4` / builder `--rpm2-ki4`:

| Parameter | Stock | Candidate |
|---|---:|---:|
| Speed Kp numerator / divisor / exponent | -18600 / 1024 / 10 | unchanged |
| Speed Ki numerator / divisor / exponent | -72 / 16384 / 14 | **-288 / 16384 / 14** |
| PI integral time, Kp / (1000 Ki) | 4.133 s | **1.033 s** |
| Speed PI output / integral accumulator bounds | +/-1632 / +/-26738688 | unchanged |
| Iq/Id loop gains and rates | original | unchanged |
| Sampled L1 stop / hardware protection | original | unchanged |
| Ramp / no-Hall stall / timeout / count stop | 500 ms / 2 s / 30 s / 480 edges | unchanged |
| RPM bounds and measured-speed guard | +/-900 / 900 | unchanged |
| CAL, arming, addressing, bootloader and FWU behavior | original | unchanged |

The Ki numerator remains signed int16-safe and preserves gain polarity. All four
selectors use the same candidate. CLI and preprocessor reject missing guarded
RPM2/offset-app prerequisites, zero/multiple wheel selections, conflicting
profiles, P2 and runtime gain-set combinations. Ordinary builds remain stock;
runtime GAIN_SET and automatic sweeps are not enabled.

## Why 4x, not the runbook's illustrative 8–12x tiers

For a stationary wheel, ideal linear 500 ms ramp, 1 kHz speed PI, initial
integral zero, and target magnitude U in 0.1 Hz SDK units, the unclipped demand
at t >= 0.5 s is approximately:

`|Iq_ref(t)| = U * (18600/1024 + (72*multiplier/16384)*(1000*t - 250)) / 544`.

Positive 239/253 RPM targets truncate to 39/42 units; negative integer division
truncates toward zero for the corresponding target magnitudes. At 1 s:

| Ki multiplier | Ideal stationary demand at U=39–42 | Ideal time to 3 A at U=39–42 |
|---|---:|---:|
| Stock 1x | 1.54–1.66 A | 4.96–5.64 s |
| 2x | 1.77–1.91 A | 2.60–2.94 s |
| **4x** | **2.25–2.42 A** | **1.43–1.60 s** |
| 8x | 3.00 A, clipped | 0.84–0.92 s |
| 12x | 3.00 A, clipped | 0.64–0.70 s |

Stock predictions resemble the measured current range but do not prove current
reference tracking under load. 4x is a cautious intermediate demand experiment
at the actual tested speed; the larger examples would drive a stationary wheel
to the existing ceiling within the one-second comparison. **4x is not claimed
sufficient at 0.5 m/s, validated stable, or optimal.** The original runbook's
200–450 ms integral-time window is a proposed heuristic, not a measured
requirement that justifies skipping a smaller step.

Actual STC implementation uses 501 ramp steps, fixed-point increments and signed
shift/division quantization; startup state delays and motor motion also change
these estimates. `tests/test_rpm2_ki4.py` links the real SDK PI, with a synthetic
integer ramp, not the actual STC/state machine or a motor/load simulation.

## Saturation and release review

SDK `PI_Controller` clamps the accumulator and total output, then adds
`wDischarge` to the stored accumulator. This is the existing implementation,
not newly designed anti-windup. Do not assume immediate post-breakaway recovery:
at zero speed error a nonzero integral term can remain. Faster Ki may worsen
overshoot or Hall-feedback hunting even with Kp unchanged. No SDK changes here.

Host tests exercise both signs and SDK shift/division variants, output and
accumulator bounds, saturation, error reversal, initialization and explicit
integral clearing. The application calls `STC_Clear`, which uses
`PID_SetIntegralTerm(...,0)` in speed mode. These code checks are not proof of
physical stopping, closed-loop stability or motor thermal margin.

## Offline builds and checks

Artifacts: `/tmp/opencode/rpm2-ki4-review-20261016/`:

- `build_review.py`: reproducible eight-build and byte-audit procedure.
- `build-manifest.json`: exact commands, toolchain, ELF/app hashes, CRC/metadata,
  linked PID initializers and archive comparisons.
- `{NE,SE,SW,NW}/{stock,ki4}/`: ELF, reconstructed `app.bin`, log and manifest.
- `source.patch`, `source-sha256.json`, `source-snapshot/`: reviewed source
  provenance; compile-source hashes unchanged across builds. New source files
  are snapshotted explicitly because ordinary `git diff` omits untracked files.

GCC 10.3.1; each build uses `--reverse-speed-diagnostic --app-offset`, candidate
additionally `--rpm2-ki4`. No updater or installed Nucleo rebuild/change.

| Wheel / CAN selector | Candidate app bytes | Candidate app CRC32 | Changed flash addresses |
|---|---:|---|---|
| FR/NE / 1 | 40868 | `75A5B016` | `0800DD4A`, `0800DD4B` |
| BR/SE / 2 | 40868 | `83B1662D` | `0800DD4A`, `0800DD4B` |
| BL/SW / 4 | 40868 | `339831FC` | `0800DD4A`, `0800DD4B` |
| FL/NW / 8 | 40860 | `C58F5643` | `0800DD42`, `0800DD43` |

Every stock app matches its archived verified installed application byte-for-byte.
Every candidate differs in exactly two bytes: Ki initializer `B8 FF` (-72) to
`E0 FE` (-288). All remaining PID bytes and the entire remainder of each app,
including executable code, are identical. This establishes unchanged addressing
code; it does not establish actual all-four hardware response. ELF hashes can
differ for stock because debug/source provenance changed while app bytes did not.
Initial stack/reset vectors and complete physical LOAD coverage checked. Runtime
gain-set diagnostic symbols are absent. Prepared manifests remain readback=false
and installed=false; they are **not** monitor installation sidecars.

Checks passed:

- `python3 tests/test_rpm2_ki4.py`: 6 groups (all-four parameters, actual SDK PI,
  saturation/recovery, invalid compiler configurations and CLI rejection).
- `python3 tests/test_fr_speed_tune.py`: 4 existing groups.
- `python3 HW/B-G431B-ESC1/motor_config/test_reverse_speed_guards.py`: all four
  selectors, stock/Ki4 and auto-CAL enabled/disabled; existing ARM/CAL/fault,
  addressing, range/ramp, current, overspeed, stall, count and timeout guards.
- `python3 scripts/monitor_esc_power.py --self-test`: 31 existing tests. This does
  **not** register Ki4 as an audited live-monitor profile.
- `git diff --check`.

## Next hardware phase — separate handoff required

Steps 1–3 and the readiness/CAL portion of step 4 below were subsequently
completed with separate authorization. Stand motion remains pending.

1. Battery OFF before moving robot; raise all four wheels and connect separate
   Nucleo and FR/NE ESC USB. Confirm exclusive access, closed TUI/tools and exact
   probe serials freshly. Current battery/probe state is historical, not assumed.
2. Before installation, add/test the exact Ki4 FR monitor identity and provenance
   sidecar, and prepare a separate motor-disabled Nucleo CAN updater. No bypass
   of existing stock-monitor hash checks. Back up current flash before writes.
3. With explicit programming approval and battery OFF, update **FR only** over
   CAN; independently verify full readback, metadata/VTOR and unchanged bootloader.
   Restore/verify the matching IK host before any later control operation.
4. Battery ON with fresh confirmation; CAL/readiness-only preparation first.
   Separately approve short raised-wheel comparisons in each direction around
   50 and 250 RPM, logging direct FR Iq reference/measured Iq/Id, Vbus, Hall/RPM,
   faults and explicit stop reason alongside Nucleo telemetry. No automatic
   sweep, repeat, or transition from CAL to RUN. Stop/review if jitter, overshoot,
   current stop, bad tracking, voltage sag or unexpected motion occurs.
5. Only after acceptable stand results: separately approved all-four matching
   rollout and all-four checks before another bounded ground test. **No ground
   driving with mixed gains.** Do not manually brake rotating wheels by hand;
   no controlled-load rig is available, so loaded validation remains ground-only.

Rollback is the preserved, independently verified stock application/metadata,
not P2. Known target-ready TUI handshake and manual CAN-CAL byte-order defects
remain open; this candidate does not fix them.

## FR-only CAN installation — 00:23Z

Operator confirmed motor battery OFF, all four wheels raised, separate FR/Nucleo
USB connections, closed TUI/tools and exclusive hardware release; explicitly
authorized FR Ki4 programming and restoration of the identical IK host, no CAL
or motion. Independently listed FR `066AFF525266555067075126` and Nucleo
`002200364142500620353451`. Existing worktree/branch and all unrelated changes
preserved. Installation/monitor support extends ownership to
`scripts/monitor_esc_power.py` and `tests/test_monitor_esc_power.py`.

- Added explicit `ne-900-ki4` monitor identity: exact ELF hash
  `dfab08bdcdb127dc649de673c24eecd42b1846595d9cc7a74ca0cb1daaeb376c`, app hash
  `e77392a7f03732ffcc8970196a91e928fc9da92a3200264c7e0dc4b535204557`, 40868 B,
  CRC `75A5B016`. Full live app/metadata/VTOR checks remain mandatory. New
  host-only test rejects live stock under Ki4 even with candidate metadata,
  rejects bad CRC metadata/VTOR, and verifies no target writes. Monitor suite
  now 32 tests; all four FWU regression suites also passed before programming.
- Fresh 128 KiB pre-update backup matched restored stock full-flash SHA256
  `df1053892346557720d8c143e2876f9672429c9c1b0c5dcda03a8919fbb42e6a`.
  Direct preflight inactive/unarmed/RPM 0, CAL 0/0, SDK Vbus ~4.574 V.
- Temporary updater: `CM7/build/FWU-FR-900-ki4/sfurs_firmware_v2_CM7.elf`, Debug,
  motor/IK producers OFF; ELF SHA256
  `7047d60af3a60f79f9471bd965e7a4825ebeed4bcbdb1dace5225787b3486e73`.
  Download/verify/reset plus independent complete host flash/payload matching
  passed before selecting device **1**. Target normal `SYSRESETREQ` exposed
  its existing bootloader window; no battery-off fault-clear bypass.
- One CAN transfer reached DONE/result 3, final DATA sequence 6811 / 6812 DATA
  frames, 6816 ACKs. No automatic retries or other wheel programming.
- Independent full 128 KiB readback equals predicted image SHA256
  `adb72c13d3594b8cefd886dbf7c90d33da49812a7d6019e2085db88e987f8c9c`.
  Metadata/CRC/app and VTOR `08004000` match. Bootloader and unused flash
  unchanged. Live default/applied speed coefficients -18600/-288, divisors
  1024/16384, exponents 10/14 and output +/-1632 verified.
- Final FR inactive/unarmed/RPM 0, CAL 0/0 after reset; SDK Vbus ~4.582 V,
  state 10, current/past faults 4/4 (battery-OFF undervoltage). This is not
  powered readiness or fault-free motion evidence.
- Completed updater/request 0 verified before restoring the **identical**
  `CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf`, SHA256
  `83cecd66e9a3ead263b40607ca909caa4a74ea9847274a4af9d99e60c419ee77`.
  Download/verify/reset and independent full segment/ABI2 match passed. Host
  IDLE, sequence/completed/request 0, zero targets/twist and no host fault.
- Both probes released. Battery last confirmed OFF; wheels raised. No CAL,
  RUN, queued FWU retry or motion request. Mixed-gain stand evaluation only.

Fresh backup/readback and verified sidecar (outside Git):
`/home/fh/sfurs_firmware/esc-backups/fr-ki4-20261016T002315Z/NE-{before,after}.bin`
and `.json`. Use `NE-after.json` with the explicit `ne-900-ki4` monitor profile.
Prepared candidate manifests remain historical readback=false records.

Evidence under `/tmp/opencode/rpm2-ki4-review-20261016/`: `install_fr.py`,
`install-session.json`, `transfer.jsonl`, `updater-manifest.json`, build/programmer
logs, `preinstall-tests.json`, `nucleo-restored.json`, and
`NE/ki4/monitor-prepared.json` (prediction only). No installation claim is based
only on TX ACK or host DONE.

## Powered preparation blocked — 00:28Z

Operator selected battery ON / CAL-only, with all wheels raised, both USB cables
unchanged and exclusive access retained. No motion was approved. Initial helper
attempt at 00:27:45Z failed before hardware reads/commands because it used an
`auto_open=False` ESC session without explicitly opening it. Both sessions were
released; helper corrected to explicit `open()` / `finally: close()`.

The corrected read-only FR preflight at 00:28:30Z verified exact Ki4 app,
metadata and VTOR, then measured SDK average Vbus **4.764 V**, ADC-derived Vbus
**4.781 V**, threshold ~8.0 V, state 10 and current/past fault **4/4**
(undervoltage). CAL 0/0, active/armed 0/0, RPM 0. **Readiness gate aborted before
opening the Nucleo session or sending any CAL/ARM/RUN/STOP command.** There is no
command JSONL for this attempt because the command logger was never opened.

Battery was operator-reported ON, but expected motor-bus voltage was absent at
FR; connector/switch/distribution versus other power-path causes are not yet
isolated. Do not clear faults or tune around this evidence. External power
connection/switch check and fresh power confirmation are required before another
read-only voltage check, followed by CAL-only preparation if voltage is valid.
Both probes released. Ki4 installation remains verified; no stand motion test,
CAL or ground driving performed. Current calibration is not valid.

Evidence: `stand-CAL-only-20261016T002745Z.json` (helper failure) and
`stand-CAL-only-20261016T002830Z.json` (read-only undervoltage abort), under the
same artifact directory. Corrected helper: `prepare_raised.py`.

### Power-restored recheck — 00:34Z

Operator reported "Battery is restored". Same probe serials independently
listed and no control/debug tools running; continued the approved readiness/CAL
scope with no motion. Exact Ki4 identity passed again, but direct FR average
Vbus was **4.606 V**, raw ADC-derived **4.640 V**, state 10, faults 4/4,
CAL 0/0, inactive/unarmed/RPM 0. Aborted before any Nucleo session or command;
both probes released and no command JSONL created. Evidence:
`stand-CAL-only-20261016T003402Z.json`. This recurrence requires distinguishing
the external battery/power-distribution path from voltage-sensing faults, not
repeated blind CAL or gain changes. Firmware cannot infer battery connectivity
from operator-reported restoration alone; no powered readiness is established.

### External feed corrected; below test-voltage gate — 00:36Z

Operator identified the motor feed as disconnected/OFF, reported a multimeter
available, then freshly confirmed connected/ON, raised clear wheels, unchanged
USB connections, closed TUI and exclusive access for CAL only. Exact FR Ki4
identity passed. FR now reports average **10.743 V**, ADC-derived **10.824 V**,
state 11, current faults 0 / past faults 4, CAL 0/0, inactive/unarmed/RPM 0.
Motor power is now present and active undervoltage cleared, but the established
**11.0 V resting test gate** was not satisfied. Preparation stopped before any
Nucleo session/command; both probes released. Do not equate this conservative
test gate with the firmware's ~8 V undervoltage threshold, or silently lower it.
Independent DC-voltage measurement/charging review is pending before further
CAL or tuning. Evidence: `stand-CAL-only-20261016T003624Z.json`, no command JSONL.

Operator subsequently selected independent meter voltage **11 V or higher**;
exact value and measurement point were not supplied. A bounded five-second
read-only trace (20 samples, 00:38:35–40Z) found FR average **10.678–10.683 V**,
ADC-derived **10.691–10.700 V** (raw ADC 20608/20624), state 11, faults 0/4,
CAL 0/0, inactive/unarmed/RPM 0. It did not settle above the 11 V gate. This is
not proof of an ADC scaling fault or a wiring drop: exact contemporaneous meter
value/location are still needed. No writes, reset, fault clear, CAL or RUN;
monitor exited normally and released FR, Nucleo never opened. Evidence:
`FR-voltage-settling-20261016.jsonl`. Keep preparation paused while reconciling
independent supply versus board-reported voltage.

## Current-battery CAL-only exception — completed 00:39:46Z

Operator requested using this battery for now while another charges. Explicitly
announced a **10.5 V floor for raised-wheel CAL only**, relaxing the previous
11 V procedural gate for this operation, not firmware protection or motion
testing. Helper default remains 11 V; explicit `--allow-current-battery-cal`
records this separately authorized exception. Existing firmware protections and
all current/RPM/stall/count/timeout limits remain unchanged. No RUN approval.

Exact FR Ki4 and host IK identities passed. FR initial Vbus 10.600 V, state 11,
faults 0/4, CAL 0/0. Fresh all-four replies initially state 11, CAL-invalid,
inactive/disarmed/RPM 0, no active faults. One PREPARE acknowledged cleared
faults; fresh all-four IDLE confirmed. Second PREPARE triggered RAM auto-CAL;
fresh all-four IDLE/CAL-valid confirmed. Auto-CAL completion did not ARM/RUN.

Cleanup STOP followed by **fresh advancing all-four IDLE, zero RPM, CAL-valid,
inactive/disarmed and active faults 0**. Direct FR independently CAL **2/1**,
state 0, current/past faults **0/0**, inactive/disarmed/RPM 0. FR Vbus sampled
throughout preparation, **10.596–10.600 V** across 16 samples; no RUN or active
fault observed. Logged mailbox commands `[1,1,3]` are two PREPARE and STOP;
there is no START. Both probes released, no queued motion/FWU requests.

Evidence: `stand-CAL-only-20261016T003946Z.json` and `.jsonl`. Battery last
operator-confirmed ON, all wheels raised, both cables unchanged. Calibration
is volatile; future power/reset changes require renewed checks. This is
successful powered calibration preparation only, **not stand-motion, power-load
margin, resolved voltage-scaling discrepancy or ground validation**. The 10.5 V
exception is not motion authorization. FR Ki4 / other three stock remains the
installation; no ground driving with mixed gains.

## First stand pulse authorized, blocked before ARM

After successful CAL, operator separately authorized one **FR-only fixed -100
RPM diagnostic for ~1 s**, existing 500 ms ramp, current battery, monitored
10.5 V abort floor and unchanged firmware protections. All wheels raised/clear,
cables unchanged, closed TUI, exclusive access and reachable cutoff confirmed.
No repeat, other-wheel RUN or ground motion authorized. Exact-image/direct
mailbox/Hall symbols and request constants checked offline before execution.

The actual preflight found FR average **10.119 V**, ADC-derived **10.127 V**,
below the approved floor. It **aborted before any direct ARM/RUN/STOP request or
Nucleo command**. The JSONL contains only one `sample` event. FR direct request,
last request, result and run counter are all 0; Hall edges/net edges/invalid 0.
FR CAL 2/1, IDLE, inactive/disarmed/RPM 0, faults 0/0. All four CAN replies fresh,
IDLE, zero RPM, disarmed/inactive, CAL-valid and no active faults; no ever-RUN
flag. Host remained IDLE/zero targets. Both probes released, no pending request.

Evidence: `FR-stand-neg100-1s-20261016.json` and `.jsonl`; helper
`stand_fr_neg100.py` refuses repeat use. **No Ki4 motor-motion trial occurred.**
Board-reported voltage fell from ~10.60 V during CAL to ~10.12 V at this preflight
without a motor run. Low battery versus feed/sensing discrepancy is not fully
isolated; exact independent meter value/location still missing. Operator advised
to switch battery OFF and wait for the charged replacement, not repeatedly lower
the gate. Battery OFF is a request, not yet a fresh confirmation. A replacement
requires new power-state/readiness/CAL checks and separate motion approval.
