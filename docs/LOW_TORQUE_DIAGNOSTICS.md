# Keyboard IK: free-air motion, buzzing under load

## Evidence and checkpoint

Operator report after `ik-first-working-20261015` (`572d4d8`):

- Keyboard IK spins the wheels off the ground.
- All translations feel weak; on the ground the robot does not drive and
  wheels buzz. Sometimes some wheels rotate while others buzz.
- No time-aligned voltage/current/RPM capture yet. The cause is **not diagnosed**.

Do not hold a stalled/buzzing motor under power. Stop ground tests until the
telemetry image is installed and a short, separately authorized capture is ready.
Keep the first working checkpoint intact; do not tune PID, raise current limits
or remove diagnostic guards to compensate for an unidentified failure.

## What to record on every wheel

- Requested body twist and signed motor RPM target.
- Actual RPM, SDK motor state, Hall bits and Hall-change counter.
- Raw Iq/Id, sampled peak L1, current-fault bits, CAL-valid, armed and active.
- Independent ages for speed and current replies; missing/stale is not zero.
- Timestamped keyboard/mailbox events and samples in JSONL.

These already exist in ESC CAN speed/current replies; only the Nucleo receiver
and TUI need changes. Current packets do **not** contain requested Iq, bus
voltage, detailed CAL result or stop reason. They cannot prove battery sag or
current saturation on their own. Use a second, read-only ESC debug connection
with the exact installed ELF for those extra fields, or separately extend ESC
telemetry later. Host/SWD sampling can miss fast voltage/current transients.

Iq is torque-axis motor current, NOT pack current. Firmware-scaled A values
require fresh RUN, CAL-valid, active and fault-free evidence; offsets/current
scaling remain unverified physical calibration. Keep raw counts in the log.
The peak is a sampled L1 (|Iq|+|Id|) maximum, not instantaneous phase-current peak.

## Interpretation guide (hypotheses, not automatic diagnoses)

| Observation | Next distinction |
|---|---|
| No target RUN / CAL invalid / inactive | Start or calibration rejection rather than weak torque |
| Undervoltage fault (0x0004) | Measure Vbus and wiring drop during the event; fault alone does not establish pack health |
| RUN, little Hall progress, low measured Iq | Compare requested Iq: low controller demand vs inability to deliver current |
| RUN, little Hall progress, high current | Possible load/commutation/current ceiling; inspect Iq reference, Vbus and mechanical drag |
| Different wheels stop independently | Read stop reasons: stall, sampled-current, count and timeout guards can stop without an SDK current-fault bit |
| RPM tracks targets but robot does not translate | Check physical rolling directions, geometry/polarity, traction and wheel forces cancelling |

The requested speed is not a torque limit. Low target RPM and a short tap
through the 500 ms speed ramp can reduce observed effort; they do not alone
explain a loaded stall. The installed RPM2 source uses a 3 A reference bound
and sampled-current stop, but source bounds are not measured delivered torque.

## Scope of this telemetry step

No ESC reflash, no PID/current-limit changes and no new motor-control framework.
The Nucleo will poll all four selectors equally and pause polling during ARM,
SPEED and STOP transport ownership. Read-only polls use a separate enqueue path
so they cannot overwrite the drive TX completion mask. Existing drive behavior
and first-working tag remain available. A new Nucleo image requires explicit
programming authorization and fresh motor-battery-OFF confirmation.

## Two-cable capture tools (implemented; live capture not yet performed)

The Nucleo telemetry image is ABI2 at
`CM7/build/IK-telemetry/sfurs_firmware_v2_CM7.elf` (22,020 B flash / 2,088 B RAM).
The first-working, previously flashed ABI1 ELF in `CM7/build/IK` is preserved
byte-for-byte. The new TUI intentionally refuses an old flashed image; do not
bypass its flash/ABI checks. ABI2 has now been programmed with operator
authorization and battery OFF; see KEYBOARD_IK.md for flash/readback evidence.
Powered telemetry validation is still pending.

After programming and after fresh target/power/ownership confirmation:

1. Secure robot with wheels off ground. Have the motor cutoff accessible.
2. Connect Nucleo data cable to probe `002200364142500620353451`; connect the
   second cable to front-right ESC probe `066AFF525266555067075126`. Close any
   tool already owning either probe. Both loggers may run together only because
   they use different probes and the ESC logger never commands CAN or motors.
3. In terminal A, start the front-right **read-only** monitor:

   ```sh
   python3 scripts/monitor_esc_power.py --seconds 60 --period 0.05
   ```

   It resolves the archived NE manifest/ELF and refuses any other image/layout.
   Before sampling it compares the complete live application, validity metadata
   and vector-table address; mismatch aborts, never triggers a reflash/reset.
   It performs no application writes, CAL, fault clear, motor commands or STOP
   (including on exit). Attach still initializes debugger infrastructure.

4. In terminal B, start the Nucleo IK TUI:

   ```sh
   python3 scripts/keyboard_drive.py
   ```

   Both programs create unique JSONL logs under `/tmp/opencode` and print their
   paths. Retain both files. Samples use UTC and host monotonic timestamps
   (ESC block timestamps in ns; TUI monotonic timestamps in seconds).
5. Begin with stationary baseline readings. On battery power, P performs only
   existing ARM/auto-CAL preparation. Check fresh CAL/state replies on all four
   wheels; bus ACK alone is not readiness. No automatic RUN or blind start retry.
6. A separately authorized short stand run can then capture target versus actual
   RPM/current. Stop with Space, confirm physical rest and review both logs.
   Plan any brief loaded comparison only after reviewing the free-air evidence;
   do not hold a buzzing/stalled wheel under power.

`--monitor-only` on the Nucleo TUI is a read-only display/log mode, not another
concurrent Nucleo session. It ignores control keys and never STOPs on exit.

### Additional fields from the front-right logger

- Averaged ESC DC-link voltage and pre-IIR ADC-equivalent voltage, conversion
  factor and undervoltage threshold. This is not direct battery-terminal voltage.
- Requested Iq/Id and measured Iq/Id, with current-valid/CAL/RUN/fault context.
- Actual linked PID output and torque limits (counts); a near-limit indication
  is an observation, not an automatic tuning recommendation.
- Motor state, current/past faults, actual RPM, armed/active and explicit stop
  reason (`current`, `stall`, `hall_invalid`, `count`, `timeout`, etc.).

The tool is deliberately restricted to the exact audited NE ELF SHA256
`a84a0dc7c40bcaad55b7e58f33639c8d4fdff539d28f005c4e5b76df821d9b95` and manifest
`/home/fh/sfurs_firmware/esc-backups/ne-fwu-rollout-20261014/NE-fixed.json`.
Other ESCs/images require their own verified layouts; do not reuse these RAM
offsets. Its archived source/symbols establish a nominal 3 A reference bound
(1632 counts), NOT measured delivered torque or an instantaneous protection
guarantee. The linked float current scale is approximately 0.001836397 A/count;
the CAN TUI's integer 544-counts/A scaling is slightly different and labeled.

Each RAM block has its own read-time bounds and raw bytes; the complete sample
is not atomic. Vbus averaging uses a 1/256 IIR, and even pre-IIR/host sampling
can miss short dips. A normal sampled voltage cannot rule out supply transients.

## Verification before programming

- 65 TUI/transport/logging tests; 25 read-only ESC-monitor tests.
- Nine actual-C telemetry scenarios at both -O0 and -O2.
- Four actual-C decoder-to-TUI ABI/interpretation tests; 13 drive-adapter scenarios.
- All 15 ABI2 symbols and the 4x52-byte telemetry layout verified in the built ELF.
- Independent review findings on frozen-clock freshness and missing STOP-failure
  log events fixed and regression-tested. Existing ESC/PID/current settings untouched.

## First read-only FR baseline (2026-10-15)

After operator connected FR USB, both expected Nucleo and NE probes were
detected and no competing probe tools were found. With the operator instructed
to keep the motor battery OFF, a one-second read-only NE capture passed full
application/metadata/VTOR identity checks and completed without application writes.

- Log: `/tmp/opencode/esc-power-20261015T115641.120333Z-3658686.jsonl`.
- RPM 0, Iq reference 0, measured current invalid, state FAULT_NOW (10),
  current/past fault `0x0004` (undervoltage), CAL result/valid `0/0`.
- SDK-scaled DC-link reading about 4.42 V in this USB-only setup. This is
  **not a powered pack-voltage measurement** and cannot diagnose battery sag.
- A pyOCD board-ID warning appeared; the explicit STM32G431 target and full
  firmware identity checks succeeded. No reset, CAL, ARM, RUN, STOP or fault
  acknowledgement was issued. Session released after capture.

Battery-off undervoltage and invalid current/CAL here do not establish the
cause of the loaded buzzing. Next obtain a stationary powered baseline before
any operator-authorized motion capture.

## First powered stationary baseline (2026-10-15)

Operator confirmed battery connected. Both captures were read-only, on separate
probes, with no CAL/ARM/RUN/STOP or fault-acknowledgment writes:

- FR power log: `/tmp/opencode/esc-power-20261015T120542.441202Z-3659094.jsonl`.
  Averaged DC-link about 11.77 V throughout; RPM 0, Iq reference 0,
  state FAULT_OVER (11), current faults 0, past faults `0x0004`, CAL `0/0`.
- All-wheel log: `/tmp/opencode/ik-telemetry-20261015T120543.034454Z-3659096.jsonl`.
  All four selectors answered both speed/current requests; latest reply ages
  were 19–159 ms. All RPM 0, state FAULT_OVER (11), profile flag present,
  CAL/armed/active/ever-run false and current faults 0. Measured current is
  invalid outside calibrated fault-free RUN; the raw zeros are not load-current
  measurements. Nucleo IDLE, drive fault 0, zero targets.

The present undervoltage condition cleared after battery connection, but old
fault acknowledgment and RAM calibration preparation are still needed. A first
ARM-only preparation can consume the cleared-fault acknowledgment before CAL;
observe state/CAL rather than following it with an immediate RUN. No start is
authorized by these observations. 11.77 V at rest does not establish voltage
behavior under load or battery health. Probe sessions released after sampling.

## First operator-motion CAN log review (2026-10-15)

Reviewed the running TUI's file only; no probe access or motor commands:
`/tmp/opencode/ik-telemetry-20261015T120645.577393Z-3659259.jsonl`.
Captured samples through approximately 12:08:50 UTC include translation requests
at 0.2 m/s. Whether each interval was free-air or grounded is not yet confirmed.

Fresh, valid current samples while the host reports RUNNING show approximately:

| Wheel | Median absolute Iq (SDK-scaled A) | Maximum sampled absolute Iq |
|---|---:|---:|
| FR | 0.191 | 0.517 |
| BR | 0.154 | 0.390 |
| BL | 0.119 | 0.381 |
| FL | 0.127 | 0.425 |

Many samples report RUN with zero measured RPM. No current SDK fault bits
were observed in the fresh current replies reviewed. Maximum reported per-run
sampled L1 peaks were FR 872, BR 715, BL 732, FL 861 SDK counts (not instantaneous
physical current peaks), below the RPM2 source's 1632-count sampled-stop threshold.
This is evidence of small sampled torque-axis currents, **not** evidence that a
3 A limit was reached; sampling alone cannot exclude fast excursions/other stops.

A source-level hypothesis: RPM2 speed gains have magnitude Kp=18600/1024 and
Ki=72/16384 per SDK speed error (6 RPM/unit), with a 1000 Hz speed loop. A
stalled 48 RPM reference is 8 SDK units, giving only about 145 current counts
(~0.267 A) from the proportional term; integral torque builds over time. The
500 ms ramp and 2 s no-Hall-progress guard can matter before high torque develops.
This calculation is a **hypothesis about controller demand**, not a measurement
of requested Iq or permission to change gains/guards. Verify reference versus
feedback on the matching live image before choosing a fix.

The available FR power logs contain stationary baselines only, so there is still
no simultaneous motion capture of requested Iq, Vbus and stop reason. Do not
diagnose battery sag from the 11.77 V stationary reading. Next correlate a short,
explicitly authorized run with the second-USB logger and record whether wheels
are off the ground or loaded. No current/PID/firmware changes made for this review.

## Completed free-air log: unreliable low-speed starts (2026-10-15)

The operator clarified that these runs were **off the ground** and closed the
TUI. The completed file above contains 3448 sample events and 25 START requests.
Analysis below excludes the first 0.7 s after START and uses fresh speed replies
before the next mailbox command. Low unloaded current alone is not a defect;
failed starts and early per-wheel stops are the actionable findings.

| START sequence | Request / targets (FR,BR,BL,FL RPM) | Observations before operator STOP |
|---|---|---|
| 4 | forward 0.2 m/s / `(51,48,-48,-51)` | FR stays 0 RPM with no Hall progress, enters STOP at ~2.144 s; other three continue RUN with Hall progress |
| 6 | right 0.2 m/s / `(-29,33,33,-29)` | FR and BR have no Hall progress; STOP at ~2.199/2.095 s. BL/FL continue with intermittent progress |
| 40 | forward 0.2 m/s / `(51,48,-48,-51)` | All four remain RUN through the ~9.482 s request; post-ramp median RPM about `(42,48,-42,-42)` with Hall progress |
| 52 | yaw +0.5 rad/s / `(13,12,12,13)` | All four stay 0 RPM/no Hall progress, enter STOP at ~2.105–2.212 s, before the ~2.422 s operator STOP |

The repeatable ~2 s self-stop with no Hall progress is **consistent with** the
existing 2000 ms Hall-stall guard, not an independently read stop-reason result.
The speed/current CAN packets cannot prove that reason. CAL remained valid and
current-fault bits were zero in the affected fresh replies; the host continued
RUNNING after some ESCs had stopped. Successful bus transmission is therefore
not equivalent to a sustained four-wheel run.

The slow IK references are substantially below the old single-wheel TUI's
typical 150–200 RPM settings. Low-reference startup torque/controller demand
is a plausible contributor. Static friction, rotor-position-dependent startup,
current tracking or commutation remain alternatives until requested Iq and Vbus
are captured during an affected start. Do not call this proof of battery sag,
a 3 A current ceiling, or a particular required gain change.

Final logged cleanup: sequence 54 completed, Nucleo IDLE, commanded RPM all 0,
drive TX/ACK 290/290, host fault 0. This is bus/host completion, not independent
physical-stop feedback. Both available FR power logs still contain only prior
stationary baselines. Next diagnostic capture must run the separate FR power
logger concurrently with a short, explicitly authorized stand start. No probes,
motor commands, code tuning or flashes were used to analyze this completed file.

## Synchronized FR power/current capture (2026-10-15)

Operator confirmed battery ON and asked to start the FR read-only logger before
starting the separate TUI. Exact-image preflight succeeded; stationary averaged
Vbus was ~11.72 V. The three-minute logger owns only the FR probe and issues no
application/motor commands. Log paths:

- FR: `/tmp/opencode/fr-iq-capture-20261015T1220.jsonl`.
- TUI: `/tmp/opencode/ik-telemetry-20261015T122029.848353Z-3661549.jsonl`.

Matched the logs using host monotonic timestamps (FR block ns converted to s).
Current values below use valid, CAL-complete, fault-free RUN samples; fields
across RAM blocks remain non-atomic. The user reported that FR turned.

| TUI sequence | FR target (RPM) | FR RPM median | Absolute Iq reference median/max (SDK A) | Absolute measured Iq median/max (SDK A) | Outcome |
|---|---:|---:|---|---|---|
| 59, reverse translation | -51 | -42 | 0.112 / 0.397 | 0.116 / 0.386 | Rotation, then operator STOP |
| 61, right translation | -29 | 0 | 0.158 / 0.189 | 0.152 / 0.286 | Explicit stop reason 9 (stall) at ~2.051 s, before host STOP |
| 63, left translation | +29 | 0 | 0.157 / 0.189 | 0.149 / 0.285 | Explicit stop reason 9 (stall) at ~2.053 s, before host STOP |
| 65, forward translation | +51 | +48 | 0.103 / 0.536 | 0.114 / 0.564 | Rotation, then operator STOP |

Median absolute requested-versus-measured Iq difference was ~0.024–0.031 SDK A
for these runs. No sample was near the actual linked +/-1632-count (~3 A)
PID output limit. Current/past faults were zero in the valid RUN samples and
CAL stayed `2/1`. Measured peak L1 during sampled RUN remained <=782 counts.

Averaged Vbus during these motion samples was ~11.725–11.732 V; pre-IIR
ADC-equivalent readings were ~11.779–11.878 V. There is no observed major
supply collapse in this free-air capture. Filtering and host sampling still
cannot exclude fast dips or diagnose voltage under a ground load.

**Finding:** the failed +/-29 RPM starts have low requested torque as well as
low measured torque, no Hall/RPM progress, and a directly observed stall stop.
This is not evidence that a 3 A current ceiling prevented delivery. The current
loop broadly follows the small request in the sampled data; this does not by
itself prove correct commutation or sufficient mechanical torque.

The next candidate investigation is ESC speed-loop/startup demand at low
reference, including breakaway friction/rotor-position sensitivity, not raising
the current ceiling. A controlled FR-only tuning/startup comparison should keep
existing current/stall protections and be separately authorized. No gains,
current limits, IK geometry, ESC binaries or Nucleo binaries changed for this
analysis. The read-only logger finishes automatically after its requested duration.

Logger subsequently completed its requested three minutes and released the FR
session. Final captured state was IDLE, RPM 0, current/past faults 0 and CAL
`2/1`; no cleanup motor command was sent. The opt-in FR-only P2 candidate was
subsequently built/reviewed, explicitly authorized, installed over CAN and
independently read back; see [FR_LOW_SPEED_P2.md](FR_LOW_SPEED_P2.md).
Current/stall protections are retained. Nucleo telemetry firmware is restored;
the first P2 stand comparison subsequently recorded FR breakaway at +/-29 RPM,
but the operator reports much worse jitter. Detailed per-run evidence and the
still-reproducing readiness gap are in that document; no loaded torque or
all-wheel improvement is established.
