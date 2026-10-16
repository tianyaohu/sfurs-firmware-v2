# Loaded drive baseline — 2026-10-16

Status: **0.5 m/s ground attempts produced only twitch/buzz; a 1 m/s, 1 s
attempt produced slight forward creep, not sustained speed tracking**.
No gains, current limits, caps or protections changed. No successful loaded
tune is established. See [FR_LOW_SPEED_P2.md](FR_LOW_SPEED_P2.md) for the restored
original-gain stand results, and the `esc-foc-tuning` skill for evidence limits.

## Configuration and evidence boundary

- All four ESCs retain original RPM2 speed gains, -18600/1024 and -72/16384,
  +/-900 RPM caps, +/-1632-count speed PI current output, and existing guards.
  Prior installation/readback evidence is in
  [HIGH_SPEED_CAPS.md](HIGH_SPEED_CAPS.md); ESC identity was not read directly
  during this Nucleo-only ground session.
- Nucleo: exact installed `CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf`,
  SHA256 `83cecd66e9a3ead263b40607ca909caa4a74ea9847274a4af9d99e60c419ee77`,
  full flash segment/ABI2 identity checked before commands, attach/no reset.
- Only Nucleo USB is connected because of cable reach. Fresh CAN provides
  per-wheel RPM, Hall changes, state, CAL/active/armed/fault flags and measured
  Iq/Id. It does **not** provide Vbus, Iq reference, direct CAL result, or an
  explicit stop reason. These readings cannot establish current-reference
  tracking, current-demand saturation, inner-loop stability or voltage margin.
- Operator confirmed battery ON after moving to the ground, closed TUI and
  exclusive access, clear surroundings, reachable cutoff, and safe USB slack.
  The motion approval additionally confirmed at least 1 m clear forward path
  and sufficient cable slack. No lateral/reverse/yaw or repeated starts approved.

All evidence below is under `/tmp/opencode/fr-900-stock-20261015/`; UTC crossed
midnight during this continuation, hence the 20261016 filenames.

## Ground preparation (23:59:25Z on 2026-10-15)

All four initially reported state 11, CAL-valid, RPM 0, disarmed/inactive, and
no active faults. One ARM-only PREPARE acknowledged cleared faults; fresh replies
confirmed all four IDLE. No additional CAL was needed. Final STOP/disarm and
fresh all-four IDLE, RPM 0, CAL-valid and no active faults confirmed. No speed,
RUN, gain, flash or reset commands. Probe released.

Evidence: `ground-preparation-20261015T235925Z.json` and `.jsonl`.

## Aborted preflight — no START (00:00:42Z)

The first authorized timed-test wrapper failed while logging its first preflight
sample: the wrapper passed its own `monotonic` field to a logger that already
adds that field. Cleanup sent STOP. Command log independently confirms **only
STOP, no START**. Fresh read-only follow-up confirmed all four IDLE, zero RPM,
disarmed/inactive, no active faults, host STOP completed. No automatic retry.

Evidence: `ground-forward-05ms-20261016T000042Z.json` and `.jsonl`,
`ground-abort-stopped-20261016T000042Z.json`. The corrected logging call passed
an offline check (`ground-logger-offline-check-20261016T000115Z.jsonl`) before
separate authorization for the actual attempt.

## First actual attempt (00:02:45Z)

One forward START, vector `(0,500,0)` mm/s, mm/s, mrad/s; STOP cleanup entered
0.500065 s after starting the submission. The ESC ramp is also 500 ms, so this
pulse provides essentially no post-ramp observation. Host timing is not an
exact measure of PWM activation or physical stop duration.

| Wheel | Command RPM | Sampled RPM | Hall-change delta | Peak sampled valid RUN \|Iq\|, SDK A |
|---|---:|---:|---:|---:|
| FR | +126 | 0 | 1 | 0.585 |
| BR | +119 | 0 | 1 | 0.395 |
| BL | -119 | 0 | 0 | 0.460 |
| FL | -126 | 0 | 0 | 0.590 |

All four entered RUN in fresh CAN telemetry, with no sampled active fault bits.
Operator report: **"No visible movement. Slightly twitched"**. Therefore this
was not successful loaded travel; isolated Hall edges are not sustained motion.
Current values are asynchronous, low-rate, firmware-scaled samples, not
continuous peaks or battery current. Repeated cached rows are not independent
measurements. No explicit stall/current stop reason or Vbus was observable.

STOP completion was followed by fresh advancing all-four replies confirming
IDLE, RPM 0, inactive/disarmed, CAL-valid, active faults 0; Nucleo host IDLE,
zero targets and no host fault. Probe released. No repeat was issued.

Evidence: `ground-forward-05ms-20261016T000245Z.json` and `.jsonl`.

## One-second duration comparison (00:05:43Z)

After separate operator approval with the ground/cable/power setup unchanged,
retained the same forward `(0,500,0)` target and entered STOP cleanup after
1.000228 s. No gain/current-limit change, other direction or repeated START.
Fresh advancing all-four IDLE/CAL-valid/disarmed/fault-free readiness was checked
before the START. All four entered RUN but sampled RPM remained 0 throughout;
**Hall-change delta was 0 on all four wheels**, including the post-STOP check.

| Wheel | Command RPM | Hall delta | Peak sampled valid RUN \|Iq\|, SDK A |
|---|---:|---:|---:|
| FR | +126 | 0 | 0.820 |
| BR | +119 | 0 | 0.756 |
| BL | -119 | 0 | 0.721 |
| FL | -126 | 0 | 0.807 |

No sampled active fault bits. The current values are sampled measured Iq, not
Iq reference, battery current, or proof of instantaneous limit headroom. This
longer record still does not show a successful loaded start. Operator confirmed
**still only twitch/buzz**, current robot mass **under 5 kg**, and a **smooth
hard floor**. Exact mass, rolling resistance and breakaway force are unmeasured.
No explicit stop reason or Vbus
was available. No stall-guard claim is made for this one-second attempt.

STOP completed, followed by fresh advancing all-four IDLE, RPM 0, disarmed/
inactive, CAL-valid and active faults 0. Host IDLE, zero targets, no host fault;
Nucleo probe released. Evidence:
`ground-forward-05ms-1s-20261016T000543Z.json` and `.jsonl`.

## Representative target comparison: 1 m/s for 1 s (00:09:14Z)

Separately authorized by the operator with all safety/power/cable conditions
unchanged. Verified expected Nucleo probe serial, exact host flash/ABI, and fresh
advancing all-four IDLE, CAL-valid, disarmed/inactive, zero RPM and no active
faults before one START, vector `(0,1000,0)`. STOP cleanup entered 1.000062 s
after starting submission. Same gains, ramp, limits and guards; no repeated
START or other direction.

| Wheel | Command RPM | Sampled RPM range | Hall delta | Peak sampled valid RUN \|Iq\|, SDK A |
|---|---:|---:|---:|---:|
| FR | +253 | 0 | 6 | 1.664 |
| BR | +239 | 0 | 5 | 1.546 |
| BL | -239 | -36 to 0 | 11 | 1.511 |
| FL | -253 | 0 | 9 | 1.597 |

All four entered RUN. Operator confirmed **creeped forward slightly**, then
estimated **maybe 0.5 cm (5 mm)**. This is a visual estimate, not an instrumented
distance measurement. Thus there was minimal forward chassis movement, but not
sustained tracking of the requested wheel RPM. Sampled RPM 0 must not be read
as no physical motion when Hall transitions and observation show otherwise.
Different Hall totals on this short startup do not by themselves establish
wheel mismatch or a commutation fault. No sampled active fault bits.

STOP completed; fresh advancing all-four replies confirmed IDLE, RPM 0, CAL-valid,
inactive/disarmed, active faults 0. Host IDLE, zero targets and no host fault.
Nucleo probe released. Battery last confirmed ON; no new motion queued.
The ~5 mm creep is classified as a failed useful loaded start, not successful
ground driving or a measured 1 m/s chassis response.

Evidence: `ground-forward-1ms-1s-20261016T000914Z.json` and `.jsonl`.
Current summary de-duplicates telemetry signatures; snapshots remain low-rate,
asynchronous firmware-scaled measurements, not current references or continuous
peaks. No Vbus or explicit stop-reason inference is possible from this record.

## Next step, not authorized by these trials

Subsequent separately approved offline review prepared an opt-in Ki4 candidate;
see [RPM2_KI4_REVIEW.md](RPM2_KI4_REVIEW.md). A later separately approved FR-only
installation is now verified there; the other three wheels remain stock.
No ground motion with mixed gains. These baseline trials all used stock gains.

The unchanged baseline has now shown insufficient loaded startup/tracking at
both tested target speeds. Stop escalating target speed or locked-rotor dwell
blindly. Review the outer speed PI demand and a conservative integral-gain
candidate while retaining original Kp, inner current PI and all protections.
Do not treat sampled measured Iq below 3 A as proof that the reference never
saturated, that power did not dip, or that raising current limits is needed.

Distinguish insufficient speed-loop demand from current tracking, power and
mechanical issues before selecting or deploying gains.
Nucleo-only telemetry cannot resolve all those causes. No automatic current-limit
increase or gain sweep. Any candidate tune must be checked on the stand and
matched/verified on all four before further ground driving. Known target-ready
handshake and manual CAN-CAL byte-order defects remain open.
