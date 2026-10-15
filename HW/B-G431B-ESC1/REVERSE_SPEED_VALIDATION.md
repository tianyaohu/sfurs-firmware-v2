# Reverse-direction speed validation (installed)

Uses local firmware-v2 SDK sources as reference (no new SDK vendoring).
Plant sign on front-right: positive torque drives reverse, so the standard
positive-gain speed loop is positive feedback. This profile inverts the loop
by negating both speed-PI gains and commanding a negative target, giving
stable reverse tracking instead of the previously observed +60 runaway.

Opt-in `build_cli.py --motor NE --reverse-speed-diagnostic`:
- Fixed target -60 motor RPM, 500 ms ramp; ARM magic RPM2 `0x52504D32`.
- Speed PI Kp=-18600/1024, Ki=-72/16384 (ELF `.data` confirms `B758/FFB8`).
  Reference/sampled-current 3 A nominal, divisors/anti-windup unchanged.
- Local STOP: 10 s backup timeout, 2 s no-Hall-progress stall, invalid Hall,
  speed magnitude >=300 RPM, faults/calibration loss. No turn-count endpoint.
- Existing calibration, explicit ARM/RUN, one-shot, UART exclusion unchanged.
  Legacy +60 speed frames rejected; CAN speed path shares the same start gate
  (separate Nucleo sender still required for a later CAN test).

Offline `test_reverse_speed_guards.py` passes (fixed -60/ramp, reverse Hall
sequence 5->4->6->2->3->1->5, invalid/stall/timeout/speed/fault gates,
one-shot, manual STOP). Existing torque-count/RPM/torque suites still pass.
ELF `/tmp/opencode/esc-ne-reverse-speed-direct/motor_NE_reverse_speed_diagnostic.elf`:
FLASH=40,452 bytes, RAM=8,728 bytes. Flashed with explicit user permission and battery-off confirmation to expected front-right probe `066AFF525266555067075126`. Download, verification and reset succeeded. HOTPLUG readback confirms negated gains B758/FFB8, zero calibration/request/armed/active/used/Hall fields, nonzero SysTick. No CAL1/ARM/RUN written. Await battery-off confirmation and explicit flash
permission; re-resolve symbols, repeat calibration, then separate single-run
authorization for direct ESC test before any CAN test.

## First reverse direct test (stable tracking, count hook missing)

Authorized single -60 RPM reverse test after calibration. One RPM2 arm, one
DRUN. Log `/tmp/opencode/front-right-reverse-speed-20261012-223203.jsonl`: 5 RUN
samples over ~2 s, RPM 0->-48 stable (no runaway to guard), iq reference
18->191->57..63, peak L1=322 SDK counts. STOP reason=9 (2 s no-Hall-progress),
then IDLE. Hall edges/net/invalid all zero because the IRQ hook in
`hall_speed_pos_fdbk.c` was gated on RPM/TORQUE_COUNT macros and excluded the
new REVERSE_SPEED macro; the safety tick correctly treated zero progress as
stall. This confirms loop-sign stability (no overspeed trip) but not turn
counting. Fix hook gate, rebuild, reflash and rerun before CAN test.

## Installed 10-turn endpoint

Battery-off confirmed. Hook gate now includes REVERSE_SPEED; profile adds
COUNT stop at 480 net edges (10 turns), 30 s backup timeout (10 turns at 60 RPM
needs ~10 s plus ramp/margin). Offline reverse guards pass including 479/480
count boundary. ELF
`/tmp/opencode/esc-ne-reverse-speed-10turns/motor_NE_reverse_speed_diagnostic.elf`.
Flashed with explicit user permission and battery-off confirmation to expected front-right probe `066AFF525266555067075126`. Download, verification and reset succeeded. HOTPLUG readback confirms negated gains, zero fields, nonzero SysTick. No CAL1/ARM/RUN. Repeat powered calibration before authorized 10-turn direct test.

## 10-turn reverse test success (count stop)

Authorized single -60 RPM 10-turn direct test after calibration. One RPM2 arm,
one DRUN. Log `/tmp/opencode/front-right-reverse-10turns-20261012-224704.jsonl`:
30 samples over 12.8 s, 26 RUN samples ranging -96..0 RPM, Hall net=-480 with
488 valid edges, stop reason=8 (turn-count target), peak L1=378 SDK counts.
Final IDLE, faults zero, used=1. No overspeed/stall/invalid trip. Physical
observation and battery disconnect pending. Ready to extend the same guarded
reverse profile to CAN after direct validation.

## Repeatable runs (one-shot latch removed, not installed)

User requested the one-shot guard off with battery off and cable on ESC.
`test_drive_used` is now a completed-start counter, never a latch: ARM, start
and calibration no longer consult it. Fresh ARM still required per start (start
consumes ARM), plus calibration/fault/IDLE gates, count/stall/current/speed/
timeout stops and 5 s ARM expiry unchanged. Named TEST_REQ_*/TEST_STOP_*
constants replace magic numbers. All five offline suites pass with repeat-run
coverage. Prepared
`/tmp/opencode/esc-ne-reverse-repeatable/motor_NE_reverse_speed_diagnostic.elf`.
Not flashed/run. Installed firmware still one-shot reverse 10-turn.
Operator must still confirm standstill/cooldown between runs; no automatic
stillness interlock was added.

## Cleanup: run-counter rename + SRP guard helpers (not installed)

Renamed `test_drive_used` to `test_drive_run_count` across firmware and all
five host suites. Split `TestDriveSafetyTick` into one static helper per stop
condition (fault/speed/Hall-invalid/stall/count/current/timeout) with the
orchestrator preserving the original evaluation order and stop-reason
priority. All suites pass. Rebuilt repeatable reverse ELF into
`/tmp/opencode/esc-ne-reverse-repeatable/`. Not flashed/run.
Remaining SOLID note: profiles still fan out through `#ifdef` chains across
`test_profile.h`, `drive_parameters.h`, `build_cli.py`, the Hall hook and the
suites; a table-driven profile config would close that, left as follow-up.
Still required between runs: operator-confirmed standstill/cooldown, fresh ARM
per start, no stillness interlock added.

## First confirmed CAN speed run (10 reverse turns, 2026-10-14)

Repeatable ESC build (run_count, no one-shot) flashed to front-right probe
`066AFF525266555067075126`, HOTPLUG CAL1 to result 2/valid 1. Nucleo sender
from `sfurs_firmware_v2` initial commit (used_mask latch, cleared by reset).
Findings: normal-mode CLI connects reset the target (wipes RAM); all ESC RAM
access must use `mode=HOTPLUG`. A broken CAN wire caused first ARM to fail
(state FAULT, result 5, first_fault 2, tx 4/ack 0); wire replaced.
First CAN RUN with wheel stuck on standing box: Nucleo RUNNING ~14 s, 5/5 ACK,
no fault; ESC side unverified (stall stop assumed). Wheel freed, ESC
recalibrated (replugging ST-LINK USB resets ESC RAM, wiping prior evidence),
Nucleo reset, re-ARM/RUN: RUNNING ~14 s, tx/ack 5/5, IDLE result 3.
User observed ~10 reverse turns through CAN speed frames (-60 RPM profile).
ESC RAM evidence lost again on cable replug (all zero); Nucleo trace + visual
count stand as the record. Next powered run needs fresh CAL1.

## Range unlock -120..-30 (2026-10-14) + CAL saga lesson

ESC exact-match gate replaced by bounded reverse-only range (forward still
rejected: negated gains would run away). +60 profile keeps exact match via
MIN==MAX==TARGET. Host suites updated (range accepts, edges rejected) and
pass; sender follows the setting (0 keeps -100 default), TUI DEF-aware.
ELFs: `/tmp/opencode/esc-ne-reverse-range/` (md5 ba3f1046...).

CAL saga: the range build deterministically rejected CAL (result 3) across
several retries including a 10 s power cycle, while the previous exact build
calibrated first try -- looked like firmware guilt. Reproducible bytes
(md5-identical rebuild) plus a later clean CAL on the SAME range image
cleared it: insufficient rail decay / stuck driver fault, not source.
Lesson: after persistent CAL reject, power off for MINUTES (a flash cycle
provides this), not seconds, before blaming firmware.

## 300 RPM reverse validated via TUI (2026-10-14)

Unlocked +-300 build (ESC md5 49d0c14b, sender 21,592 B) installed both
sides, ESC CAL 2/1 on fresh 3S. Operator stepped 150 (subtle) then 300 RPM
reverse through the keyboard TUI: fast smooth spin, count-stop as expected,
no runaway, no fault. Forward (+ range) and other-wheel tuning still open.

## NE forward validated at +300 via TUI (2026-10-14)

Bidirectional +-300 range with negated gains holds forward too: operator ran
NE forward through the keyboard TUI up to +300 RPM, smooth spin, count-stop,
no runaway, no fault. The sign-stability argument is now hardware-confirmed
both ways. Same guards throughout (300 RPM / 3 A / stall / 480-edge / 30 s).
