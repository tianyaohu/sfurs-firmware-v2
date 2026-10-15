# Front-right torque turn-count (installed)

Requested: 300 wheel revolutions in the motor's natural (reverse) direction,
user-reported 1:1 gearing. Source pole pairs=8 => 48 valid Hall edges per
mechanical turn => 14,400 net edges for 300 turns.

Opt-in `build_cli.py --motor NE --torque-count-diagnostic`:
- Fixed positive 1.5 A nominal torque reference (816 SDK counts), 200 ms ramp.
  Positive reference previously drove reverse motion; count stop triggers on
  either sign magnitude, expecting negative net.
- IQMAX/nominal PI bounds 3 A retained; sampled L1 STOP threshold 3 A.
- Local STOP on abs(net edges) >= 14,400 (reason 8), invalid Hall (reason 7),
  2 s no-progress stall (reason 9), speed magnitude >=300 RPM (reason 5),
  faults/calibration loss (reason 6), 180 s backup timeout (reason 1).
- Existing calibration, explicit ARM (CNT1 0x434E5431)/RUN, one-shot, UART
  exclusion unchanged. No speed-loop tuning involved; avoids prior reverse
  runaway instability by using open-loop torque.
- Reference/current/speed/count values are firmware nominal, not independently
  calibrated physical guarantees. Long run requires continuous cutoff access,
  clear wheels, Nucleo unpowered, and monitoring; 180 s energized run is higher
  risk than prior short pulses.

Offline `test_torque_count_guards.py` passes, plus existing RPM/torque/old
suites. ELF `/tmp/opencode/esc-ne-torque-count-300turns/motor_NE_torque_count_diagnostic.elf`:
FLASH=40,516 bytes, RAM=8,728 bytes. Flashed with explicit user permission and battery-off confirmation to expected front-right probe `066AFF525266555067075126`. Download, verification and reset succeeded. HOTPLUG readback: all calibration/request/armed/active/used/Hall fields zero, monitor-ready=1, nonzero SysTick, expected battery-off undervoltage. No CAL1/ARM/RUN written. Await battery-off confirmation and explicit flash permission;
re-resolve symbols, repeat calibration, then separate single-run authorization.

## First 300-turn attempt outcome (early speed-guard stop)

After powered calibration (result=2/valid=1), clean preflight and explicit user
authorization, one DARM/CNT1 arm verified then exactly one DRUN; no retry. Log
`/tmp/opencode/front-right-torque-count-300turns-20261012-215515.jsonl`: RUN at
first post-write sample, then IDLE by next 0.5 s sample. Final IDLE, faults zero,
inactive/disarmed, used=1, result=3. Stop reason=5 (sampled speed magnitude
>=300), peak L1=669 SDK counts (~1.23 A nominal). Valid Hall edges=11,
invalid=0, net=-11 (reverse, ~0.23 turn), far short of 14,400 target. Post-stop
sampled speed -132 RPM decaying to 0 indicates rapid acceleration past the guard
within <0.5 s; coasting after STOP is not counted (counting gates on active).
This confirms free-wheel acceleration under 1.5 A is too aggressive for a
count-to-300 open-loop run with a 300 RPM guard. No second start/reset issued.
Physical motion observation pending; disconnect battery before next steps.

## Installed 1000 RPM guard

User requested 1000 RPM guard after 300 RPM early stop. Changed only
ESC_TEST_MAX_MOTOR_RPM 300->1000 for the torque-count profile; 1.5 A reference,
3 A IQMAX/sampled threshold, 14,400-edge target, 2 s stall, 180 s backup,
calibration/fault/one-shot gates unchanged. Offline guards pass. ELF
`/tmp/opencode/esc-ne-torque-count-1000rpm-guard/motor_NE_torque_count_diagnostic.elf`:
FLASH=40,516 bytes, RAM=8,728 bytes. Flashed with explicit user permission and battery-off confirmation to expected front-right probe `066AFF525266555067075126`. Download, verification and reset succeeded. HOTPLUG readback: calibration/request/armed/active/used/Hall fields zero, monitor-ready=1, nonzero SysTick, MCI IDLE with zero faults at that sample. No CAL1/ARM/RUN written. Repeat powered calibration before any authorized run.

## 1000 RPM guard attempt outcome (speed-guard stop at 858 RPM)

Authorized single 300-turn run after calibration. One CNT1 arm verified, one
DRUN, no retry. Log
`/tmp/opencode/front-right-torque-count-1000guard-20261012-220235.jsonl`: 11
samples over 5.5 s, states RUN/STOP/IDLE. Final IDLE, faults zero,
inactive/disarmed, used=1. Stop reason=5 (speed >=1000), max sampled 858 RPM,
peak L1=936 SDK counts (~1.72 A nominal). Valid edges=76, invalid=0, net=-76
reverse (~1.58 turns), far short of 14,400. Raising 300->1000 extended powered
travel 11->76 edges but free-wheel steady speed under 1.5 A still exceeds the
guard. Further guard raises approach rated speeds with higher kinetic risk;
reducing torque to hold speed under guard is the safer next variable. No second
start/reset. Disconnect battery; physical observation pending.

## Prepared 0.8 A slower observable run (not installed)

User missed brief 5.5 s guard-stop and requested rerun with battery off
confirmed. Prepared lower 0.8 A nominal reference (435 SDK counts), retaining
3 A IQMAX/sampled threshold, 14,400-edge target, 1000 RPM guard, 2 s stall,
180 s backup and existing gates. Goal is sustained sub-guard speed for an
observable multi-turn run instead of immediate overspeed trip. Offline guards
pass. ELF `/tmp/opencode/esc-ne-torque-count-0p8a/motor_NE_torque_count_diagnostic.elf`.
Not flashed/run; installed still 1.5 A/1000 RPM guard. Await explicit flash
permission with battery off.

## Installed 0.8 A observable run

Flashed with explicit user permission and battery-off confirmation to expected
front-right probe `066AFF525266555067075126`. Download, verification and reset
succeeded. HOTPLUG readback: calibration/request/armed/active/used/Hall fields
zero, MCI IDLE zero faults at that sample, nonzero SysTick. No CAL1/ARM/RUN.
Repeat powered calibration before any authorized run.

## 0.8 A attempt outcome (134 edges, speed-guard stop)

Authorized single run after calibration. One CNT1 arm, one DRUN. Log
`/tmp/opencode/front-right-torque-count-0p8a-20261012-220634.jsonl`: 11 samples
over 5.5 s, RUN/STOP/IDLE. Final IDLE, faults zero, used=1. Stop reason=5
(speed >=1000), max sampled 954 RPM, peak L1=560 SDK counts (~1.03 A nominal).
Valid edges=134, invalid=0, net=-134 reverse (~2.79 turns). Lower torque extended
travel 76->134 edges versus 1.5 A but still reached guard speed. No second start.
Disconnect battery; observation pending.

## Watched 0.8 A rerun (133 edges, speed-guard stop)

User watching with battery reconnected after reset/calibration. One CNT1 arm,
one DRUN while observing. Log
`/tmp/opencode/front-right-torque-count-rerun-20261012-220933.jsonl`: 11 samples
over 5.5 s. Final IDLE, faults zero, used=1. Stop reason=5 (speed >=1000), max
sampled 948 RPM, peak L1=553 SDK counts. Valid edges=133, net=-133 reverse
(~2.77 turns). No second start. Observation and battery disconnect pending.
