# Short front-right RPM validation (installed)

## Installed first speed-PI tuning trial

User confirmed motor battery off after requesting properly checked 60-100 RPM
tuning. Prepared first stage at 60 RPM: opt-in RPM-profile gains Kp=18600/1024,
Ki=72/16384 (8x original). Current PI, units, divisors, anti-windup limits and
3 A nominal reference/sampled-current thresholds unchanged. Normal, old low-
current and torque builds do not receive the gain override. Retains 500 ms ramp,
10-second hard timeout, 2-second no-valid-Hall-progress stop, invalid-Hall stop,
120 motor-RPM guard, calibration/fault gates and one-shot lockout. No 300-turn
endpoint enabled. This is an EXPERIMENTAL starting gain set, NOT validated tuning.

Actual SDK PI host check `test_speed_pi_response.py --trial` passes with UBSan
and nominal +/-3 A saturation. Stationary-feedback model predicts 60 RPM current
references ~0.349/0.471/1.118 A at 0.5/2/10 seconds; stall guard stops a no-progress
attempt at 2 seconds, so the 10-second stationary model is analytical only.
The model does not establish motor-loop stability or physical current calibration.
RPM, torque and old-profile guard tests pass. ELF data at speed PI handle
0x20000154 confirms compiled default gains 18600/72 and unchanged divisors/limits.
Prepared `/tmp/opencode/esc-ne-60rpm-pi-trial/motor_NE_rpm_diagnostic.elf`:
FLASH=40,564 bytes, RAM=8,728 bytes. Subsequently flashed with explicit user
authorization and battery-off confirmation to expected front-right probe
`066AFF525266555067075126`. Download, verification and reset succeeded.
HOTPLUG readback confirms default/live PI gains=18600/72 with unchanged limits,
zero calibration/request/armed/active/used/Hall diagnostics, monitor-ready=1,
nonzero SysTick and expected battery-off undervoltage faults. No CAL1/ARM/RUN.
Repeat powered calibration and obtain separate single-run authorization.
Measure tracking/oscillation, Hall
progress and iq reference/feedback before attempting the next speed/gain stage.

Current trial symbol addresses (re-resolved): CAL1/direct request fields through
0x200008FC unchanged; Hall edges=0x20000900, invalid=0x20000904,
last-edge tick=0x20000908, net edges=0x2000090C, id=0x20000910,
id-reference=0x20000914, iq=0x20000918, iq-reference=0x2000091C,
RPM=0x20000920, peak L1=0x20000924, start tick=0x2000092C,
stop reason=0x20000930, used=0x20000934, SysTick=0x20001C10.
Do not reuse earlier RPM field addresses after 0x20000904.

Other prepared-profile sections below are historical and superseded by this trial.

## Speed-loop arithmetic audit (no tuning writes or motor commands)

Read-only HOTPLUG speed PI handle at 0x20000154 confirms default/live Kp=2325,
Ki=9, divisors 1024/16384, shifts 10/14, integral limits +/-26,738,688 and
output limits +/-1632 SDK counts (nominal 3 A). Source loop frequency=1000 Hz,
SPEED_UNIT=10 (0.1 Hz): 60 RPM maps to 10 units; 100 RPM truncates to 16 units,
i.e. 96 RPM. Feedback therefore has 6 RPM numerical quantization at this API.

New offline `test_speed_pi_response.py` compiles the actual SDK PI function/type
and exercises stationary-feedback, 500 ms ramp response with UBSan and output
saturation tests. At 60 RPM: 23/31/75 reference counts at 0.5/2/10 seconds
(~0.042/0.057/0.138 A nominal). At requested 100/encoded 96 RPM: 38/51/121
counts (~0.070/0.094/0.222 A). The 31-count result matches prior live telemetry.
This establishes that the existing speed PI requests little current at low
speed, not that the current loop/power stage cannot deliver it. It does not
validate mechanical torque, current calibration or closed-loop stability.
No gain changes, flash, calibration, reset, ARM or RUN issued during this audit.
Physical gain tuning still requires a separately authorized, instrumented trial,
valid Hall feedback and immediate cutoff; do not claim validated new gains based
only on a stationary arithmetic simulation.

## Prepared 100 RPM / 10-second validation (not installed)

Following barely perceptible movement, user requested 100 RPM and a 10-second
backup timeout. Prepared RPM profile: fixed target 100 RPM, 500 ms ramp,
3 A reference/sampled-L1 threshold, existing 120 motor-RPM speed guard, Hall
invalid-transition stop and new 2000 ms no-valid-Hall-progress stop (reason 9),
including startup. No speed-PI tuning change. No 300-turn endpoint enabled while
counting remains unvalidated; 300 turns at 100 RPM require about 180 seconds and
cannot complete within the requested 10-second cap.
Prior RUN samples show maximum iq reference 31 SDK counts (~0.057 A nominal),
not a 3 A demand. Source speed PI is Kp=2325/1024, Ki=9/16384 in default SDK
speed units, explaining weak proportional demand at low targets and slow
integral buildup. This is a plausible torque limitation, not physical current
calibration or exclusion of sensing/mechanical/commutation problems.
Tests for RPM, torque and old profile pass, including stall boundaries and wrap.
ELF `/tmp/opencode/esc-ne-100rpm-10seconds-validation/motor_NE_rpm_diagnostic.elf`:
FLASH=40,564 bytes, RAM=8,728 bytes. Not flashed/run. Installed profile still
60 RPM / 2000 ms; await battery-off confirmation and explicit flash permission.
Re-resolve addresses before operating the replacement. Repeat powered calibration.

The remainder records the installed 60 RPM version and its test history.

## First authorized short test outcome

After user-confirmed battery reconnection, first CAL1 acknowledged cleared
undervoltage but rejected calibration during the asynchronous idle transition;
readback verified IDLE/fault-free before a second CAL1 completed (result=2/valid=1).
User authorized one RPM test. Clean preflight, one DARM with RPM1 armed readback,
then exactly one DRUN; no retry. Log
`/tmp/opencode/front-right-60rpm-validation-20261012-213616.jsonl` records writes
and 104 post-write samples, including 52 RUN samples. States IDLE/RUN/STOP;
all sampled RUN speeds zero, valid/invalid/net Hall counters all zero, raw Hall
inputs remained 5. Stop reason=1 (2000 ms timeout), peak L1=146 SDK counts.
Final IDLE, zero faults, inactive/disarmed, used=1, direct result=3.
This does NOT validate 60 RPM tracking or revolution counting. Physical motion
observation pending; no 300-revolution run, reset or second start authorized.
Disconnect battery before diagnosis or changes.

Requested eventual target: 300 wheel revolutions at 60 RPM, user-reported 1:1
gearing, 3 A reference cap. User agreed to validate speed/counting with a short
test first. This firmware DOES NOT implement or authorize the five-minute run.

Opt-in `build_cli.py --motor NE --rpm-diagnostic` uses the existing calibration,
explicit ARM/RUN, SDK start-permit, UART exclusion and one-shot fault gates.
Direct DRUN requests exactly +60 motor RPM with 500 ms ramp. Speed PI/STC
reference limits are nominal +/-3 A; sampled L1-current STOP threshold is 3 A.
Local SysTick STOP deadline is 2000 ms from start request; speed magnitude
>=120 motor RPM, faults or lost calibration also stop. Reference/current/speed
values are not independently validated physical ceilings.

ARM magic is RPM1 (`0x52504D31`), distinct from torque/old-speed profiles. Existing
direct request words DARM/DRUN/DSTP remain; derive addresses from the new ELF.
New IRQ-fed Hall diagnostics count valid adjacent SDK sector transitions:
positive 5->1->3->2->6->4->5, reverse subtracts net count; total valid edges and
invalid transitions are separate counters. Invalid transitions latch an early
STOP (reason 7). Counters reset only at a new authorized start, not each IRQ.
Polling-based Hall telemetry is not used for revolution counting. Counts cover
the active command interval, not all coasting after STOP. Count-based stopping,
stall detection and long-run timeout are NOT implemented in this short build.

Source specifies 8 pole pairs: 48 valid Hall sectors/mechanical turn; with true
1:1 gearing, 300 wheel turns would require 14,400 net sectors. Physical pole pairs,
direction, transitions and speed tracking must be checked before relying on this.
This speed-control build must not be treated as precise position control.

Offline tests `python3 test_rpm_guards.py` pass for fixed speed/ramp, profile
limits, Hall forward/reverse/invalid sequences, one-shot, calibration/fault/
current/speed stops, manual STOP, expiry, rejected start and timeout/tick wrap.
Existing low-current and torque-profile tests also pass.
ELF `/tmp/opencode/esc-ne-60rpm-validation/motor_NE_rpm_diagnostic.elf`:
FLASH=40,356 bytes, RAM=8,728 bytes. Subsequently installed with user-authorized
flash and previously confirmed motor battery disconnected on expected front-right
probe `066AFF525266555067075126`. Download, verification and reset succeeded.
HOTPLUG readback: calibration/request/result/armed/active/used/Hall counters zero,
monitor-ready=1, nonzero SysTick; battery-off undervoltage current/latched=4.
No calibration, ARM or RUN written. Powered calibration and separate short-run
authorization remain required.

Re-resolved ELF symbols: CAL1 request=0x200008D0, result=0x200008D4,
valid=0x200008DC; direct last/request/result=0x200008E4/E8/EC;
active=0x200008F0, armed=0x200008F8; Hall valid/invalid/net=0x20000900/904/908;
iq=0x20000914, iq-reference=0x20000918, RPM=0x2000091C,
peak L1=0x20000920, start tick=0x20000928, stop reason=0x2000092C,
used=0x20000930, SysTick=0x20001C0C. Earlier torque diagnostic field addresses
after 0x20000900 must not be reused. MCI=0x200000D4 unchanged.

## First tuning-trial outcome (reverse-direction runaway, guard stopped)

Authorized single 60 RPM trial after calibration (log
`/tmp/opencode/front-right-60rpm-pi-trial-20261012-214824.jsonl`): one DARM/RPM1
arm verified, exactly one DRUN, no retry. 20 RUN samples then STOP/IDLE; final
IDLE, faults zero, inactive/disarmed, used=1, result=3. Stop reason=5 (sampled
speed magnitude >=120). RUN telemetry progressed iq reference 0..384 SDK counts
(~0..0.71 A nominal), measured iq tracking reference, peak L1=545 SDK counts.
Measured speed stayed 0 through most of RUN, then -60; STOP samples -144..-78
before returning to 0. Valid Hall edges=7, invalid=0, net=-7 (reverse by our
positive 5->1->3->2->6->4->5 convention). Thus the wheel drove opposite the
requested positive direction and accelerated until the 120 motor-RPM guard
stopped it. This is NOT validated tuning and does NOT support a 300-turn run.
Direction/feedback sign, Hall order, phase wiring and speed-loop stability must
be diagnosed with battery disconnected. No second start/reset issued.
