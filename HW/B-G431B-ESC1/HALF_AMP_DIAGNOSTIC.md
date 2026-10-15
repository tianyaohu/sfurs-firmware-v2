# Front-right NE: guarded torque diagnostic history

## Installed 1.5 A / 2-second diagnostic

User explicitly requested 1.5 A after reporting a quarter-turn on the 0.5 A
attempt. Normal repository IQMAX_A/NOMINAL_CURRENT_A are indeed 3 A; that is a
reference setting, not independent current calibration or safety validation.
Prepared opt-in profile sets reference and sampled L1 STOP threshold to 1.5 A
(816 SDK counts nominal), retaining 200 ms ramp, 2000 ms local timeout, 120 RPM
sampled speed guard, calibration/fault gates and one-shot lockout. Both reference
and sampled-current threshold increase; no guaranteed physical-current ceiling.
Legacy `--half-amp-diagnostic` option/macro names remain for build compatibility.
Old-profile and torque guard host tests pass; mock torque bounds now follow the
profile, and updated tests check 816 accepted/817 rejected, sampled-current
boundary, direct fixed reference, deadline/tick wrap and existing gates.
ELF `/tmp/opencode/esc-ne-1p5a-direct-2seconds/motor_NE_half_amp_diagnostic.elf`:
FLASH=40,220 bytes, RAM=8,712 bytes. User subsequently authorized flashing
with the previously confirmed motor battery disconnected. Detected expected
front-right probe `066AFF525266555067075126`; download, verification and software
reset succeeded. Re-resolved ELF symbols remain unchanged. HOTPLUG readback:
calibration/request/result/armed/active/used variables all zero, MCI IDLE with
zero current/latched faults at that sample, nonzero SysTick. No CAL1/ARM/RUN
issued after programming. Repeat powered calibration before any authorized run.

### First authorized 1.5 A-reference pulse

User confirmed battery reconnected; CAL1 completed (result=2/valid=1), IDLE,
fault-free/disarmed/unused. After explicit authorization for one 1.5 A pulse,
DARM written once, armed readback verified, DRUN written exactly once. No retry.
Log `/tmp/opencode/front-right-direct-1p5a-20261012-212817.jsonl`: 105 post-write
samples, states IDLE/RUN/STOP, all six raw Hall input states 1..6 observed.
Final IDLE, zero faults, inactive/disarmed, used=1, start accepted result=3.
Stop reason=5: sampled speed guard (magnitude >=120 motor RPM), not current or
the 2-second timer. Sampled reference maximum 405 SDK counts; faster local peak
L1=559 SDK counts. Nominal conversion is not independent physical-current proof.
No further pulse/reset issued. Physical movement/stopping await user observation;
requested battery disconnection.

The following sections record earlier profiles and tests, not the installed limit.

Source-based build subsequently flashed/verified; NOT run. Physical front-right probe
`066AFF525266555067075126`, NE bit 1. Original recovery image belongs to this
probe despite historical filename:
`/home/fh/sfurs_firmware/esc-backups/rear-right-original-20261012.bin`.
Keep the original backup unchanged. Latest user-authorized reference cap: 0.5 A,
not 3 A. Keep motor battery disconnected during programming.

## Scope and safeguards

- Opt-in `--half-amp-diagnostic`, NE-only; separate from `--low-current-test`.
- Sets IQMAX_A and NOMINAL_CURRENT_A to 0.5 A for generated speed-PI and STC
  positive/negative torque-reference bounds. Normal builds retain original 3 A
  settings; old test profile retains 0.25 A.
- Uses existing guarded SDK start permit, UART-control exclusion, no boot start,
  zero default speed target, and explicit calibration-only CAL1 request.
- Calibration must complete successfully, offsets be valid, faults zero and state
  IDLE before ARM/drive. No automatic calibration/arming/retry/reverse.
- One drive attempt per ESC boot, even if startup rejected.
- ESC SysTick independently requests STOP at 750 ms from accepted command,
  sampled abs(iq)+abs(id) >=0.5 A, measured motor speed magnitude >=120 RPM,
  or calibration lost/current fault present. Does not depend on Nucleo STOP.
- Original Hall placement, pole pairs, commutation and voltage-sensing scaling
  retained from local repository. Existing guarded profile 18 V overvoltage
  setting retained; real battery voltage/current scaling still needs validation.

These are software/reference limits, NOT instantaneous phase-current protection
or guaranteed physical current/speed ceilings. Torque control can accelerate a
free wheel quickly. Sampled guards can miss peaks/transitions or be wrong if
sensing/scaling is wrong. Local timeout relies on functioning ESC firmware.
Secure the robot, clear all wheels, keep immediate battery cutoff accessible.
Do not conduct a powered test merely because an offline build passes.

## Deliberately incompatible with the old sender

Current Nucleo physical-button sender sends speed ID 0x081; this new diagnostic
REJECTS it. Existing legacy torque ID 0x181 also rejected. Old TEST arm magic
0x54455354 rejected. This prevents accidentally starting the new torque profile
with an old speed sequence.

ARM: ID 0x401, little-endian uint32 magic **0x48414C46**, expires in five seconds.
After calibrated/idle/fault-free and ARM confirmed, diagnostic torque command is
exact ID **0x501**, four-byte payload: signed SDK q-current counts (positive only)
and uint16 ramp milliseconds. Bounds 1..floor(0.5*CURRENT_CONV_FACTOR), ramp
200..500 ms. With current factor 544, maximum 272 counts (payload
`10 01 C8 00` for maximum reference and 200 ms ramp). A positive motor reference
does not establish physical rotation direction. STOP ID 0x781 or all-wheel
0x78F remains accepted. Reference/time bounds cannot be refreshed by new frames.

Do not reuse original ESC memory addresses; derive symbols from new ELF. Existing
speed/Hall telemetry 0x101 -> 0x281 and calibrated current telemetry
0x301 -> 0x381 retained. Added debugger-visible iq/id references, measured RPM,
variables. The unreferenced reference-limit-mA variable was linker-discarded;
use the profile definition and verified STC limits instead. Invalid measured currents remain zero and
must not be interpreted as measured zero physical current.

Next steps require explicit flash authorization, recovery-backup verification,
post-flash identity/disarmed checks, powered calibration without startup and a
matching diagnostic sender/monitor. No stronger command should be sent to the
original firmware as a substitute for installing these limits.

## Reproduce and tests

From `motor_config` with toolchain in PATH:

```sh
python3 test_half_amp_guards.py
python3 build_cli.py --motor NE --half-amp-diagnostic --output /tmp/opencode/esc-ne-half-amp-diagnostic
```

ELF `/tmp/opencode/esc-ne-half-amp-diagnostic/motor_NE_half_amp_diagnostic.elf`.
Build succeeded: FLASH 39,932 bytes, RAM 8,704 bytes. Host tests extract actual
guard code and cover old profile regression, wrong/legacy commands, bounds,
calibration/fault rejection, one-shot, rejection after failed start, CAN STOP,
current/speed/calibration-loss/time guards and wraparound. Normal NE build also
succeeded (FLASH 43,896 bytes/RAM 8,664 bytes). Tests are offline logic checks,
not evidence of physical motor function or safety.

## Flash outcome

User authorized programming and confirmed motor battery off. Original recovery
backup length/hash verified before download. Detected physical front-right probe
`066AFF525266555067075126`; ELF download, verification and reset succeeded.
Post-reset HOTPLUG reads confirm NE hardware filter `0x88010001`, all calibration/
arm/active/used/current-valid/reference/peak/stop-reason variables zero. MCI at
`0x200000D4` shows FAULT_NOW with current/latched undervoltage=4, expected with
motor battery off. Live STC torque reference bounds are +272/-272 SDK counts
(0.5 A nominal conversion), replacing original limits. No CAL1/ARM/drive written.
Existing Nucleo speed/button sender remains incompatible with this new profile.

Installed ELF symbols (resolve again after rebuild): calibration request
`0x200008D0`, result `0x200008D4`, validity `0x200008DC`, active `0x200008E4`,
armed `0x200008EC`, current-valid `0x200008F0`, id `0x200008F4`, id-reference
`0x200008F8`, iq `0x200008FC`, iq-reference `0x20000900`, measured RPM
`0x20000904`, peak L1 `0x20000908`, start tick `0x20000910`, stop reason
`0x20000914`, used `0x20000918`. Stop reasons: 1 timeout, 2 CAN STOP,
3 rejected startup, 4 sampled current, 5 sampled speed, 6 fault/calibration loss.

## 2-second direct diagnostic (now installed)

User requested extending the direct pulse maximum to 2 seconds. Changed only
the half-amp profile local timeout from 750 to 2000 ms; reference/ramp, sampled
current and speed stops, calibration/fault gates and one-shot latch are unchanged.
This does not guarantee a full 2-second run or a physical current ceiling.
Host old/half-amp guard tests pass, including deadline boundary and tick wrap.
Prepared ELF: `/tmp/opencode/esc-ne-half-amp-direct-2seconds/motor_NE_half_amp_diagnostic.elf`.
Build FLASH=40,220 bytes, RAM=8,712 bytes. Subsequently flashed after explicit
user permission and motor-battery-off confirmation to front-right probe
`066AFF525266555067075126`. Download, verification and software reset succeeded.
Re-resolved ELF symbols: calibration/direct/drive addresses below unchanged.
HOTPLUG readback shows calibration/request/result/armed/active/used variables
all zero, MCI IDLE with current/latched faults zero at that sample, and nonzero
SysTick. Battery status is operator-confirmed, not inferred from that fault sample.
Calibration must be repeated after programming/reset. No ARM/RUN issued.

### First authorized 2-second-maximum direct pulse

After user-confirmed battery reconnection, preflight IDLE/fault-free/unarmed/
unused; one CAL1 completed with result=2/valid=1. User explicitly authorized
one pulse. DARM written once and armed state verified; DRUN written exactly
once, no retries. HOTPLUG log
`/tmp/opencode/front-right-direct-2seconds-20261012-212412.jsonl` contains write
outcomes and 104 post-write samples over approximately four seconds.
States observed IDLE/RUN/STOP; final IDLE, faults zero, inactive/disarmed,
used=1, result=3. STOP reason=4 (sampled current guard), peak L1=275 SDK counts:
the guard stopped this attempt before the 2-second timeout. Raw Hall input
states 2/6 observed; actual wheel motion/stopping require operator observation.
No guard bypass, extra RUN or reset issued. Disconnect battery before further work.

## Installed direct-debugger extension (supersedes addresses above)

User authorized adding/flashing direct diagnostics with battery still off.
Shared `TestDriveArm`/`TestDriveStart` enforce the same CAN and direct guards.
Main-loop `TestDirectService` consumes each request once; none are issued at boot.
Write only via HOTPLUG; do not halt firmware during a powered attempt. Do not
blindly retry a write reported failed: first inspect consumed request/result.

ELF `/tmp/opencode/esc-ne-half-amp-direct-diagnostic/motor_NE_half_amp_diagnostic.elf`
built (FLASH 40,220 bytes/RAM 8,712 bytes), downloaded, verified, reset. Recovery
backup hash reverified. Post-reset request/result/last-request and all drive/
calibration variables zero; NE filter unchanged, live torque bounds +/-272,
expected battery-off undervoltage. No calibration, ARM or RUN issued.
Host old/half-amp guard suites pass, including direct unarmed RUN/unknown request
rejection, calibration/fault gates, explicit arm/start, no replay/repeat, STOP,
and arm expiry. A powered test remains separately authorized.

Current debugger symbols:

- CAL1 request `0x200008D0` (magic `0x43414C31`); result `0x200008D4`,
  calibration-valid `0x200008DC`.
- Direct last-request `0x200008E4`, request **`0x200008E8`**, result `0x200008EC`.
  DARM=`0x4441524D`, DRUN=`0x4452554E`, DSTP=`0x44535450`.
  Results 1 armed, 2 rejected, 3 start accepted, 4 SDK start rejected, 5 STOP
  requested. Acceptance is not measured motion or confirmed physical stopping.
- Active `0x200008F0`, armed `0x200008F8`, current-valid `0x200008FC`;
  id `0x20000900`, id-reference `0x20000904`, iq `0x20000908`, iq-reference
  `0x2000090C`, measured RPM `0x20000910`, peak L1 `0x20000914`;
  start tick `0x2000091C`, stop reason `0x20000920`, used `0x20000924`.
- MCI `0x200000D4`, STC `0x20000248` unchanged; use local source layouts.

DRUN fixes positive reference to floor(0.5*CURRENT_CONV_FACTOR)=272 counts with
200 ms ramp, 750 ms local timeout. Cannot start without successful CAL1 and
separate DARM. Existing Nucleo button speed command remains rejected. For a
CAN-independent bench test keep Nucleo unpowered; leave front-right data USB
connected. Confirm wheels clear/stationary and cutoff ready before powered
calibration, then obtain explicit authorization for the single torque pulse.

### Powered calibration outcome

User confirmed all bench conditions ready and subsequently battery connected.
Initial HOTPLUG snapshot: FAULT_OVER=11, current faults zero, remembered UV=4,
all calibration/direct/drive variables zero. First CAL1 write consumed despite
CLI exit 1; calibration result=3 (rejected), but MCI transitioned to IDLE with
both fault fields zero. Fault acknowledgement is asynchronous, so same-call
IDLE check can reject during that transition. After confirming IDLE, a second
calibration-only CAL1 request was issued (not a RUN retry). CLI again reported
download error; readback confirms request consumed, result=2 (done), validity=1,
IDLE and both fault fields zero. ARM/active/used/direct requests/results remain
zero. Firmware offset-valid flag is not independent validation of physical
current scaling. No DARM/DRUN or CAN drive command sent. Physical observation
and separate single-pulse authorization pending.

### First authorized direct positive-reference pulse

User confirmed all wheels stationary and explicitly authorized one direction
pulse. Sequential HOTPLUG preflight verified IDLE, zero current/latched faults,
calibration done/valid, no used/active/armed/request flags. DARM written once and
armed readback confirmed before DRUN written exactly once. No CAN used or retry.
Log `/tmp/opencode/front-right-direct-pulse-20261012-211603.jsonl` contains raw
write outcomes and 80 post-write snapshots over approximately three seconds.
States seen IDLE/RUN/STOP; raw Hall inputs 4,5,6 (previous baseline 5), suggesting
rotor displacement but not independent proof of sustained wheel rotation.
Sampled iq reference 0..182 SDK counts; iq 0..185, id -6..10; speed getter stayed
zero. Faster local sampled peak abs(iq)+abs(id)=274 SDK counts, approximately
0.504 A nominal conversion (factor 544); guard STOP reason=4, before the local
timeout rather than confirmed full 750 ms running. Peak is not a guaranteed
instantaneous physical-current ceiling. Last snapshot IDLE, zero faults,
active/armed/current-valid=0, direct result=3 (start accepted), used=1. No further
attempt permitted this boot; physical wheel observation pending.
