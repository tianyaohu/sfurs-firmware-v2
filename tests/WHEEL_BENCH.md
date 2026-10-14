# Original-firmware wheel bench sender

Prepared offline; installing this ELF does not authorize powering or driving.
Subsequently flashed to Nucleo `002200364142500620353451` with user-confirmed
battery disconnected. Download, verification and reset succeeded. HOTPLUG reads
confirmed all six bench command/state variables zero (idle, unselected,
unarmed, unused), and a nonzero SysTick. No commands were triggered.
Build memory: FLASH 20,428 bytes; RAM 1,880 bytes. CM4/option bytes unchanged.

## First powered attempt — blocked before RUN

After user confirmed battery connected and all four wheels stationary, preflight
readback unexpectedly showed state=4 (latched fault), result=5 (transport fault),
request/wheel/direction/used-mask all zero. No ARM or RUN was written. Source of
the earlier fault is unresolved; initial post-flash readback had been all zero.
Issued only explicit all-wheel STOP trigger. CLI write returned its known
download-error response; subsequent readback showed request consumed, three
STOP attempts completed, state still fault/result=5 and used-mask=0. Successful
STOP delivery is NOT confirmed. Requested battery disconnection; no reset or
fault bypass performed. Powered drive test must remain blocked pending diagnosis.

### Battery-off diagnosis

Correct H755 FDCAN1 registers (base 0x4000A000): ECR=0x00130095,
PSR=0x00000768, TXBTO=0x38, TXBCF=0x07, TXBRP=0.
TEC=149, REC=0; error-passive/warning set, not bus-off. Six TX message-RAM
elements starting 0x4000AE40 contain only ID 0x78F, four zero payload bytes:
slots 0-2 canceled, slots 3-5 transmitted/ACKed. Thus the later STOP sequence
did reach the bus; this still does not prove all ESCs stopped. Earlier reads at
0x4000A418/0x4000A444 concern FDCAN2, not FDCAN1; do not use those to diagnose
this sender. The trigger/source of the first STOP sequence is unresolved.

A diagnostic build was prepared and host-tested after this finding, then flashed
with user authorization and battery disconnected after reported reconnection of
a loose pin. Download, verification and reset succeeded. Readback confirmed all
request/state/diagnostic variables zero, no pending/successful/canceled TX bits,
and nonzero SysTick. No triggers were written. User reports no independent CLI
or debugger commands; original STOP trigger source remains unresolved. Loose
connection is a plausible transmission-error cause, not a confirmed root cause.
It adds last request/request count, queued TX/ACK counters, first fault reason/
tick, and captured CAN registers. All drive/STOP timings and fault gates unchanged.
FLASH=20,652 bytes, RAM=1,928 bytes. Re-resolve symbol addresses before flashing
or operating a later ELF; earlier addresses describe the prior build.

Current diagnostic ELF: request `0x2000012C`, wheel `0x20000130`, direction
`0x20000134`, state `0x20000138`, result `0x2000013C`, used-mask `0x20000140`,
last-request `0x20000144`, request-count `0x20000148`, TX-count `0x2000014C`,
ACK-count `0x20000150`, first-fault `0x20000154`, first-fault-tick `0x20000158`.

### Diagnostic STOP-only check and sender completion race fix

User confirmed battery powered, stationary wheels and cutoff ready. Preflight
all diagnostics zero, ECR=0. Issued one STOP trigger, no ARM/RUN.
Postflight: request-count=1, last-request=STOP, TX-count=3, ACK-count=2,
first-fault=2, state=4/result=5. Hardware TXBTO=7, TXBCF=0, TXBRP=0,
ECR=0: all three frames actually transmitted successfully, with no CAN errors.
This is false failure classification in the sender, not evidence of a loose
connection in this attempt. The old adapter read TXBTO first then TXBRP:
completion between those reads can clear pending after a stale success sample.
It would incorrectly report failure. Fixed adapter uses terminal success/
cancellation flags; absence of either returns pending until the next poll or
the existing 100 ms timeout. No fault gates/timings bypassed.
Regression test added; all 17 scenarios pass. Build FLASH=20,680 bytes,
RAM=1,928 bytes. Fixed build subsequently flashed with explicit user authorization
and battery-off confirmation. Download, verification and reset succeeded on
Nucleo `002200364142500620353451`. HOTPLUG readback confirmed idle/unarmed,
zero requests/TX/ACK/first-fault counters, no pending/completed/canceled TX flags,
and nonzero SysTick. No commands triggered. Diagnostic addresses unchanged.
Source of the earlier unexplained STOP sequence is still not established.

### Corrected sender recheck after power-connection clarification

User reports earlier STOP check occurred before battery fully connected (after
a conversation history reset). Corrected-build preflight now found last-request
STOP, request-count=1, TX-count=3, ACK-count=0, first-fault=2, used-mask=0.
Issued only another explicit STOP with user-confirmed battery fully connected.
Readback request-count=2, TX-count=6, ACK-count=3, TXBTO=0x38, TXBCF=0x07,
TXBRP=0; all three new STOPs successfully transmitted/ACKed and recognized by
fixed sender. ECR=0x15, no error-passive/bus-off flags. Prior fault intentionally
remains latched (state=4/result=5); no ARM/RUN issued. Requested battery-off
confirmation before resetting sender, rather than bypassing its fault latch.

User subsequently confirmed battery off. Software reset performed on Nucleo;
HOTPLUG readback confirmed all bench requests/state/counters/fault fields zero,
TXBRP/TXBTO/TXBCF zero and nonzero SysTick. No STOP/ARM/RUN triggered while
battery off. Ready for a new powered preflight after user confirmation.

### First corrected-sender NE positive drive request

User confirmed fully reconnected battery and stationary wheels. Clean preflight
followed by explicit STOP: TX=3/ACK=3, idle/result=3, first-fault=0, ECR=0.
Selected physically front-right NE=1, sign=+1; ARM2 confirmed state=1/result=1.
Issued exactly one RUN2, payload `1E 00 F4 01` to ID 0x081. Sender readback
showed running then stopping then idle/result=3, used-mask=1, TX=7/ACK=7,
first-fault=0. Thus one drive frame and three timed STOP frames transmitted
successfully after preflight STOP. No reverse/other wheel command issued.
CLI trigger writes returned error despite being consumed; no RUN retry.
Log: `/tmp/opencode/ne-positive-wheel-test-20261012.log`.
Physical movement, actual speed and stopping await user observation; CAN ACK
does not establish those outcomes. No further starts until physical confirmation.

### NE observation and negative-direction request

User reported no observed movement (possibly missed) in positive test and all
wheels stationary. No motion/actual speed confirmed. User explicitly authorized
negative test. Preflight idle/result=3, used-mask=1, TX=ACK=7, fault=0.
Selected NE=1/sign=-1, ARM2 confirmed, issued exactly one RUN2
(`E2 FF F4 01`, ID 0x081). Readback running -> stopping -> idle/result=3,
used-mask=3, TX=ACK=11, first-fault=0. Three timed STOP frames ACKed.
Log: `/tmp/opencode/ne-negative-wheel-test-20261012.log`.
No retry or other-wheel drive commands. Negative motion observation pending;
both NE directions are now used for this boot. Speed/duration not increased.

User also reports no observed movement in negative test. Paused drive testing;
requested battery disconnection and front-right ESC USB for read-only diagnosis.
Original binary confirms +/-30 RPM decodes to signed +/-5 SDK speed units, not
zero. Its start routine at 0x080020C0 rejects non-IDLE state or either current/
latched fault; an ACK cannot prove these gates passed. Binary pMCI[0] pointer
stored at 0x20001C00; after validating live pointer, MCI state is byte offset 1,
current fault halfword offset 2, historical fault halfword offset 4, speed-ramp
reference halfword offset 20 and duration offset 24. Do not use guarded-profile
ELF symbols for this original image. No additional start or fault clear issued.

### Front-right original ESC read-only inspection, battery off

Detected expected front-right probe `066AFF525266555067075126`. pMCI[0] at
0x20001C00 points to 0x20000000. MCI bytes show state=10 (FAULT_NOW), current
fault=0x0004, occurred/latched fault=0x0004 (SDK undervoltage). Speed reference
and duration are zero. NE live filter still 0x88010001 (fields 0 0 2 1 1 1).
Undervoltage with battery disconnected is expected; this does not prove it was
present during either drive request. However, original start routine rejects
latched faults too, and its CAN speed/STOP dispatch does not acknowledge faults.
Next diagnostic: read state/faults with confirmed battery connected, no start
or acknowledgement writes. No flash/reset/drive/fault-clear commands issued.

### Powered front-right state check

User confirmed battery connected/stationary wheels. Read-only MCI now shows
IDLE=0, current fault=0, latched fault=0, direct command=2 (fault acknowledge),
speed reference/duration=0. Further binary inspection at 0x08001D74 and
0x08001E98-0x08001EA6 confirms the original main loop automatically acknowledges
FAULT_OVER when current faults clear. Thus persistent latched undervoltage is
not established as the nonmovement cause; previous hypothesis must not be
treated as diagnosis. No operator fault clear/reset/RUN performed.
Need capture ESC acceptance/start state during an explicitly authorized run,
not infer it from Nucleo ACK or the state after power cycling.

### Unexpected STOP after corrected-build flash

Following a clean zero-counter post-flash readback, the next preflight found
request-count=1, last-request=STOP, TX-count=3, ACK-count=0, first-fault=2 at
tick 0xCDEF, state=4/result=5, used-mask=0. No trigger had been issued in this
conversation after that flash. Snapshot CAN ECR=0x00010018, PSR=0x0000071B,
TXBTO=0, TXBCF=7, TXBRP=0 confirms canceled STOP transmissions, with ACK-error
code 3 in the captured PSR. This is not the earlier false-completion case.
Requested battery disconnection; issued no new commands/reset/fault bypass.
Process snapshot shows no active external programmer/debugger besides our
read-only CLI, but cannot rule out short-lived processes, another control
session or memory corruption. The recorded trigger origin remains unresolved.
Do not restart a powered drive test until its origin is identified.

### Corrected sender: genuine STOP-only ACK failure

After corrected flash, user confirmed battery reconnected/stationary wheels.
Preflight diagnostics all zero. One STOP trigger only; no ARM/RUN.
Postflight state=4/result=5, TX-count=3, ACK-count=0, first-fault=2,
last-request=STOP, request-count=1, used-mask=0. Captured ECR=0x00010018,
PSR=0x0000071B (LEC=3, ACK error), TXBTO=0, TXBCF=7. Live TEC=24,
TXBRP=0. This is genuine lack of CAN acknowledgement, not the fixed completion
race. All three frames failed/canceled and the fault gate correctly blocked
starts. Requested battery disconnection. CAN wiring/transceiver power/receiver
readiness must be diagnosed before any powered RUN attempt; no reset bypass.

## First powered baseline attempt

User confirmed wheels stationary after battery connection and cutoff in hand.
Only explicit STOP baseline was triggered (`0x53544F50` at `0x20000118`).
CLI write reported its previously observed download error, but the request was
consumed. Subsequent HOTPLUG readback: request=0, wheel=0, direction=0,
state=4 (latched transport fault), result=5, used-mask=0. Baseline STOP
transmission did not complete successfully; no ARM or RUN/speed command sent.
User instructed to disconnect battery; CAN transport must be diagnosed before
another powered attempt. Never reset/retry merely to bypass this lockout.
No ESC changes. Nucleo PB8/PB9 CAN pin configuration unchanged. CM7-only build:
`CM7/build/Debug/sfurs_firmware_v2_CM7.elf`.

## Preconditions for any powered attempt

Robot secured on a suitable stand, all rotating parts clear of people/objects,
accessible immediate battery cutoff. Verify the physical wheel/probe mapping
recorded in `/home/fh/sfurs_firmware/esc-backups/physical-wheel-mapping-verified-20261012.json`.
No competing CAN command sender. Keep battery disconnected during programming.
Do not begin a reverse run until the wheel is visibly stationary. Each run needs
separate operator authorization/ARM/RUN, not an automatic eight-run sequence.

This is NOT a current-limited test. Original ESC current protection is unvalidated.
Low requested speed does not guarantee slow motion or low current. No ESC-local
test timeout exists; Nucleo reset/halt/loss of power or CAN failure can strand a
running wheel. Sender STOP is best effort, not independent safety protection.

## Commands (debugger-visible variables, use current ELF symbol addresses)

Installed ELF addresses (re-resolve after any rebuild): request `0x20000118`,
wheel `0x2000011C`, direction `0x20000120`, state `0x20000124`, result
`0x20000128`, used-mask `0x2000012C`. Old ARM1/RUN1 addresses must not be used.

- `wheel_bench_wheel`: one bit only, NE=1, SE=2, SW=4, NW=8.
- `wheel_bench_direction`: signed +1 or -1 (`0xFFFFFFFF`). Positive is a motor
  reference sign, not a verified physical forward/clockwise direction.
- `wheel_bench_request`: ARM2=`0x41524D32`, RUN2=`0x52554E32`, STOP=`0x53544F50`.
- `wheel_bench_state`: 0 idle, 1 armed, 2 running, 3 stopping, 4 latched fault.
- `wheel_bench_result`: 0 none, 1 armed, 2 run queued, 3 stop frames ACKed,
  4 rejected, 5 transport fault. ACK is not proof any particular ESC stopped.
- `wheel_bench_used_mask`: each wheel/sign may run once per Nucleo boot.

In the initial CLI-only build, no frame is transmitted at startup, by button or
solely by selecting a wheel. The gated button extension below supersedes only
the button restriction, not the no-startup/no-selection rule.
The old ARM1/RUN1 NE-profile sender is disabled in this build. First issue STOP
explicitly and verify physical rest; wait at least two seconds after the STOP
sequence. Set wheel/sign, ARM2, read armed state, then explicitly issue RUN2.
Arming expires in five seconds. Changing wheel/sign between ARM/RUN rejects RUN.

RUN sends one 4-byte CAN FD frame, BRS off, 500 kbit/s:
`0x080 | wheel`, signed little-endian RPM +/-30 and ramp 500 ms.
Payload positive=`1E 00 F4 01`; negative=`E2 FF F4 01`. Original binary speed
dispatch sign-extends the low halfword and uses signed division by six before
calling the SDK, so negative references are supported. This is motor RPM, not
measured wheel RPM; Hall quantization/gearing/control may prevent tracking.

At 1000 ms from enqueue (not confirmed motion), the sender attempts three
STOP frames `0x78F`, zero payload, spaced at least 50 ms after each completion.
All four individually verified mask filters accept bitmask 0xF. STOP is sent to
all wheels, not just the selected wheel. Sender TX timeout is 100 ms; any TX
failure triggers STOP attempts and permanently disables further starts until
reset. Drive frames are never retried. A manual STOP works while idle/armed/
running/faulted, and cancels a pending drive transmission best effort.

If any unexpected wheel moves, speed is unexpectedly high, stopping fails or
fault status appears: disconnect battery immediately. Do not blindly retry RUN.
HOTPLUG debugger trigger writes previously reported a CLI error despite being
consumed; always read state/result instead of assuming a failed write was not
delivered. Never halt the Nucleo to inspect state during a powered run.

Tests: `python3 tests/test_wheel_bench.py` (actual state machine/fake transport)
and `python3 tests/test_wheel_sender.py` (retained old selection gates).
Host tests prove logic/payload selection, not physical safety or actual stopping.

## One-data-cable button extension

User can power Nucleo independently from a power bank while the sole data cable
connects to front-right ESC. Added explicit ABTN trigger `0x4142544E`: snapshots
selected wheel/sign like ordinary ARM, but allows one blue USER-button press
within 60 seconds. Ordinary ARM retains its 5-second window and does not enable
button starts. Nucleo USER B1=PC13, active high, confirmed from official ST H7
Nucleo BSP; polled with 50 ms debounce. Requires stable release after arming,
then stable press. Held-at-arm/bouncing button cannot start; holding/repressing
cannot repeat/advance a test. Existing used-mask/fault gates/1000 ms STOP unchanged.
Yellow LED steady=armed; red steady=latched sender fault; green=heartbeat.
No ESC modification or increased speed/current/run duration.

With battery OFF, flash Nucleo and verify button input while unarmed. Then set
NE/sign and explicitly arm button mode, maintaining uninterrupted Nucleo power
while moving data cable to ESC. Establish read-only monitoring before any
authorized press. If yellow LED extinguishes (expiry), do not press to start;
return to Nucleo with battery off to explicitly rearm. The button is not an
independent emergency stop; battery cutoff remains required. Do not press during
cable/power setup. Disconnect battery after attempt before resetting sender.
No automatic delayed RUN used. All 21 host scenarios pass; FLASH=21,152 bytes,
RAM=1,936 bytes. Battery off confirmed; data probe still front-right ESC, so
Nucleo flashing awaits moving the data cable. Earlier symbol addresses must
be re-resolved from the button ELF before writing any command.

Subsequently detected Nucleo after cable move and flashed button ELF: download,
verification/reset succeeded. Battery-off HOTPLUG readback shows all bench
commands/state/counters/fault fields zero, PC13 input low (released). No arming
or test trigger written. Hardware press/release validation pending.
Installed button ELF addresses: request=0x20000130, wheel=0x20000134,
direction=0x20000138, state=0x2000013C, result=0x20000140, used=0x20000144,
last-request=0x20000148, request-count=0x2000014C, TX=0x20000150,
ACK=0x20000154, first-fault=0x20000158, first-fault-tick=0x2000015C.

Battery-off hardware button validation completed: user held B1, PC13 read
0x2000 (pressed high); after release PC13 read zero. Both checks show every
bench request/state/counter/fault field zero. No automatic start or transmit
from unarmed press/release. Sender remains unarmed; cable-transfer arming pending.

## Physical ARM/RUN after power interruption (current installed version)

User cannot keep Nucleo continuously powered during cable transfer. Following
confirmation of battery off/data cable to Nucleo, installed physical-gesture
extension: stable release must first be seen after boot; a subsequent >=2-second
hold while idle/fault-free/NE+ unused selects NE=1/+1 and generates ABTN (arm
only). Yellow LED steady indicates armed. Release, then one separate debounced
press generates RUN2. Startup-held button never arms, short triple presses do
nothing while unarmed, and holding the arm gesture never generates RUN.
Arming expires in 60 seconds. Physical gesture is fixed to front-right NE/+30
motor RPM, 500 ms ramp, 1000 ms sender stop deadline. NE+ used-mask prevents
retries until a battery-off reset. No boot/power-restoration automatic arming.
Existing fault lockout and three all-wheel STOP attempts unchanged. No increased
speed/duration/current cap or ESC modification. Host tests: all 24 scenarios pass.

Flash/download verification/reset succeeded on Nucleo; readback zero requests,
state/counters/faults, PC13 released low, SysTick nonzero at correct new ELF
address 0x20000030 (older 0x2000002C no longer applies). FLASH=21,292 bytes,
RAM=1,936 bytes. Bench variable addresses match preceding button build.
No arming/RUN/STOP triggered. Next: move data cable to ESC with battery off,
power Nucleo from power bank, establish read-only ESC monitoring before any
authorized powered button sequence. Do not press during setup.

### Monitored physical-button NE positive request

User confirmed motor battery connected, stationary wheels; ESC read-only
preflight IDLE/current=latched=0. Started HOTPLUG monitor, then authorized one
2-second arm-hold/release/separate RUN press, cutoff ready.
Log `/tmp/opencode/front-right-button-start-monitor-20261012.jsonl`:
at 45.319 s direct command=start (1), SDK reference=5 (+30 motor RPM), ramp=500;
at 45.3539 s state=6 (RUN), faults=0; at 46.3416 s state=8 (STOP), direct stop=5,
reference/ramp=0; at 46.7236 s IDLE, current/latched faults still zero. 1,427
samples captured. This confirms the ESC accepted a drive request, entered RUN
and then STOP/IDLE. Startup lasting the whole one-second request is not supported
in this attempt. Does NOT prove physical rotation/speed/current; user observation
pending. No automatic additional request issued. No ESC reset/fault-clear/write.

User subsequently confirms one physical sequence only and no wheel motion;
reports LD2 still on. Steady yellow is inconsistent with sender's expected
post-run idle indication and must be checked on Nucleo, not inferred from ESC
state. Instructed no more presses and battery disconnection. ESC subsequent
read-only snapshot remains IDLE, reference/ramp zero, current/latched faults zero.
No higher speed/duration/torque command authorized or sent. Need validate sender
state/LED and obtain actual FOC references/current/Hall data before further drive.

### Subsequent return to Nucleo, read-only check

Detected Nucleo probe. Bench variables all zero: idle/unarmed, used-mask=0,
request/TX/ACK/fault counters zero. Yellow LD2/PE1 output is low (off), red
LD3/PB14 output low (off), PC13 released low, SysTick=0x3DFC. This indicates a
fresh boot/reset since monitored drive (consistent with interrupted cable/power
transfer); no retained test history is available on sender. It does not negate
the independently logged ESC RUN/STOP sequence. No reset/flash/request writes
performed in this check. Awaiting explicit motor-battery-off confirmation.

User confirmed battery off. Extended read-only ESC monitor prepared (`--foc`):
resolves/validates original MCI/STC/FOC/speed/scale pointers instead of assuming
newer SDK layout. Binary getters establish FOC phase currents offsets 0/2,
iq/id 8/10, references 12/14, voltage commands 18/20 and 22/24; measured
speed path MCI+28 -> STC+20 -> SPD+12. Original float current conversion is
ScaleParams+4 (verified in 0x080021C4 getter); report raw SDK and explicitly
firmware-scaled values, not verified physical currents. Also reads raw PB6..8
inputs at GPIOB IDR 0x48000410, not a claimed validated abstract Hall mapping.
Multi-register HOTPLUG samples are non-atomic; brief peaks/transitions may be
missed. Decoder host tests and all 24 sender cases pass. No new firmware build/
flash or motor command needed for this monitor. Next cable move to front-right
ESC with Nucleo separately powered; powered attempt requires renewed preflight
and explicit single button sequence authorization, same speed/duration only.

Extended-monitor battery-off validation succeeded on front-right: FOC pointer
0x20000944, speed pointer 0x20000054, firmware current scale approximately
0.001836397 A/SDK count. Raw PB6..8 inputs=5, measured speed=0, current/latched
faults=undervoltage, current references and voltage commands zero. Measured
SDK currents outside RUN are large/nonphysical (battery off, offsets/sampling
unvalidated); must NOT interpret them as physical amperes. Monitor now suppresses
ampere conversion of measured currents outside fault-free RUN and explicitly
marks offset calibration unvalidated even during RUN. Raw counts retained for
diagnosis. Baseline log `/tmp/opencode/front-right-foc-battery-off-check-20261012.jsonl`.
## Extended FOC capture result — subsequent NE positive test

Log `/tmp/opencode/front-right-foc-run-monitor-20261012.jsonl`, 619 samples,
including 26 RUN samples. User authorized same one-second +30 RPM button test,
cutoff ready and all wheels stationary before test. ESC again RUN -> STOP ->
IDLE, current/latched faults zero throughout. Sampled SDK iq reference 0..15,
id reference=0; installed float scale corresponds to iq-reference approximately
0..0.02755 A. This is commanded reference, NOT an independently measured current
limit/phase-current peak. Sampled iq=-23..41, id=-44..52 SDK units; offset
calibration not independently validated. Measured speed stayed zero and raw
PB6..8 Hall inputs stayed 5 throughout RUN. Voltage commands nonzero in RUN.

Very small torque reference is a plausible explanation for no rotation at this
very low requested speed; it does not prove the motor or power stage healthy,
nor rule out friction/wiring/current scaling/commutation problems. No increased
speed/duration/current command issued. Sender/ESC returned stopped reference;
physical observation pending. Pause and disconnect battery before next test.
