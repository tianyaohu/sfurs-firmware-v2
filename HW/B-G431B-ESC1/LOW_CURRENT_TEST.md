# Front-right low-current bench profile (NOT validated for powered operation)

This opt-in profile is separate from the telemetry firmware currently installed.
It does not change the normal build or make the Nucleo issue drive commands.

## Build only

From `motor_config`:

```bash
PATH=/tmp/opencode/cm7-toolchain/root/usr/bin:$PATH python3 build_cli.py \
  --motor NE --low-current-test --output /tmp/opencode/esc-ne-low-current-test
```

Output: `motor_NE_low_current_test.elf`. Building does not flash or run it.

Offline validation: `python3 test_low_current_guards.py` in `motor_config` checks
disarming, rejected command bounds, one-shot behavior, rejected starts, stop,
deadline and tick wrap using the actual guard snippets with mocked SDK calls.
Both normal and test builds pass. The test ELF's initialized speed-PID and
torque bounds were checked as +/-136 SDK current units; default speed is zero.
These checks do not validate real current, commutation or physical stopping.

## Guards

- Speed PID output and torque ramp bounds: 0.25 A in SDK q-axis reference units.
- Default target: zero. CAN torque commands disabled.
- Boot is disarmed. An exact NE `TEST_ARM` request with payload `0x54455354`
  arms it for 5 seconds. This requires successful calibration-only completion,
  calibrated offsets, no current fault and no prior test attempt.
  It explicitly acknowledges a cleared fault (e.g. USB-only undervoltage).
  No arming command is sent automatically.
- Exact NE speed-write ID only; requested positive rotor speed 12–60 RPM,
  ramp 500–1000 ms. These are motor RPM, not geared-wheel RPM.
- One attempt per ESC boot; repeated commands cannot restart or extend it.
- ESC SysTick requests stop 1500 ms after accepting the command, including
  calibration/startup time. This does not depend on a later CAN stop arriving.
- Addressed CAN stop remains enabled and disarms the test.
- Button/UART starts are blocked; UART tuning/control processing is disabled.
- Faults are not automatically acknowledged in this profile.
- SysTick samples measured SDK iq/id and retains peak `abs(iq)+abs(id)` during
  the attempt. At approximately 0.4 A in these units it requests a stop (reason 4).
  This conservative sampled guard is NOT independent peak-current protection.
- Test-current requests/replies use IDs 0x301/0x381. The 8-byte reply contains
  signed iq and id, flags/faults, and peak L1 magnitude, each 16-bit little-endian.
  Current scale is 544 SDK counts/A, according to existing board configuration.
  Flags: bit15 test profile, bit14 armed, bit13 used, bit12 active,
  bit11 offsets calibrated, low11 faults.
  iq/id fields are zero unless calibrated, fault-free and in RUN; zero while
  invalid is NOT evidence that physical motor current is zero.
- Overvoltage trip changed from 14 V to 18 V for the reported 16 V battery;
  undervoltage and existing hardware current protection remain unchanged.

The timed stop depends on functioning firmware/interrupts and the SDK stop path.
It is not a hardware emergency stop or an independent watchdog. A battery cutoff
must remain accessible. An MCU reboot clears the one-attempt latch but disarms it.

## Unresolved checks BEFORE flashing, arming or applying motor power

Repository consistency check: tracked `pmsm_motor_parameters.h`, `main.h`,
`mc_config.c` and `motor_config.ioc` agree on 8 pole pairs, 120-degree Hall
placement, 120-degree phase shift, H1=PB6, H2=PB7, H3=PB8 and TIM4. The bench
profile preserves these settings and the existing PWM mapping/current-loop
gains. The user confirms this motor previously drove correctly with this
repository firmware and unchanged phase/Hall wiring. This supports retaining
the existing commutation configuration; it is not a new physical alignment
measurement. The .ioc also agrees with RS=0.22 ohm and LS=0.11 mH. A separate
profiling JSON contains different R/L values, so it is not substituted.

1. Confirm battery chemistry, fully charged voltage, fuse and accessible cutoff.
2. Verify motor U/V/W and H1/H2/H3 mapping and configured 120-degree Hall phase
   shift. Reading Hall speed successfully does NOT verify commutation alignment.
3. Resolve the electrical parameter discrepancy. Nanotec DF45L024048-A lists
   16 poles, line-line R=0.56 ohm and L=0.27 mH. The existing SDK profile has
   8 pole pairs, RS=0.22 ohm and LS=0.11 mH. Line-line specifications cannot be
   substituted blindly for SDK phase parameters; winding connection and tuning
   history must be established. Existing current-loop gains are unchanged.
4. Validate current-sense scaling/offset calibration on this board. Measured
   iq/id telemetry and the sampled L1 guard are prepared, not hardware-validated.
5. Diagnose intermittent CAN errors and verify the timed-stop behavior.

A 0.25 A reference cap is NOT a guarantee that instantaneous motor phase current
stays below 0.5 A. Sensor errors, commutation errors and control transients can
exceed the reference. Existing hardware overcurrent protection is not a 0.5 A
trip. If an absolute 0.5 A peak ceiling is required, do not run this setup without
appropriate independently verified current protection.

Datasheet: https://www.nanotec.com/fileadmin/files/Datenblaetter/BLDC/DF45/DF45L024048-A.pdf

## Prepared Nucleo sender (not yet flashed)

`CM7/Core/Src/main.c` alternates speed and test-current queries at 10 Hz each.
It never automatically arms or starts the motor. Debugger variable
`wheel_test_request` accepts ARM1=0x41524D31, RUN1=0x52554E31, STOP=0x53544F50.
ARM1 and RUN1 require a fresh current reply identifying the test profile, zero
current faults, calibrated offsets and an unused attempt. RUN1 additionally requires a fresh IDLE
speed reply and the armed flag. RUN1 sends exactly one 60 rotor RPM request with
a 1000 ms ramp; it is never retried, even after a TX failure. STOP is available
without a telemetry freshness gate. The ESC-local timeout is still authoritative.
Nucleo `wheel_test_result`: 0 none, 1 queued, 2 rejected, 3 CAN ACK, 4 TX failure.
A CAN ACK proves frame transmission, not successful motor startup or stopping.
Peak telemetry is sampled L1 magnitude, not a physical phase-current peak.

## Latest USB-only flash verification

Updated ESC profile downloaded, verified and reset successfully. Live RAM
confirms disarmed/inactive/unused, +/-136 current-reference units and the expected
undervoltage fault. However, `PWM_Handle_M1._Super.offsetCalibStatus` is false
and iq/id values are implausible under USB-only operation. Treat these values as
invalid, not physical motor current. Before any drive attempt, add explicit
calibration-valid reporting and ensure the sampled-current guard cannot interpret
uncalibrated startup readings as valid. Nucleo sender is built but not flashed.

## Calibration-only update (built, not yet flashed)

Keep the data cable on the ESC for this step. Explicit debugger variable
`test_calibration_request` accepts CAL1 (`0x43414C31`). No request is written
automatically. The application rejects it with a present fault, an armed/active
test, a used drive attempt, a pending calibration, or a non-IDLE state. It may
explicitly acknowledge a cleared fault before requesting the SDK's
`MC_StartPolarizationOffsetsMeasurementMotor1()` calibration-only path.
That path measures offsets with phase outputs inactive and returns to IDLE;
it never grants motor-start permission. Calibration result values are 0 none,
1 pending, 2 completed, 3 rejected and 4 failed/timed out. A 1000 ms software
deadline requests stop; it is not an independent hardware timeout.

`test_calibration_valid` reports the SDK offset-calibration flag. Successful
completion additionally requires returning to IDLE without a current fault.
ARM and SPEED_WRITE are blocked until this explicit completion. A flag is not
proof of accurate current scaling; inspect the offsets and subsequent measured
current before treating sensing as verified. `test_drive_current_valid` is true
only with calibrated offsets, no fault and state RUN; uncalibrated/idle samples
are excluded from reported iq/id and from the sampled-current guard.

USB-only undervoltage prevents accepting calibration. Motor-bus power must be
present for the normal SDK fault checks, but do not apply it until the updated
firmware is flashed and the operator confirms the lifted wheel and cutoff.
Do not arm or run merely to obtain offset calibration.

## Calibration-only hardware result

With battery power connected, explicit CAL1 was issued over ESC SWD. The first
request acknowledged the cleared undervoltage fault but was rejected while the
SDK asynchronously transitioned from FAULT_OVER to IDLE. After checking IDLE,
zero faults and no completed calibration, a second request succeeded.
Live readback: result=2, valid=1, SDK offsetCalibStatus=1, state=IDLE,
current/past faults=0, drive armed/active/used=0. Measured offsets were
A=0x9DFE, B=0x9E71, C=0x9EF6. No ARM or motor-start request was sent.
The CLI reported failed RAM writes, but live application results prove the
requests were consumed; do not blindly retry a trigger after a CLI error.
Current scale/physical peak protection and the approximately 12.1 V firmware
reading versus the user's reported 16 V remain unvalidated. Calibration is RAM
state and must be repeated after an ESC reset. Nucleo sender remains unflashed.

## First powered one-shot result

The Nucleo sender was subsequently flashed/verified/reset. After another
ESC calibration-only operation and cable move, CAN flags confirmed calibrated,
IDLE, disarmed and fault-free. One ARM attempt expired without a RUN because the
first confirmation sample did not yet contain the armed flag. A subsequent ARM
was explicitly confirmed before exactly one RUN request was sent; no RUN retry.
The ESC entered RUN (state 6), then STOP (8), then IDLE (0). Final flags=0xA800:
profile/calibrated/used, inactive, disarmed, no reported current faults.
Observed RPM=0 and Hall transitions=0 throughout the collected snapshots.
Sampled peak abs(iq)+abs(id)=0.744 A in SDK-derived units, above the sampled
guard threshold. Earlier snapshots reported peak L1 0.231 and 0.286 A.
This is NOT a measured instantaneous phase-current peak or proof of a 0.5 A
hardware ceiling. Powered rotation and current-limit performance are NOT
validated. Operator instructed to disconnect battery; no further drive attempt
or current increase is authorized by this result. Diagnose sensing/transients
and lack of motion before any further powered test.
