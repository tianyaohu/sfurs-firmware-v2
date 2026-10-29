# Master design log

Append only.

## 2026-10-29  Phase 1 kinematics is the linear wheel map  (who: cloud agent)

- Decision: the first owned module maps a body twist to four raw motor RPM
  (FR, BR, BL, FL) with the provisional wheel map: contacts hand-measured,
  front rollers ±30° from forward, rear ±35°, radius 32.75 mm, direct drive,
  left motors mirrored (`motor_sign` −1). A command past 900 RPM scales every
  wheel by the same factor. Zero is left at zero. The function is host-tested
  and not called from CM7 yet.
- Why: Phase 1 of the from-scratch tree is the math the drive will need, checked
  on the PC. The map matches the reviewed provisional geometry. One shared scale
  keeps the requested direction; clamping a single wheel would skew the twist.
- Rejected: per-wheel clamp; a speed floor; copying the previous tree's test
  harness and validation matrix into this change.
- Touches: `src/shared/inc/rover_kinematics.h`, `src/shared/src/rover_kinematics.c`,
  `tests/test_rover_kinematics.c`, `scripts/test.sh`.

## 2026-10-29  ESC vendor image is MCSDK 6.4.1 without the drive application  (who: cloud agent)

- Decision: vendor the B-G431B-ESC1 MCSDK 6.4.1 project and Cube HAL, and
  cross-compile one image at `0x08000000` with the same host toolchain as
  the Nucleo. `main` is the generated init and an empty loop. The
  medium-frequency task stays in SysTick and the TIM1 break IRQ.
- Why: the wheel-controller vendor drop has to build on the PC before any
  owned drive code is written. The reference tree's command, speed, and
  Hall-angle modules are that owned code, so they are not copied.
- Stripped: `drive_app`, `motor_cmd`, `motor_speed`, `hall_angle`; the CAN
  bring-up and boot-entry code in `main`; the FOC angle and capture hooks;
  Hall `EdgesPos` / `EdgesNeg` and the `HALL_Clear` speed wipe; the
  compile-time Hall-shift override; stall macros only the speed module
  read. The generated `MCI_ExecSpeedRamp` of the default target (0 RPM)
  is restored at the end of FOC init. The link needed no stubs.
- Kept: the parameter values the generated code reads. Current-loop gains
  4000/4400, speed-loop gains 1024/553, `IQMAX_A` 8, nominal current 6 A,
  Hall FIFO depth 6, over-voltage 18 V, `SPEED_UNIT` `U_RPM`.
- Not in this change: the CAN bootloader, the `0x08004000` link, four
  wheel builds, and the Motor Control Workbench project files.
- Touches: `src/esc/motor_config/`, `scripts/test.sh`.

## 2026-10-29  Frame codec is CONTROL, STATUS, and DIAG  (who: cloud agent)

- Decision: one shared codec encodes and decodes CONTROL v1 and the STATUS
  and DIAG polls and replies. CONTROL structural checks stop at the wire
  (version, mode, flags, unused fields, which wheels a frame may name).
  Reason `BAD_LENGTH` stays 20 so codes 6-19 remain free for the ESC's
  range, session, and sequence checks. Those checks are not in this codec.
- Why: both ends of the bus have to agree on the bytes before either end
  grows a drive loop. STATUS and DIAG are the telemetry frames already
  fixed by the old service layout; they are a different id scheme from
  CONTROL, and they belong in the same codec so there is still one packer.
- Rejected: copying the reference codec's CONFIG, CONTROL_REPLY, session
  stepper, and sequence window; linking this file into the Nucleo or ESC
  image.
- Touches: `src/shared/inc/motor_protocol.h`, `src/shared/src/motor_protocol.c`,
  `tests/test_motor_protocol.c`, `architecture/protocol.md`, `scripts/test.sh`.

## 2026-10-29  CONTROL_REPLY and the session and sequence helpers  (who: cloud agent)

- Decision: the shared codec also packs CONTROL_REPLY (`0x410` plus one
  wheel select bit) and exposes `motor_session_next` and `motor_seq_classify`.
  Reason codes 6-19, 21-22, and 32-36 are named in the header. CONTROL
  decode still does not return them. Boot session (`prev` 0) steps modulo
  255, so every nonzero byte is reachable; select-bit entropy is still
  `1 + select`. A later session steps modulo 254 so it cannot repeat `prev`.
- Why: both ends need the reply bytes and the two pure rules before either
  end grows a drive loop. Naming the reason codes now keeps the wire numbers
  from being reused by accident. The reply id uses bits 10-4 as the base,
  the same split as CONTROL, because bit 4 is part of `0x410`.
- Rejected: porting `motor_cmd` (inbox, watchdog, arming, CONFLICT versus
  DUPLICATE); linking the codec into the Nucleo or ESC image; CONFIG.
- Touches: `src/shared/inc/motor_protocol.h`, `src/shared/src/motor_protocol.c`,
  `tests/test_motor_protocol.c`, `architecture/protocol.md`.

## 2026-10-29  Command slice arms and records a setpoint  (who: cloud agent)

- Decision: host-only `motor_cmd` handles one CONTROL for one wheel. Init is
  DISARMED and the session is `motor_session_next(0, entropy)`. ARM moves to
  READY. Motion that passes the session, the sequence window, and the rpm/mA
  limits is stored and answered APPLIED while the lifecycle stays READY.
  `CLEAR_FAULT` is REJECTED(UNSUPPORTED). This slice does not enter FAULTED;
  if a caller sets the latch, ARM answers with that reason before it checks
  the session. Plain DISABLE clears the recorded setpoint and does not change
  the lifecycle, because nothing here turns the bridge on.
- Why: the codec already names the reply and the sequence classes. The next
  checkable piece is those checks plus arming. Staying in READY keeps the
  applied setpoint visible to a host test, with no plant model and no MCSDK.
- Rejected: porting the reference `motor_cmd` (inbox, watchdog, HOLD, park,
  MCSDK start timeout, stall rest); a CLEAR_FAULT path that invents a fault
  word; linking this file into an image.
- Touches: `src/shared/inc/motor_cmd.h`, `src/shared/src/motor_cmd.c`,
  `tests/test_motor_cmd.c`, `architecture/command-lifecycle.md`.
