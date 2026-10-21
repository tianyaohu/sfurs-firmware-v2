# C. Control path and ownership (design, for review)

Status: design freeze draft for [phase1-esc-command-v2](../phase1-esc-command-v2.md). States: [command-lifecycle](../../architecture/command-lifecycle.md).

## Signal path (SPEED mode, RUNNING)

```
Nucleo, every 20 ms (ik_drive.c)
  body twist (mailbox) --rover_ik--> wheel RPM target --slew 900 RPM/s, all wheels together--> setpoint[w]
  body twist (mailbox) --friction_ff_wheel_amps--> ff[w] (mA)          (0 unless the friction model is enabled)
  motor_protocol encode --> CONTROL(setpoint, ff, seq, session) --> CAN
ESC FDCAN ISR (motor_rx.c)
  decode + structural validation --> snapshot {fields, ordinal, rx time} --> lifecycle FIFO / motion slot
ESC 1 kHz tick (MCSDK medium-frequency hook; motor_cmd.c, motor_safety.c)
  lifecycle, watchdog, session/sequence --> applied target {mode, setpoint, ff}
  reference r(t): linear from the present value to the setpoint over 20 ms (C5b)
ESC 1 kHz speed step (motor_speed.c, called from FOC_CalcCurrRef in place of the MCSDK speed PI output)
  w_hat, valid  = speed_estimate()                 the only speed estimate (chosen in Stage 4a)
  e             = r - (valid ? w_hat : 0)          w_hat = 0 only before the first edge of a start
  p             = Kp * e  (+ i, Ki = 0 by default)
  ff_applied    = slew(ff toward the target along sign(r), 0 against it, reset to 0 on a sign change of r)
  u             = p + i + ff_applied
  L             = min(IQMAX, configured limit, I2T allowance)
  iq_ref        = clamp(u, -L, L); integrate only if u == iq_ref or the error pulls back in
ESC 16 kHz FOC (MCSDK, unchanged): iq_ref, id_ref = 0 --> current PI --> SVPWM --> motor
```

CURRENT mode: `iq_ref = clamp(setpoint_mA in counts, -L, L)`; no speed loop and no feed-forward.
HOLD: `iq_ref = 0`. STARTING: MCSDK's start-up owns the references; `motor_speed` is not called.
DISARMED, READY, FAULTED: output off; nothing is computed.

## One owner per quantity

| Quantity | Owner | Nobody else may |
|---|---|---|
| acceleration limit | Nucleo `ik_drive.c` slew | the ESC only reconstructs the trajectory (C5b) |
| target wheel speed | Nucleo, via CONTROL | the ESC never invents a target (watchdog -> disarm, not "hold last") |
| friction compensation | Nucleo `friction_ff.c` -> ff field | no ESC-side friction constants (legacy static/kinetic ff deleted) |
| lifecycle state | ESC `motor_cmd.c` | tools and the Nucleo request transitions; they never set state |
| q-current request | ESC `motor_speed.c` (SPEED) or the setpoint (CURRENT), RUNNING only | single insertion point in `FOC_CalcCurrRef` |
| current limit | ESC: min(IQMAX compile-time, CONFIG limit, I2T) | the Nucleo can only lower it |
| speed estimate | ESC `speed_estimate()` | the reply, recorder and stall monitor all read this one value |
| protections | MCSDK (over-current, voltage, temperature), ESC I2T, watchdog, stall monitor | none can be disabled over CAN |

## What replaces the DSC startup machine's jobs

| DSC job (`drive_speed_ctrl.c`) | New owner |
|---|---|
| ramp current until breakaway | P on the full error (est = 0 before the first edge) plus the commanded ff; bounded by L |
| hand-off hold, hold decay, positive-P cap | removed: nothing to hand off; one law from the first tick |
| restick detection | stall monitor (B) |
| learned breakaway preload | removed: the Nucleo's friction model provides breakaway current |
| reversal filter, edge-timing speed, plausibility cap | inputs to `speed_estimate()`: kept only where Stage 4a recorded data needs them; each lesson first becomes a test |
| FAILED latch | FAULTED(STALL) / FAULTED(HALL_LOST) with explicit clear |
