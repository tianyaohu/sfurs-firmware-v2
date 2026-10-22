# E. Stage 4a speed loop (design, decisions by Frank H 2026-10-21)

For [phase1-esc-command-v2](../phase1-esc-command-v2.md) Stage 4a. Amends C4/C5 and C-ownership where marked.
Evidence: hardware-test-results/2026-10-21-stage0-baseline-raised.md, 2026-10-21-h2-install-raised.md; raw data in
/home/fh/sfurs_firmware/hw-data/2026-10-21-*.

## Why (from the recorded data)
- Mat: breakaway needed 3.3-4.7 A (BL 0.47 s, FL 0.50 s, FR 0.67 s; BR still stuck at 4.45 A); creep edges 94-160 ms
  apart before. After breakaway FR went 110 -> 450 RPM in ~20 ms: the I term wound up while stuck.
- Raised: the same wind-up on a sticky start (BL 2.1 A, then 2.46x); the 1.3-1.5x overshoot is I plus the ~3-edge
  (~31 ms at 120 RPM) lag of the MCSDK 6-edge average. Reference slip on clean raised starts <= 2.75 edges (123/124).
- Sector widths +-6 % around the 6-edge mean (raised, all wheels). 60 RPM "scatter" is mostly real speed variation.

## Control law (1 kHz, `motor_speed.c`, pure C)
e = r - w_hat (w_hat = 0 while invalid); p = Kp*e; u = p + I + ff; iq = clamp(u, +-L), L = min(IQMAX, CONFIG limit,
I2T allowance). Gains unchanged from today (decision: Ki stays, C5's Ki 0 default is amended): Kp 1024/256 counts/RPM
(7.35 mA/RPM), Ki 553/16384 counts/RPM/ms (62 mA/s per RPM).
Integrator, first matching rule per tick:
- (a) Stalled ramp: slip >= 4 edges and |r| >= 10 RPM and |w_hat| < 45 RPM -> I moves along sign(r) at 10 A/s
  (CONFIG `STALL_RAMP_MA_PER_S`), never past L - p - ff. Slip = reference travel (|r|*48/60000 edges per ms) minus
  progress edges (new furthest position along the command; rocking adds nothing), clamped at 4: the loop never tries
  to win back distance lost while stuck.
- (b) Otherwise I += Ki*e, conditional integration on the saturated total (only if inside +-L or moving back inside).
- (c) Over-speed bleed: on a progress edge, if I has the command's sign and the speed of the latest run of consecutive
  progress edges spanning >= 8 ms (1250*k/(span_ms+1) RPM) is >= 1.15*|r|, I <- I/2 (truncating toward 0).
- I cleared on start, leaving HOLD, sign change of r, gain change, disable, fault.
Fixed point: p = (1024*e)>>8 with e clamped to +-5952, p clamped to +-L; I int32 Q14 counts, |I| <= L*2^14;
Ki step 553*e; stall ramp 89129 Q14/tick at 10 A/s; ff mA -> counts (ff*544 +- 500)/1000.
ff: slews toward ff_target when sign(ff) == sign(r), else 0, at 50 A/s; reset on sign change of r; default 0
(Stage 5 measures it). Reference: C5b interpolation over 20 ticks, a new setpoint restarts from the current value.

## Speed estimate (the only one)
Invalid until 2 consecutive same-direction edges since start/reversal, and while MCSDK's sign disagrees with the last
edge direction. Edges 2-6: from 1 kHz edge-tick intervals; from a full 6-edge buffer: MCSDK average. Always
|w_hat| <= 1250/(ms since last edge); 0 after the stale window clamp(2*last period, 20, 100) ms. Reply, recorder,
stall monitor and park check read this value. The plausibility cap and the reversal filter go.

## Faults and HOLD
- HALL_LOST (decision, amends C4): MCSDK's speed-feedback fault (Hall code 0/7) or an invalid Hall code longer than
  the stale window. An invalid estimate after motion is not a fault (a re-stuck wheel is the stall ramp's job).
- STALL: |iq_ref| >= 0.8*L, |r| >= 10 RPM, no progress edge for 500 ms (CONFIG `STALL_MS`); resets on progress edge,
  sign flip, |r| < 10, leaving RUNNING; while |iq_ref| < 0.8*L it pauses (keeps its count). Replaces loop_failed /
  STOP_LOOP_FAILED. "Saturated" also counts a P term clipped to +-L.
- HOLD: RUNNING, setpoint 0, |w_hat| < 45 RPM for 150 ms -> output stays on, I = ff = slip = 0, Iq ref 0. Any
  non-zero SPEED frame -> RUNNING with no MCSDK start. DISABLE/watchdog/fault leave HOLD like RUNNING; "MCSDK left
  RUN" also checked in HOLD. STOP_PARKED retired.

## Ownership (decision, amends C-ownership)
The loop runs in drive_app's 1 kHz tick right after the command lifecycle and publishes with
`MC_SetCurrentReferenceMotor1` (MCSDK torque mode). Our edits to mc_tasks_foc.c and speed_torq_ctrl.c are reverted
to stock. +1 ms latency, small against the estimator lag.

## Recorder and keys
Slots keep their meaning: s0 applied ref, s1 w_hat, s2/s3/s4 p/I/ff, s5 iq_ref, s6 measured Iq; s7 bits 0-1 mode
{off, run, hold, stall-ramp}, 2 valid, 3 saturated, 4 bleed, 5 stall timer, 8-15 unchanged. CONFIG: keep 1-6, add 25
STALL_RAMP_MA_PER_S (2000-30000), 26 STALL_MS (200-2000); 7-24 retired and rejected, numbers never reused.

## Deleted
drive_speed_ctrl.c/.h and its test + fakes; DSC, legacy static/kinetic ff, rev filter, plausibility cap, edge-speed
estimator and their keys, tool options (`--dsc*`, `--ff-static/kinetic`, `--edge-speed`), tests and doc sections;
motor_cmd `loop_failed`, `ff_supported`, FF_UNUSED path, STOP_PARKED/STOP_LOOP_FAILED.

## Host tests (production decisions only, no plant)
P rounding/overflow; conditional integration; limit change mid-tick; stall-ramp start/stop/pause/clamp and rocking;
bleed threshold incl. -6 % sector + 1-tick quantization not firing at 50/120/400 RPM; reversal clears; ff slew/sign;
HOLD entry/exit and stall timer; estimator validity/fill/1-over-t/stale window/counter wrap; motor_cmd HOLD and
HALL_LOST transitions; interpolation; drive_app publishes Iq only in RUNNING/HOLD; recorder bits. Replays of recorded
edge/reference traces assert decisions only (stall predicate timing on mat vy_p_0; no bleed on creep edges; stall
predicate false on raised starts except vy_p_1 BL).

## Gate H4a (decision: 4a goal = no runaway + correct faults; ff 0, D deadlines on the mat may still fail)
Raised: spins 60/120 RPM vs today; the 30-start set + wz 4.8 rad/s; bleed/stall-ramp counters 0 in steady spins; no
new failure types; overshoot no worse. HOLD 5 min: Iq, noise, temperature vs output off; wheel turns by hand.
Faults on purpose: blocked wheel -> STALL; Hall unplugged -> HALL_LOST; recover via DISABLE + CLEAR_FAULT + ARM.
Mat: the vy+ start that failed (breakaway earlier than 0.47-0.67 s, no overspeed abort), then 5 starts per axis/sign.
