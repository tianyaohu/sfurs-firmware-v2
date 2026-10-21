# D. Acceptance metrics and stage gates (design, for review)

Status: design freeze draft for [phase1-esc-command-v2](../phase1-esc-command-v2.md). The startup definition is
the user's (2026-10-21); this file makes it computable.

## Measurement facts that shape the rules

- 8 pole pairs (`POLE_PAIR_NUM`, consistent with the alignment sweep: 2 electrical turns ~ 1/4 wheel turn), so
  6 x 8 = 48 Hall transitions per wheel turn, nominally 7.5 deg apart. Stage 0 confirms by counting edges over one
  marked turn.
- The Hall state tells which of the 6 electrical sectors the rotor is in, never which pole pair. So the calibration
  is per wheel and per electrical sector (6 widths in mechanical degrees, summing to one electrical turn = 45 deg,
averaged over the 8 pole pairs). The pole-pair spread of each
  width is measured and becomes part of the tolerance.
- The rotor's position inside its starting sector is unknown.
- Edge times come from the ESC recorder at 1 kHz (+-1 ms). Each record row carries the net edge count (direction)
  and the raw Hall state of that tick (packed bits 12-14). s0 is the sector of the Hall state on the last
  zero-reference row; sectors are numbered in the MCSDK positive order 5-1-3-2-6-4 (sector 0 = state 5), and every
  later row's Hall state must be the sector its edges lead to, else the start is not measurable (`hall_sequence`).
  Without a calibration map sectors are nominal 7.5 deg and the tolerance is widened by the width spread measured in
  Stage 0.

## Startup tracking (per active wheel, per start)

Inputs: applied reference r(t) (RPM, the ESC's interpolated reference), Hall transitions with times and directions,
starting sector s0, calibrated widths w[0..5].
- Reference angle travelled since apply: theta_r(t) = integral of |r| dt (commanded direction).
- Boundary k (k = 1, 2, ...) ahead of the start in the commanded direction lies at distance D_k - u from the start,
  where D_k = sum of the widths from the start of sector s0 up to boundary k (known) and u in [0, w_s0] is the
  unknown offset.
- Deadline for transition k: T_k = t_ref(D_k) + 20 ms, where t_ref(x) is when theta_r reaches x. Taking u = 0 makes
  the deadline the latest the reference can cross that boundary, so a wheel that follows the reference exactly
  always meets it.
- Tolerance of transition k: tol_k = t_ref(D_k) - t_ref(D_k - w_s0 - delta_w), where delta_w is the measured width
  spread. It is the most by which a wheel can lag and still meet T_k. It shrinks as the reference speeds up.
- A transition is *measurable* when tol_k <= 20 ms. At 120 RPM with 900 RPM/s these are about k >= 3 (k = 1: ~53 ms,
  k = 2: ~22 ms, k = 3: ~17 ms, k = 4: ~14 ms, then decreasing).

**Pass, per wheel per start:** every transition k = 1..8 occurs by its deadline T_k, in the commanded direction;
and at least 3 of them are measurable. Missing any deadline fails, measurable or not (a stuck wheel fails early).
A start with fewer than 3 measurable transitions is "not measurable", never a pass.

## Synchronization

Command-apply skew: for the first non-zero CONTROL cycle of a start, skew = max - min over active wheels of the
Nucleo receipt times of their APPLIED replies. **Pass: skew <= 2 ms.** Error budget: replies leave on the tick that
applied the frame; 4 replies queue behind each other at about 0.25 ms each. Measured at H2 (p99 of reply latency).

## Overshoot

Measured speed = calibrated edge-interval speed, independent of the firmware's estimator: summed sector widths over
the shortest run of consecutive same-direction transitions that spans >= 25 ms, divided by that time. With 1 ms
edge timestamps this bounds the quantization error to about 4 % (a single 10 ms sector at 120 RPM would be +-10 %).
Evaluated at each transition once such a run exists. **Pass: max from the first transition to the end of the
recording <= 1.3 x that wheel's steady commanded speed** (the plateau of its applied reference; wheels differ within one twist,
the fastest is 120 RPM at the gate). The recorder holds
384 ms per start, so with the default 50 ms pre-trigger this window is at least 200 ms after the first transition at
the gate speed; a start whose window is shorter is "not measurable" for overshoot.

## Other failures (any one fails the start)

MCSDK fault, I2T derate, FAULTED(STALL / HALL_LOST), REJECTED or LOST on a motion frame, any of the first 8
transitions against the commanded direction (rocking is a startup failure, not noise).
Missing evidence never passes: a record without a zero-reference baseline, a reversing reference, or a wheel whose
reference has not plateaued (|ref| constant within +-1 RPM for the last 50 ms; the plateau is that wheel's steady
target for the overshoot ratio) is "not measurable". Two transitions inside one 1 ms tick in opposite directions
cancel in the recorder's net edge count and are not seen; edge times are taken at the end of their tick.

## Low-speed operating points

Where the startup tracking metric is not measurable (e.g. wz 1 rad/s, about 25 RPM per wheel), the report says "not
measured by Hall". Such points need an independently validated estimator (the AS5047P `wheel_encoder` driver, if fitted,
or the rig repo's poses) before anything can be claimed. They are not part of the gate.

## Gate operating points

120 RPM peak wheel speed per axis: vx ~ 0.72 m/s, vy ~ 0.47 m/s, wz ~ 4.8 rad/s, each sign; Nucleo slew 900 RPM/s;
on the mat; from rest (wheels in HOLD from Stage 4a on, motor off before); out and back so the robot stays on the
mat. Logged per start: Vbus, temperature class (cold: first 5 min; warm after), starting Hall state, firmware hashes.

## Stage gates

| Gate | Where | Must show | Starts |
|---|---|---|---|
| H0 | raised, then mat; current firmware | C tools work on the robot; IDLE Iq reading explained; 48 transitions per marked turn; sector-width spread; MCSDK start latency; baseline of all metrics above | 5 per axis/sign (30) |
| H1 | none (host) | design approved; codec golden vectors pass | - |
| H2 | raised, then mat | every reject reason produced on purpose and seen at the Nucleo; watchdog trips at 4 lost frames and not at 3; LOST disables all four; 5 min with no false trip; reply p99 and bus load; nominal traces match H0; metrics no worse than H0 (ff off) | 30 |
| H3 | raised | every CONFIG key set and read back on all four wheels; refused when not IDLE; effective limit tracks I2T | - |
| H4a | raised, then mat | new loop: metrics no worse than H0 with ff off; HOLD idle current, noise and temperature acceptable; stall and Hall-loss faults triggered on purpose | 30 |
| H5 | mat | current-mode roll-outs per axis and sign; friction fit with residuals; Ke fit | per fit |
| H6 | mat | full set: startup tracking, synchronization, overshoot, other failures; ff on with H5 values | 20 per axis/sign (120) |

A gate's result file in `hardware-test-results/` lists every start's verdict and the counts; counts are reported as
counts (120 starts is an agreed gate, not a reliability estimate).
