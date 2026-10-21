# B. ESC command lifecycle (design, for review)

Status: design freeze draft for [phase1-esc-command-v2](../phase1-esc-command-v2.md), Stage 1. Wire format:
[A](A-protocol.md). Who owns the current request in each state: [C](C-ownership.md).

## States

| State | Output stage | Iq request owner | Meaning |
|---|---|---|---|
| DISARMED | off | none | after boot, timeout, or a cleared fault; motion rejected (NOT_ARMED) until ARM |
| READY | off | none | armed, motor stopped; HALL_ALIGN, CAPTURE and CONFIG allowed here and in DISARMED |
| STARTING | on | MCSDK start-up | `MC_StartMotor1` issued, waiting for MCSDK RUN |
| RUNNING | on | `motor_speed` (SPEED) or the setpoint (CURRENT) | regulating |
| HOLD | on | fixed Iq = 0 | parked: zero speed reached; no speed loop, so it cannot buzz; restarts without an MCSDK start |
| FAULTED | off | none | latched cause in `reason` (32+); needs DISABLE+CLEAR_FAULT |

"IDLE" (for CONFIG, CLEAR_FAULT, align, capture) = state DISARMED or READY, MCSDK state IDLE and output off. A zero
setpoint is not IDLE.

## Transitions

The 1 kHz tick evaluates, in this order: (1) MCSDK fault, (2) lifecycle FIFO, (3) watchdog, (4) motion slot,
(5) monitors. The first transition that fires in a tick wins for that step.

| From | Event | To | Action |
|---|---|---|---|
| any with output on | MCSDK fault (FAULT_NOW/FAULT_OVER) | FAULTED | reason FAULT_MCSDK; MCSDK already disabled the bridge |
| DISARMED | ARM with the current session | READY | `last_seq = seq`, `last_applied_seq = 0` |
| any except FAULTED | ARM with the current session while armed | unchanged | re-baselines `last_seq`; reply APPLIED |
| any | ARM with a wrong session | unchanged | REJECTED(BAD_SESSION) |
| STARTING, RUNNING, HOLD | DISABLE | READY | `MC_StopMotor1`, integrator and ff cleared, motion slot cleared |
| DISARMED, READY | DISABLE | unchanged | none (idempotent) |
| FAULTED | DISABLE | FAULTED | none (output already off) |
| FAULTED | CLEAR_FAULT, MCSDK fault gone and MCSDK IDLE | DISARMED | acknowledge the MCSDK fault, new session |
| FAULTED | CLEAR_FAULT, cause still present or not IDLE | FAULTED | REJECTED(NOT_IDLE or the latched reason) |
| not FAULTED | CLEAR_FAULT | unchanged | APPLIED (nothing latched) |
| READY | accepted motion frame, aligned | STARTING | `MC_StartMotor1`, mode latched, reply INHIBITED(STARTING) |
| READY | accepted motion frame, not aligned | READY | REJECTED(NOT_ALIGNED) |
| STARTING | MCSDK reaches RUN | RUNNING | the integrator starts at 0, ff slew starts at 0 |
| STARTING | no RUN within 500 ms | FAULTED | reason FAULT_MCSDK, `MC_StopMotor1` |
| RUNNING | motion frame with a different mode | unchanged | REJECTED(MODE_CHANGE) |
| RUNNING, SPEED | applied reference = 0 and \|speed\| < 45 RPM for 150 ms | HOLD | speed loop off, integrator cleared, Iq request 0 |
| HOLD | SPEED frame with setpoint != 0 | RUNNING | speed loop on, integrator 0, ff slew from 0 |
| HOLD | CURRENT frame | unchanged | REJECTED(MODE_CHANGE): mode is latched until DISABLE |
| RUNNING | stall monitor fires | FAULTED | reason FAULT_STALL, `MC_StopMotor1` |
| RUNNING | speed estimate invalid after it was valid, beyond the stale window | FAULTED | reason FAULT_HALL_LOST, `MC_StopMotor1` |
| armed, not FAULTED | watchdog age >= 100 ms | DISARMED | `MC_StopMotor1` if on, new session, reason TIMEOUT (sticky in DIAG until the next ARM) |
| any | lifecycle FIFO overflow | DISARMED (FAULTED stays FAULTED) | `MC_StopMotor1`, new session, LIFECYCLE_OVERFLOW counted |

Notes:
- A DISABLE keeps the session: the Nucleo can resume with the next sequence without a new ARM. Losing commands
  (timeout) or clearing a fault always requires a fresh ARM against a new session.
- Mode (SPEED or CURRENT) is latched on READY -> STARTING and only changes through DISABLE. CURRENT with setpoint 0
  keeps the output on with zero q-current; it is not HOLD and has no parking timer.
- No automatic restart after FAULTED: the Nucleo must DISABLE, CLEAR_FAULT, ARM.
- Stall monitor (RUNNING only): |Iq request| >= 80 % of the effective limit, |applied reference| >= 10 RPM, and no
  Hall edge along the command for 500 ms (CONFIG 200-2000 ms). The timer resets on any edge along the command, a
  reference sign change, |reference| < 10 RPM, or leaving RUNNING. In CURRENT mode the stall monitor is off (a
  roll-out against a wall is a legitimate test); the current limit and I2T protect the motor there.

## Ordering of received frames (ISR -> tick)

- The ISR decodes and structurally validates each frame, stamps it with a monotonically increasing ordinal and the
  1 kHz receive time, then publishes it under a short critical section (interrupts off around a copy of under 16
  bytes):
  - DISABLE, ARM and CLEAR_FAULT go into a 4-entry ordered FIFO. When it is full, the ISR sets an overflow flag
    instead (tick: forced DISABLE + disarm).
  - SPEED and CURRENT go into one motion slot (latest wins). Overwriting an unconsumed motion frame records its
    seq for a SUPERSEDED reply.
  - Structural rejects go straight to the reply ring with outcome REJECTED.
- The tick drains the lifecycle FIFO in order, then takes the motion slot. A motion frame whose ordinal is lower
  than that of the last consumed DISABLE is discarded (outcome SUPERSEDED), so "motion, DISABLE" never ends moving,
  and nothing re-arms after a stop.
- A motion frame whose receive age is >= 100 ms when the tick reaches it expires (REJECTED, reason TIMEOUT) and does
  not refresh the watchdog.

## Retired behaviour

- Starting a motor as a side effect of the first non-zero SPEED_WRITE, with no arming: replaced by ARM + motion.
- MOTOR_STOP: replaced by a broadcast DISABLE (honoured whatever its session or sequence). It also aborts a running
  HALL_ALIGN or CAPTURE.
- Output-off parking (`ESC_STOP_PARKED`, ~30 ms MCSDK restart): replaced by HOLD in Stage 4a. The Stage 2 adapter
  keeps the old parking so nominal traces stay comparable.
