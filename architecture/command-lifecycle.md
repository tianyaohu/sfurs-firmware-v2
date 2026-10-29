# ESC command lifecycle

Host slice in `src/shared/src/motor_cmd.c`. It is not linked into the Nucleo
or ESC image. Wire format: [protocol.md](protocol.md). One wheel, one
CONTROL at a time. `motor_cmd_control` decodes with `motor_control_decode`
and answers with `motor_reply_encode` on `cfg.select`.

`motor_cmd_inputs` is the caller's clock (`now_ms`) and bridge sample
(`mc` IDLE / STARTING / RUN / FAULT, `aligned`, `busy`). NULL inputs do
not sample: the watchdog does not run and an accepted motion stays READY.
`motor_cmd_tick` is that sample with no frame. `motor_cmd_actions` is
`start`, `set_target`, and `stop`. A set-target reads `mode`, `setpoint`,
and `ff_ma` from the state.

## States

| State | Meaning here |
|---|---|
| DISARMED | after `motor_cmd_init`, or after the command watchdog. Motion is `NOT_ARMED`. |
| READY | ARM accepted. The bridge is off. A nonzero target starts when the sample allows it. |
| STARTING | `start` and `set_target` were requested. Waiting for `mc == RUN`. |
| RUNNING | the sample reported RUN. A further accepted motion is `APPLIED` and sets `set_target`. |
| FAULTED | STARTING waited 500 ms without RUN, or a caller set `lifecycle` and `latched`. ARM and motion are `REJECTED` with `latched`. |

HOLD is not used.

## Clock

`MOTOR_WATCHDOG_MS` is 100. `watch_ms` is the receive time of the ARM that
left DISARMED, or of the last accepted fresh motion. A repeated ARM,
DISABLE, a duplicate, and any reject do not change it.

On a sampled `motor_cmd_control`, and on `motor_cmd_tick`, in this order:

1. If armed (READY, STARTING, or RUNNING) and `rx_ms - watch_ms >= 100`
   (signed), disarm before the frame is judged. A motion timestamped on
   that deadline is then `NOT_ARMED`. The signed difference is so a frame
   older than the last command is not treated as an expiry.
2. A motion frame with age `now_ms - rx_ms >= 100` is `REJECTED(TIMEOUT)`
   and is not applied. It does not refresh `watch_ms`. DISABLE and ARM are
   not expired by age. The TIMEOUT reply is the lifecycle at that verdict.
3. After the verdict, if still armed and `now_ms - watch_ms >= 100`, disarm.

A fresh motion is answered after step 3 and the start rules, so its reply
already shows STARTING or RUNNING. ARM, DISABLE, duplicates, and rejects
are answered at the verdict; step 3 still runs afterwards. A late ARM
(receive time already 100 ms behind `now_ms`) is APPLIED, then that same
call disarms.

Disarm: lifecycle DISARMED, `session = motor_session_next(session, now_ms)`,
clear the setpoint, feed-forward, mode, and applied sequence, `last_valid`
0. `disarm_reason` stays `TIMEOUT` until the next accepted ARM.
`actions.stop` is set only when the bridge was on. The next motion frame
is `NOT_ARMED`, not TIMEOUT.

## Start

Sampled after the frame (and on tick) when the watchdog did not trip:

- READY, nonzero setpoint, `aligned`, not `busy`, `mc == IDLE` → STARTING,
  `state_ms = now_ms`, `actions.start` and `actions.set_target`.
- STARTING and `mc == RUN` → RUNNING.
- STARTING and `now_ms - state_ms >= 500` (`MOTOR_CMD_START_TIMEOUT_MS`)
  without RUN → FAULTED, `latched = FAULT_MCSDK`, `actions.stop`, target
  cleared. The session is kept.

An accepted nonzero motion answered while the lifecycle is still READY or
STARTING is `INHIBITED(STARTING)`. Not aligned, busy, or `mc` other than
IDLE records the target and waits; it does not reject. RUNNING is
`APPLIED`. A zero setpoint does not start.

DISABLE while STARTING or RUNNING: `actions.stop`, lifecycle READY, target
and applied sequence cleared, session kept. DISABLE while DISARMED, READY,
or FAULTED does not change the lifecycle.

## One CONTROL

Checked in this order. NULL inputs stop before the clock and the start
rules, and an accepted motion is then APPLIED in READY.

A frame that is not a DLC-8 CONTROL is `BAD_LENGTH` and produces no reply.
The state is unchanged. Any other structural failure (`BAD_VERSION`,
`BAD_MODE`, `BAD_FLAGS`, `RESERVED_NONZERO`, `BAD_SELECT`) is `REJECTED`
with that reason and does not change state, apart from a sampled watchdog
trip. A frame whose select does not include this wheel is `BAD_SELECT`. A
plain DISABLE may name any non-empty subset that includes this wheel.

Plain DISABLE is `APPLIED`. The recorded setpoint, feed-forward, and
applied sequence are cleared. From STARTING or RUNNING the lifecycle
returns to READY and `actions.stop` is set. Otherwise the lifecycle stays,
including when the caller has set FAULTED. The last accepted payload is
kept, so a retransmission of that motion is still `DUPLICATE` and does not
restore the setpoint.

ARM (`DISABLE` + `ARM`):

- FAULTED → `REJECTED` with `latched`, before the session is checked.
- Wrong session → `BAD_SESSION`, state unchanged.
- DISARMED → READY, `last_seq` is this frame's sequence, `watch_ms` is
  `rx_ms`, `disarm_reason` clears, `APPLIED`.
- Already armed → stay in that state, re-baseline `last_seq`, forget the
  stored payload (`last_valid` clears). `watch_ms` is not refreshed. The
  recorded setpoint stays until DISABLE or a newer accepted motion frame.

`CLEAR_FAULT` is `REJECTED(UNSUPPORTED)` and changes nothing. This slice
has no MCSDK fault word and does not acknowledge one.

Motion (`SPEED` or `CURRENT`), first failure wins:

1. Age `>= 100` ms when inputs were passed → `REJECTED(TIMEOUT)`, not
   applied. This is before the lifecycle check, so a late frame is
   TIMEOUT even when the wheel is disarmed or faulted.
2. FAULTED → `REJECTED(latched)`. DISARMED → `NOT_ARMED`. READY, STARTING, and RUNNING continue.
3. Session matches. Otherwise `BAD_SESSION`.
4. `motor_seq_classify`:
   - `DUPLICATE` and the same mode, flags, setpoint, feed-forward, and session → outcome `DUPLICATE`, reason none, state unchanged. Does not refresh `watch_ms`.
   - `DUPLICATE` with a different payload, or with no payload stored yet → `CONFLICT`.
   - `STALE`, `SEQ_AMBIGUOUS`, `SEQ_JUMP`.
5. `|setpoint|` above `max_rpm` (SPEED) or `max_ma` (CURRENT), or `|feed-forward|` above `max_ma` → `OUT_OF_RANGE`. The limit itself is accepted.
6. `CURRENT` while `current_supported` is 0 → `UNSUPPORTED`.

On accept: update `last_seq`, the stored payload, and `watch_ms` (`rx_ms`);
record the setpoint, feed-forward, mode, and applied sequence. With inputs,
the start rules then run and a nonzero target that is not yet RUNNING is
`INHIBITED(STARTING)`. Without inputs the lifecycle stays where it was and
the reply is `APPLIED`. Measured RPM and current in the reply are 0.

Boot session is `motor_session_next(0, entropy)`. Entropy equal to the
wheel select bit is `1 + select` (FR 2, BR 3, BL 5, FL 9). A watchdog
disarm uses `now_ms` as the entropy.

## Deferred

- ISR inbox, ordered drain, `SUPERSEDED`, `LIFECYCLE_OVERFLOW`
- HOLD, park
- Hall loss, stall rest, `MODE_CHANGE`, `NOT_ALIGNED` and `BUSY` as rejects, `LIMITED`
- MCSDK fault while RUNNING, and `CLEAR_FAULT` back to DISARMED
- CONFIG frames
- Linking this file into the ESC or Nucleo image
