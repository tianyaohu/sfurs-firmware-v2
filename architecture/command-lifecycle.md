# ESC command lifecycle

Host slice in `src/shared/src/motor_cmd.c`. It is not linked into the Nucleo
or ESC image. Wire format: [protocol.md](protocol.md). One wheel, one
CONTROL at a time. `motor_cmd_control` decodes with `motor_control_decode`
and answers with `motor_reply_encode` on `cfg.select`.

## States

| State | Meaning here |
|---|---|
| DISARMED | after `motor_cmd_init`. Motion is `NOT_ARMED`. |
| READY | ARM accepted. An accepted motion frame is recorded here. The bridge stays off. |
| FAULTED | not entered by this slice. A caller that sets `lifecycle` and `latched` gets ARM `REJECTED` with that latched reason. |

STARTING, RUNNING, and HOLD are not used. An accepted setpoint stays in
READY so a host test can read it.

## One CONTROL

Checked in this order.

A frame that is not a DLC-8 CONTROL is `BAD_LENGTH` and produces no reply.
The state is unchanged. Any other structural failure (`BAD_VERSION`,
`BAD_MODE`, `BAD_FLAGS`, `RESERVED_NONZERO`, `BAD_SELECT`) is `REJECTED`
with that reason and does not change state. A frame whose select does not
include this wheel is `BAD_SELECT`. A plain DISABLE may name any non-empty
subset that includes this wheel.

Plain DISABLE is `APPLIED`. The recorded setpoint, feed-forward, and
applied sequence are cleared. The lifecycle stays where it is: there is no
output-on state to bring back to READY, including when the caller has set
FAULTED. The last accepted payload is kept, so a retransmission of that
motion is still `DUPLICATE` and does not restore the setpoint.

ARM (`DISABLE` + `ARM`):

- FAULTED → `REJECTED` with `latched`, before the session is checked.
- Wrong session → `BAD_SESSION`, state unchanged.
- DISARMED → READY, `last_seq` is this frame's sequence, `APPLIED`.
- Already READY → stay READY, re-baseline `last_seq`, forget the stored
  payload (`last_valid` clears). The recorded setpoint stays until DISABLE
  or a newer accepted motion frame.

`CLEAR_FAULT` is `REJECTED(UNSUPPORTED)` and changes nothing. This slice
has no MCSDK fault word and does not acknowledge one. A later slice can
return FAULTED to DISARMED when `mc_fault_present` is 0.

Motion (`SPEED` or `CURRENT`), first failure wins:

1. Lifecycle is READY. Anything else, including FAULTED and DISARMED, is `NOT_ARMED`.
2. Session matches. Otherwise `BAD_SESSION`.
3. `motor_seq_classify`:
   - `DUPLICATE` and the same mode, flags, setpoint, feed-forward, and session → outcome `DUPLICATE`, reason none, state unchanged.
   - `DUPLICATE` with a different payload, or with no payload stored yet → `CONFLICT`.
   - `STALE`, `SEQ_AMBIGUOUS`, `SEQ_JUMP`.
4. `|setpoint|` above `max_rpm` (SPEED) or `max_ma` (CURRENT), or `|feed-forward|` above `max_ma` → `OUT_OF_RANGE`. The limit itself is accepted.
5. `CURRENT` while `current_supported` is 0 → `UNSUPPORTED`.

On accept: update `last_seq` and the stored payload, record the setpoint,
feed-forward, mode, and applied sequence, stay READY, reply `APPLIED`.
Measured RPM and current in the reply are 0 because the bridge is off.

Boot session is `motor_session_next(0, entropy)`. Entropy equal to the
wheel select bit is `1 + select` (FR 2, BR 3, BL 5, FL 9).

## Deferred

- ISR inbox, ordered drain, `SUPERSEDED`, `LIFECYCLE_OVERFLOW`
- 100 ms watchdog and per-frame `TIMEOUT`
- STARTING, RUNNING, HOLD, park, and the MCSDK start and stop
- Start-timeout fault, Hall loss, stall rest, `MODE_CHANGE`, `NOT_ALIGNED`, `BUSY`, `LIMITED`
- `CLEAR_FAULT` back to DISARMED, and the new session that goes with a clear or a disarm
- CONFIG frames
- Linking this file into the ESC or Nucleo image
