# Motor command protocol v1

Implemented by the shared codec `src/shared/src/motor_protocol.c` (`src/shared/inc/motor_protocol.h`), used by the
Nucleo, the ESCs and the tools. Lifecycle semantics: [command-lifecycle.md](command-lifecycle.md).

## Transport

- CAN FD frames, no bit-rate switch, 500 kbit/s nominal (both ends: ESC 170 MHz / 2 / 170 tq, Nucleo
  40 MHz / 4 / 20 tq). Every frame of this protocol has DLC exactly 8. A frame on a protocol ID with any other
  DLC is dropped and counted (`rx_bad_length`); it gets no reply.
- 11-bit ID = slot (bits 10-7) | sub-ID (bits 6-4) | select (bits 3-0). Select bits: FR 0x1, BR 0x2, BL 0x4,
  FL 0x8 (unchanged, `can_datatypes.h`). The ESC hardware filter accepts any ID with its select bit set.
- All multi-byte fields little endian. Signed fields two's complement.
- One codec, `src/shared/src/motor_protocol.c`, compiled into the ESC, the Nucleo and the PC tools. Nothing else
  packs or unpacks these frames.

## ID allocation

The 4-bit slots 1-7, 9-11, 14, 15 are taken by the existing protocol and 12-13 by firmware update (`0x600-0x6FF`).
Slots 0 and 8 are free; like the firmware-update protocol, they use bits 6-4 as a sub-ID.

| ID | Name | Direction | Note |
|---|---|---|---|
| `0x010 \| sel` | CONTROL | Nucleo -> ESC | highest bus priority of all traffic |
| `0x020 \| sel` | CONFIG | Nucleo -> ESC | one wheel; IDLE only |
| `0x410 \| sel` | CONTROL_REPLY | ESC -> Nucleo | one per received CONTROL frame |
| `0x420 \| sel` | CONFIG_REPLY | ESC -> Nucleo | one per CONFIG |

Retired and never reused: `SPEED_WRITE 0x080` (replaced by CONTROL), `MOTOR_STOP 0x780` (replaced by a broadcast
DISABLE), `CONFIG_SET 0x500` (replaced by CONFIG). Unchanged: STATUS, DIAG, IDENT,
HALL_ALIGN, CAPTURE/record, firmware update, boot request. An old-protocol frame can never decode as a new one:
the IDs differ.

## CONTROL (8 bytes)

| Byte | Field | Meaning |
|---|---|---|
| 0 | version (bits 7-4) \| mode (bits 3-0) | version 1; mode 0 DISABLE, 1 SPEED, 2 CURRENT |
| 1 | flags | bit 0 ARM, bit 1 CLEAR_FAULT; bits 7-2 reserved, must be 0 |
| 2-3 | int16 setpoint | SPEED: motor RPM (mechanical; direct drive, so wheel RPM). CURRENT: q-axis phase current, mA. DISABLE: must be 0 |
| 4-5 | int16 ff | SPEED only: friction feed-forward, q-axis mA, acting along the setpoint only. Must be 0 in other modes |
| 6 | seq | u8 sequence, per wheel, chosen by the Nucleo |
| 7 | session | u8, the session the ESC announced (reply byte 4) |

Current is always signed q-axis (torque-producing) motor phase current; positive q-current produces positive RPM.
It is never battery current. The ESC converts with `CURRENT_CONV_FACTOR` counts per A (544 in this build).

### Structural validation (codec, stateless; reason in brackets)

1. DLC != 8: dropped, counted, no reply.
2. version != 1 [BAD_VERSION]; mode > 2 [BAD_MODE].
3. reserved flag bits set [RESERVED_NONZERO]; ARM or CLEAR_FAULT with mode != DISABLE, or both together
   [BAD_FLAGS].
4. DISABLE with setpoint or ff != 0, CURRENT with ff != 0 [RESERVED_NONZERO].
5. No select bit, or more than one select bit on anything but a plain DISABLE (mode DISABLE, no flags); only a
   plain DISABLE may be broadcast [BAD_SELECT].

The rules are checked in this order and the first violated one is reported. NULL pointers count as BAD_LENGTH.

### Range validation (ESC, against its own compile-time limits)

SPEED |setpoint| <= `MAX_APPLICATION_SPEED_RPM` (2976); |ff| and CURRENT |setpoint| <= `IQMAX_A` (6000 mA)
[OUT_OF_RANGE]. Out-of-range values are rejected, never clamped. A value within the hard limit but above the
effective limit (configured limit or I2T derate) is applied and limited; the reply says INHIBITED(LIMITED).

### Session and sequence (ESC, stateful)

- The ESC holds `session` (nonzero u8). It picks a new one on boot, command timeout, FAULTED -> cleared, and every
  disarm: `next = 1 + (prev - 1 + 1 + (entropy mod 254)) mod 255`, which is never 0 and never equal to `prev`.
  `entropy` is the 1 kHz tick counter. At boot (`prev = 0`): `next = 1 + entropy mod 255`. Every reply carries the current `session` in byte 4.
- ARM (mode DISABLE, flag ARM) is accepted only when byte 7 equals the current session [BAD_SESSION]. It sets the
  sequence baseline `last_seq = seq`.
- Motion frames (SPEED, CURRENT) need lifecycle armed [NOT_ARMED] and byte 7 == session [BAD_SESSION]. Then, with
  `d = (int8_t)(seq - last_seq)`:
  - `d == 0`: payload identical to the last accepted frame -> DUPLICATE (reply, no apply, no watchdog refresh);
    different -> REJECTED [CONFLICT].
  - `d == -128`: REJECTED [SEQ_AMBIGUOUS]. `-127 <= d < 0`: REJECTED [STALE].
  - `1 <= d <= 16`: accepted (up to 15 lost frames, 300 ms). `d > 16`: REJECTED [SEQ_JUMP].
- A plain DISABLE is honoured whatever its session or sequence (stopping is the safe direction); it does not
  change `last_seq`, and it never refreshes the command watchdog.
- Freshness assumption (documented, not enforced): CAN has no store-and-forward, and the Nucleo TX FIFO holds at most
  a few frames, so no frame is older than a few ms when it arrives. A frame from an earlier session matches the new
  session with probability 1/255 per disarm, and it must then also pass the sequence window.

### Timing

- The Nucleo sends CONTROL to each wheel every 20 ms (`MOTOR_PROTOCOL_PERIOD_MS`).
- The ESC moves its applied speed reference linearly from its present value to a new SPEED setpoint over
  `MOTOR_PROTOCOL_PERIOD_MS` (a protocol constant, not a field). That reconstructs the Nucleo's slewed trajectory;
  acceleration limiting happens only on the Nucleo.
- Command watchdog: age = now - receive time of the last accepted fresh motion frame (ISR timestamp, 1 ms). Trip at
  age >= 100 ms: 3 lost frames are tolerated, a 4th trips. The age is also checked at each frame's receive time, so a
  frame arriving exactly at the deadline finds the wheel disarmed even when the tick drains it in the same millisecond. DISABLE, ARM, duplicates, rejects, CONFIG and polls do not
  refresh it. Trip action: output off, lifecycle DISARMED, new session, reason TIMEOUT.

## CONTROL_REPLY (8 bytes, one per received CONTROL frame of DLC 8)

| Byte | Field |
|---|---|
| 0 | request_seq: byte 6 of the frame answered |
| 1 | last_applied_seq: seq of the motion frame the controller is using now (0 after a disarm) |
| 2 | bits 2-0 lifecycle (0 DISARMED, 1 READY, 2 STARTING, 3 RUNNING, 4 HOLD, 5 FAULTED); bits 5-3 outcome (0 APPLIED, 1 INHIBITED, 2 SUPERSEDED, 3 REJECTED, 4 DUPLICATE); bit 6 first_edge (a Hall edge along the command since the current start); bit 7 0 |
| 3 | reason (table below; 0 for a plain APPLIED) |
| 4 | session the ESC accepts now (the challenge while DISARMED) |
| 5-6 | int16 measured speed, RPM (the one speed estimate of [C](../technical-development-plans/esc-command-v2/C-ownership.md)); 0 when not valid |
| 7 | int8 measured q-current, 50 mA units, nearest step with halves away from zero, saturated at +-127 (+-6.35 A covers the 6 A limit); 0 while the output is off (the old STATUS showed ~25 A on IDLE wheels on 2026-10-20, so MCSDK's Iq is not meaningful with the bridge off; cause confirmed in Stage 0) |

- Produced by the 1 kHz tick that processed the frame, not by the ISR, so APPLIED means "the target is handed to
  the controller this tick". The speed loop runs it from its next step (at most 1 ms later: the application hook
  follows MCSDK's medium-frequency task); that offset is the same on every wheel, so it does not change the
  command-apply skew. It does not mean the speed was reached.
- SUPERSEDED: accepted, then replaced in the single motion slot by a newer frame before a tick consumed it.
- INHIBITED: applied, but the output cannot follow. Reasons: LIMITED (effective limit below the request), STARTING
  (MCSDK start in progress), FF_UNUSED (18, retired with the old speed loop: never sent, never reused), and
  the FAULTED reasons.
- Pending replies sit in a 4-entry ring; when it is full the newest is dropped and counted (`reply_dropped`, ESC
  state over SWD); see [command-lifecycle.md](command-lifecycle.md) for why.
- The Nucleo timestamps each reply on receipt. Command-apply skew across wheels is the spread of those receipt
  times within one 20 ms cycle, with error = reply queueing (bounded, measured at H2).

### Reasons (u8)

| Code | Name | Code | Name |
|---|---|---|---|
| 0 | NONE | 12 | NOT_ALIGNED |
| 1 | BAD_VERSION | 13 | BUSY (align or capture running) |
| 2 | BAD_MODE | 14 | MODE_CHANGE (mode differs from the running one; DISABLE first) |
| 3 | BAD_FLAGS | 15 | NOT_IDLE (CLEAR_FAULT while the motor is not stopped) |
| 4 | RESERVED_NONZERO | 16 | LIMITED |
| 5 | BAD_SELECT | 17 | STARTING |
| 6 | OUT_OF_RANGE | 18 | FF_UNUSED |
| 7 | NOT_ARMED | 32 | FAULT_MCSDK (over-current, voltage, temperature) |
| 8 | BAD_SESSION | 33 | FAULT_STALL |
| 9 | STALE | 34 | FAULT_HALL_LOST |
| 10 | SEQ_AMBIGUOUS | 35 | TIMEOUT |
| 11 | SEQ_JUMP | 36 | LIFECYCLE_OVERFLOW |
| 19 | CONFLICT | 20 | BAD_LENGTH (DLC != 8 or wrong ID base; counted, never sent) |
| 21 | UNSUPPORTED (valid mode this firmware does not implement yet) | | |

Codes 32 and up are latched causes. While FAULTED every reply carries the cause as its reason (and the Nucleo keeps
it as `ik_drive_link.fault_reason`). The cause of the last disarm (timeout, overflow) is kept in the ESC's
`drive_cmd.disarm_cause` until the next ARM, readable over SWD. DIAG is unchanged (its 8 bytes are full).

## CONFIG and CONFIG_REPLY

CONFIG: byte 0 version (bits 7-4) | op (bits 3-0: 1 SET, 2 GET); byte 1 config seq; bytes 2-3 u16 key; bytes 4-5
int16 value (SET; 0 for GET); bytes 6-7 reserved 0. Accepted only when the motor is IDLE (lifecycle DISARMED or
READY, MCSDK IDLE, output off); a CONFIG can never start or arm the motor.
CONFIG_REPLY: byte 0 config seq; byte 1 status (0 OK, else a reason code above or 64 UNKNOWN_KEY, 65 BAD_VALUE
(range or cross-key check)); bytes 2-3 key; bytes 4-5 stored value; bytes 6-7 effective value (for a limit key: the
limit in force after I2T derate; else equal to the stored value).
Keys: `0x0001-0x00FF` limits, `0x0100-0x01FF` tuning. RAM only: compiled defaults on every boot. A CONFIG goes to
exactly one wheel (BAD_SELECT otherwise). Retires the 4-byte `CONFIG_SET 0x500`.

| Key | Name | Unit, range |
|---|---|---|
| `0x0001` | IQ_LIMIT_MA | mA, 1..6000 (IQMAX); effective = min(stored, I2T derate) |
| `0x0002` | HALL_SHIFT_DEG | electrical degrees, -359..359; a SET marks the wheel aligned |
| `0x0100` | SPEED_KP | counts per RPM x 256, 0..32767 |
| `0x0101` | SPEED_KI | counts per RPM per ms x 16384, 0..32767 |
| `0x0102` | CURRENT_KP | MCSDK units (Iq and Id), 0..32767 |
| `0x0103` | CURRENT_KI | MCSDK units (Iq and Id), 0..32767 |
| `0x0104` | STALL_RAMP_MA_PER_S | mA/s, 2000..30000 |
| `0x0105` | STALL_MS | ms, 200..2000 |

Reply status order: structural reason, then NOT_IDLE (15), then UNKNOWN_KEY, then BAD_VALUE. A refused SET leaves the
stored value unchanged and the reply carries it.

## Golden vectors (CTest, shared by the ESC, Nucleo and tool tests)

| Case | ID | Bytes | Decode |
|---|---|---|---|
| SPEED +120 RPM, FR, seq 5, session 0x3C | `0x011` | `11 00 78 00 00 00 05 3C` | ok |
| SPEED -120 RPM, ff -1500 mA, seq 6 | `0x011` | `11 00 88 FF 24 FA 06 3C` | ok |
| CURRENT +2500 mA, FL, seq 7 | `0x018` | `12 00 C4 09 00 00 07 3C` | ok |
| ARM, challenge 0x9D, seq 0 | `0x012` | `10 01 00 00 00 00 00 9D` | ok |
| DISABLE broadcast | `0x01F` | `10 00 00 00 00 00 2A 00` | ok |
| CLEAR_FAULT | `0x014` | `10 02 00 00 00 00 00 3C` | ok |
| version 2 | `0x011` | `21 00 78 00 00 00 05 3C` | BAD_VERSION |
| mode 3 | `0x011` | `13 00 78 00 00 00 05 3C` | BAD_MODE |
| ARM with SPEED | `0x011` | `11 01 78 00 00 00 05 3C` | BAD_FLAGS |
| ARM and CLEAR_FAULT | `0x011` | `10 03 00 00 00 00 05 3C` | BAD_FLAGS |
| reserved flag bit 2 | `0x011` | `11 04 78 00 00 00 05 3C` | RESERVED_NONZERO |
| CURRENT with ff | `0x011` | `12 00 C4 09 01 00 05 3C` | RESERVED_NONZERO |
| DISABLE with setpoint | `0x011` | `10 00 01 00 00 00 05 3C` | RESERVED_NONZERO |
| SPEED broadcast | `0x013` | `11 00 78 00 00 00 05 3C` | BAD_SELECT |
| reply: seq 5 applied, RUNNING, first edge, 118 RPM, 1.25 A | `0x411` | `05 05 43 00 3C 76 00 19` | ok |
| CONFIG SET IQ_LIMIT_MA 4000, FR, seq 7 | `0x021` | `11 07 01 00 A0 0F 00 00` | ok |
| CONFIG GET SPEED_KP, FL, seq 8 | `0x028` | `12 08 00 01 00 00 00 00` | ok |
| CONFIG op 3 | `0x021` | `13 07 01 00 A0 0F 00 00` | BAD_MODE |
| CONFIG GET with a value | `0x021` | `12 08 00 01 05 00 00 00` | RESERVED_NONZERO |
| CONFIG to two wheels | `0x023` | `11 07 01 00 A0 0F 00 00` | BAD_SELECT |
| CONFIG_REPLY seq 7 OK, IQ_LIMIT 4000 stored, 3000 effective | `0x421` | `07 00 01 00 A0 0F B8 0B` | ok |

Sequence cases (last_seq = 250): 251 ok, 10 ok (d = 16), 11 SEQ_JUMP (d = 17), 250 DUPLICATE or CONFLICT,
249 STALE, 122 SEQ_AMBIGUOUS (d = -128).

## Bus load (estimate; measured at H2)

Per 20 ms: 4 CONTROL + 4 CONTROL_REPLY + up to 4 STATUS/DIAG polls and replies = about 16 frames of ~110-130 bits
(8-byte FD frame without BRS, with stuffing) = ~1900 bits = ~4 ms of every 20 ms, about 20 % at 500 kbit/s.
Recorder readouts (CAPTURE_REPLY bursts) only run while the drive is stopped.
