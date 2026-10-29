# Motor frames

Codec: `src/shared/src/motor_protocol.c`. It is not linked into the Nucleo
or ESC image. Multi-byte fields are little endian, signed fields two's
complement. The codec does not apply RPM or current limits, and it does
not run the ESC command lifecycle. The session and sequence helpers are
pure: they do not remember a wheel.

11-bit ids. Wheel select bits: FR `0x1`, BR `0x2`, BL `0x4`, FL `0x8`.

CONTROL and CONTROL_REPLY put the select in bits 3-0 and the base in bits
10-4 (`0x010` and `0x410`). STATUS and DIAG put the command in bits 10-7
and the select in the low bits; bits 6-4 stay 0.

| Id | Name | Payload |
|---|---|---|
| `0x010 \| select` | CONTROL | 8 bytes, below |
| `0x410 \| select` | CONTROL_REPLY | 8 bytes, exactly one wheel |
| `0x100 \| select` | STATUS poll | 4 bytes, ignored. Select may be any non-empty subset of the four wheels (`0x10F` asks all of them). |
| `0x280 \| select` | STATUS reply | 8 bytes, exactly one wheel |
| `0x300 \| select` | DIAG poll | 4 bytes, ignored. Same select rule as a STATUS poll. |
| `0x380 \| select` | DIAG reply | 8 bytes, exactly one wheel |

A poll whose DLC is not 4, or whose command nibble is neither STATUS nor
DIAG, is `BAD_LENGTH` and does not write its outputs. A select of no wheel,
or of a bit outside the four wheel bits, is `BAD_SELECT` and also does not
write them. Encode stores four zero bytes; decode ignores whatever four
bytes arrived.

## CONTROL

| Byte | Field |
|---|---|
| 0 | version (bits 7-4) \| mode (bits 3-0). Version 1. Mode 0 DISABLE, 1 SPEED, 2 CURRENT. |
| 1 | flags. Bit 0 ARM, bit 1 CLEAR_FAULT. Bits 7-2 are 0. |
| 2-3 | int16 setpoint. SPEED: mechanical RPM. CURRENT: q-axis mA. DISABLE: 0. |
| 4-5 | int16 feed-forward, q-axis mA. SPEED only; 0 in the other modes. |
| 6 | sequence, chosen by the sender. |
| 7 | session byte. Carried, not interpreted. |

Positive q-axis current is the direction of positive RPM. It is motor phase
current, not battery current.

Checked in this order; the first failure is the one returned:

1. NULL, DLC other than 8, id above `0x7FF`, or id base other than `0x010`: `BAD_LENGTH` (20). The output struct is left untouched.
2. Version nibble other than 1: `BAD_VERSION` (1). The struct is still filled, because byte 6 is the sequence whatever the later check says.
3. Mode above 2: `BAD_MODE` (2).
4. A reserved flag bit set: `RESERVED_NONZERO` (4).
5. ARM or CLEAR_FAULT with a mode other than DISABLE, or both flags together: `BAD_FLAGS` (3).
6. DISABLE with a non-zero setpoint or feed-forward, or CURRENT with a non-zero feed-forward: `RESERVED_NONZERO` (4).
7. No wheel bit, a bit outside the four wheel bits, or more than one wheel bit on anything except a plain DISABLE (mode DISABLE and flags 0): `BAD_SELECT` (5). A plain DISABLE may name any non-empty subset.

Encode refuses the same field rules and does not write the id or the bytes
when it refuses. Encode always writes version 1.

## CONTROL_REPLY

One reply frame per CONTROL of DLC 8. The ESC lifecycle that would produce
it is not in this codec. Encode and decode only pack the bytes and require
exactly one wheel.

| Byte | Field |
|---|---|
| 0 | request sequence: byte 6 of the CONTROL this answers |
| 1 | applied sequence: the motion frame in use now, 0 after a disarm |
| 2 | bits 2-0 lifecycle (0 DISARMED, 1 READY, 2 STARTING, 3 RUNNING, 4 HOLD, 5 FAULTED); bits 5-3 outcome (0 APPLIED, 1 INHIBITED, 2 SUPERSEDED, 3 REJECTED, 4 DUPLICATE); bit 6 first Hall edge along the command since this start; bit 7 is 0 |
| 3 | reason, below. 0 when the frame was applied with nothing to add |
| 4 | session the ESC accepts now |
| 5-6 | int16 measured mechanical RPM, 0 when not valid |
| 7 | int8 measured q-axis current, 50 mA per step. Nearest step, halves away from zero, saturated at ±127 (±6.35 A). Decode multiplies the step by 50 mA. |

Checked in this order; the first failure is the one returned:

1. NULL, DLC other than 8, id above `0x7FF`, or id base other than `0x410`: `BAD_LENGTH` (20). The output struct is left untouched.
2. Bit 7 of byte 2 set, lifecycle above 5, or outcome above 4: `RESERVED_NONZERO` (4). The struct is left untouched.
3. No wheel bit, or more than one: `BAD_SELECT` (5). The struct is filled first.

Encode refuses a lifecycle above 5 or an outcome above 4 before it looks at
the select, and it does not write the id or the bytes when it refuses. A
nonzero first-edge field sets bit 6; decode returns 0 or 1. The reason byte
is copied either way. It is not checked against the table.

| Code | Name | Code | Name |
|---|---|---|---|
| 0 | NONE | 16 | LIMITED |
| 1 | BAD_VERSION | 17 | STARTING |
| 2 | BAD_MODE | 18 | FF_UNUSED (retired, not reused) |
| 3 | BAD_FLAGS | 19 | CONFLICT |
| 4 | RESERVED_NONZERO | 20 | BAD_LENGTH |
| 5 | BAD_SELECT | 21 | UNSUPPORTED |
| 6 | OUT_OF_RANGE | 22 | STALLED |
| 7 | NOT_ARMED | 32 | FAULT_MCSDK |
| 8 | BAD_SESSION | 33 | FAULT_STALL (retired, not reused) |
| 9 | STALE | 34 | FAULT_HALL_LOST |
| 10 | SEQ_AMBIGUOUS | 35 | TIMEOUT |
| 11 | SEQ_JUMP | 36 | LIFECYCLE_OVERFLOW |
| 12 | NOT_ALIGNED | | |
| 13 | BUSY | | |
| 14 | MODE_CHANGE | | |
| 15 | NOT_IDLE | | |

CONTROL decode still returns only codes 1-5 and 20. Codes 6 and up are
named so a later lifecycle cannot reuse them. This codec does not arm, apply
a setpoint, or latch a fault.

## STATUS reply

| Byte | Field |
|---|---|
| 0-1 | int16 mechanical RPM |
| 2-3 | int16 measured q-axis current, 544 counts per ampere |
| 4-5 | int16 q-axis current reference, same unit |
| 6 | MCSDK state byte. RUN is 6 in the vendored `MCI_State_t`. |
| 7 | flags. Bits 3-0 are the last stop reason: 0 none, 1 DISABLE, 2 timeout, 3 fault, 4 align, 5 capture, 8 parked. Bit 4 output on, bit 5 I²t derated, bit 6 aligning, bit 7 Hall shift known. |

The codec does not check the state byte or the flag bits. A reply id that
matches the STATUS base but does not name exactly one wheel is
`BAD_SELECT` after the payload has been written. A wrong length or a
different command leaves the output untouched.

## DIAG reply

| Byte | Field |
|---|---|
| 0-1 | uint16 bus voltage, centivolts |
| 2-3 | uint16 latched MCSDK fault word |
| 4-5 | int16 Hall phase shift, electrical degrees |
| 6 | align result: 0 none, 1 ok, 2 reversed, 3 no edges, 4 aborted, 5 running |
| 7 | uint8 I²t load, percent |

Same select and length rules as a STATUS reply.

## Session and sequence

`motor_session_next(prev, entropy)` is the next session byte. It is never
0 and never equal to `prev`. It does not store the session.

- Boot, `prev` 0: `1 + (entropy mod 255)`. Every nonzero byte is reachable.
  The boot entropy is that wheel's select bit, so the session is
  `1 + select`: FR 2, BR 3, BL 5, FL 9.
- Otherwise: `1 + (prev - 1 + 1 + (entropy mod 254)) mod 255`. The shorter
  step is what skips `prev`.

`motor_seq_classify(last_seq, seq)` is the signed 8-bit distance
`d = seq - last_seq`:

| d | Class |
|---|---|
| 0 | DUPLICATE |
| −128 | SEQ_AMBIGUOUS |
| −127 to −1 | STALE |
| 1 to 16 | NEXT |
| 17 to 127 | SEQ_JUMP |

DUPLICATE does not compare payloads. CONFLICT (same sequence, different
payload) is an ESC check and is not returned here. With `last_seq` 250:
251 is NEXT, 10 is NEXT (`d` 16), 11 is SEQ_JUMP (`d` 17), 250 is
DUPLICATE, 249 is STALE, 122 is SEQ_AMBIGUOUS (`d` −128).

## Golden vectors

SPEED +120 RPM to FR, sequence 5, session `0x3C`. Id `0x011`.
Bytes `11 00 78 00 00 00 05 3C`.

- `0x11`: version 1, mode SPEED.
- `0x0078`: 120 RPM.
- feed-forward 0, sequence 5, session `0x3C`.

STATUS from that FR wheel, still at 120 RPM, measuring 1 A while the
current reference is 2 A, MCSDK RUN, bridge on, Hall shift known.
Id `0x281`. Bytes `78 00 20 02 40 04 06 90`.

- `0x0078`: 120 RPM, the same speed as the CONTROL frame.
- `0x0220` = 544 counts = 1 A. `0x0440` = 1088 counts = 2 A.
- `0x06`: RUN. `0x90`: bit 4 (output on) and bit 7 (Hall shift known). The low nibble is 0, so the wheel was not stopped.

DIAG from FR on a full 3S pack, Hall shift −30 electrical degrees,
alignment not run, no fault, I²t cold. Id `0x381`.
Bytes `EC 04 00 00 E2 FF 00 00`.

- `0x04EC` = 1260 centivolts = 12.60 V.
- `0xFFE2` = −30 degrees.

CONTROL_REPLY for that FR wheel: sequence 5 was applied, lifecycle RUNNING,
outcome APPLIED, first Hall edge seen, session `0x3C`, measured 118 RPM and
1.25 A. Id `0x411`. Bytes `05 05 43 00 3C 76 00 19`.

- `0x43`: lifecycle 3, outcome 0, bit 6 set.
- `0x0076`: 118 RPM.
- `0x19`: 25 × 50 mA = 1250 mA.

## Not in this codec

CONFIG. Arming, the session check, CONFLICT versus DUPLICATE, the
setpoint limit, the 100 ms watchdog, STARTING/RUNNING, and HOLD/park live
in `motor_cmd` ([command-lifecycle.md](command-lifecycle.md)), which calls
these helpers and is not linked into an image either. The inbox is not
in that module yet. Nothing here is linked into the Nucleo or ESC image.
