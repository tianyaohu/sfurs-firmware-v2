# Motor frames

Codec: `src/shared/src/motor_protocol.c`. It is not linked into the Nucleo
or ESC image. Multi-byte fields are little endian, signed fields two's
complement. The codec is stateless: it does not apply RPM or current
limits, and it does not track session or sequence.

11-bit ids. Wheel select bits: FR `0x1`, BR `0x2`, BL `0x4`, FL `0x8`.

CONTROL uses a different id layout from STATUS and DIAG. CONTROL puts the
select in bits 3-0 and the base `0x010` in bits 10-4. STATUS and DIAG put
the command in bits 10-7 and the select in the low bits; bits 6-4 stay 0.

| Id | Name | Payload |
|---|---|---|
| `0x010 \| select` | CONTROL | 8 bytes, below |
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

## Not in this codec

CONTROL_REPLY, CONFIG, and the ESC's range, session, and sequence rules.
Reason codes 6 through 19 belong to those rules and are not returned here.
