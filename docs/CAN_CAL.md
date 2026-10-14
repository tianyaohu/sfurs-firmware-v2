# CAN-triggered ESC calibration (CAL1 sender)

Nucleo-H755 CM7 sends one `0x480|dev` + `"CAL1"` frame per wheel so the
bench — not a debugger RAM poke (`0x200008D0`) — can trigger the ESC's
existing `TestCalibrationService()`. The ESC side is a port checklist
below; until it lands, the frame is bus traffic the stock ESC ignores.

## Command

500 kbit/s, `FD_NO_BRS`, 4-byte payload (same framing as bench drive).

| ID (Nucleo→ESC) | Payload bytes | Meaning                              |
|-----------------|---------------|--------------------------------------|
| `0x480\|dev`     | `43 41 4C 31` | `"CAL1"` = `CAL_MAGIC 0x43414C31U`   |

`dev` = wheel device bits (`NE=1, SE=2, SW=4, NW=8`). Slot 9
(`9U<<7 = 0x480`) is free: drive/telemetry use 1,2,3,5,6,7,8,15,
FWU owns `0x600..0x6FF`, STOP is `0x78F`.

## Gates (Nucleo `can_cal_sender`, mirrors `wheel_bench`)

A frame goes out only when ALL hold:

1. Debugger explicitly wrote `can_cal_request` (`1/2/4/8`, or
   `CAN_CAL_STOP 0x53544F50` to abort). Consumed once; never periodic.
2. Caller passes bench-idle true (`wheel_bench_state == BENCH_IDLE`).
3. Device not already sent this boot (`can_cal_used_mask` one-shot;
   marked *before* enqueue, like bench RUN — a possibly delivered CAL
   is never retried).
4. Transport enqueue (`wheel_bench_send`) succeeds.

In-flight frame settles via `wheel_bench_poll` with a 100 ms budget:
ACK → `CAN_CAL_ACKED`; cancel/fail/timeout → `CAN_CAL_FAILED` and a
latched fault (`can_cal_first_fault` 1/2/3) that blocks further CAL
sends. STOP aborts a pending frame with no fault latch.

## Confirmation

Nucleo-side `ACKED` proves only bus TX. Real confirmation is the ESC's
existing `0x381` telemetry: **flags bit11** set after
`TestCalibrationService()` completes (result/valid vars in ESC RAM).

## ESC-side port checklist (`firmware-v2`, still needed)

In `HW/B-G431B-ESC1/motor_config/Src/main.c`:

- [ ] RX callback: on `0x480|dev` with bytes `43 41 4C 31`, set
      `test_calibration_request = 0x43414C31` ("CAL1") — but only when
      MCI is IDLE and no faults are present; otherwise ignore (count
      the rejection for telemetry if cheap).
- [ ] Widen the FDCAN exact-match mask filter to also accept
      `0x480|dev` (or at minimum `0x481` for the NE-first bench test).
- [ ] Keep `TestCalibrationService()` gating (`MCI IDLE + no faults`)
      untouched; the CAN path only *requests*, same as the debugger
      poke today.
- [ ] Bench-test on NE first: send CAL with the motor known-stopped,
      watch `0x381` bit11, confirm no motion on reject paths.

## Safety — read before any powered run

1. Default state sends nothing (`can_cal_request` boots 0).
2. CAL while the bench is ARMED/RUNNING is refused; STOP the bench
   (`0x78F x3`) first, same battery discipline as `WHEEL_BENCH.md`.
3. One CAL per device per boot; a fault latches until reset.
4. `ACKED != calibrated`: trust only `0x381` bit11 + ESC result vars.
5. SWD stays the recovery path; never mass-erase, never touch option
   bytes without sign-off.

## Files

- `Common/Inc/can_cal.h` — `CAL_REQUEST`, `CAL_MAGIC`, id/pack
  helpers, result enum (header-only, shared Nucleo/ESC/host).
- `CM7/Core/Inc|Src/can_cal_sender.{h,c}` — gated one-shot service;
  transport is `can_cal_send/poll/abort` (firmware wires the
  `wheel_bench_*` trio, host tests fake it).
- `tests/test_can_cal.py` — compiles the real service with
  `cc -std=c11 -Wall -Wextra -Werror`; 8 scenarios.
