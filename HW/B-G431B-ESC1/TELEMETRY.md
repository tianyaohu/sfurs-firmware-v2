# Read-only hand-spin telemetry

This extends the existing CAN FD protocol without changing motor parameters.
Existing drive commands remain supported; do not send speed or torque writes
during this test. Keep motor power off while flashing. Only MOTOR_NE has been
updated in this session; the other ESCs will not implement these replies yet.

- Nominal bitrate: 500 kbit/s, CAN FD without BRS.
- Front-right (NE) select bit: 0x01, from common/CAN/can_datatypes.h.
- Request: standard ID SPEED_READ | MOTOR_NE_CAN_ID = 0x101,
  CAN FD data frame, four zero payload bytes.
- Reply: standard ID SPEED_READ_REPLY | MOTOR_NE_CAN_ID = 0x281,
  CAN FD data frame, eight payload bytes.

| Bytes | Meaning |
| --- | --- |
| 0–1 | Signed int16 RPM, little endian, SDK measured average |
| 2 | Motor SDK state (MCI_State_t), not a universal enum |
| 3 | Raw H1/H2/H3 GPIO levels in bits 0/1/2 |
| 4–7 | uint32 observed Hall-state change count since boot, little endian |

The change count is sampled in the main loop. It is a slow hand-spin diagnostic,
not a lossless encoder counter or calibrated position measurement. RPM may be
zero or unreliable while idle/faulted; verify Hall changes first. The configured
motor has eight pole pairs (normally 48 Hall transitions per mechanical rotor
revolution), which must match the actual motor. Hall sensor power must be present
to observe transitions; USB-only operation does not guarantee it.

The ESC CAN ISR marks a request pending; the main loop samples Hall inputs and
enqueues one response when TX FIFO space is available. The response handler does
not invoke a motor-start API. Other existing CAN/UART drive paths are unchanged.

## Build without CubeIDE

With arm-none-eabi-gcc on PATH, from motor_config:

```sh
python3 build_cli.py --motor NE --output /tmp/opencode/esc-ne-telemetry
```

The CLI build uses the CubeIDE .project linked-source list, its Cortex-M4F hard
float settings, -Ofast, and existing flash linker script. It does not compile the
currently unused common/CAN/can_datatypes.c wrapper.

## Nucleo diagnostics

The test project at Project1/sfurs_firmware_v2 requests NE telemetry once per
second using FDCAN1 PB8 RX / PB9 TX. Its debugger-visible variables include
esc_telemetry_reply_count, esc_ne_rpm, esc_ne_motor_state, esc_ne_hall_state,
esc_ne_hall_changes, and esc_ne_last_reply_tick. Re-resolve their addresses with
arm-none-eabi-nm after each build; do not reuse old addresses blindly.

Check a baseline snapshot, hand-spin slowly, and read again. A rising reply count
verifies application-level communication; changing Hall bits/count verifies Hall
feedback. A TX ACK alone does not identify which node acknowledged.

Original front-right flash backup (128 KiB at 0x08000000):
`/home/fh/sfurs_firmware/esc-backups/front-right-before-telemetry-20261012.bin`.
This is not a backup of option bytes. Do not restore or alter option bytes without
explicit authorization. Flashing the repository build preserves its source motor
parameters, but does not prove they matched the previously installed binary.
