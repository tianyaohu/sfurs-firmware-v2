# Protocol v1 install and first H2 runs, wheels raised  (2026-10-21, Frank H + claude)
- Firmware: feature/esc-command-v2. ESC apps (89de998, built with `--shift NE=-60 SE=-64 SW=-59 NW=-62`) CRC/size
  NE B82DCCCF/56208, SE 2A4D0EC6/56208, SW B938A36F/56200, NW 9130898D/56200, installed with `esc_update <W> --can`,
  each verified by IDENT. Nucleo drive image: 89de998 (ELF sha256 cbea5f266ae3...) for the runs below, then d8c0609
  (mailbox ABI 7, reply latency histogram; ELF sha256 c4102b22f603...) flashed with STM32_Programmer_CLI, verified.
  Rollback set (previous ESC apps, CRC-matched, plus b15ac8a Nucleo images): /home/fh/sfurs_firmware/esc-backups/rollback-20261021/.
- Setup: robot raised, battery 4S 16.8 -> 16.2 V over the session. Probe: Nucleo ST-LINK-V3 002200364142500620353451 only.
- IDLE Iq after install, before any start: 0.00 A on all four (1cc9340 fix confirmed).
- `esc_tool spin`, each wheel alone: 60 RPM mean 58.7-61.8 (sd 21-31; old firmware 57.5-60.8, sd 18-33); 120 RPM mean
  119.9-120.1 (sd 1.5-4.9; old 120.2-121.0, sd 1.9-3.8). No faults; every run armed, applied, stopped, disarmed;
  link misses 0.
- Startup set, same command and points as the Stage 0 baseline (2026-10-21-stage0-baseline-raised.md), 5 per axis/sign:
  | axis | overshoot median / max (old) | wheel-starts missing a deadline (old) |
  |---|---|---|
  | vx- | 1.38 / 1.54 (1.39 / 1.49) | 0/20 (2/20) |
  | vx+ | 1.43 / 1.49 (1.43 / 1.49) | 2/20 (0/20) |
  | vy- | 1.40 / 1.45 (1.40 / 1.45) | 0/20 (0/20) |
  | vy+ | 1.45 / 1.54 (1.40 / 1.45) | 3/20 (2/20); one run aborted by the tool, BL -250 RPM > 238 (old: one, FL) |
  | wz- (2 rad/s) | 1.32 / 1.43 (1.32 / 1.43) | 0/20 (0/20) |
  | wz+ (2 rad/s) | 1.39 / 1.52 (1.28 / 1.52) | 2/20 (1/20) |
  Wheel-starts: 0 PASS, 105 FAIL (98 overshoot, 7 deadline), 15 not measurable (old: 1 / 98 / 21).
  Command-apply skew (new): PASS on all 30 starts, 793-1085 us (bar 2 ms).
  Same speed loop as before (Stage 2 keeps it), and the numbers match within the spread of 5 runs; wz+ median moved most
  (1.28 -> 1.39), to be watched in the mat runs.
- On the mat (Nucleo d8c0609, 16.1 V, one `move_test` vy+ 0.47 m/s run, same flags): no wheel turned within 360 ms.
  Iq followed its reference on all four (current loop fine); the speed PI's I term rose ~7.4 A/s to ~3 A total while the
  wheels crept 1-2 Hall edges (static friction; earlier breakaway measurements 2.2-3.2 A). BL/FL broke away at
  ~480 ms, then FR overshot to 450 RPM (tool abort at 238): wound-up I term after breakaway. The protocol did its part
  (skew 1074 us, no misses, clean stop). This is the old speed loop, unchanged in Stage 2; the remaining mat runs were not
  done, since every start would end the same way. Fix belongs to Stage 4a/5 (new loop, friction ff).
- H2 protocol checks on the mat, no motion (tools 7151aaa, Nucleo d8c0609):
  - `esc_tool reject W`, all four wheels: 17/17 checks each (16 cases + the second row of BAD_SELECT): BAD_VERSION,
    BAD_MODE, BAD_FLAGS, RESERVED_NONZERO, BAD_SELECT, BAD_SESSION, NOT_ARMED, OUT_OF_RANGE (rpm, ff), STALE,
    SEQ_AMBIGUOUS, SEQ_JUMP, DUPLICATE, CONFLICT, UNSUPPORTED, INHIBITED/FF_UNUSED, each seen at the Nucleo with the
    expected outcome, reason and lifecycle. Not produced from outside in Stage 2: MODE_CHANGE, LIMITED, NOT_ALIGNED
    (shifts baked), FAULT_HALL_LOST (Stage 4a), TIMEOUT as a reject, LIFECYCLE_OVERFLOW, BAD_LENGTH (no reply by
    design), and STARTING/BUSY/FAULT_STALL/NOT_IDLE (need motion or a real fault).
  - Watchdog, `esc_tool spin W --rpm 0 --gap-ms N`, gap measured between Nucleo reply stamps: no trip at 72 and
    91-93 ms (all four wheels at ~92 ms), trip (REJECTED NOT_ARMED, DISARMED, new session) from 103 ms (all four at
    171 ms). 3 lost frames (80 ms) never trip, 4 (100 ms) do; finer gaps than ~10 ms are not placeable from the PC.
  - `esc_tool stream --seconds 300`: per wheel 15044 replies to 15044 frames, tx_fail 0, fault word 0, 1 reject per
    wheel (the session-0 ARM probe). Reply latency (Nucleo tick to reply, 250 us bins, upper edges) p50/p99:
    FR 1.50/2.00, BR 2.00/2.25, BL 2.25/2.75, FL 2.50/2.75 ms; exact max 1.94/2.77/2.68/2.79 ms. Rising in send order
    (frames queue FR, BR, BL, FL), which is the ~1 ms apply skew. Bus load ~19.6 % of 500 kbit/s (130 bits per 8-byte
    frame, 100 per 4-byte poll; 120357 frames received, 60167 sent + ~60060 polls in 300.3 s).
  - LOST: BL put in its bootloader (one-off helper sending the documented 0x654 `BOOT 01 04 FB 00` through the raw
    mailbox; status "no reply"), then `esc_tool stream --seconds 5`: fault word 0x40 (BL LOST), drive STOPPING,
    FR/BR/FL answered the broadcast DISABLE (APPLIED/NONE), no motion (needs all four armed); latched while enable was
    held 2 s, cleared at enable 0. Tests LOST at session start, not mid-stream. BL restored with
    `esc_update SW --can --target-in-boot` (same CRC B938A36F), all four IDLE.
- H2 result: protocol checks pass. Mat starts not repeated (old loop stalls on static friction; design log 2026-10-21).
- Data: /home/fh/sfurs_firmware/hw-data/2026-10-21-h2-raised/ (runs, log); spins in
  /home/fh/sfurs_firmware/hw-data/2026-10-21-stage0-baseline/new_spin_*.jsonl.
