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
- Not done yet (H2): reject reasons, watchdog 3 vs 4 lost frames, LOST disables all, 5 min without false trip, reply
  latency p99 and bus load (tools in progress), mat runs.
- Data: /home/fh/sfurs_firmware/hw-data/2026-10-21-h2-raised/ (runs, log); spins in
  /home/fh/sfurs_firmware/hw-data/2026-10-21-stage0-baseline/new_spin_*.jsonl.
