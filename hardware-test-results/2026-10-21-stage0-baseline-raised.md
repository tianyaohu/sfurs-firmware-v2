# Stage 0 baseline, wheels raised  (2026-10-21, Frank H + claude)
- Firmware: as installed. Nucleo = 77e9377 image (ELF sha256 89c7bac67d4a...; Nucleo sources identical at b15ac8a).
  ESC apps CRC NE 8CBA6720/53248, SE C2465711/53248, SW 8B8B2CDF/53240, NW C86A592F/53240 (CAN install 2026-10-18,
  most likely source 776994f). Tools built at b15ac8a (mailbox ABI 4); default DSC off, ff 0, kp 1024 ki 553.
- Setup: robot raised, all four wheels free. Battery 4S, Vbus 16.8 V at start, >= 16.45 V throughout. Probe: Nucleo
  ST-LINK-V3 002200364142500620353451 only; CAN through the Nucleo.
- IDLE Iq: before the first start after boot three wheels read about +-24-26 A at rest (varies per boot and wheel);
  after one start/stop all read 0.00-0.11 A. Cause: MCSDK computes Iq from current offsets calibrated only at the first
  start. Not a real current; i2t integrates only in RUN. Fixed in reporting by 1cc9340 (feature/esc-command-v2).
- Single wheel, `esc_tool spin`: 60 RPM, 3 s: mean 57.5-60.8 RPM, sd 18-33 RPM, 1-7 of ~78 samples wrong sign.
  120 RPM step (no ramp), 3 per wheel: mean 120.2-121.0 RPM, sd 1.9-3.8; MCSDK RUN seen 13-53 ms after the command
  (typ. 23-35), first non-zero speed 46-92 ms (status polled every ~5-10 ms, so coarse). No faults.
- Startup baseline, `move_test --accel 900 --ff 0 --record --record-pre 20 --startup-metrics --sector-spread-deg 0`,
  5 starts per axis/sign: vx +-0.72 m/s, vy +-0.47 m/s (120 RPM peak wheel), wz +-2 rad/s (~50 RPM; the b15ac8a
  `move_test` caps every axis at 2, so the 4.8 rad/s gate point was not run). Nominal 7.5 deg sectors (no Hall state in
  this firmware's recorder, no sector map yet).
  | axis | overshoot median / max | wheel-starts missing a tracking deadline |
  |---|---|---|
  | vx- | 1.39 / 1.49 | 2/20 |
  | vx+ | 1.43 / 1.49 | 0/20 |
  | vy- | 1.40 / 1.45 | 0/20 |
  | vy+ | 1.40 / 1.45 | 2/20 (one run aborted by the tool: FL -270 RPM > 238 limit) |
  | wz- | 1.32 / 1.43 | 0/20 |
  | wz+ | 1.28 / 1.52 | 1/20 |
  Verdicts (wheel-starts): 1 PASS, 93 overshoot FAIL, 5 deadline FAIL (k=2: 4, k=3: 1), 21 not measurable (mostly
  wz, 50 RPM). Synchronization NOT_MEASURABLE (no APPLIED replies in this protocol). Overall FAIL against
  D-acceptance; the overshoot (1.3-1.5x with no load) is the baseline H2/H4a compare against.
- Not done yet: edges per marked wheel turn, sector-width map (no tool; needs the Hall state, which only the new
  recorder logs), mat runs.
- Data: /home/fh/sfurs_firmware/hw-data/2026-10-21-stage0-baseline/ (runs/*/record.json, result.json, startup.tsv,
  spin and latency logs, scripts).
