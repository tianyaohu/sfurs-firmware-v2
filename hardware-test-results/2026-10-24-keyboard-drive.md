# Keyboard drive after reinstalling develop_ai firmware  (2026-10-24, user driving)

- Firmware: `9720d91` reinstalled over the TIGERs experiment image (`61a7e3e`; FR ran an unrecorded `F1C29315/55956`). `scripts/test.sh` 5/5; `scripts/build.sh --shift NE=-60 SE=-64 SW=-59 NW=-62` without warnings. Installed ESC CRC/size FR `34887E92/55636`, BR `0B8442A5/55636`, BL `94E17FEF/55636`, FL `C85C810A/55628` (identical to the 2026-10-23 install); Nucleo drive ELF SHA256 `abc51ecf…`.
- Setup: 4S pack, Vbus 16.43-16.59 V before, all four IDLE, no faults. Probe `002200364142500620353451` on this PC, no other tool holding it. `esc_update NE/SE/SW/NW --can` sequentially: bootloader CRC, IDENT CRC/size and all-IDLE checks passed on each. `iq_limit_ma` 8000 (default). Mat, wheels down.
- Procedure: user ran `keyboard_drive --speed 0.3 --log …/run1.jsonl` (twist up to 1000 mm/s, yaw 2000 mrad/s), then an unlogged run at 2.0 m/s.
- Result, 1.0 m/s (logged, 107 s, 42831 frames, `tx_fail` 0, `bad_input` 0, no faults; peak |Iq| 5.1/5.9/6.3/5.8 A, I2T ≤ 36 %, min Vbus 16.07 V):
  - Left/right curves visibly, same curve mirrored for A vs D (user). Wheels lag their targets unequally, e.g. vy +1000: 189/169/-193/-227 RPM vs 253/239/-239/-253 target within the ~0.5-1 s presses.
  - Yaw steps ~40° fast then slow, both directions (user). Log: all four wheels at 50 RPM target average 40 RPM; Iq ref on all four rises together to 1.2-2.3 A and drops to ~0 every 0.35-0.45 s (2 rad/s × 0.4 s ≈ 46° per cycle). Matches the speed loop's stuck ramp (|ŵ| < 45 RPM, 10 A/s) followed by the per-edge overspeed bleed halving I to ~0, below the rolling current.
- Result, 2.0 m/s (user, no log): left/right unusable; wheels break traction and spin, the body keeps rotating.
- Final: user quit `keyboard_drive`; battery on. Final ESC state not read: the TIGERs worktree's `esc_tool` (BR CONFIG writes) held the probe afterwards.
- Data: `/home/fh/sfurs_firmware/hw-data/2026-10-24-keyboard/run1.jsonl`.
- Friction (same day, mat, user authorized): the TIGERs session had reinstalled its revision on all four ESCs (FR `0D312472`, BR `1E3E0004`, BL `71BEF2AF`, FL `A28AD730`, 56.6 KB) and the Nucleo after the drive; reinstalled `9720d91` again over CAN, CRCs as above, all IDLE, 15.87-16.02 V.
  - `friction_map --breakaway --body wz|vx|vy` (2 A/s ramp, 4 A cap, 3 reps per sign), current at the moment two wheels turned, per wheel: wz 0.84-1.26 A, vx 2.09-2.62 A, vy 1.30-1.81 A. All 18 trials moved.
  - `friction_map --axis vx|vy` speed legs: no fit. vx ±0.1/0.2, ±0.3/0.5 and ±0.7/1.0 m/s (1 rep, 0.7 m): every point `speed_not_steady` (SD 15-56 RPM, > 10 % of mean); at 0.7-1.0 m/s wheels ran 65-128 RPM against 146-167 RPM targets with 1.7-4.3 A. vy ±0.7/1.0 (0.7 m): 3 steady points at 175-188 RPM, 0.77-1.17 A; 1.0 m/s legs too short (3 samples). vy ±0.6/0.9 (0.9 m) aborted after the first leg on `BR: unreadable telemetry`; robot left ~0.9 m from start.
  - After: all four IDLE/stopped, no faults, Vbus 15.82-15.94 V, probe released, battery on.
  - Data: `/home/fh/sfurs_firmware/hw-data/2026-10-24-friction/` (`brk-*`, `roll-*`, result.json each).
