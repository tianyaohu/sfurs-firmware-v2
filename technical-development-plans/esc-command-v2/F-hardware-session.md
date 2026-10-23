# F. Hardware session: gates H4a, H3, H5, H6 in one sitting

For [phase1-esc-command-v2](../phase1-esc-command-v2.md). Stages 3, 4a, 5 were built and host-tested without hardware
(design log 2026-10-22); this session runs their gates in dependency order. Stop at the first gate that fails, record it,
and decide before going on: later gates sit on earlier ones.

## Before anything moves

1. Battery state: 4S, `build/tools/esc_tool status` shows Vbus (15-16.8 V); < 14 V: charge first.
2. Probe: only the Nucleo ST-LINK `002200364142500620353451`; no other tool on it or on CAN.
3. Install (wheels raised, battery on, all four IDLE, no faults): `build/tools/esc_update <W> --can` for NE, SE, SW, NW
   (each verifies IDENT CRC; the first run also flashes the Nucleo drive image, mailbox ABI 8). Rollback set:
   `/home/fh/sfurs_firmware/esc-backups/rollback-20261021/` (`esc_update <W> --can --build <that dir>`, b15ac8a tools).
4. `esc_tool status` and `esc_tool get all`: four IDLE, no faults, Hall shifts -60/-64/-59/-62 known, keys at defaults
   (kp 1024, ki 553, iq_limit_ma 6000, stall_ramp_ma_per_s 10000, stall_ms 500).
Record every step in `hardware-test-results/<date>-h4a-h3-h5-h6.md` as it happens.

## H4a: new speed loop (raised, then mat)

Raised:
1. `esc_tool spin <W> --rpm 60 --seconds 3 --log ...` and `--rpm 120` per wheel. Compare with
   2026-10-21-h2-install-raised.md (60 RPM mean 58.7-61.8, sd 21-31; 120 RPM 119.9-120.1, sd 1.5-4.9). Pass: no faults,
   means within 5 %, no worse scatter.
2. The 30-start set, same flags as 2026-10-21 (`move_test ... --accel 900 --record --record-pre 20 --startup-metrics
   --sector-spread-deg 0`, 5 per axis/sign) plus wz at the gate point `--axis wz --speed 4.8 --distance 1.0`. Pass: no
   overspeed abort, no fault, overshoot no worse than 2026-10-21 (medians 1.28-1.45); expected lower. In each record,
   channel 7 mode should be run/hold only (no stall_ramp on free wheels) and bleed rare.
3. HOLD: `keyboard_drive`, drive a short move, release the keys and leave it enabled 5 min. `esc_tool status` every
   minute in another terminal is not possible (one tool per probe): instead stop after 5 min and read the temperature
   by hand (finger test on the ESC/motor) and listen for buzz. Pass: no buzz, motors/ESCs not warmer than after normal
   driving, wheel turns by hand with mild resistance.
4. Faults on purpose: block one wheel by hand during `esc_tool spin <W> --rpm 60 --seconds 3`: expect FAULTED(STALL)
   after ~1 s (stall ramp + 500 ms), status shows fault, `esc_tool clear` recovers. Hall loss: unplug a Hall connector
   ONLY if the user agrees (it is a connector stress); otherwise mark HALL_LOST "host-tested only".
Mat:
5. `move_test --out ... --axis vy --speed 0.47 --distance 0.28 --accel 900 --record --record-pre 20 --record-delay 200
   --startup-metrics --sector-spread-deg 0` (the start that failed 2026-10-21). Pass: no overspeed abort, breakaway earlier
   than 0.47-0.67 s, no fault. Then 5 starts per axis/sign. D's tracking deadlines may still fail with ff 0 (expected).

## H3: CONFIG (raised, idle)

1. Every key on all four wheels: `esc_tool set <W> <key> <value>` then `esc_tool get <W> <key>`: status OK, stored =
   value, effective = value (iq_limit_ma: = value unless derated). Out-of-range values: BAD_VALUE, stored unchanged.
2. While a wheel spins (`esc_tool spin` in progress is not possible with one tool): arm a wheel at 0 and command motion
   with `keyboard_drive`, quit, then immediately `esc_tool set` before the ESC stops: expect NOT_IDLE if caught running;
   the host test covers this exactly, so a single attempt is enough.
3. Power-cycle (battery off/on): `esc_tool get all` shows compiled defaults again (RAM only).

## H5: friction (mat)

1. `friction_map --out ... --check` (preflight, no motion), then `friction_map --out ... --axis vx --speeds=-0.1,-0.2,-0.3,0.1,0.2,0.3`
   and the same for vy and wz. Pass: every wheel has >= 4 accepted segments per direction and fit residual RMS < 20 % of Ic.
2. `friction_map --out ... --breakaway --wheel all --rate-ma-s 2000 --cap-ma 4000 --reps 3`. Compare the breakaway
   currents with 2026-10-21 (3.3-4.7 A on the mat).
3. Turn the fit into the Nucleo friction model: `move_test` / `keyboard_drive` take `--ff-coulomb-x-n`,
   `--ff-coulomb-y-n`, `--ff-viscous-xy-n-per-mps`, `--ff-coulomb-w-nm`, `--ff-viscous-w-nm-per-radps`,
   `--ff-breakaway-x-n`, `--ff-breakaway-y-n`, `--ff-breakaway-w-nm`, `--ff-breakaway-ms`, `--ff-a-per-nm` (1/Kt) and
   `--ff-scale FR,BR,BL,FL`. Mat 2026-10-22 starting point (1/Kt placeholder 10): coulomb y 10.5 N (~1 A per wheel),
   x ~13 N (~2 A), breakaway x ~21 N (~3.2 A), y ~21 N (~2 A); tune from there. The fit is per wheel in mA; the model is in body-frame N and N*m, so converting needs the
   wheel radius and Kt (`--ff-a-per-nm` = 1/Kt): take Kt from the motor datasheet and record its source (Ke is not
   measured). Per-wheel differences go into `--ff-scale`. Check one keyboard_drive run with the values before H6.

## H6: final gate (mat)

With ff on (from H5, `move_test ... --ff-*`): 20 starts per axis/sign at the gate points (vx 0.72 m/s, vy 0.47 m/s, wz 4.8 rad/s), D-acceptance
rules, `startup_report` over all runs. Report counts, not percentages.

## Leave the hardware

`esc_tool stop`; all four IDLE, no faults; battery state and installed firmware (CRCs) in the results file.
