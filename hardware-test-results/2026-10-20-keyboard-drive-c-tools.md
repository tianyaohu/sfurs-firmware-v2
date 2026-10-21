# keyboard_drive (C tool over OpenOCD), first hardware run  (2026-10-20, Frank H + claude)
- Firmware: commit 77e9377 (chore/openocd = develop_ai); Nucleo drive image flashed with STM32_Programmer_CLI
  (verified), ELF sha256 89c7bac67d4a...; ESC apps unchanged (CRC 8CBA6720, C2465711, 8B8B2CDF, C86A592F).
- Setup: wheels raised, battery on at 15.5 V (ESC Vbus over CAN), all four ESCs IDLE, no faults, Hall shifts -60/-64/-59/-62.
  Probe: Nucleo ST-LINK-V3 only; OpenOCD 0.11 via `lib/ocd.c`; CAN through the Nucleo.
- Procedure: `esc_tool status` (read-only), then operator ran `build/tools/keyboard_drive --speed 0.3` in a terminal.
- Result: telemetry read and the keyboard drive work as with the Python tool (operator: "everything works").
  Open: sideways (vx, A/D) motion - one wheel starts slipping before the others. Not diagnosed. Operator: it "still" happens,
  i.e. it predates the port (the Nucleo and ESC firmware are unchanged by it).
  Observation: BR and FL reported Iq of about 25 A while IDLE at 0 rpm in `status`; unexplained, probably an idle offset.
- Data: none kept.
- Not run on hardware: esc_update, esc_tool align/capture/spin/pulse, move_test, tune_ground, sweep.
