# Mat keyboard drive, Hall-angle + 100 Hz firmware  (2026-10-23, user driving)

- Firmware: `9720d91` as installed (ESC CRCs FR `34887E92`, BR `0B8442A5`, BL `94E17FEF`, FL `C85C810A`; Nucleo `drive-9720d91-hall-100hz.elf`). No reflash since the install.
- Setup: robot on the mat, wheels down, area clear (user confirmed). Vbus 11.31-11.37 V before, all four IDLE, no faults. `iq_limit_ma` was back at the 8 A default after the CAN rewire power cycle; set to 7000 (RAM) on all four and read back. No other probe/CAN tool running.
- Procedure: `keyboard_drive --speed 0.3 --log ...` run by the user from their terminal for 51 s; forward/back, left/right, diagonals, taps up to 1000 mm/s command vectors (speed raised from 0.3 with `+`).
- Result: no faults, no derate, no ESC stop reasons on any wheel; `tx_fail` 0 and `bad_input` 0 over 66990 frames; no status-count resets in the log. Peak |Iq| 4.7-5.3 A (limit 7 A), peak I2T 5-15 %, peak wheel speed 254-388 RPM, min Vbus 10.97 V under load. After the run all four IDLE, 0 RPM, 0 A, Vbus 11.20-11.26 V.
- Open: `esc_tool status` shows `restarts 1` on every wheel afterwards (was 0 before the drive, nothing in the log shows a reset). Probably the first running session after the rewire power cycle, not a mid-drive restart; unconfirmed. 100 Hz bus load and reply latency were not analysed. No position measurement, so no accuracy number.
- Data: `/home/fh/sfurs_firmware/hw-data/2026-10-23-mat-keyboard/run1.jsonl`. Battery on, probes released.
