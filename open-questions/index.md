# Open questions

Questions raised but not yet answered. Delete a line when answered and put the
answer in the doc it changes.

- Exact accuracy spec: along-track vs cross-track vs heading, speed, floor.
- Speed loop on the mat with the own Hall angle: wheel starts within 24-62 ms and overshoot <= 1.4x, except one
  first-free wheel at 2.1x (2026-10-22). Is that the stall-ramp I held at breakaway, and does it need a limit?
- ESC restarts: all four ESCs restarted together at least twice during keyboard drives on 2026-10-22 (boot
  sessions back, RAM settings lost); the Nucleo (USB-powered) did not. Battery connector, switch or ESC power
  wiring? `esc_tool status` / keyboard_drive now count restarts.
- 100 Hz command stream (fix/drive-100hz, merged): bus load and reply latency not yet measured on hardware.
- Current sense: the phase-U reading looks wrong (ΔIq ≈ −0.55·ΔId); ADC1
  shared with the Vbus/temperature conversions is the open lead (DRIVE.md).
- Kinematics contact positions are hand-measured and provisional.
- Drive motor part number (read the motor label). TIGERs' Nanotec DF45L024048 has nearly the same
  R, L and pole pairs; if ours matches, its Ke 0.0225 V·s/rad is a prior for the tuning plan.
- Is an IMU fitted and readable on the robot? The tuning plan (stage 7) uses one for yaw inertia if so.
- Does open loop meet ±10 cm, or is IMU heading hold needed?
- Rig ground truth: slide the tag a known 210 mm several times in the centre;
  one hand move read 193 mm. Suspects: `--tag-mm` print size, calibration error.
- Rig calibration: RMS 1.76 px; corners beyond ~450 px from the centre are
  unverified (disagreements up to ~200 px). Needs rigid board, still views,
  3-4 views per corner at ~40 cm, then `xval` below 2 px at the edges.
- Rig geometry: the camera is ~1 m up (cable length), so the reliable area is
  about ±0.29 m; a 1 m drive needs ~2 m height (active USB extension) or a
  corner-covering recalibration. The floor survey for `rig_track`/`rig_analyze`
  is not done.
- Rig timing: exposure/USB latency is unknown; `rig_analyze --estimate-latency`
  is untested on real data. Camera and `esc_tool` must share a host clock.
- Rig friction-ID pipeline (`sweep --execute`, then `rig_analyze.py` in the rig repo) has only
  run on synthetic logs.
