# Open questions

Questions raised but not yet answered. Delete a line when answered and put the
answer in the doc it changes.

- Exact accuracy spec: along-track vs cross-track vs heading, speed, floor.
- Speed-loop gains: which rung holds the robot on the ground without limit
  cycling a raised wheel (see LESSONS)?
- Current sense: the phase-U reading looks wrong (ΔIq ≈ −0.55·ΔId); ADC1
  shared with the Vbus/temperature conversions is the open lead (DRIVE.md).
- Kinematics contact positions are hand-measured and provisional.
- Does open loop meet ±10 cm, or is IMU heading hold needed?
