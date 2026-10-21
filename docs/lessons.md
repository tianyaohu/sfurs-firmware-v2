# Lessons learned

Short, durable facts from past debugging. Add a line only when it would change
what someone does next; this is not a log.

- **CAN ACK proves nothing about the target.** Any node acknowledges a frame.
  "No motion with good ACKs" means the ESC ignored the command; read its
  status reply or RAM.
- **Speed-PI gains must be checked in physical units.** The Oct 2026 RPM2
  builds asked a stalled wheel for ~0.15 A (≈3 counts/RPM, integral taking
  minutes), so the robot could not move on the ground while spinning freely
  in the air.
- **Stall/timeout/count stops fight a loaded low-speed start.** A 2 s
  no-Hall-edge stop ended every ground start before torque could build. Use
  thermal (I²t) protection instead.
- **Negating speed gains hides a Hall-offset error.** If +Iq turns the motor
  in the −Hall direction, the Hall phase shift is ~180° off. Measure it
  (`esc_tool.py align`) instead of flipping signs.
- **The boards as received run a sensorless (observer) image** whose observer
  needs ≥1743 RPM; at robot speeds it buzzes in rev-up at up to 6 A.
- **Rebuilding does not update the board.** Resolve RAM addresses from the
  ELF that is actually flashed; stale images mimic dead motors.
- **Shared per-wheel code fails per wheel.** A gate like
  `if wheel == 1 || wheel == 2` silently skips the others; test all four
  wheels in host tests for anything addressed per wheel.
- **Attach, don't connect.** A normal SWD connect resets a running ESC and
  stops the motor; use attach/HOTPLUG for monitoring.
- **Compare flashed bytes, not ELF files.** Two builds of the same tree give
  different ELF hashes; `scripts/build.sh` prints hashes of the loadable image.
- **A frozen telemetry row looks live.** The Nucleo keeps each wheel's last
  reply; a powered-off ESC still "reads" 4.6 V and FAULT_NOW. Only a row
  whose reply count advanced is live (`esc_tool.py status` checks this).
- **USB before battery latches undervolt.** An ESC brought up on its ST-LINK
  USB sits in undervolt; when the battery comes on it stays in FAULT_OVER.
  `esc_tool.py stop` clears it once no fault is present.
- **Idle Iq of ±25 A is not current.** Current offsets are calibrated at each
  motor start; before that the reading is offset garbage with PWM off.
- **Speed gains that hold the robot on the ground limit-cycle in free air.**
  4096/4424 swung a raised wheel ±300 RPM at ±6 A (loud buzz, no rotation);
  1024/553 is calm in the air but may be too weak loaded. Judge on the ground.
- **The Workbench motor profile was a placeholder.** Measured dq R is
  0.30–0.32 Ω (not 0.22), L 135–150 µH (not 110); capture before trusting
  any motor constant.
- CAN bus-off is permanent on STM32 FDCAN unless software clears CCCR.INIT; a jolted connector once took the whole drive offline until reset (2026-10-16). Both Nucleo and ESC main loops now recover.
- Check that the planned path fits on the mat before any ground drive; a 0.6 m forward run started 0.14 m off-centre on the wrong side and left the mat (2026-10-17). `move_test.py` now refuses such paths.
- MCSDK starts the speed ramp at the last measured average speed, which goes stale while IDLE: a wheel pushed by hand started at -92 RPM and limit-cycled. HALL_Clear now zeroes it; never treat stale idle telemetry as harmless (2026-10-17).
- **A low calibration RMS can hide an invalid model.** One run had RMS 0.52 px
  with fx/fy = 1.15 and k3 = 72; hold the board ~40 cm from the lens (at ~1.5 m
  the fit is degenerate). `rig_calibrate.py solve` warns about both.
- **Single-frame tag orientation is noisy by several degrees.** Average it; that
  removed a -1% distance bias in `rig_measure.py`.
