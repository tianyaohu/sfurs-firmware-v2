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
  different ELF hashes; `build_all.py` prints hashes of the loadable image.
