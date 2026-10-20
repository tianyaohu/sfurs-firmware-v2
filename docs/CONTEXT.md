# System context

Firmware for an omni-wheel robot built for RoboCup Small Size League
(SSL) Division B. A Nucleo-H755 computes body-twist inverse kinematics and
commands four B-G431B-ESC1 wheel controllers over CAN.

## Goal (MVP: motion control)

Command a velocity or a 1 m move and have the robot do it accurately in every
direction on a flat floor: **1 m command lands within ±10 cm** (along-track,
cross-track and heading; exact spec still to agree with the team lead).

Everything else (kicker, vision, strategy, the sensor boards under `HW/`) is
out of scope until motion control meets that number.

## Where to read next

[PRD.md](PRD.md) what must be built, in phases ·
[ARCHITECTURE.md](ARCHITECTURE.md) what each file does ·
[DRIVE.md](DRIVE.md) drive data flow, CAN protocol, tuning ·
[HARDWARE.md](HARDWARE.md) boards, wheel map, safety ·
[OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) · [test-results/](test-results/)
