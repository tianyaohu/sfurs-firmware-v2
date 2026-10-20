# System context

Firmware for an omni-wheel robot built for RoboCup Small Size League
(SSL) Division B. A Nucleo-H755 computes body-twist inverse kinematics and
commands four B-G431B-ESC1 wheel controllers over CAN.

## Goal (MVP: motion control)

Command a velocity or a 1 m move and have the robot do it accurately in every
direction on a flat floor: **1 m command lands within ±10 cm** (along-track,
cross-track and heading; exact spec still to agree with the team lead).

Everything else (kicker, vision, strategy, the sensor app in
`src/sensors/`) is out of scope until motion control meets that number.

## Where to read next

- [prd/](../prd/index.md): what must be built, in phases
- [technical-development-plans/](../technical-development-plans/README.md): phased plans
- [design-chronological/](../design-chronological/README.md): why things were done
- [architecture/](../architecture/index.md): system, what each file does, drive and CAN protocol
- [system-info/](../system-info/hardware.md): boards, part numbers, wheel map, safety
- [open-questions/](../open-questions/index.md): unanswered questions
- [hardware-test-results/](../hardware-test-results/README.md): one file per session
- [references/](../references/README.md): datasheets and manuals
- [lessons](lessons.md): hard-won one-liners
