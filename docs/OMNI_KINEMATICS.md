# Omni drive: measurements and rebuild plan

Status: IK math rebuilt in `fh/omni-ik`, based on `hw-session` at `4fe8e8c`.
The failed quad sender/TUI remain removed. A separate opt-in IK image and TUI
are described in [KEYBOARD_IK.md](KEYBOARD_IK.md); normal firmware behavior and
the single-wheel TUI are unchanged. See KEYBOARD_IK.md for the authorized
Nucleo CM7 programming/readback record and first working checkpoint. The user
now reports wheel motion through the keyboard IK path, but insufficient effort
to drive the robot; loaded driving and physical axis validation remain open.

The user confirms reliable wheel motion with the latest development firmware
and requests that work now focus on IK, not additional bring-up or safety
frameworks. This is operator evidence; it does not independently resolve the
raw command signs or geometry below.

## Recorded hardware observations

### Latest operator-reported setup (2026-10-15)

- Robot secured off the ground; motor battery OFF.
- USB data connection currently to the Nucleo.
- Two USB data cables are now available, so Nucleo and one ESC can be
  connected simultaneously without transferring the sole data cable.
- A simultaneous ESC connection has not yet been confirmed; verify both probe
  serials before access. Probe/CAN ownership remains exclusive per target/bus.
- Other TUI/debugger/agent release is not yet confirmed. No hardware access,
  flashing, calibration or motion is authorized by this setup note.
- This is a setup snapshot, not a standing assumption about later power state.

User measurements, approximate (2026-10-15):

| Item | Observation |
|---|---|
| Wheel outside diameter | About 65.5 mm on all four wheels |
| Derived rolling radius | About 32.75 mm; provisional |
| Wheel-center height above floor | About 32 mm |
| Small rollers per wheel | 16 |
| Small roller length | About 12 mm; diameter unknown |
| Overall robot outside diameter | About 200 mm, coarse |
| Reported front track | About 144 mm |
| Reported rear track | About 141 mm |
| Reported left/right wheelbase | About 95 mm each |

The user subsequently described measuring center-to-outer-wheel contact.
The endpoints/reference of the track measurements therefore remain ambiguous.
Do not turn these numbers into authoritative contact coordinates. In particular,
wheelbase/2 assumes a chosen midpoint origin, not a measured chassis center.
The noncircular plate outline does not define the kinematic origin by itself.

SolidWorks sources supplied locally:

- `/home/fh/Downloads/Middle Plate.SLDPRT`
- `/home/fh/Downloads/Bottom Plate.SLDPRT`

These files have not been geometrically decoded. Earlier CAD-image dimension
interpretations and simulation poses are references, not measured hardware truth.

## Original team formula (reference, not validated)

```c
#define ROBOT_RADIUS_M 0.034f

motor1_wheel_vel = COSINE_60 * vx + SINE_60 * vy - rotation * ROBOT_RADIUS_M;
motor2_wheel_vel = -COSINE_55 * vx + SINE_55 * vy + rotation * ROBOT_RADIUS_M;
motor3_wheel_vel = COSINE_55 * vx + SINE_55 * vy - rotation * ROBOT_RADIUS_M;
motor4_wheel_veloc = -COSINE_60 * vx + SINE_60 * vy + rotation * ROBOT_RADIUS_M;
```

60/55 degrees describe the assumed rolling-direction projections, not measured
motor directions. Their assignment to physical wheels and the rotation signs
must be re-derived. The meaning of `ROBOT_RADIUS_M` is unconfirmed; its numerical
similarity to a wheel radius does not prove a naming error.

For an eventual model, choose x right, y forward, positive yaw CCW as viewed
from above. For each contact position (x_i, y_i) and unit rolling direction
(d_x, d_y), the geometric projection is:

```
v_i = d_x * vx + d_y * vy + (x_i * d_y - y_i * d_x) * omega
wheel_rpm_i = v_i * 60 / (2*pi*r_i)
```

Apply the independently verified motor sign and motor-to-wheel gear ratio once.
The yaw coefficient is a projected lever arm, not automatically a shared radial
distance. Do not recover wheel rolling angles from contact positions alone.

## Direction observations: distinguish TUI from raw CAN

User reported: FR/BR `+` moves forward, BL/FL `+` moves reverse. What `+`
referred to (TUI direction, displayed effective RPM, or raw CAN RPM) and what
"forward" described must be resolved before using this as a sign table.

The `hw-session` single-wheel sender already negates SW/NW before transmission.
Consequently a TUI `f` request is not a raw positive shaft-RPM request for those
wheels. The previous IK implementation assumed the observations were raw and
added another sign interpretation without establishing the command path.
Also, bottom-tread motion is opposite the wheel's ideal ground-travel direction.

## Rebuilt math and explicit provisional geometry

Files: `Common/Inc/rover_kinematics.h`, `Common/Src/rover_kinematics.c`,
`tests/test_rover_kinematics.py`. The function takes a four-row geometry table;
it is independent of HAL, calibration and CAN. The opt-in IK image calls it;
normal bench/FWU builds do not.

The working origin is midway between the front/rear contact lines, assuming
left-right symmetry. The supplied track/base measurements are used provisionally:

| Wheel / selector | x / y (mm) | Forward rolling dx / dy | Raw motor sign |
|---|---|---|---|
| FR / NE=1 | +72 / +47.5 | -cos60 / +sin60 | +1 |
| BR / SE=2 | +70.5 / -47.5 | +cos55 / +sin55 | +1 |
| BL / SW=4 | -70.5 / -47.5 | -cos55 / +sin55 | -1 |
| FL / NW=8 | -72 / +47.5 | +cos60 / +sin60 | -1 |

**These are assumptions, not a newly measured sign/mounting table.** Front
rolling vectors are perpendicular to assumed 30-degree axle lines; rear vectors
use 35 degrees. Wheel position alone does not determine the rolling direction.
Motor numbering from the original snippet is deliberately not assigned.

With radius 0.03275 m and provisional direct drive (gear ratio 1), before
saturation, raw motor RPM is approximately:

```
FR = -145.791 vx +252.517 vy +25.106 omega
BR = +167.244 vx +238.850 vy +24.783 omega
BL = +167.244 vx -238.850 vy +24.783 omega
FL = -145.791 vx -252.517 vy +25.106 omega
```

Thus this model predicts forward `++--`, right `-++-`, CCW `++++`.
An independent read-only geometry review confirmed these derivations under
the stated assumptions, not their correspondence to the physical robot.
The previous all-positive-right/all-negative-yaw model is not reused.

Motor sign and gearing are applied exactly once. A later CAN adapter must send
these raw outputs directly; do not pass them through the single-wheel sender's
left-sign mirror. Provisional limits match the existing sender: 300 RPM for
NE, 150 for the other wheels (not independently validated ESC limits). Saturation
scales ALL four RPMs by one factor to preserve the requested twist direction.
No minimum-speed floor; exact zero remains zero. Float outputs deliberately
leave wire quantization and small-speed ESC behavior to later integration.

Verification: `python3 tests/test_rover_kinematics.py` — seven tests pass,
including independent axis coefficients, zero/small commands, mixed linearity,
reversal, common saturation, sign/gearing, invalid inputs, and 100 deterministic
forward-model reconstructions. Host tests prove math, not physical travel.

## Integration progress and remaining checks

Accept reliable single-wheel motion as operator-confirmed. Do not repeat
commissioning as a prerequisite to math work. No new lease/watchdog framework
or general-purpose quad controller framework; keep existing protections intact.

1. **Math complete offline.** Keep geometry and polarity explicit and editable;
   retain the coefficient and forward-reconstruction tests.
2. **Interface inspected.** Guarded `TestDriveStart()` requires IDLE and fresh
   ARM; zero and running-target updates are rejected. Keep ESC behavior intact.
   Reconcile the provisional motor-sign table against actual travel at testing.
3. **Minimal transport integrated in opt-in build.** Four signed targets from
   the C model go through existing CAN framing with no second sign mirror.
   PREPARE is ARM-only; START is ARM plus nonzero SPEED; STOP broadcasts three
   times and respects existing stop/re-arm timing. Separate keyboard inputs use
   this mailbox; no new safety framework or PID changes.
4. **Verify actual chassis axes.** Short stand observations followed by explicitly
   authorized ground checks distinguish wheel spin from forward/strafe/yaw motion.
   Adjust provisional geometry only from evidence; do not treat host tests as
   physical validation.

Each step ends with recorded evidence and review before the next begins.
Hardware access still requires exclusive ownership and fresh power/stand
confirmation. This plan authorizes no flashing or movement by itself.

## Cleanup provenance

The abandoned source files and full tracked diff were preserved outside Git:
`/tmp/opencode/omni-ik-abandoned-20261015-030143/`.
Other worktrees (including `sfurs-hw-session`, drive-safety and host-lease lanes)
were left untouched. Old build artifacts are not a valid new drive image.
