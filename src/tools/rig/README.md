# Overhead wheel-control rig

One fixed overhead camera measures one robot with AprilTag 36h11 ID 0.
Four surveyed floor tags establish a metric frame; ChArUco calibration removes
the webcam's barrel distortion. This is an offline measurement tool, with no
closed-loop vision control or SSL network output.

## Setup

Start with the camera **2.0 m above the floor**, a **100 mm robot tag**, and
**120 mm floor tags**. Sizes refer to the outside of the black square, excluding
the white quiet zone. Keep at least a quarter-tag white margin. Print the PDFs
at 100% / actual size and measure both axes with a ruler. Tape flat to a rigid,
matte surface. Mount the robot tag horizontally, preferably centered over the
body origin, with printed top pointing forward. Measure its height from the
floor and its center offset from the robot origin. Avoid a flexible mast.

Clamp the camera rigidly, looking approximately vertically down. Secure the
cable; moving the camera invalidates the floor frame. Put floor tags around
the working area without putting tape in the wheel paths. Keep the robot,
including its stopping distance, inside the calibrated area.

The exact field of view must come from the fitted K, rather than the webcam's
marketing specification. For an approximately vertical camera at height H,
floor coverage is `1280*H/fx` by `720*H/fy` and horizontal FOV is
`2*atan(640/fx)`. A tag of side S at height h spans about `fx*S/(H-h)` pixels
near the center. For example, **fx=fy=900 px** gives:

| Camera height | Floor coverage at 1280×720 | 100 mm top tag at h=150 mm |
|---|---|---|
| 1.5 m | 2.13 × 1.20 m | 67 px |
| 2.0 m | 2.84 × 1.60 m | 49 px |
| 2.5 m | 3.56 × 2.00 m | 38 px |

Barrel distortion and tilt change edge coverage and resolution. Aim for at
least 40–60 pixels across the robot tag throughout the useful area; 16 px is
only the software rejection threshold. Use a larger tag or lower camera if
the measured focal length gives fewer pixels. The example floor layout spans
1.82 × 1.00 m including tags; survey your actual positions.

Use bright, diffuse, steady lighting with no glare or visible LED flicker.
The Microdia 0c45:636b needs V4L2 **MJPG, 1280×720, manual exposure** to sustain
30 fps. Auto exposure has been observed to reduce this mode to 5 fps. Both
capture and tracking request these settings through OpenCV, including
`AUTO_EXPOSURE=1`, `EXPOSURE=50` (5 ms), and one-frame buffering. `--exposure`
accepts 1–100 in 100 µs units. Property refusals and exposure readback are
printed; check achieved FPS on the real device. At 0.3 m/s, 5 ms exposure and
the 2 m example above produce about 0.73 px translation blur; 10 ms doubles
that. Add lighting before increasing exposure. Fix focus, zoom and resolution
before calibration and retain them. `--preview` requires a graphical desktop;
the regular opencv-contrib wheel supplies GUI support. No global installs,
sudo, or v4l2-ctl are needed.

## Calibrate and establish coordinates

All commands run from the repository root with PEP 723 environments managed by
uv. Store images, logs and outputs outside git. These example paths use /tmp;
use persistent storage for real measurements.

```sh
uv run src/tools/rig/rig_calibrate.py board --out /tmp/charuco
uv run src/tools/rig/rig_calibrate.py capture --out /tmp/rig-views --count 30 --preview
uv run src/tools/rig/rig_calibrate.py solve --images '/tmp/rig-views/*.png' --out /tmp/camera.yaml
```

The default ChArUco board is 7×5 squares, 30 mm square side, 22 mm markers,
DICT_5X5_100, with a 210×150 mm printed area. Keep these arguments identical
for board/capture/solve if changing them. PNG, dimensioned PDF and board JSON
are produced by `board`. `capture` saves a view at most once per second when
at least eight internal corners are detected. Move and tilt the board between
captures, covering the image edges, corners, center and multiple distances;
repeated identical views do not constrain calibration. `capture --video file`
can extract views offline (`--interval 0` to accept every usable frame).
`solve` accepts multiple image glob arguments and saves K, dist, image size,
RMS and per-view RMS in JSON or YAML. At least eight usable views are required;
20–30 diverse views are preferable. Aim for RMS below 0.5 px; inspect views
above 1 px and validate against surveyed distances. RMS alone cannot detect
poor coverage or wrong printed dimensions.

Copy [floor.example.json](floor.example.json) to a measurement directory, then
replace the tag corners with your survey. Each tag has **four black-square
corners in decoded TL, TR, BR, BL order**, in floor millimeters. The example
tags all have their printed tops toward floor +y. Do not substitute tag
centers or the white paper corners. The frame has x right, y forward at
robot yaw zero, and z upward. Positive yaw rotates body +x toward floor +y
(CCW viewed from above). Robot x right/y forward matches the firmware geometry.
The reported pose refers to the body origin; tag offset is in body x/y.

```sh
uv run src/tools/rig/rig_track.py --floor /tmp/floor.json --print-tags /tmp/rig-tags
uv run src/tools/rig/rig_track.py --calibration /tmp/camera.yaml --floor /tmp/floor.json \
  --tag-mm 100 --tag-height-mm 150 --out /tmp/rig.jsonl --preview
```

Replace 150 with the measured tag height. `--tag-offset-mm X Y` specifies tag
center relative to the body origin; `--tag-yaw-rad` specifies tag +x yaw
relative to body +x. Tag size must match the print. The tracker waits until
all four floor tags are visible, solves their floor-to-camera pose, rejects
floor reprojection RMS above 2 px, then locks the transform for the run.
Floor tags may subsequently be occluded. Restart tracking after any camera
movement; it does not automatically detect a bumped camera. Robot corner
rays intersect the measured **top-tag height plane**, avoiding floor-plane
parallax. A 150 mm height error at a 1 m lateral offset can cause roughly
80 mm position error with a 2 m camera, so height must be measured.

Alternatively supply a homography configuration:

```json
{
  "image_points_px": [[120, 620], [1160, 620], [1160, 100], [120, 100]],
  "floor_points_mm": [[-900, -450], [900, -450], [900, 450], [-900, 450]],
  "camera_center_mm": [0, 0, 2000]
}
```

Those pixel values are illustrative; measure your own matching points on the
floor. Input pixel points are raw distorted pixels and are undistorted before
homography fitting. Four noncollinear point pairs are the minimum; more are
preferable. An explicit `"H": [[...], [...], [...]]` can replace the pairs:
it must map **undistorted K-pixel coordinates to floor mm**. An elevated robot
tag requires surveyed `camera_center_mm` in this mode for parallax correction;
the floor-tag method estimates that center automatically.

Each tracking JSONL row contains `t`, `grab_t`, `frame`, `detected`, `clock`,
`source`, and `quality`. Detected poses add `x_mm`, `y_mm`, `theta_rad`.
Missing/rejected detections still produce a row. Quality includes visible IDs,
rejected candidates, floor readiness/RMS, minimum robot edge length in pixels,
and square-shape RMS in mm. Floor-tag RMS is in px; homography fit RMS is in
mm. Tags below 16 px or with square-shape RMS above 10% of side are rejected.
These are screening measures, not pose covariance estimates. Tracker rejects
images with dimensions different from calibration.

## Quick displacement check (no floor survey)

`rig_measure.py` needs one AprilTag 36h11 ID 0 (100 mm black square) and the
lens calibration. Tape the tag flat on the robot, start it with the robot
still, press `r` to set the reference, run the drive, wait about 1 s, press `q`:

```sh
uv run src/tools/rig/rig_measure.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml \
  --log /tmp/drive.jsonl --target-mm 500 --tolerance-mm 20
```

It prints distance moved in the tag plane, `max_sideways_mm` (drift off the
start-end line), heading change, error against the target and pass/fail.
Noise with the tag still is about 1.5 mm and ±0.2° near the image centre.
An out-of-plane reading above ~10 mm means the tag tilted; do not trust it.
Keep drives inside the reliable centre of the calibrated view (see
open-questions for the current coverage limit). Webcam: Microdia 0c45:636b,
MJPG 1280x720 at 30 fps needs manual exposure (at most 100) and 2 V4L2
buffers; fixed focus keeps the calibration valid after remounting.

## Acquire control measurements

Read [hardware rules](../../../system-info/hardware.md) first. A human must confirm battery state
before each hardware step and after every cable/power change. Battery OFF for
ESC flashing; wheels raised for alignment and the first motion after firmware
changes. Ground motion needs a clear floor, explicit human authorization and
an operator ready to stop. Use one process for the probe/CAN bus; the camera
process can run alongside it. These scripts do not flash or align boards.

Start tracking before telemetry. Use the same Linux host and boot so both
tools share `time.monotonic()`. Start at a low speed, measure actual stopping
distance, and inspect pose/velocity before increasing speed. Keep floor,
battery loading, gains, feed-forward settings and tag mounting consistent.
Use zero existing friction feed-forward for the baseline. Configure gains
and FF through the existing `esc_tool.py set` commands while IDLE, recording
chosen settings externally with the measurement data.

Existing `esc_tool pulse` gives an initial axial tracking/breakaway check:

```sh
src/tools/esc_tool.py pulse --axis vy --speed 0.1 --seconds 1.5 --reps 3 --log /tmp/pulse.jsonl
```

It drives out and back and reports per-wheel breakaway times. Its current
`t` field is **elapsed since pulse start**; earlier keyboard-drive logs use
absolute monotonic time, `vector`, `status`, and `rows`. The analyzer accepts
both, and accepts `twist`/`target_rpm`/`rpm`/`iq_ref_a` pulse arrays or nested
per-wheel `rows`. Raw Iq counts use ik_link's 544 counts/A. FR, BR, BL, FL is
the array order. A one-wheel `spin` log cannot provide four-wheel odometry.

Axial pulses alone cannot identify all direction coefficients. The small
`rig_sweep.py` helper uses the same `ik_link` as esc_tool for oblique
out-and-back commands, reads all four telemetry rows, and logs **absolute
monotonic time**. It changes no gains or feed-forward settings. It prints a
plan by default without opening hardware:

```sh
uv run src/tools/rig/rig_sweep.py --angles-deg 11.25 --speeds 0.1
# Human-authorized motion only, after battery confirmation and clearance:
uv run src/tools/rig/rig_sweep.py --angles-deg 11.25 --speeds 0.1 \
  --seconds 1.5 --out /tmp/sweep.jsonl --execute
```

Once the initial measurement is safe and useful, omitting `--angles-deg` and
`--speeds` plans eight directions over a half-circle, their opposites, at
0.1/0.2/0.3 m/s (48 moving plateaus). Default durations are 1.5 s motion and
1 s rest, for 121 s overall. Out-and-back commands reduce travel but cannot
guarantee return to the starting position. The helper requires all wheels
IDLE and aligned initially, aborts on faults/derating, missing rows or status
counts stale for >200 ms, disables drive in `finally`, waits for STOPPED,
then releases the probe. Ctrl-C also takes that stop path. Check the printed
state and robot afterward. Extend rest or collect shorter batches as needed.

## Align and analyze

```sh
uv run src/tools/rig/rig_analyze.py --rig /tmp/rig.jsonl --esc /tmp/sweep.jsonl \
  --estimate-latency --out /tmp/rig-analysis
```

Live `t` is sampled immediately after `grab()` and before decode/detection.
It is a host delivery timestamp, not the sensor exposure midpoint. Positive
`--latency-s L` means vision arrives late: vision at t compares with telemetry
at `t-L`. `--estimate-latency` searches ±0.5 s around that supplied offset in
5 ms steps, maximizing demeaned vision-speed/wheel-odometry-speed correlation.
It needs speed changes, adequate overlap, and no severe slip during the
alignment events. Search-edge and competing-peak warnings require inspection;
a repetitive pulse pattern can be ambiguous. This estimates the aggregate
camera-versus-wheel delay, including Hall filtering and host polling; it
does not independently measure each device's latency.

For elapsed-time pulse logs, add the original pulse-start monotonic time:

```sh
uv run src/tools/rig/rig_analyze.py --rig /tmp/rig.jsonl --esc /tmp/pulse.jsonl \
  --telemetry-origin-t 12345.0 --estimate-latency --lag-search-s 2 --out /tmp/pulse-analysis
```

Replace 12345.0 with the known or approximate origin. If the origin was not
recorded, estimate it from a recognizable start in the vision trace, then
search around that estimate; the recovered offset includes the origin error.
The analyzer never assumes that two files started simultaneously. Logs from
different hosts/reboots need an independently measured clock mapping.

For recorded video, reconstruct the original timeline explicitly:

```sh
uv run src/tools/rig/rig_track.py --calibration /tmp/camera.yaml --floor /tmp/floor.json \
  --video /tmp/run.avi --video-start-t 12345.0 --tag-height-mm 150 --out /tmp/replay.jsonl
```

This uses `start + frame_index/video_fps`, suitable for constant-frame-rate
recordings with no dropped frames. The original first-frame time must be
recorded by the capture system. Without it the tracker uses replay grab times
for offline inspection; analysis refuses to align those with telemetry.
Variable-frame-rate or dropped-frame videos need the original frame-time log;
they cannot be recovered reliably from nominal FPS. Live JSONL remains the
preferred measurement record.

Analysis unwraps yaw and differentiates local quadratic pose fits over 0.3 s
using actual sample times. It does not bridge missing detections; telemetry
interpolation also rejects gaps >150 ms. Velocities rotate into body x/y.
`ik_link.py` supplies wheel order and current scale; the IK matrix is read
directly from `src/shared/src/rover_kinematics.c` to preserve radii, motor signs
and the provisional contact geometry. Pseudoinversion supplies wheel odometry.
Wheel slip is **measured motor RPM minus motor RPM predicted from vision
body twist**. Geometry error also contributes to this residual; it is not a
pure tire-slip measurement or a dimensionless slip ratio.

Steady segments require an unchanged command, 0.5 s settling, at least 0.4 s
usable data, translation >=30 mm/s, acceleration <100 mm/s², near-zero yaw,
wheel tracking error <30 RPM, wheel/vision residual <30 RPM, and current below
95% of the configured 6 A limit. Logged faults/derating are excluded. Thresholds
are CLI options; tune against plots instead of relaxing them to obtain a fit.
Old pulse logs do not contain derating flags, and wheel rows are polled
asynchronously; absolute timing precision is limited by that telemetry.

For every wheel the least-squares model on equally weighted segment means is:

```text
u_A = d_A_per_RPM * w_ref_RPM + f(theta)
f(theta) = a_A * sin(theta) + b_A * sign(sin(theta))
theta = atan2(body_vy, body_vx), measured driving direction, not robot yaw
```

Signed `iq_ref_a` is the default drive effort; `--effort iq_a` fits measured
current instead. This is a current/torque-equivalent feed-forward fit, not a
PWM or voltage fit. Near the sine discontinuity, |sin(theta)|<0.001 is treated
as zero. Collect oblique directions away from that boundary and several speeds
in each direction. The fit reports a, b, d, segment RMSE, R², normalized design
condition and approximate 95% confidence half-widths in coefficient order
`[a,b,d]`. Rank-deficient or poorly conditioned sweeps report unidentifiable
instead of invented coefficients. Per-direction effort-versus-reference-RPM
line fits also report their extrapolated zero-speed intercept when at least
two distinct speeds exist. Confidence estimates describe scatter of segment
means, not calibration bias, model error or independent repeated experiments.

The intercept estimates **running friction extrapolated toward zero speed**.
Actual static breakaway friction can be larger. Validate the camera-free
current-ramp breakaway idea against the first sustained vision displacement,
body yaw kick and Hall-edge timing. Do not use a steady-state intercept as
proof that a wheel has physically broken free. The analyzer does not apply
its coefficients to firmware; current firmware's static/kinetic scalar FF is
not yet the direction-dependent model fitted here.

Outputs are `summary.json`, `aligned.jsonl`, `tracking.png` (command/vision/
odometry), `slip.png` (trajectory and four wheel residuals), and `friction.png`
(directional effort after subtracting viscous effort). Review all wheels,
sample coverage, rejected intervals and residual patterns before interpreting
coefficients. A high R² does not establish the physical model.

## Accuracy and offline verification

At roughly 2 mm/px, a realistic initial target is 3–10 mm position accuracy,
0.5–2° yaw, and roughly 10–30 mm/s differentiated-velocity noise with the
0.3 s window. These are setup targets, not hardware-verified specifications.
Survey/tag-height error, blur, rolling shutter, floor tilt, camera motion,
Hall filtering and provisional wheel geometry can dominate. Verify static
points and measured straight distances at the center and edges, then repeated
translations and yaw before using data for tuning. More smoothing reduces
noise but blurs short transients; retain the raw pose log.

```sh
uv run src/tools/rig/test_rig.py -v
```

Tests generate distorted AprilTags with a known raised, offset top tag,
rotated/translated camera and yaw wrapping, encode a 720p MJPG trajectory,
run the real tracker CLI and check pose/timestamp/dropout behavior. Other tests
exercise homography height correction, missing floor, size mismatch, ChArUco
solve/print, telemetry units/order, gaps and rank-deficient sweeps. Synthetic
analysis supplies 16 directions × three speeds, noise and missing detections,
then invokes the real analysis CLI to recover latency and all four wheels'
known a/b/d and produce plots. All generated data stays in temporary storage.

Later, a small adapter can send these body poses/velocities into skynet with
an explicit coordinate/time conversion and robot ID. Replacing the webcam
front end with SSL-Vision can preserve the downstream comparison and fitting
workflow once SSL capture-time clocks are mapped to the telemetry clock.
Emitting `SSL_WrapperPacket`, multi-robot association, ball tracking and
multi-camera fusion are outside this MVP; a packet adapter is a possible next
step after the single-camera measurements have been verified.
