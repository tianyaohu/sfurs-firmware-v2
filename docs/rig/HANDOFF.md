# Rig handoff: camera ground truth for motor and drive calibration

Status as of 2026-10-17. Branch `feature/vision-rig`, worktree `Project1/sfurs-vision-rig`.

## What works today (verified on the real camera)
- **Webcam:** `/dev/video0`, Microdia 0c45:636b, MJPG 1280x720 at **30.0 fps**.
  - Requirements: manual exposure ≤100 and 2 V4L2 buffers (1 buffer halves it to 15 fps).
  - Fixed focus: no autofocus control exists, so the lens calibration survives remounting.
- **Lens calibration:** `~/sfurs_firmware/rig-calib/camera.yaml`.
  - 65 views covering all 9 image regions (≥4 views each). fx 880, fy 879, centre (645, 372),
    k1 −0.33, k2 0.09, RMS 1.76 px.
  - Cross-validation, comparing two half-calibrations:
    - within 250 px of the centre: agree to ≤1.6 px
    - 250–450 px: median 2.9 px, max 8.5 px
    - beyond 450 px (the corners): median 13 px, outliers up to ~200 px
  - **Prefer the centre; treat the corners as unverified until a physical edge test.**
- **Displacement measurement, no floor survey needed:** `scripts/rig/rig_measure.py`.
  - Works from one AprilTag 36h11 (ID 0, 100 mm black square) plus `camera.yaml`.
  - Press `r` with the tag still to set the reference. It then reports the distance moved
    within the reference plane and the heading change.
  - Noise with the tag still: **≤1.5 mm, ±0.2°** at about 1 m camera distance.
  - Synthetic check through the real calibration: 0.2–1.8 mm error on 200–360 mm moves.
  - **Real-world accuracy against a known distance is NOT yet confirmed.** One clean hand move
    read 193.2 mm, but its true distance wasn't measured.
  - `--log drive.jsonl --target-mm 500 --tolerance-mm 20` records every frame after `r`. On
    quit it prints: distance, `max_sideways_mm` (drift off the start→end line), heading change,
    error vs target, pass/fail, and a count of frames outside the reliable centre.
  - "out-of-plane" is a sanity check. More than ~10 mm means the tag tilted or lifted, so
    don't trust that reading.

## Using it to check "command 1 m, did it drive 1 m, straight?"
1. Tape tag 0 **flat** on top of the robot, centred if possible. Set the reference with the
   tag on the robot. The tool measures in the tag's plane, so the tag height doesn't matter
   as long as it stays constant.
2. Start: `uv run scripts/rig/rig_measure.py --calibration ~/sfurs_firmware/rig-calib/camera.yaml --log /tmp/drive.jsonl --target-mm <cmd>`.
   Put the robot still at the start point, press `r`, run the drive, wait about 1 s still,
   then press `q`.
3. Read the drive summary. Tolerance is your call; noise is about 2 mm in the centre.
4. **Field-of-view limit:** the camera is currently ~1 m up on a rail, constrained by cable
   length.
   - Visible floor is ~1.47 × 0.83 m, and the **reliable centre is only about ±0.29 m**.
   - So test drives of ≤0.5 m centred in view for now. A 1 m drive needs either the camera at
     ~2 m (active USB extension) or a recalibration with corner coverage.

## Gaps (highest value first)
1. **Ground-truth accuracy test.** Slide the tag a known 210 mm (the checkerboard's long side)
   several times in the centre. If readings are consistently short (one read 193 mm), find the
   cause before trusting absolute distances. Suspects: tag print size (`--tag-mm`) and calibration error.
   (The principal point was 34 px off in the 45-view calibration; it is centred in the 65-view one.)
2. **Calibration quality.** RMS 1.76 px; corners still unverified (2026-10-17 edge round helped the mid-ring). Fixes:
   - glue the board to rigid foam board
   - hold still for each view (capture has no motion-blur check yet)
   - 3–4 views in every image corner, with the board ~40 cm from the lens
   - re-run `~/sfurs_firmware/rig-calib/diagnostics/xval.py` and aim for <2 px edge disagreement
3. **Camera height and coverage.** Get it to ~2 m, or shrink the test area to the reliable zone.
4. **Floor survey for `rig_track` / `rig_analyze`** (the friction-ID pipeline). Not done.
   - `~/sfurs_firmware/rig-calib/floor.json` is a template: tag centres at ±400 × ±250 mm,
     assuming ~2 m height.
   - At the current 1 m height those positions fall outside the reliable zone. Re-plan the
     layout for the real height, then measure the actual tag positions.
5. **Timing.** Rig timestamps are host `time.monotonic()` at frame grab. Exposure/USB latency
   is unknown (`rig_analyze --estimate-latency` exists, untested on real data). The camera and
   `esc_tool` must run on the same host for timestamps to line up.
6. **Motion blur.** Exposure 100 = 10 ms; at 0.3 m/s that's 3 mm of smear. Use more light and
   lower exposure for fast tests.
7. **End-to-end friction ID never run on hardware.**
   - `rig_sweep.py --execute` drives the robot; it is dry-run by default, so get human
     authorization and confirm the battery first.
   - `rig_analyze.py` is only tested on synthetic logs.
8. **Push.** Neither this branch nor `feature/ssl-vision-mvp` (sfurs/software) is pushed.
   GitLab SSH needs `ssh-add ~/.ssh/gitlab_key`.

## Pitfalls already hit (don't repeat)
- A low calibration RMS can hide an invalid model. One run had RMS 0.52 px but fx/fy = 1.15
  and k3 = 72. `rig_calibrate.py solve` now warns about this, and about boards that are too small.
- The calibration capture only detects the ChArUco board, not the AprilTags.
- The board ~1.5 m from the lens gives a degenerate fit. Hold it ~40 cm away.
- Single-frame tag orientation is noisy by several degrees. Averaging it removed a −1%
  distance bias.

## Files
- `scripts/rig/rig_measure.py`: displacement and drive check (new)
- `scripts/rig/rig_calibrate.py`: board / capture / solve
- `scripts/rig/rig_track.py`, `rig_analyze.py`, `rig_sweep.py`: friction-ID pipeline
  (needs gap 4)
- `scripts/rig/test_rig.py`: 11 offline tests; run with `uv run scripts/rig/test_rig.py`
- `~/sfurs_firmware/rig-calib/`: `camera.yaml`, `floor.json` template, `views/`,
  `rejected/`, `diagnostics/` (outside git)
- `~/sfurs_firmware/rig-print/`: printable tag and board PDFs (print at 100%)
