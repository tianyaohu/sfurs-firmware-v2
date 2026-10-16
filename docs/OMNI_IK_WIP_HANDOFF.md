# Omni IK WIP checkpoint — hardware work stopped

Operator requested stopping all hardware work and committing this lane as WIP.
Branch `fh/omni-ik`, prior HEAD `572d4d8`; integration destination remains
`hw-session` first, with no merge or push authorized by this checkpoint.
Existing source/test/doc changes, including preserved experimental tuner code,
are checkpointed; this is **not** a production-readiness or tuning-approval claim.

## Hardware handoff (historical observations, not fresh readiness)

- All agent-owned probe sessions closed. Process check found no keyboard TUI,
  ESC logger/tuner, STM32 programmer, pyOCD, OpenOCD or ST-LINK debugger running.
  No new hardware connection/command was made to create this checkpoint.
- FR/NE **Ki4 installed and full-readback verified**: Kp -18600/1024,
  Ki -288/16384. Other three ESCs remain original Ki -72/16384.
  **No ground driving with mixed gains.** No Ki4 motor-motion test occurred.
- Nucleo restored to exact verified
  `CM7/build/IK-900-reviewed/sfurs_firmware_v2_CM7.elf`, SHA256
  `83cecd66e9a3ead263b40607ca909caa4a74ea9847274a4af9d99e60c419ee77`.
  Preserve this ignored artifact; rebuilding does not change the board.
- Latest read-only preflight: fresh all-four IDLE, zero RPM, CAL-valid,
  disarmed/inactive, no active faults; FR direct CAL 2/1 and faults 0/0.
  Attempted stand pulse aborted **before ARM/RUN**, at FR ~10.12 V. No pending
  motion or FWU retry was left. No hardware recheck at checkpoint time.
- Battery last confirmed ON, all wheels raised, separate FR/Nucleo USB connected.
  Battery OFF was recommended afterward but **not confirmed**. Do not assume
  power state or CAL survives the handoff. Fresh setup, exclusive hardware
  ownership, power and operation authorization are required before resuming.

## Evidence, limitations and checks

- [Ki4 review, installation and blocked stand test](RPM2_KI4_REVIEW.md).
- [Original-gain ground baseline](LOADED_DRIVE_BASELINE.md): ~5 mm creep at
  1 m/s input for 1 s; failed useful loaded start, not speed validation.
- [Caps/rollout](HIGH_SPEED_CAPS.md), [historical P2 comparison](FR_LOW_SPEED_P2.md),
  [tuning research](ESC_TUNING_RESEARCH.md).
- 5 m/s input ceiling and +/-900 RPM bounds do not establish actual travel speed.
  Yaw ceiling remains 1 rad/s; low-yaw startup remains unresolved.
- Target-ready TUI handshake and manual CAN-CAL byte-order defects remain open:
  [RCA](NE_SE_POST_ROLLOUT_RCA.md). Preserved runtime gain-set/auto-tuner code is
  not enabled in installed images or authorized for a physical sweep.
- Latest offline candidate checks: 6 Ki4 groups, 4 prior PI groups, all-four
  stock/Ki4 guard matrix, 32 monitor tests and four FWU regression suites passed.
  Eight audited stock/candidate ESC builds passed; every stock app matched its
  verified archive and each candidate changed only two Ki initializer bytes.
  Those runs predate this commit; full project regression was not rerun just
  to checkpoint. `git diff --check` passed before staging.

Build outputs, payloads, capture logs and full-flash backups remain outside the
source commit. Key artifact roots (local, not durable Git storage):

- `/tmp/opencode/rpm2-ki4-review-20261016/`
- `/tmp/opencode/fr-900-stock-20261015/`
- `/home/fh/sfurs_firmware/esc-backups/fr-ki4-20261016T002315Z/`

Use verified FR sidecar `NE-after.json` in that backup directory with monitor
profile `ne-900-ki4`; stock identity must reject the installed Ki4 image.
