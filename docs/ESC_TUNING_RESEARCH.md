# ST FOC tuning research — 2026-10-15

Read-only web/source research, not a tuning or motion authorization. The requested
`explore` worker hit a usage limit; the parent checked sources directly.
Cross-links: [measured baseline](LOW_TORQUE_DIAGNOSTICS.md),
[FR P2 experiment](FR_LOW_SPEED_P2.md),
[tuning runbook](../.opencode/skills/esc-foc-tuning/SKILL.md).

## Separate the loops

This firmware's `mc_tasks_foc.c` computes Vq/Vd from separate Iq/Id PI controllers
and applies voltage-vector circle limitation. `speed_torq_ctrl.c` computes the
Iq request from a speed PI. Current control executes at PWM 16 kHz; speed control
at 1 kHz. Current loop PI outputs are motor voltage commands, not an additional
DC-bus voltage regulator. Vbus monitoring/protection does not regulate the battery
voltage. Optional field weakening in other configurations is a separate feature,
not a remedy for low-speed stiction or permission to enable it here.

The P2 experiment doubled **speed proportional response only**, not Iq/Id gains.
Captured ~0.024 A median requested/measured current error does not prove current
loop bandwidth/stability: host sampling at 20 Hz cannot resolve a 16 kHz loop.
Likely speed-loop hunting after P2 remains a hypothesis; Hall alignment,
commutation, friction and feedback delay still need discrimination.

## Automatic calculation is not automatic validation

For an ideal decoupled electrical RL plant, continuous SI starting gains can be
chosen approximately `Kp = L * bandwidth_rad_s`, `Ki = R * bandwidth_rad_s`.
The firmware's discrete Ki includes sample time; SDK current/voltage scaling and
integer dividers must be accounted for before inserting any coefficients. These
are design equations, not proposed numeric gain changes.

Workbench calculates initial torque/flux PI coefficients using motor parameters
and the selected bandwidth. Speed-loop design additionally depends on inertia,
friction, torque constant, load and feedback dynamics. Parameter identification
and offline calculation reduce blind trial-and-error; a few bounded stand and
load-validation tests remain necessary. Current-offset CAL is neither motor
identification nor a PI auto-tune. A custom speed-gain sweep is also not ST's
electrical Motor Profiler or proof of a safe optimum.

## Sources checked

1. **ST Technical Moderator, 2022-07-12:** cut-off frequency is used to compute
   torque/flux PI coefficients; execution rate is the FOC execution rate.
   https://community.st.com/stm32-mcus-motor-control-34/what-does-mean-cut-off-frequency-and-execution-rate-in-st-motor-control-workbench-13326
2. **ST employee Gael A, 2025-07-09:** Workbench speed-gain determination from
   `J`, friction, torque constant, bandwidth and SDK units/sample rate. The answer
   explicitly reports a pole-pair scaling issue for manually entered inertia/
   friction in the discussed release. Do not transplant its formulas into another
   version without checking the installed documentation and units.
   https://community.st.com/stm32-mcus-motor-control-34/speed-loop-pid-tuning-in-esc001v1-153312
3. **Board/version-specific ST Community thread, 2025-10-09:** moderator directs
   users to Workbench > About > Documentations > Documentation > User manual >
   Motor Profiler Application Note. The original user's follow-up reports that
   its Known Problems excludes B-G431B-ESC1/ESC boards with MCSDK 6.4.1 and that
   manually entering motor parameters in the board example worked with Pilot.
   This limitation is a user's report on ST's forum, not independently read
   application-note text; verify the exact installed release before promising
   one-click profiling. Older third-party 5.4.8 examples are not proof of 6.4.1
   support. Main ST/wiki fetches timed out during this session.
   https://community.st.com/stm32-mcus-motor-control-34/b-g431b-esc1-with-mcsdk-6-4-1-cannot-generate-or-find-motor-profiler-firmware-157943
4. **ST-hosted community discussion, 2026:** reports that 6.4.2 documentation adds
   `md_docs_2speed__pi__gains__determination.html`. That is a documentation pointer,
   not authorization to upgrade this 6.4.1 project.
   https://community.st.com/stm32-mcus-motor-control-34/speed-pi-gains-used-in-mcsdk-162211

## Better bounded procedure

1. Verify motor Rs/Ls/pole pairs, sense scaling/polarity and Hall phase/sequence;
   distinguish measured motor parameters from copied project settings.
2. Start from analytically calculated/Workbench current gains. If current-loop
   dynamics are suspect, use appropriately fast capture (buffered firmware,
   DAC/scope or supported high-rate telemetry) and small controlled references,
   not only 20 Hz host medians. A torque-mode check needs separate authorization.
3. With current control and commutation established, model/measure speed response
   at representative operating RPM and design conservative outer-loop gains.
   Change one effective variable at a time; retain original image/gain evidence.
4. Validate both directions unloaded, then controlled load and all four wheels.
   Free-air success is not loaded validation. Do not manually brake rotating
   wheels by hand or disable guards to complete identification.

Unmeasured example inertia, friction and ground breakaway-current numbers in
the runbook must not be treated as measurements of this robot. Small Id alone
also does not independently prove Hall alignment. SDK-equivalent Iq is neither
battery current nor an independently calibrated phase-current measurement.
