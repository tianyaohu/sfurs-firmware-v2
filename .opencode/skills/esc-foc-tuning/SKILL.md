---
name: ESC FOC Tuning
description: Tune B-G431B-ESC1 (STM32G431 / MCSDK v6.4.1) Hall-sensored FOC current and speed PI loops from free-air stand to loaded ground driving — battery/CAL procedures, speed selection, load translation physics, quantitative gain formulas, and dual-cable verification.
---

# ESC FOC & Speed-Loop Tuning Runbook (B-G431B-ESC1)

> Evidence boundary (2026-10-15): see `docs/ESC_TUNING_RESEARCH.md` for checked
> ST sources and profiling-version limitations, and `docs/HIGH_SPEED_CAPS.md`
> for current-source versus installed speed caps. Numerical inertia/load-current
> examples and proposed gain windows below are **not measured robot constants
> or validated tuning bounds**. P2 hunting is a hypothesis supported by operator
> reports/logs, not proof of its exact Hall/FIFO mechanism. A 20 Hz logger cannot
> establish 16 kHz current-loop stability; small Id alone cannot prove Hall
> alignment. Do not apply proposed gains, auto-tune sweeps, profiler firmware or
> motion without a separately agreed experiment. Phase/dq current is not battery
> current: adding four 3 A Iq limits does not establish a 12 A battery draw.

**Latest installation (2026-10-16): FR/NE Ki4 is installed and independently
verified**, not yet stand-motion validated. Other three wheels remain stock:
**no ground driving with mixed gains**. FR speed Kp -18600/1024, Ki -288/16384;
all caps/current limits/guards unchanged. Use explicit monitor profile
`ne-900-ki4` with verified sidecar
`/home/fh/sfurs_firmware/esc-backups/fr-ki4-20261016T002315Z/NE-after.json`.
Identical IK host restored/verified; both probes released. Battery last confirmed
OFF, all wheels raised, FR CAL 0/0 and battery-OFF undervoltage 4/4 after reset.
No powered readiness, CAL or motion authorized by the installation. Next requires
fresh battery-ON readiness/CAL-only approval, then separate stand-motion approval.
Evidence and full payload/readback identities: `docs/RPM2_KI4_REVIEW.md`.

Later CAL-only approval with operator-reported battery ON did **not** establish
readiness: direct Ki4 preflight measured FR ~4.76 V average / ~4.78 V ADC,
active undervoltage fault 4, CAL 0/0. Aborted before any Nucleo command; no CAL,
ARM or RUN sent. Both probes released. Check external power path with battery
OFF before adjusting connections; fresh power confirmation and voltage check
are needed. Do not infer that user-reported ON means valid ESC motor-bus power.
At 00:34Z, operator-reported battery restoration still yielded FR ~4.61 V
average / ~4.64 V ADC and active undervoltage, CAL 0/0. Again no Nucleo commands;
both probes released. Resolve external feed versus sensing before more CAL or
tuning; see the recurring-power evidence in `docs/RPM2_KI4_REVIEW.md`.
Operator then found the motor feed disconnected/OFF and confirmed it corrected.
At 00:36Z FR measured ~10.74 V average / ~10.82 V ADC, active faults 0 / past 4,
CAL 0/0. This remains below the 11 V resting test gate, although above firmware
undervoltage. Again no Nucleo command or CAL; both probes released. Independent
battery/feed voltage measurement or recharge is needed before proceeding.

**Subsequent preparation succeeded (00:39:46Z):** operator requested use of the
current battery while another charges. Explicit 10.5 V procedural floor allowed
**raised-wheel CAL only**, not RUN/ground tests; firmware protections unchanged.
Two PREPARE transactions acknowledged cleared faults then completed RAM CAL on
all four. STOP plus fresh advancing all-four IDLE, RPM 0, CAL-valid, disarmed/
inactive and active faults 0 verified. FR direct CAL 2/1, faults 0/0; preparation
Vbus 10.596–10.600 V. Both probes released, battery last ON, all wheels raised.
No RUN sent; further motion needs separate approval. Power changes invalidate
readiness. Exact evidence: `docs/RPM2_KI4_REVIEW.md`. Earlier CAL 0/0 notices
above are historical; this is not loaded power-margin or motor tune validation.

The subsequently authorized FR-only -100 RPM / ~1 s stand pulse **did not run**:
preflight Vbus fell to ~10.12 V (ADC ~10.13 V), below its separately approved
10.5 V floor. Log has only a sample; direct ARM/RUN/request/run counter all 0,
no Nucleo commands. Fresh all-four IDLE, RPM 0, CAL-valid, inactive/disarmed,
active faults 0; both probes released. Advised battery OFF and waiting for the
charged replacement. OFF is not yet confirmed. No Ki4 stand-motion validation
exists; do not infer that CAL success validates power under load. See
`docs/RPM2_KI4_REVIEW.md` for evidence and unresolved independent-voltage check.

Historical installed baseline (2026-10-15): the completed all-four cap rollout and
separately approved FR gain restoration leave **all four wheels on original
RPM2 speed gains** (-18600/1024 and -72/16384), with +/-900 RPM ESC/IK caps and
5 m/s host input bounds. FR logger uses explicit `ne-900-stock` identity; see
`docs/FR_LOW_SPEED_P2.md` for the verified restoration sidecar. P2 below is a
historical comparison, not the current tune. Powered calibration-only preparation
is recorded in `docs/FR_LOW_SPEED_P2.md` (all-four fresh CAN CAL-valid, FR direct
2/1). Subsequent raised-wheel capture established all-four WASD rotation at
1 m/s input; operator reported good translation. Default Q/E (0.5 rad/s) sent
only +/-12–13 RPM: all four entered RUN without sustained Hall rotation; FR
requested <0.10 SDK A and longer attempts reached stall reason 9. This is not
loaded validation or quantitative smoothness acceptance. TUI exit STOP was
initially unconfirmed, then independently verified completed with all four
stopped/disarmed; see that document for exact evidence and remaining caveats.
CAL is volatile and that observation is not a permanent readiness claim. Use selection/checklist handoffs per
`can-fwu-update`; never infer authorization for an automatic gain sweep.

First Nucleo-only ground baselines (2026-10-16) are recorded in
`docs/LOADED_DRIVE_BASELINE.md`: one 0.5 m/s, 0.5 s forward attempt entered RUN
on all four but produced only a slight twitch, sampled RPM 0, 0/1 Hall changes,
and peak sampled |Iq| ~0.40–0.59 SDK A. STOP and all-four disarmed/IDLE verified.
The pulse only spans the 500 ms ramp; it does not prove steady loaded torque,
current-limit saturation, a stall stop, or a need for increased current limits.
Ground connection is Nucleo-only: Iq reference, Vbus and explicit stop reason
are unavailable. A separately approved 1 s pulse at the same speed also produced
only twitch/buzz, zero sampled RPM and zero Hall changes on all four; peak sampled
|Iq| ~0.72–0.82 SDK A, no active faults. STOP/disarm independently verified.
Operator reports mass under 5 kg and smooth hard floor. A separately approved
1 m/s, 1 s forward comparison then produced only ~5 mm forward creep (operator
visual estimate, failed useful loaded start): Hall deltas
6/5/11/9, only BL sampled nonzero RPM (-36 briefly), peak sampled |Iq|
~1.51–1.66 SDK A. No sustained target tracking. No active faults; STOP and
fresh all-four IDLE/disarmed independently verified, Nucleo probe released.
Further motion/tuning needs separate approval. Do not infer a need to raise
current limits, absence of reference saturation, or voltage margin from these
Nucleo-only tests. Full measurements/identities are in the loaded-baseline doc.

Cross-links:
- Ki-only candidate review and pre-install gates: `docs/RPM2_KI4_REVIEW.md`
- Commissioning & CAN/TUI topology: `.opencode/skills/wheel-motor-setup/SKILL.md`
- Over-CAN firmware update (protocol v2): `.opencode/skills/can-fwu-update/SKILL.md`
- Baseline torque & current diagnostics: `docs/LOW_TORQUE_DIAGNOSTICS.md`
- Nucleo-only loaded baseline and evidence limits: `docs/LOADED_DRIVE_BASELINE.md`
- FR doubled-$K_p$ (`ne-low-speed-p2`) stand experiment: `docs/FR_LOW_SPEED_P2.md`
- Post-rollout CAL/ARM handshake RCAs: `docs/NE_SE_POST_ROLLOUT_RCA.md`

---

## 1. Control Architecture & Hardware Constants

The ESC (`HW/B-G431B-ESC1/motor_config/`) runs ST MCSDK v6.4.1 Field-Oriented Control (FOC) with **three cascaded PI regulators** (all derivative $D$ terms are `DISABLE`) and **no voltage PID loop**:

```
Target RPM ──► [STC 500 ms Ramp] ──► [Speed PI @ 1 kHz] ──► Iq_ref (±1632 cts = ±3.0 A) ──► [Iq PI @ 16 kHz] ──► Vq ──┐
                                            ▲                                                     ▲                   ├──► Circle Limit & SVPWM
                                    hAvrMecSpeedUnit                                         Measured Iq              │    (scaled by Vbus)
                                  (Hall 16-edge FIFO)                                                                 │
                                                             Id_ref = 0 A ────────────────► [Id PI @ 16 kHz] ──► Vd ──┘
```

### Fixed Plant & Scaling Constants
| Parameter | Macro / Symbol | Value | Physical / Control Meaning |
|---|---|---|---|
| Pole pairs | `POLE_PAIR_NUM` | `8` (16 poles) | $6 \times 8 = 48$ Hall edges per mechanical rev ($7.5^\circ$ mech / $60^\circ$ elec per edge) |
| Stator $R_s, L_s$ | `RS`, `LS` | $0.22\ \Omega$, $110\ \mu\text{H}$ | Electrical time constant $\tau_e = L_s / R_s = 0.50\text{ ms}$ ($\omega_p = 2000\text{ rad/s}$) |
| Back-EMF / Torque const | `MOTOR_VOLTAGE_CONSTANT` | $3.0\text{ V}_{\text{rms,ll}}/\text{kRPM}$ | $K_t \approx 0.0351\text{ N}\cdot\text{m/A}_{\text{peak}}$; direct drive (`gear_ratio = 1.0`, $r_w = 32.75\text{ mm}$) |
| Current sense scale | `CURRENT_CONV_FACTOR` | `544` counts/A | $R_{\text{shunt}} = 3\text{ m}\Omega$, net gain $9.14$, $3.3\text{ V}$ ADC (`0.001836397` A/count) |
| Nominal / max $I_q$ | `IQMAX_A` / `IQMAX` | `3.0 A` (`1632` counts) | Clamps Speed PI output (`±1632`) & accumulator (`±1632 * SP_KIDIV`); $\tau_{\max} \approx 0.105\text{ N}\cdot\text{m}$ |
| Sampled L1 current stop | `ESC_TEST_SAMPLED_STOP_A` | `3.0 A` (`1632` counts) | Trips `TEST_STOP_CURRENT` (`4`) if $\|I_q\| + \|I_d\| \ge 1632$ at 2 kHz SysTick |
| Hardware OCP | `COMP1/2/4 -> TIM1_BRK` | Cycle-by-cycle | Nanosecond PWM break; latches `MC_OVER_CURR` (`0x0040`) $\rightarrow$ `TEST_STOP_FAULT` (`6`) |
| Speed unit | `SPEED_UNIT` (`U_01HZ`) | `10` ($0.1\text{ Hz}$ mech) | **`1 unit = 0.1 rev/s = 6.0 RPM`**; `RPM_2_SPEED_UNIT(rpm)` truncates via `(rpm * 10) / 60` |
| Hall FIFO & timeout | `HALL_AVERAGING_FIFO_DEPTH` | `16` edges ($120^\circ$ mech) | `HallTimeout = 150 ms` (`MaxPeriod = 149.6 ms` $\approx 8.36\text{ RPM}$); startup deadzone $< 25.11\text{ RPM}$ |
| Stall guard | `ESC_TEST_HALL_STALL_MS` | `2000 ms` | Trips `TEST_STOP_STALL` (`9`) if no valid Hall edge occurs for $2.0\text{ s}$ |
| Guarded `RPM2` limits | `ESC_REVERSE_SPEED_DIAGNOSTIC` | $\pm 900\text{ RPM}$ (`test_profile.h`) | `500 ms` ramp, `480` net-edge count stop (10 revs, `reason=8`), `30 s` timeout (`reason=1`) |

### Current & Speed PI Gains in Code
| Loop | Rate | Numerator (`int16_t`) | Divisor (`uint16_t` / shift) | Effective Gain & Bandwidth / Time Constant |
|---|---|---|---|---|
| **Inner $I_q$ & $I_d$** | $16\text{ kHz}$ | `KP = 3391`<br>`KI = 3343` | `TF_KPDIV = 512` (`>> 9`)<br>`TF_KIDIV = 4096` (`>> 12`) | Pole-zero cancellation ($K_i/K_p = R_s T_s / L_s \approx 0.1232$) at $\omega_{bw} = 6000\text{ rad/s}$ ($\sim 955\text{ Hz}$) |
| **Outer Speed (`RPM2` baseline)** | $1\text{ kHz}$ | `KP = -18600`<br>`KI = -72` | `SP_KPDIV = 1024` (`>> 10`)<br>`SP_KIDIV = 16384` (`>> 14`) | $k_p = 18.164\text{ cts/unit}$ ($33.4\text{ mA per }6\text{ RPM}$)<br>$k_i = 0.004395\text{ cts/(unit}\cdot\text{ms)}$ ($8.08\text{ mA/s per }6\text{ RPM}$), $T_i = 4.13\text{ s}$ |
| **Outer Speed (`ne-low-speed-p2`)** | $1\text{ kHz}$ | `KP = -18600`<br>`KI = -72` | `SP_KPDIV = 512` (`>> 9`)<br>`SP_KIDIV = 16384` (`>> 14`) | $k_p = 36.328\text{ cts/unit}$ ($66.8\text{ mA per }6\text{ RPM}$)<br>$k_i = 0.004395\text{ cts/(unit}\cdot\text{ms)}$, $T_i = 8.27\text{ s}$ (hunts in free air) |

> **Why `RPM2` Speed PI gains are negative (`-18600`, `-72`):**
> On these motors, $+I_q$ drives the rotor in the `NEGATIVE` Hall sequence (`hMeasuredSpeed < 0`). Because `STC_CalcTorqueReference` computes `hError = hTargetSpeed - hMeasuredSpeed`, negative PI gains are required so that `hError > 0` commands $I_{q,\text{ref}} < 0$ (`POSITIVE` Hall rotation), preserving negative feedback in both directions.
>
> **Critical `int16_t` overflow rule:**
> `hDefKpGain`, `hDefKiGain`, `hKpGain`, and `hKiGain` are **`int16_t` (`-32768..32767`)**. Never double `-18600` to `-37200` (which wraps to `+28336` and reverses loop sign into positive-feedback runaway). To increase effective gain beyond `|numerator| = 32767`, halve `SP_KPDIV` or `SP_KIDIV` and decrement `SP_KPDIV_LOG` or `SP_KIDIV_LOG` in lockstep.

---

## 2. Battery, Power-State, and Calibration Procedure

### A. Battery Rules by Operation
1. **ESC Flashing (SWD or CAN-FWU): Battery MUST be OFF (USB 5 V power only).**
   - Prevents unintended gate-driver energization or shoot-through while GPIO pins initialize across reset/erase/jump, and prevents immediate spin if a candidate build has a sign or gain error.
   - *CAN-FWU note with battery OFF:* With battery OFF, $V_{\text{bus}} \approx 4.4\text{ V} < 8\text{ V}$, holding the app in `FAULT_NOW` (`10`) with `MC_UNDER_VOLT` (`0x0004`), which blocks remote CAN boot entry (`0x650|dev`). To flash over CAN-FWU with battery OFF, arm the Nucleo FWU host first, then issue a normal SWD system reset (`Target.ResetType.SYSRESETREQ`) on the target ESC probe so it enters the bootloader's 3-second listening window (`0x08000000` holds all 6 gate pins LOW and ignores $V_{\text{bus}}$).
2. **Calibration (`CAL`) & Motor Running: Battery MUST be ON ($V_{\text{bus}} \approx 11.5\text{--}12.3\text{ V}$, within $8\text{--}14\text{ V}$).**
   - `R3_2_CurrentReadingPolarization()` activates `TIM1` triggers with PWM channels disabled to sample the zero-current baseline of the 3-shunt (`3 mΩ`) + internal `OPAMP1/2/3` (`PGA gain 16`) network (`PhaseAOffset`, `PhaseBOffset`, `PhaseCOffset`).
   - If battery is OFF (`MC_UNDER_VOLT` active), both `TestDriveArm()` and `TestCalibrationService()` reject immediately (`result = 3`).
3. **Power-Cycle / Rail Decay Rule:**
   - Turning the battery OFF then ON wipes volatile SRAM calibration (`test_calibration_result = 0, test_calibration_valid = 0`) on un-cabled ESCs and latches `MC_UNDER_VOLT` (`0x0004`) in `PastFaults` (`mci_state = 11` `FAULT_OVER`) on USB-powered ESCs.
   - If the first CAL after battery-ON rejects (`result = 3` or `4`), retry once; if it rejects persistently, turn the battery OFF for **several minutes** (not seconds) so bulk/rail capacitors fully decay before powering back ON.

### B. Exact State Machine After Turning Battery ON (`11 -> 0 -> CAL 2/1 -> ARMED -> RUN`)
Because `ESC_LOW_CURRENT_TEST` disables automatic fault acknowledgment in `main.c`, and because `MCI_FaultAcknowledged()` only queues `DirectCommand = MCI_ACK_FAULTS` for the next 1 kHz medium-frequency task tick, an ESC sitting in `mci_state = 11` (`FAULT_OVER`) after battery-ON requires a **three-step preparation sequence** before `RUN`:

1. **Step 1 — Clear `FAULT_OVER` (`mci_state: 11 -> 0`):**
   - Send 1st `ARM` (`P` in `keyboard_drive.py`, or CAN `0x400|id` with `"RPM2"`).
   - `TestDriveArm()` calls `MC_AcknowledgeFaultMotor1()`, sees `MC_GetSTMStateMotor1()` is still `11` during the RX ISR, and returns `false` without starting CAL. Within `1 ms`, `TSK_MediumFrequencyTaskM1` transitions `mci_state` from `11` to `0` (`IDLE`).
2. **Step 2 — Trigger & Complete RAM Auto-CAL (`CAL: 0/0 -> 1/0 -> 2/1`):**
   - Send 2nd `ARM` (`P`) once `mci_state == 0`.
   - `TestDriveArm()` sees `cal_valid == 0`, starts `MC_StartPolarizationOffsetsMeasurementMotor1()` (`mci_state = 17` `OFFSET_CALIB`), and returns `false` with `test_drive_armed = 0`.
   - Wait `~100–200 ms` for `test_calibration_result == 2` and `test_calibration_valid == 1` (`mci_state == 0`). **Auto-CAL completion NEVER arms or starts the motor.**
3. **Step 3 — Arm Within the 5-Second Window & Start (`armed = 0x52504D32 -> RUN`):**
   - Send 3rd `ARM` (or press movement key in `keyboard_drive.py`, which sends `ARM` then `SPEED_WRITE` after `CAL == 2/1`).
   - Now `TestDriveArm()` sets `test_drive_armed = 0x52504D32` (expires after `5000 ms`), and the subsequent `SPEED_WRITE` (`0x080|id`, `ramp_ms = 500`) enters `CHARGE_BOOT_CAP` (`10 ms`) $\rightarrow$ `RUN` (`mci_state = 6`).

> **Golden Probe Rule:** Whenever battery is ON or `CAL == 2/1` is in RAM, **never normal-connect** over SWD. Always use `connect_mode="attach"` (pyOCD / `monitor_esc_power.py`) or `mode=HOTPLUG` (`STM32_Programmer_CLI`), or the debugger connection will reset the MCU and wipe RAM calibration back to `0/0`.

---

## 3. Physics: How an In-the-Air Tune Translates (and Fails to Translate) to a Loaded Tune

Because the rover uses **direct-drive hub/wheel coupling (`gear_ratio = 1.0`, $r_w = 32.75\text{ mm}$)**, there is no gearbox ($1/G^2$) to isolate the motor from the robot's mass or multiply startup torque:

| Property | In the Air (On Stand, No Load) | On the Ground (Under Robot Weight & Omni Scrub) | Ratio / Impact |
|---|---|---|---|
| **Effective Inertia $J_{\text{eff}}$** | $J_{\text{air}} = J_{\text{rotor}} + J_{\text{wheel}} \approx (0.6\text{--}1.2)\times 10^{-4}\text{ kg}\cdot\text{m}^2$ | $J_{\text{load}} = J_{\text{air}} + \frac{1}{4}M_{\text{robot}} r_w^2 \approx (0.8\text{--}3.2)\times 10^{-3}\text{ kg}\cdot\text{m}^2$ | **$10\times\text{ to }50\times$ higher inertia on ground** |
| **Accel Sensitivity $\alpha / I_q$** | $\sim 4190\text{ RPM/s per Amp}$ | $\sim 85\text{--}400\text{ RPM/s per Amp}$ | Free air is $10\text{--}50\times$ more twitchy to torque steps |
| **Breakaway Current $I_{\text{break}}$** | $0.20\text{--}0.35\text{ A}$ (bearing + seal stiction + cogging) | $1.0\text{--}2.6\text{ A}$ (tire flattening + angled $30^\circ/35^\circ$ sub-roller pin scrub) | **$4\times\text{ to }10\times$ higher breakaway current on ground** |
| **Steady Running Current** | $0.10\text{--}0.19\text{ A}$ | $0.6\text{--}1.5\text{ A}$ (surface & translation direction dependent) | Requires sustained integral action to hold target speed |

### Three Failure Modes When Translating Between Stand and Ground

1. **Why doubling $K_p$ (`P2`) caused severe hunting in the air:**
   - At low speeds, Hall edges are sparse ($48\text{ edges/rev} \implies f_{\text{edge}} = 0.8 \times \text{RPM}$): at $29\text{ RPM}$, edges arrive every $43.1\text{ ms}$; at $51\text{ RPM}$, every $24.5\text{ ms}$.
   - The 16-edge moving average (`HALL_AVERAGING_FIFO_DEPTH = 16`) introduces a group delay $\tau_{\text{FIFO}} \approx \frac{10}{\text{RPM}}\text{ s}$ ($\sim 200\text{ ms}$ at $50\text{ RPM}$), and on the 16th Hall edge after startup (`BufferFilled == 16`), `hall_speed_pos_fdbk.c` abruptly switches from single-edge period to the 16-edge sum (which still holds slow startup edges $1\dots 15$).
   - Because free-air inertia $J_{\text{air}}$ is tiny, doubling $K_p$ (`36.3 cts/unit`) pushes the free-air loop crossover $\omega_c = \frac{K_p K_t}{J_{\text{air}}}$ into the $200\text{ ms}$ FIFO phase lag, causing limit-cycle hunting ($-36$ to $+90\text{ RPM}$ swings and $1.1\text{ s}$ zero-RPM dropouts when reverse torque spikes reset the Hall direction FIFO).
2. **Why a low-$K_i$ tune that spins in the air stalls and buzzes on the ground:**
   - With baseline `KP = -18600/1024` and `KI = -72/16384`, the PI integral time constant is:
     $$T_i = \frac{k_p}{1000\,k_i} = \frac{18.164}{4.3945} = 4.133\text{ seconds}$$
   - Because $T_i = 4.13\text{ s}$ is **more than twice the $2.0\text{ s}$ stall guard (`ESC_TEST_HALL_STALL_MS = 2000`)**, the integrator adds less than $42\%$ of the P-term before the safety guard kills the motor!
   - At $\pm 51\text{ RPM}$ ($8\text{ SDK units}$), total requested current at $t = 2.0\text{ s}$ is only **$0.38\text{ A}$** ($0.267\text{ A}$ P-term $+ 0.113\text{ A}$ I-term). That is enough to spin the wheel in the air ($> 0.25\text{ A}$), so the stand test looks "fine"—but on the ground ($1.0\text{--}2.5\text{ A}$ stiction), the rotor stays locked inside one Hall sector, buzzing at $16\text{ kHz}$ for $2.0\text{ s}$ until `stop_reason = 9` trips.
3. **Why ground inertia damps high-frequency jitter—but requires anti-windup care:**
   - On the ground, the $10\times\text{--}50\times$ larger inertia $J_{\text{load}}$ acts as a mechanical low-pass filter, reducing high-frequency Hall quantization ripple by $20\text{--}34\text{ dB}$.
   - However, because static breakaway current on the ground ($1.5\text{--}2.5\text{ A}$) is higher than steady rolling current ($0.6\text{--}1.0\text{ A}$), the integrator winds up during breakaway. Once stiction breaks, excess integral charge accelerates the robot until $\omega_{\text{meas}} > \omega_{\text{target}}$. Therefore, $K_i$ must be fast enough to beat the $2.0\text{ s}$ stall guard ($T_i \approx 250\text{--}400\text{ ms}$), while keeping $K_p$ at its free-air-stable baseline so post-breakaway recovery is well damped ($\zeta = \frac{K_p}{2}\sqrt{\frac{K_t}{J_{\text{eff}} K_i}}$).

---

## 4. What Speeds to Use for Tuning (Creep vs. Tournament Speeds)

Never tune a Hall-sensored FOC loop exclusively at creep speed ($12\text{--}30\text{ RPM}$):

| Speed Regime | Rover Speed | Motor RPM | SDK Units (`0.1 Hz`) | Hall Edge Rate & Period | 16-Edge FIFO Delay | Baseline P-Term at $t=0.5\text{ s}$ | Suitability for Tuning |
|---|---:|---:|---:|---|---|---:|---|
| **Sub-deadzone** | $< 0.09\text{ m/s}$ | $< 25.1\text{ RPM}$ | $0\text{--}4$ | $< 20\text{ Hz}$ ($> 49.8\text{ ms}$) | $> 800\text{ ms}$ | $\le 0.13\text{ A}$ | **Unusable:** `TIM4->PSC` stays at `388`; `hAvrMecSpeedUnit` locked at `0` |
| **Creep / Low** | $0.11\text{--}0.20\text{ m/s}$ | $29\text{--}51\text{ RPM}$ | $4\text{--}8$ | $23\text{--}41\text{ Hz}$ ($24\text{--}43\text{ ms}$) | $390\text{--}690\text{ ms}$ | $0.13\text{--}0.27\text{ A}$ | **Breakaway/stiction stress test only** (125 ms zero-output ramp deadtime at 29 RPM) |
| **Mid / Transition** | $0.4\text{--}0.6\text{ m/s}$ | $100\text{--}150\text{ RPM}$ | $16\text{--}25$ | $80\text{--}120\text{ Hz}$ ($8.3\text{--}12.5\text{ ms}$) | $133\text{--}200\text{ ms}$ | $0.53\text{--}0.83\text{ A}$ | Good for initial step/ramp tracking and moderate load checks |
| **Tournament Cruise** | **$1.0\text{--}2.0\text{ m/s}$** | **$240\text{--}500\text{ RPM}$** | **$40\text{--}83$** | **$192\text{--}400\text{ Hz}$ ($2.5\text{--}5.2\text{ ms}$)** | **$40\text{--}83\text{ ms}$** | **$1.34\text{--}2.77\text{ A}$** | **Primary speed-loop tuning regime:** smooth Hall feedback, immediate $1.3\text{--}2.8\text{ A}$ startup torque |
| **Tournament Sprint** | $2.5\text{--}3.0\text{ m/s}$ | $600\text{--}758\text{ RPM}$ | $100\text{--}126$ | $480\text{--}606\text{ Hz}$ ($1.65\text{--}2.1\text{ ms}$) | $26\text{--}33\text{ ms}$ | $3.00\text{ A}$ (clamped) | High-speed voltage-margin / back-EMF / current-limit check (within `RPM2` $\pm 900\text{ RPM}$) |

> **Note on current host vs. ESC speed caps:**
> Current installed `ESC_REVERSE_SPEED_DIAGNOSTIC` (`test_profile.h`) and all-four IK limits allow **`±900 RPM`**. Nucleo vx/vy input bounds are +/-5000 mm/s per axis, and TUI `--speed` accepts 0.05–5 m/s (default still 0.2). The former 300/150 RPM IK and 0.3 m/s host caps are historical. Uniform saturation still applies: 5 m/s forward requests scale to about 3.56 m/s under provisional geometry, not proven ground speed.
> Also note that `RPM2` has `ESC_TEST_TARGET_NET_EDGES = 480` ($10\text{ revolutions}$): at $250\text{ RPM}$ ($1\text{ m/s}$), a wheel completes $10$ revolutions in **$2.4\text{ seconds}$** and stops cleanly with `stop_reason = 8` (`TEST_STOP_COUNT`). That is normal guard behavior, not a fault!

---

## 5. Proper Inside-Out Staged Tuning Procedure

### Current next experiment: Ki4, not automatic adoption of the example table

Operator approved offline preparation of `--rpm2-ki4` / `ESC_RPM2_KI4`:
Ki -288/16384 (4x), original Kp -18600/1024, all current loops/limits/guards
unchanged. Eight offline builds establish every stock app matches its archived
installation, and each all-four candidate changes only two Ki initializer bytes.
Subsequently installed/verified on FR only with separate approval; **not yet
motor-validated**. See the current installation notice above and
`docs/RPM2_KI4_REVIEW.md` for provenance.

At the actual 1 m/s test's 39–42 SDK units, the stationary linear-ramp model
predicts 2.25–2.42 A demand after 1 s for Ki4; the example 8x/12x steps would
already saturate the 3 A command ceiling within that second. Hence choose the
smaller hypothesis-testing step first, not a blind gain jump based on unmeasured
breakaway current or the illustrative Ti window. This does not prove sufficient
ground torque, current tracking, power margin, or stability. Review post-breakaway
integral persistence/overshoot on the stand. Hardware/flash/motion require fresh
handoffs, exact new monitor identity, backup/readback and separate approvals.

### Stage 0: Pre-Flight Offline Guard & Unit Tests (Battery State Unchanged, No Probe Access)
Before compiling or flashing any candidate tune, run the host-side C/Python verification suites:
```sh
python3 tests/test_fr_speed_tune.py && \
python3 HW/B-G431B-ESC1/motor_config/test_reverse_speed_guards.py && \
python3 scripts/monitor_esc_power.py --self-test
```

### Stage 1: Inner Current Loop ($I_q / I_d$ at 16 kHz) & Sensor Verification (On Stand, In the Air)
**When to run:** Once per motor type or when diagnosing a suspect wheel/power stage.
1. **Analytical / Workbench Baseline:**
   - Because $R_s = 0.22\ \Omega$ and $L_s = 110\ \mu\text{H}$ depend only on the stator windings (not mechanical load), the inner loop gains (`PID_TORQUE_KP_DEFAULT = 3391`, `TF_KPDIV = 512`, `PID_TORQUE_KI_DEFAULT = 3343`, `TF_KIDIV = 4096`, $\omega_{bw} = 6000\text{ rad/s}$) **do not need retuning between free air and ground load**.
2. **In-Air Verification via Synchronized Read-Only Monitor (`monitor_esc_power.py`):**
   - With wheels raised on the stand and battery ON, run `monitor_esc_power.py` on the target ESC probe (`connect_mode="attach"`) while commanding motion from the Nucleo.
   - **Pass Criteria:**
     - `calibration_result == 2`, `calibration_valid == 1`, `current_faults == 0`.
     - Median current tracking error $\|I_{q,\text{ref}} - I_q\| \le 0.05\text{ A}$ during `mci_state == 6` (`RUN`).
     - Direct-axis current $\|I_d\| \le 0.10\text{ A}$ median is a screening
       observation only; it does not independently prove Hall/flux alignment.
     - `test_drive_hall_invalid == 0` (no `stop_reason = 7`).
   - *(Optional pure Torque Mode check if inner loop is suspect: use `--torque-count-diagnostic` (`CNT1`, $0.8\text{ A}$) or `--half-amp-diagnostic` (`HALF`, $1.5\text{ A}$), which sets `MCM_TORQUE_MODE` and bypasses the Speed PI completely).*

### Stage 2: Outer Speed Loop $K_p$ Stability Ceiling & Mid-Speed Tracking (On Stand, In the Air)
**Objective:** Use free air (worst-case low inertia $J_{\text{air}}$) to set $K_p$ below the high-frequency hunting threshold, and verify smooth tracking at normal operating speeds ($100\text{--}300\text{ RPM}$, $0.4\text{--}1.2\text{ m/s}$).
1. **Select $K_p$ (`ESC_TEST_SPEED_KP`, `SP_KPDIV`):**
   - Keep effective proportional gain in the free-air-stable window:
     $$18.0 \le k_p = \frac{|\text{hKpGain}|}{\text{SP\_KPDIV}} \le 24.0\text{ counts/unit}\quad (0.033\text{--}0.044\text{ A per }6\text{ RPM unit})$$
   - The baseline setting (`ESC_TEST_SPEED_KP = -18600`, `SP_KPDIV = 1024`, $k_p = 18.164\text{ cts/unit}$) is proven stable in free air. Do **not** use `SP_KPDIV = 512` ($k_p = 36.33\text{ cts/unit}$), which exceeds the 16-edge Hall FIFO phase margin in free air.
2. **Test at Mid / Tournament Speeds ($150\text{--}250\text{ RPM}$, $\sim 0.6\text{--}1.0\text{ m/s}$) on the Stand:**
   - Verify clean acceleration through the $500\text{ ms}$ ramp, steady RPM within $\pm 1\text{--}2\text{ SDK units}$ ($\pm 6\text{--}12\text{ RPM}$) of the quantized target `6 * int(rpm / 6)`, zero sign reversals during `RUN`, and clean termination at the 480-edge count stop (`stop_reason = 8`) or operator `STOP` (`stop_reason = 2`).

### Stage 3: Outer Speed Loop $K_i$ Design for Breakaway & Loaded Torque (Bench Drag $\rightarrow$ Ground)
**Objective:** Choose $K_i$ (`ESC_TEST_SPEED_KI`, `SP_KIDIV`) so that when a wheel is held back by ground stiction ($I_{\text{break}} \approx 1.5\text{--}2.0\text{ A}$), the Speed PI ramps $I_{q,\text{ref}}$ past $I_{\text{break}}$ **well before the $2000\text{ ms}$ stall guard (`ESC_TEST_HALL_STALL_MS`) trips**, without causing excessive post-breakaway windup.

1. **Quantitative $K_i$ Design Formula:**
   For a stalled wheel ($\omega_{\text{meas}} = 0$) with a $T_{\text{ramp}} = 500\text{ ms}$ linear speed ramp to target $U_{\text{ref}} = \lfloor |\text{RPM}_{\min}| / 6 \rfloor$ (in `0.1 Hz` units), the current command in SDK counts ($544\text{ cts/A}$) at time $t_{\text{break}} \in [500\text{ ms}, 2000\text{ ms})$ is:
   $$I_q(t_{\text{break}}) = k_p U_{\text{ref}} + k_i U_{\text{ref}}\left(t_{\text{break}} - 250\right) \ge 544 \cdot I_{\text{break}}$$
   Solving for $k_i = \frac{|\text{hKiGain}|}{\text{SP\_KIDIV}}$:
   $$k_i = \frac{|\text{hKiGain}|}{\text{SP\_KIDIV}} \ge \frac{544\,I_{\text{break}} - k_p U_{\text{ref}}}{U_{\text{ref}}\left(t_{\text{break}} - 250\right)}$$
   while keeping the PI integral time constant $T_i = \frac{k_p}{1000\,k_i} \in [200\text{ ms}, 450\text{ ms}]$.

2. **Reference Gain Table (with Baseline $K_p = -18600 / 1024 = 18.164\text{ cts/unit}$):**

| Candidate Tier | `ESC_TEST_SPEED_KI` / `SP_KIDIV` | $k_i$ (cts/unit$\cdot$ms) | $T_i$ (ms) | Stalled $\pm 51\text{ RPM}$ ($8\text{ u}$): $I_q$ at $1.0\text{ s}$ / $1.5\text{ s}$ / $2.0\text{ s}$ | Stalled $\pm 100\text{ RPM}$ ($16\text{ u}$): $I_q$ at $1.0\text{ s}$ / $1.5\text{ s}$ | Recommended Use |
|---|---:|---:|---:|---|---|---|
| **Baseline (`RPM2`)** | `-72 / 16384` | $0.00439$ | $4133$ | $0.32\text{ A}$ / $0.35\text{ A}$ / $0.38\text{ A}$ | $0.63\text{ A}$ / $0.70\text{ A}$ | Free-air $\ge 60\text{ RPM}$ only; stalls on ground at low speed |
| **Mild ($8\times K_i$)** | `-576 / 16384` | $0.03516$ | $517$ | $0.65\text{ A}$ / $0.91\text{ A}$ / $1.17\text{ A}$ | $1.31\text{ A}$ / $1.83\text{ A}$ | Conservative first step if testing $\ge 100\text{ RPM}$ |
| **Balanced ($12\times K_i$)** | **`-864 / 16384`** | **$0.05273$** | **$344$** | **$0.85\text{ A}$ / $1.23\text{ A}$ / $1.62\text{ A}$** | **$1.70\text{ A}$ / $2.47\text{ A}$** | **Recommended starting point for loaded low/mid speed** |
| **Strong ($16\times K_i$)** | **`-1152 / 16384`** | **$0.07031$** | **$258$** | **$1.04\text{ A}$ / $1.56\text{ A}$ / $2.07\text{ A}$** | **$2.08\text{ A}$ / $3.00\text{ A}$** | **Strong low-speed ground breakaway ($>2\text{ A}$ before $2\text{ s}$)** |

3. **Stand + Controlled Load Verification Before Ground Placement:**
   - First verify the candidate (`-864` or `-1152`) in free air on the stand at both $50\text{ RPM}$ and $250\text{ RPM}$ to confirm zero hunting.
   - Apply light/moderate mechanical drag on the stand (or test at $100\text{--}250\text{ RPM}$) and confirm via `monitor_esc_power.py` that $I_{q,\text{ref}}$ and $I_q$ ramp smoothly up to $1.0\text{--}2.5\text{ A}$ under load without tripping `stop_reason = 9` (stall) or `stop_reason = 4` ($3.0\text{ A}$ sampled L1 current stop).

### Stage 4: All-Four-Wheel Rollout & On-Ground Validation
1. **Rollout Discipline (`.opencode/skills/wheel-motor-setup/SKILL.md` Rule 8):**
   - Never test ground driving with mixed gains across wheels (e.g., FR on candidate, BR/BL/FL on baseline). On omni wheels, a single slower or stalled wheel acts as an angled brake and drags the chassis off-axis.
   - Back up flash, deploy the validated tune to all four ESCs (`NE=1, SE=2, SW=4, NW=8`) over CAN-FWU with battery OFF, verify full readback + metadata + `VTOR = 0x08004000`, and restore the Nucleo sender/IK image.
   - Note: For sustained ground driving beyond $10$ wheel revolutions ($\sim 2.06\text{ m}$ of wheel travel), `ESC_TEST_TARGET_NET_EDGES` (`480` edges) must be raised or disabled across all 4 wheels with explicit operator approval.
2. **On-Ground Pre-Start Check:**
   - Battery ON $\rightarrow$ run the `FaultAck` + Auto-CAL preparation sequence (`P` twice in `keyboard_drive.py`, waiting for all 4 wheels to report `cal=1` and `IDLE` on CAN telemetry).
   - Verify $V_{\text{bus}} \ge 11.0\text{ V}$ at rest and adequate voltage during
     simultaneous breakaway; do not approach the $8.0\text{ V}$ undervoltage
     cutoff. Four 3 A dq limits do not establish a 12 A battery-current limit
     or physical phase-current peak guarantee.
3. **On-Ground Motion Sequence:**
   - Start with **Forward (`W`) and Backward (`S`) at $0.5\text{--}1.0\text{ m/s}$** ($\sim 120\text{--}250\text{ RPM}$), where startup proportional current is immediately $> 0.7\text{--}1.4\text{ A}$ per wheel.
   - Next test **Yaw (`Q`/`E`)** and **Lateral Strafe (`A`/`D`)** (lateral strafe has the highest omni-roller scrubbing load because rollers slide at $55^\circ\text{--}60^\circ$).
   - Inspect post-run JSONL logs for all 4 wheels: confirm synchronized first nonzero RPM, zero `stop_reason = 9` (stall) or `stop_reason = 4` (overcurrent) events, and balanced steady-state $|I_q|$ across left/right pairs.
