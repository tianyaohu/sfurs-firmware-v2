"""Current-loop capture analysis (ESC CAPTURE, can_datatypes.h). Pure Python.

The ESC holds the rotor on the d axis at ESC_CAPTURE_HOLD_MA, then steps the
Id reference. From Id and the applied Vd at 16 kHz:
  R  = dVd / dId between the two steady states (cancels dead-time offset),
  L  = least-squares fit of Vd - R*Id - Voff = L dId/dt over the transient,
       with the PWM update delay (0..2 periods) chosen by best fit,
plus the step response of the closed loop and PI gains for a target
bandwidth by pole-zero cancellation (Kp = L*wc, Ki = R*wc).
"""
import math

COUNTS_PER_AMP = 544
PWM_HZ = 16000
TF_KPDIV, TF_KIDIV = 512, 4096
STEP_WINDOW = 64  # samples (4 ms) after the step judged as step response
SQRT3 = math.sqrt(3.0)


def volts_per_s16(vbus_v):
    """MCSDK s16V: 32767 = Vbus/sqrt(3) phase-voltage amplitude."""
    return vbus_v / SQRT3 / 32767.0


def _mean(values):
    values = list(values)
    return sum(values) / len(values)


def analyze(info, samples, pre=64, pwm_hz=PWM_HZ):
    vbus = info["vbus_centivolts"] / 100.0
    vs = volts_per_s16(vbus)
    n = len(samples)
    i = [s["id"] / COUNTS_PER_AMP for s in samples]
    v = [s["vd"] * vs for s in samples]
    ref = [s["id_ref"] / COUNTS_PER_AMP for s in samples]
    i0, v0 = _mean(i[8:pre]), _mean(v[8:pre])
    tail = range(n - n // 4, n)
    i1, v1 = _mean(i[k] for k in tail), _mean(v[k] for k in tail)
    if abs(i1 - i0) < 0.2:
        raise ValueError(f"step too small to fit ({i0:.2f} -> {i1:.2f} A)")
    r = (v1 - v0) / (i1 - i0)
    v_off = v0 - r * i0
    t = 1.0 / pwm_hz
    noise = math.sqrt(_mean((x - i0) ** 2 for x in i[8:pre]))
    # Isolated current-sense spikes (paired +-0.5 A, about every 20 ms on
    # this hardware) would dominate dI/dt; drop samples far from the local
    # median. The fit window covers the transient, where the signal is large.
    def spike(k):
        window = sorted(i[max(0, k - 2):k + 3])
        return abs(i[k] - window[len(window) // 2]) > max(4 * noise, 0.1)
    best = None
    for delay in (0, 1, 2):
        xs, ys = [], []
        for k in range(pre, min(pre + 32, n - 1)):
            if k - delay < 0 or spike(k) or spike(k + 1):
                continue
            xs.append((i[k + 1] - i[k]) / t)
            ys.append(v[k - delay] - r * (i[k] + i[k + 1]) / 2 - v_off)
        sxx = sum(x * x for x in xs)
        if sxx == 0:
            continue
        ell = sum(x * y for x, y in zip(xs, ys)) / sxx
        resid = sum((y - ell * x) ** 2 for x, y in zip(xs, ys))
        if best is None or resid < best[2]:
            best = (ell, delay, resid)
    ell, delay = (best[0], best[1]) if best else (float("nan"), None)

    # Step metrics on a 4-sample moving average: single-sample current noise
    # (tens of mA, the odd spike) would otherwise read as overshoot.
    r0, r1 = ref[pre - 1], ref[pre]
    span = r1 - r0
    smooth = {k: _mean(i[max(pre, k - 3):k + 1]) for k in range(pre, n)}
    def first(cond):
        return next((k - pre for k in range(pre, n) if cond(i[k])), None)
    rise10 = first(lambda x: (x - r0) >= 0.1 * span)
    rise90 = first(lambda x: (x - r0) >= 0.9 * span)
    # Overshoot and settling inside STEP_WINDOW after the step; later
    # excursions are disturbances (counted separately), not step response.
    end = min(n, pre + STEP_WINDOW)
    peak = max(smooth[k] for k in range(pre + 4, end))
    band = max(0.05 * abs(span), 3 * noise)
    outside = [k for k in range(pre + 4, end)
               if abs(smooth[k] - r1) > band and not any(spike(j) for j in range(k - 3, k + 1))]
    settle = (outside[-1] + 1 - pre) if outside else 4
    disturbances = sum(1 for k in range(end, n) if abs(i[k] - r1) > 5 * max(noise, 0.01))
    us = 1e6 / pwm_hz
    return dict(
        vbus_v=vbus, hold_a=round(r0, 3), step_a=round(r1, 3),
        r_ohm=r, l_h=ell, tau_ms=ell / r * 1e3 if r > 0 else float("nan"), delay_periods=delay,
        v_offset=v_off,
        rise_us=(rise90 - rise10) * us if rise10 is not None and rise90 is not None else None,
        overshoot_pct=100.0 * (peak - r1) / span if span else 0.0,
        settle_us=settle * us if settle < end - pre else None,
        disturbance_samples=disturbances,
        steady_error_a=i1 - r1, noise_a=noise, settle_band_a=band,
    )


def pi_gains(r_ohm, l_h, vbus_v, fc_hz=1000.0, pwm_hz=PWM_HZ):
    """Digital MCSDK current PI (dividers TF_KPDIV/TF_KIDIV) for bandwidth fc."""
    wc = 2 * math.pi * fc_hz
    per_count = volts_per_s16(vbus_v) * COUNTS_PER_AMP  # V per (s16V/count)
    kp = l_h * wc * TF_KPDIV / per_count
    ki = r_ohm * wc * TF_KIDIV / (per_count * pwm_hz)
    return round(kp), round(ki)


def loop_bandwidth_hz(kp_digital, l_h, vbus_v):
    """Closed-loop bandwidth of the current PI with pole-zero cancellation."""
    kp = kp_digital / TF_KPDIV * volts_per_s16(vbus_v) * COUNTS_PER_AMP
    return kp / l_h / (2 * math.pi)
