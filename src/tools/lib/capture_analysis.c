#include "capture_analysis.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double current(const capture_sample *s, size_t k) {
  return s[k].id / (double)CAPTURE_COUNTS_PER_AMP;
}

static double rounded_milliamps(double a) {
  char text[64];
  /* Decimal conversion preserves Python's rounding of binary halfway values. */
  snprintf(text, sizeof(text), "%.3f", a);
  return strtod(text, NULL);
}

static int spike(const capture_sample *s, size_t n, size_t k, double noise) {
  double window[5];
  size_t start = k > 2 ? k - 2 : 0;
  size_t end = n - k > 3 ? k + 3 : n;
  size_t count = 0;
  for (size_t j = start; j < end; ++j) {
    double x = current(s, j);
    size_t at = count++;
    while (at && window[at - 1] > x) {
      window[at] = window[at - 1];
      --at;
    }
    window[at] = x;
  }
  return fabs(current(s, k) - window[count / 2]) > fmax(4 * noise, 0.1);
}

double capture_volts_per_s16(double vbus_v) {
  return vbus_v / sqrt(3.0) / 32767.0;
}

int capture_analyze(double vbus_centivolts, const capture_sample *s,
                    size_t n, size_t pre, double pwm_hz, capture_result *out) {
  if (!s || !out || pre <= 8 || n <= pre || n - pre < 5 ||
      !isfinite(pwm_hz) || pwm_hz <= 0 || !isfinite(vbus_centivolts))
    return CAPTURE_INVALID;
  double vbus = vbus_centivolts / 100.0;
  double vs = capture_volts_per_s16(vbus);
  double i0 = 0, v0 = 0, i1 = 0, v1 = 0;
  for (size_t k = 8; k < pre; ++k) {
    i0 += current(s, k);
    v0 += s[k].vd * vs;
  }
  i0 /= pre - 8;
  v0 /= pre - 8;
  for (size_t k = n - n / 4; k < n; ++k) {
    i1 += current(s, k);
    v1 += s[k].vd * vs;
  }
  i1 /= n / 4;
  v1 /= n / 4;
  if (fabs(i1 - i0) < 0.2)
    return CAPTURE_STEP_TOO_SMALL;
  double r = (v1 - v0) / (i1 - i0);
  double v_off = v0 - r * i0;
  double t = 1.0 / pwm_hz;
  double noise = 0;
  for (size_t k = 8; k < pre; ++k) {
    double d = current(s, k) - i0;
    noise += d * d;
  }
  noise = sqrt(noise / (pre - 8));
  double ell = NAN, best_resid = 0;
  int best_delay = -1;
  size_t fit_count = n - 1 - pre < 32 ? n - 1 - pre : 32;
  for (int delay = 0; delay <= 2; ++delay) {
    double xs[32], ys[32], sxx = 0, sxy = 0;
    size_t used = 0;
    for (size_t k = pre; k < pre + fit_count; ++k) {
      if (k < (size_t)delay || spike(s, n, k, noise) || spike(s, n, k + 1, noise))
        continue;
      double x = (current(s, k + 1) - current(s, k)) / t;
      double y = s[k - delay].vd * vs - r * (current(s, k) + current(s, k + 1)) / 2 - v_off;
      xs[used] = x;
      ys[used++] = y;
      sxx += x * x;
      sxy += x * y;
    }
    if (sxx == 0)
      continue;
    double l = sxy / sxx, resid = 0;
    for (size_t j = 0; j < used; ++j) {
      double d = ys[j] - l * xs[j];
      resid += d * d;
    }
    if (best_delay < 0 || resid < best_resid) {
      ell = l;
      best_delay = delay;
      best_resid = resid;
    }
  }
  double r0 = s[pre - 1].id_ref / (double)CAPTURE_COUNTS_PER_AMP;
  double r1 = s[pre].id_ref / (double)CAPTURE_COUNTS_PER_AMP;
  double span = r1 - r0;
  size_t rise10 = n, rise90 = n;
  for (size_t k = pre; k < n; ++k) {
    if (rise10 == n && current(s, k) - r0 >= 0.1 * span)
      rise10 = k - pre;
    if (rise90 == n && current(s, k) - r0 >= 0.9 * span)
      rise90 = k - pre;
  }
  size_t end = n - pre < CAPTURE_STEP_WINDOW ? n : pre + CAPTURE_STEP_WINDOW;
  double peak = -INFINITY;
  double band = fmax(0.05 * fabs(span), 3 * noise);
  size_t settle = 4, disturbances = 0;
  for (size_t k = pre + 4; k < end; ++k) {
    double smooth = 0;
    int has_spike = 0;
    for (size_t j = k - 3; j <= k; ++j) {
      smooth += current(s, j);
      has_spike |= spike(s, n, j, noise);
    }
    smooth /= 4;
    peak = fmax(peak, smooth);
    if (fabs(smooth - r1) > band && !has_spike)
      settle = k + 1 - pre;
  }
  for (size_t k = end; k < n; ++k)
    if (fabs(current(s, k) - r1) > 5 * fmax(noise, 0.01))
      ++disturbances;
  double us = 1e6 / pwm_hz;
  *out = (capture_result){
    .vbus_v = vbus, .hold_a = rounded_milliamps(r0), .step_a = rounded_milliamps(r1),
    .r_ohm = r, .l_h = ell, .tau_ms = r > 0 ? ell / r * 1e3 : NAN,
    .delay_periods = best_delay, .v_offset = v_off,
    .rise_us = rise10 != n && rise90 != n ? ((double)rise90 - (double)rise10) * us : NAN,
    .overshoot_pct = span ? 100.0 * (peak - r1) / span : 0.0,
    .settle_us = settle < end - pre ? settle * us : NAN,
    .disturbance_samples = disturbances, .steady_error_a = i1 - r1,
    .noise_a = noise, .settle_band_a = band
  };
  return CAPTURE_OK;
}

static int64_t round_even(double x) {
  double lo = floor(x), part = x - lo;
  return (int64_t)(part > 0.5 || (part == 0.5 && fmod(lo, 2) != 0) ? lo + 1 : lo);
}

void capture_pi_gains(double r_ohm, double l_h, double vbus_v,
                      double fc_hz, double pwm_hz, int64_t *kp, int64_t *ki) {
  double wc = 2 * acos(-1.0) * fc_hz;
  double per_count = capture_volts_per_s16(vbus_v) * CAPTURE_COUNTS_PER_AMP;
  *kp = round_even(l_h * wc * CAPTURE_TF_KPDIV / per_count);
  *ki = round_even(r_ohm * wc * CAPTURE_TF_KIDIV / (per_count * pwm_hz));
}

double capture_loop_bandwidth_hz(double kp_digital, double l_h, double vbus_v) {
  double kp = kp_digital / CAPTURE_TF_KPDIV * capture_volts_per_s16(vbus_v) * CAPTURE_COUNTS_PER_AMP;
  return kp / l_h / (2 * acos(-1.0));
}
