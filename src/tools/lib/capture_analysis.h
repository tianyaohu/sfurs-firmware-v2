/** @file capture_analysis.h
 * @brief Current-loop identification from ESC capture samples.
 */
#ifndef CAPTURE_ANALYSIS_H
#define CAPTURE_ANALYSIS_H

#include <stddef.h>
#include <stdint.h>

#define CAPTURE_COUNTS_PER_AMP 544
#define CAPTURE_PWM_HZ 16000
#define CAPTURE_TF_KPDIV 512
#define CAPTURE_TF_KIDIV 4096
#define CAPTURE_PRE_SAMPLES 64
#define CAPTURE_STEP_WINDOW 64

typedef struct {
  int32_t id, vd, id_ref; /**< ESC current counts and s16V voltage counts. */
} capture_sample;

typedef struct {
  double vbus_v, hold_a, step_a;
  double r_ohm, l_h, tau_ms;
  int delay_periods; /**< -1 when the inductance fit is unavailable. */
  double v_offset; /**< V. */
  double rise_us, overshoot_pct, settle_us; /**< NaN for unavailable times. */
  size_t disturbance_samples;
  double steady_error_a, noise_a, settle_band_a;
} capture_result;

enum { CAPTURE_OK = 0, CAPTURE_INVALID = -1, CAPTURE_STEP_TOO_SMALL = -2 };

/** @brief Phase voltage in V per s16V count for a DC bus voltage in V. */
double capture_volts_per_s16(double vbus_v);

/** @brief Fit R [ohm], L [H], PWM delay [periods] and step metrics [A, us, %].
 * @param vbus_centivolts DC bus voltage [0.01 V].
 * @param samples Array of n capture samples [ESC counts].
 * @param n Number of samples; at least pre + 5.
 * @param pre Step index [samples], greater than 8 (normally 64).
 * @param pwm_hz Sampling frequency [Hz], positive (normally 16000).
 * @param out Result; unavailable fit/times are NaN, delay is -1.
 * @return CAPTURE_OK, CAPTURE_STEP_TOO_SMALL for a step below 0.2 A,
 * or CAPTURE_INVALID for invalid arguments. No allocation is needed.
 */
int capture_analyze(double vbus_centivolts, const capture_sample *samples,
                    size_t n, size_t pre, double pwm_hz, capture_result *out);

/** @brief MCSDK digital PI gains [counts] for R [ohm], L [H], bus [V],
 * bandwidth [Hz], and PWM frequency [Hz]; round ties to even like Python.
 */
void capture_pi_gains(double r_ohm, double l_h, double vbus_v,
                      double fc_hz, double pwm_hz, int64_t *kp, int64_t *ki);

/** @brief Closed-loop bandwidth [Hz] from digital Kp [counts], L [H], bus [V]. */
double capture_loop_bandwidth_hz(double kp_digital, double l_h, double vbus_v);

#endif
