/** @file friction_fit.h
 * @brief Pure steady-segment statistics and Iq = Ic*sign(RPM) + b*RPM least squares.
 */
#ifndef FRICTION_FIT_H
#define FRICTION_FIT_H
#include <stdbool.h>
#include <stddef.h>

/** @brief One telemetry sample: speed (RPM) and q-current (mA). */
typedef struct { double rpm, iq_ma; } friction_sample_t;
/** @brief Whether a plateau segment is usable for the fit. */
typedef enum { FRICTION_STEADY, FRICTION_TOO_FEW, FRICTION_UNSTEADY, FRICTION_NO_MOTION,
               FRICTION_BAD_SAMPLE } friction_verdict_t;
/** @brief Plateau mean speed (RPM), mean current (mA), speed SD (RPM), sample count and verdict. */
typedef struct {
  double rpm, iq_ma, speed_sd;
  size_t samples;
  friction_verdict_t verdict;
} friction_point_t;
/** @brief Fit Iq = Ic*sign(RPM) + b*RPM: Ic (mA), b (mA per RPM), RMS residual (mA). */
typedef struct {
  bool valid;
  size_t points;
  double ic_ma, b_ma_per_rpm, residual_rms_ma;
} friction_fit_t;
/** @brief Accept >=5 finite samples with nonzero mean, consistent sign and population speed SD <=10% of |mean|.
 *  @param samples steady-plateau samples (RPM, mA)
 *  @param n       sample count
 *  @return segment mean speed (RPM), mean current (mA), speed SD (RPM) and verdict */
friction_point_t friction_segment(const friction_sample_t *samples, size_t n);
/** @brief Stable name of a verdict for JSON output ("unknown" when out of range).
 *  @param verdict segment verdict */
const char *friction_verdict_name(friction_verdict_t verdict);
/** @brief Equal-weight segment means; direction -1/+1 filters by sign, 0 fits both. Reject singular designs.
 *  @param points    segments; only FRICTION_STEADY ones are used
 *  @param n         segment count
 *  @param direction -1 or +1: only that speed sign; 0: both
 *  @return Coulomb current Ic (mA), viscous slope b (mA per RPM), RMS residual (mA); valid false if singular */
friction_fit_t friction_fit(const friction_point_t *points, size_t n, int direction);
/** @brief Motion detector state: last reply count seen and current streak. */
typedef struct { unsigned count, streak; } friction_motion_t;
/** @brief Confirm motion after three consecutive fresh reads with signed RPM >= threshold.
 * A read is fresh when reply_count advanced by one or more since the last read (the host
 * poll is slower than the reply rate, so gaps are normal and do not reset). A fresh read
 * below the threshold, in the wrong direction or non-finite resets the streak; a read with
 * no new reply changes nothing. Caller initializes count to the baseline reply_count.
 * @param state       detector state
 * @param reply_count the wheel's link reply counter
 * @param rpm         measured speed in that reply, RPM
 * @param sign        expected direction, -1 or +1
 * @param threshold   speed that counts as motion, RPM (> 0)
 * @return true once three such fresh reads arrived in a row
 */
bool friction_motion(friction_motion_t *state, unsigned reply_count, double rpm, int sign, double threshold);
#endif
