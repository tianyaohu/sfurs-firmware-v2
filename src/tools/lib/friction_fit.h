/** @file friction_fit.h
 * @brief Pure steady-segment statistics and Iq = Ic*sign(RPM) + b*RPM least squares.
 */
#ifndef FRICTION_FIT_H
#define FRICTION_FIT_H
#include <stdbool.h>
#include <stddef.h>

typedef struct { double rpm, iq_ma; } friction_sample_t;
typedef enum { FRICTION_STEADY, FRICTION_TOO_FEW, FRICTION_UNSTEADY, FRICTION_NO_MOTION,
               FRICTION_BAD_SAMPLE } friction_verdict_t;
typedef struct {
  double rpm, iq_ma, speed_sd;
  size_t samples;
  friction_verdict_t verdict;
} friction_point_t;
typedef struct {
  bool valid;
  size_t points;
  double ic_ma, b_ma_per_rpm, residual_rms_ma;
} friction_fit_t;
/** @brief Accept >=5 finite samples with nonzero mean, consistent sign and population speed SD <=10% of |mean|. */
friction_point_t friction_segment(const friction_sample_t *samples, size_t n);
const char *friction_verdict_name(friction_verdict_t verdict);
/** @brief Equal-weight segment means; direction -1/+1 filters by sign, 0 fits both. Reject singular designs. */
friction_fit_t friction_fit(const friction_point_t *points, size_t n, int direction);
/** @brief Count three consecutive fresh, contiguous replies with signed RPM >= threshold; duplicates do not count.
 * Sequence gaps reset the streak. Caller initializes count to the baseline reply_count.
 */
typedef struct { unsigned count, streak; } friction_motion_t;
bool friction_motion(friction_motion_t *state, unsigned reply_count, double rpm, int sign, double threshold);
#endif
