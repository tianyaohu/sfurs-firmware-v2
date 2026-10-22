#include "friction_fit.h"
#include <math.h>

friction_point_t friction_segment(const friction_sample_t *s, size_t n)
{
  friction_point_t p = {.samples = n, .verdict = FRICTION_TOO_FEW};
  if (!s || !n) return p;
  double variance = 0;
  for (size_t i = 0; i < n; i++) {
    if (!isfinite(s[i].rpm) || !isfinite(s[i].iq_ma)) { p.verdict = FRICTION_BAD_SAMPLE; return p; }
    double delta = s[i].rpm - p.rpm;
    p.rpm += delta / (i + 1);
    variance += delta * (s[i].rpm - p.rpm);
    p.iq_ma += (s[i].iq_ma - p.iq_ma) / (i + 1);
  }
  p.speed_sd = sqrt(fmax(0, variance / n));
  if (n < 5) return p;
  if (fabs(p.rpm) < 1) { p.verdict = FRICTION_NO_MOTION; return p; }
  p.verdict = p.speed_sd > 0.1 * fabs(p.rpm) ? FRICTION_UNSTEADY : FRICTION_STEADY;
  for (size_t i = 0; i < n; i++) if (s[i].rpm * p.rpm <= 0) p.verdict = FRICTION_UNSTEADY;
  return p;
}

const char *friction_verdict_name(friction_verdict_t v)
{
  static const char *const names[] = {"steady", "fewer_than_5_samples", "speed_not_steady", "no_motion", "nonfinite_sample"};
  return (unsigned)v < sizeof names / sizeof *names ? names[v] : "unknown";
}

static bool usable(const friction_point_t *p, int direction)
{
  return p->verdict == FRICTION_STEADY && p->samples >= 5 && isfinite(p->rpm) && isfinite(p->iq_ma) &&
         fabs(p->rpm) >= 1 && (direction == 0 || (p->rpm > 0 ? 1 : -1) == direction);
}

friction_fit_t friction_fit(const friction_point_t *p, size_t n, int direction)
{
  friction_fit_t f = {0};
  if (!p || direction < -1 || direction > 1) return f;
  /* Multiplying each equation by sign(v) gives Iq*sign(v) = Ic + b*|v|.
   * Centering avoids cancellation in the two-column normal equations. */
  double xmean = 0, ymean = 0, xx = 0, xy = 0;
  for (size_t i = 0; i < n; i++) if (usable(&p[i], direction)) {
    double x = fabs(p[i].rpm), y = p[i].iq_ma * (p[i].rpm > 0 ? 1 : -1);
    f.points++;
    double dx = x - xmean;
    xmean += dx / f.points;
    ymean += (y - ymean) / f.points;
    xx += dx * (x - xmean);
    xy += dx * (y - ymean);
  }
  if (f.points < 2 || xx <= 1e-12 * f.points * fmax(1, xmean * xmean)) return f;
  f.b_ma_per_rpm = xy / xx;
  f.ic_ma = ymean - f.b_ma_per_rpm * xmean;
  double sum = 0;
  for (size_t i = 0; i < n; i++) if (usable(&p[i], direction)) {
    double residual = p[i].iq_ma - (f.ic_ma * (p[i].rpm > 0 ? 1 : -1) + f.b_ma_per_rpm * p[i].rpm);
    sum += residual * residual;
  }
  f.residual_rms_ma = sqrt(sum / f.points);
  f.valid = isfinite(f.ic_ma) && isfinite(f.b_ma_per_rpm) && isfinite(f.residual_rms_ma);
  return f;
}

bool friction_motion(friction_motion_t *s, unsigned count, double rpm, int sign, double threshold)
{
  /* The host polls slower than the ESC replies, so several replies may land between
   * reads; any advance is one fresh observation, and only the newest reply is seen. */
  if (count == s->count) return false;
  s->count = count;
  if ((sign == 1 || sign == -1) && isfinite(rpm) && isfinite(threshold) && threshold > 0 && rpm * sign >= threshold)
    ++s->streak;
  else s->streak = 0;
  return s->streak >= 3;
}
