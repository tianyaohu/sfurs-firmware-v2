/** @file friction_run.h
 * @brief Guarded normal-drive acquisition for friction_map; no motor/plant model.
 */
#ifndef FRICTION_RUN_H
#define FRICTION_RUN_H
#include "friction_fit.h"
#include "ik_link.h"
#include <signal.h>

typedef struct {
  double min_vbus_v, max_rpm, transient_s;
  const volatile sig_atomic_t *interrupted;
} friction_limits_t;
typedef struct {
  bool moved;
  double command_ma, measured_ma;
  bool attempted, capped; /**< A cap without motion is censored; an unattempted/aborted trial is distinct. */
} friction_breakaway_t;
/** @brief Publish zero current and enable=0, then require STOPPED and all four IDLE within 4 s. */
int friction_stop(ik_link_t *link);
/** @brief One SPEED move; collect fresh replies after transient and 0.2 s of unslewed reference plateau.
 * Always stops; outputs rejected points as well as accepted points. duration <=5 s.
 */
int friction_speed_move(ik_link_t *link, const int32_t twist[3], double duration_s,
                        const friction_limits_t *limits, friction_point_t out[IK_WHEELS]);
/** @brief One CURRENT ramp on selected wheels; zero each at its third consecutive motion reply, then stop all.
 * selected is a wheel-index bit mask. Missing motion at cap is recorded as moved=false.
 */
int friction_breakaway_run(ik_link_t *link, unsigned selected, int sign, double rate_ma_s,
                           double cap_ma, double motion_rpm, const friction_limits_t *limits,
                           friction_breakaway_t out[IK_WHEELS]);
#endif
