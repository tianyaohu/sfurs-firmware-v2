/** @file friction_run.h
 * @brief Guarded normal-drive acquisition for friction_map; no motor/plant model.
 */
#ifndef FRICTION_RUN_H
#define FRICTION_RUN_H
#include "friction_fit.h"
#include "ik_link.h"
#include <signal.h>

/** @brief Run guards: Vbus floor (V), speed ceiling (RPM), settling time (s), interrupt flag. */
typedef struct {
  double min_vbus_v, max_rpm, transient_s;
  const volatile sig_atomic_t *interrupted;
} friction_limits_t;
/** @brief Breakaway trial of one wheel: commanded and measured current (mA) at motion, at the stop or at the cap. */
typedef struct {
  bool moved;
  double command_ma, measured_ma;
  bool attempted, capped; /**< A cap without motion is censored; an unattempted/aborted trial is distinct. */
} friction_breakaway_t;
/** @brief Publish zero current and enable=0, then require STOPPED and all four IDLE within 4 s.
 *  @param link open mailbox link
 *  @return 0, or -1 with link->err set */
int friction_stop(ik_link_t *link);
/** @brief One SPEED move; collect fresh replies after transient and 0.2 s of unslewed reference plateau.
 * Always stops; outputs rejected points as well as accepted points. duration <=5 s.
 * @param link       open mailbox link
 * @param twist      vx, vy (mm/s), omega (mrad/s)
 * @param duration_s move length, s
 * @param limits     Vbus floor (V), speed ceiling (RPM), settling time (s) and interrupt flag
 * @param out        out: one plateau point per wheel (RPM, mA)
 * @return 0, or -1 with link->err set
 */
int friction_speed_move(ik_link_t *link, const int32_t twist[3], double duration_s,
                        const friction_limits_t *limits, friction_point_t out[IK_WHEELS]);
/** @brief One CURRENT ramp: wheel w gets weight[w] * min(cap, t * rate); stop all when the rule is met.
 * stop_wheels 0 zeroes each wheel at its own third consecutive motion reply (one wheel at a time);
 * stop_wheels n > 0 keeps the pattern until n wheels move, then zeroes all four (a body-force ramp, so a wheel
 * that frees first does not change the direction of the force while the others are still stuck). Motion
 * counts along sign(weight[w]). Missing motion at the cap is recorded as capped.
 * @param link        open mailbox link
 * @param weight      per-wheel signed share of the ramp, |weight| <= 1, largest |weight| = 1; 0 = not driven
 * @param stop_wheels 0 (per wheel) or the number of moving wheels that ends the ramp for all
 * @param rate_ma_s   ramp rate of the largest wheel, mA/s
 * @param cap_ma      largest |current|, mA
 * @param motion_rpm  speed that counts as breakaway, RPM
 * @param limits      Vbus floor (V), speed ceiling (RPM), settling time (s) and interrupt flag
 * @param out         out: per wheel, moved flag and the commanded and measured current (mA) when it moved, or
 *                    (stop_wheels > 0) when the ramp stopped
 * @return 0, or -1 with link->err set
 */
int friction_breakaway_run(ik_link_t *link, const double weight[IK_WHEELS], unsigned stop_wheels, double rate_ma_s,
                           double cap_ma, double motion_rpm, const friction_limits_t *limits,
                           friction_breakaway_t out[IK_WHEELS]);
#endif
