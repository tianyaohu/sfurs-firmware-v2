#ifndef MOTOR_SPEED_H
#define MOTOR_SPEED_H

#include "motor_protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file motor_speed.h
 *  @brief ESC speed loop (Stage 4a): reference interpolation, the Hall speed estimate, P + I + ff with
 *         stall ramp, over-speed bleed and stall timer.
 *
 *  Pure logic in fixed point: no MCSDK or HAL calls and no floats. The caller samples the Hall counters
 *  and MCSDK's average speed into ::motor_speed_inputs, calls ::motor_speed_tick every 1 ms and publishes
 *  ::motor_speed_outputs.iq_ref_counts (RUNNING/HOLD). Specification:
 *  technical-development-plans/esc-command-v2/E-speed-loop.md.
 *
 *  Units: current in counts (::MOTOR_SPEED_COUNTS_PER_A per A), speed in RPM, time in 1 ms ticks,
 *  integrator in Q14 counts. HOLD entry timing belongs to the command lifecycle; this module only
 *  reports ::motor_speed_outputs.standstill and behaves as HOLD when told. */

#define MOTOR_SPEED_COUNTS_PER_A 544      /**< q-current counts per A (CURRENT_CONV_FACTOR) */
#define MOTOR_SPEED_EDGE_RPM_MS 1250      /**< 48 Hall edges/rev: one edge per ms is 1250 RPM */
#define MOTOR_SPEED_KP_SHIFT 8            /**< Kp divisor 256 */
#define MOTOR_SPEED_KI_SHIFT 14           /**< Ki divisor 16384 (integrator Q14) */
#define MOTOR_SPEED_E_MAX_RPM 5952        /**< |speed error| clamp before the gains, RPM */
#define MOTOR_SPEED_FF_SLEW_MA_PER_MS 50  /**< ff slew rate, mA per tick (50 A/s) */
#define MOTOR_SPEED_MOVING_RPM 10         /**< |r| below this: no stall ramp, no stall timer, RPM */
#define MOTOR_SPEED_STANDSTILL_RPM 45     /**< |w_hat| below this is standstill (Hall rocking reads 25-32), RPM */
#define MOTOR_SPEED_SLIP_MAX_EDGES 4      /**< slip clamp and stall-ramp threshold, edges */
#define MOTOR_SPEED_BLEED_SPAN_MS 8       /**< shortest progress run the bleed judges, ms */
#define MOTOR_SPEED_BLEED_PCT 115         /**< bleed when run speed >= this % of |r| */
#define MOTOR_SPEED_VALID_EDGES 2         /**< same-direction edges before the estimate is valid */
#define MOTOR_SPEED_MCSDK_EDGES 6         /**< edges after which MCSDK's 6-edge average is used */
#define MOTOR_SPEED_STALE_MIN_MS 20       /**< stale window lower bound, ms */
#define MOTOR_SPEED_STALE_MAX_MS 100      /**< stale window upper bound, ms */
#define MOTOR_SPEED_STALL_REST_MS 300     /**< after a stall: Iq 0 this long, then a fresh push, ms */
#define MOTOR_SPEED_BLEED_RING 16U        /**< progress-tick records kept (>= BLEED_SPAN_MS + 1) */

/** @brief What the lifecycle wants from the loop this tick. */
enum motor_speed_mode {
  MOTOR_SPEED_OFF = 0,     /**< output off (DISARMED/READY/STARTING/FAULTED): all loop state cleared */
  MOTOR_SPEED_RUNNING = 1, /**< closed loop */
  MOTOR_SPEED_HOLD = 2     /**< output on, Iq 0; I, ff, slip held at 0 */
};

/** @brief Tunables (CONFIG keys 1-6, 25, 26). */
struct motor_speed_config {
  int16_t kp;                   /**< counts per RPM x 256 (1024 = 4 counts/RPM = 7.35 mA/RPM) */
  int16_t ki;                   /**< counts per RPM per ms x 16384 (553 = 62 mA/s per RPM) */
  uint16_t stall_ramp_ma_per_s; /**< stall-ramp integrator slope, mA/s */
  uint16_t stall_ms;            /**< high current without progress this long -> stall rest, ms */
};

/** @brief Sampled by the caller before each tick. */
struct motor_speed_inputs {
  uint8_t mode;              /**< ::motor_speed_mode */
  int16_t limit_counts;      /**< L = min(IQMAX, CONFIG limit, I2T allowance), counts; < 0 reads as 0 */
  uint16_t edges_pos;        /**< Hall edges counted in + direction (wrapping) */
  uint16_t edges_neg;        /**< Hall edges counted in - direction (wrapping) */
  int16_t mcsdk_avg_rpm;     /**< MCSDK 6-edge average speed, RPM */
  uint8_t mcsdk_buffer_full; /**< MCSDK's speed buffer holds 6 edges */
  uint8_t hall_code_invalid; /**< Hall code 0 or 7 this tick */
};

/** @brief Results of one tick. */
struct motor_speed_outputs {
  int16_t iq_ref_counts; /**< q-current request, counts (0 unless RUNNING) */
  int16_t ref_rpm;       /**< applied (interpolated) reference r, RPM */
  int16_t w_hat_rpm;     /**< speed estimate, RPM; 0 while invalid */
  int16_t p_counts;      /**< P term, counts */
  int16_t i_counts;      /**< integrator, counts (Q14 >> 14) */
  int16_t ff_counts;     /**< applied feed-forward, counts */
  uint8_t valid;         /**< w_hat is a measurement */
  uint8_t standstill;    /**< |w_hat| < ::MOTOR_SPEED_STANDSTILL_RPM (input to the HOLD rule) */
  uint8_t hall_lost;     /**< Hall code invalid for longer than the stale window */
  uint8_t saturated;     /**< P + I + ff was clamped to +-L */
  uint8_t stall_ramp;    /**< rule (a) moved the integrator this tick */
  uint8_t bleed;         /**< rule (c) halved the integrator this tick */
  uint8_t stall_timer;   /**< the stall timer counted this tick */
  uint8_t stall_rest;    /**< resting after a stall: Iq 0 for ::MOTOR_SPEED_STALL_REST_MS, then retry */
};

/** @brief Loop state; one per wheel controller, statically allocated. Fields are private to motor_speed.c. */
struct motor_speed {
  struct motor_speed_config cfg; /**< tunables */
  int32_t stall_step_q14;  /**< stall-ramp integrator step, Q14 counts per tick */
  uint32_t now_ms;         /**< ticks since init */
  uint8_t mode;            /**< mode of the previous tick */
  /* reference */
  int16_t ref_rpm;         /**< applied reference r */
  int16_t ref_from_rpm;    /**< interpolation start */
  int16_t ref_to_rpm;      /**< latest setpoint */
  uint8_t ref_k;           /**< interpolation step, 0..MOTOR_PROTOCOL_PERIOD_MS */
  int8_t ref_sign;         /**< sign of r last tick */
  int8_t ref_dir;          /**< last nonzero sign of r */
  int16_t ff_target_ma;    /**< requested feed-forward, mA */
  int16_t ff_ma;           /**< applied (slewed) feed-forward, mA */
  int32_t i_q14;           /**< integrator, Q14 counts */
  /* estimator */
  uint16_t prev_pos;       /**< Hall counters last tick */
  uint16_t prev_neg;       /**< see prev_pos */
  int8_t edge_dir;         /**< direction of the current same-direction run */
  uint8_t run_edges;       /**< edges in that run (saturating) */
  uint8_t seen_edge;       /**< an edge since init, so period is defined */
  uint16_t run_span_ms;    /**< first to last edge tick of the run (saturating) */
  uint16_t since_edge_ms;  /**< ticks since the last edge tick (saturating) */
  uint16_t period_ms;      /**< last edge period, ms per edge */
  uint16_t hall_bad_ms;    /**< consecutive ticks with an invalid Hall code */
  /* progress, slip, bleed, stall */
  int32_t back_edges;      /**< distance behind the furthest position along r, edges */
  int32_t slip;            /**< reference travel minus progress, 1/1250 edge, 0..4 edges */
  uint32_t progress_total; /**< progress edges since the run ring was cleared */
  uint32_t ring_ms[MOTOR_SPEED_BLEED_RING];   /**< tick of each progress record */
  uint32_t ring_edges[MOTOR_SPEED_BLEED_RING]; /**< progress_total at that record */
  uint8_t ring_head;       /**< next write slot */
  uint8_t ring_len;        /**< records in the current consecutive run */
  uint16_t stall_count_ms; /**< stall timer */
  uint16_t rest_ms;        /**< stall rest ticks left */
};

/**
 * @brief Initialise: mode OFF, reference 0, estimator invalid.
 * @param s         state
 * @param cfg       tunables (copied)
 * @param edges_pos current Hall + edge counter, so the first tick counts no edges
 * @param edges_neg current Hall - edge counter
 */
void motor_speed_init(struct motor_speed *s, const struct motor_speed_config *cfg, uint16_t edges_pos,
                      uint16_t edges_neg);

/**
 * @brief Change tunables; a Kp or Ki change clears the integrator (the old I means nothing under new gains).
 * @param s   state
 * @param cfg new tunables (copied)
 */
void motor_speed_configure(struct motor_speed *s, const struct motor_speed_config *cfg);

/**
 * @brief New target from an accepted CONTROL frame: r moves linearly from its present value to
 *        @p setpoint_rpm over ::MOTOR_PROTOCOL_PERIOD_MS ticks (C5b).
 * @param s            state
 * @param setpoint_rpm wheel speed target, RPM
 * @param ff_ma        feed-forward target, mA (applied only along sign(r), slewed)
 */
void motor_speed_set(struct motor_speed *s, int16_t setpoint_rpm, int16_t ff_ma);

/**
 * @brief 1 kHz step: reference, estimate, control law.
 * @param s   state
 * @param in  sampled inputs
 * @param out results; fully written
 */
void motor_speed_tick(struct motor_speed *s, const struct motor_speed_inputs *in, struct motor_speed_outputs *out);

#ifdef __cplusplus
}
#endif

#endif
