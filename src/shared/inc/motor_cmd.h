#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

/** @file motor_cmd.h
 *  @brief One-wheel CONTROL lifecycle: arm, watchdog, and start.
 *
 *  Host logic only. It calls the frame codec and does not touch HAL or
 *  MCSDK. The caller samples the clock and the bridge into
 *  ::motor_cmd_inputs and performs ::motor_cmd_actions. Specification:
 *  architecture/command-lifecycle.md.
 */

#include "motor_protocol.h"

#include <stdint.h>

/** @brief No accepted fresh motion for this long disarms the wheel, ms. */
#define MOTOR_WATCHDOG_MS 100u

/** @brief STARTING without the bridge in RUN for this long latches a fault, ms. */
#define MOTOR_CMD_START_TIMEOUT_MS 500u

/** @brief Bridge state the caller sampled. Not an MCSDK enum. */
enum motor_cmd_mc {
    MOTOR_CMD_MC_IDLE = 0,     /**< stopped, a start is allowed */
    MOTOR_CMD_MC_STARTING = 1, /**< start in progress, not regulating yet */
    MOTOR_CMD_MC_RUN = 2,      /**< regulating */
    MOTOR_CMD_MC_FAULT = 3     /**< bridge reports a fault */
};

/** @brief Limits and the wheel this state answers for. Copied at init. */
struct motor_cmd_cfg {
    uint32_t select;           /**< one wheel select bit; CONTROL_REPLY uses it */
    int16_t max_rpm;           /**< largest |SPEED setpoint| accepted, RPM */
    int16_t max_ma;            /**< largest |CURRENT setpoint| and |feed-forward| accepted, mA */
    uint8_t current_supported; /**< 0 rejects CURRENT with ::MOTOR_R_UNSUPPORTED */
};

/**
 * @brief Clock and bridge sample.
 *
 * NULL on ::motor_cmd_control keeps the arming checks only: the watchdog
 * does not run and the lifecycle does not leave READY for STARTING, because
 * this call did not observe a bridge.
 */
struct motor_cmd_inputs {
    uint32_t now_ms; /**< caller clock, ms; the same clock as a frame's receive time */
    uint8_t mc;      /**< ::motor_cmd_mc */
    uint8_t aligned; /**< 1 when the Hall phase shift is known */
    uint8_t busy;    /**< 1 while alignment or capture is in progress */
};

/**
 * @brief What the caller does with the bridge after a call. Cleared first.
 *
 * When @c set_target is 1, program @c motor_cmd::mode, @c setpoint, and
 * @c ff_ma. @c start and @c stop are requests; several may be set, and a
 * fault clears @c start and @c set_target so only @c stop remains.
 */
struct motor_cmd_actions {
    uint8_t start;      /**< 1: start the bridge */
    uint8_t set_target; /**< 1: program the recorded target */
    uint8_t stop;       /**< 1: switch the output off */
};

/**
 * @brief Lifecycle memory for one wheel.
 *
 * FAULTED is entered when STARTING waits longer than
 * ::MOTOR_CMD_START_TIMEOUT_MS. A caller may also set @c lifecycle and
 * @c latched; ARM and motion then report @c latched.
 */
struct motor_cmd {
    struct motor_cmd_cfg cfg;
    uint8_t lifecycle;     /**< ::motor_lifecycle */
    uint8_t session;       /**< session byte this wheel accepts */
    uint8_t last_seq;      /**< last ARM, or the last accepted motion sequence */
    uint8_t applied_seq;   /**< sequence of the recorded motion; 0 when none is recorded */
    uint8_t last_valid;    /**< @c last_frame is an accepted motion payload */
    uint8_t latched;       /**< ::motor_reason while FAULTED */
    uint8_t mode;          /**< mode of the recorded target */
    uint8_t disarm_reason; /**< ::MOTOR_R_TIMEOUT after a watchdog disarm, else 0 */
    int16_t setpoint;      /**< recorded target: RPM (SPEED) or q-axis mA (CURRENT) */
    int16_t ff_ma;         /**< recorded feed-forward, q-axis mA */
    struct motor_control last_frame; /**< accepted motion payload, for a duplicate check */
    uint32_t watch_ms;     /**< ARM time, or receive time of the last fresh motion */
    uint32_t state_ms;     /**< now_ms when STARTING was entered */
    uint32_t now_ms;       /**< last sampled clock */
};

/**
 * @brief DISARMED, no target, session from ::motor_session_next(0, entropy).
 *
 * @param m       state; overwritten
 * @param cfg     limits and this wheel's select bit
 * @param entropy boot entropy; the select bit yields session `1 + select`
 */
void motor_cmd_init(struct motor_cmd *m, const struct motor_cmd_cfg *cfg, uint32_t entropy);

/**
 * @brief Decode one CONTROL, apply it, and encode the CONTROL_REPLY.
 *
 * A frame that is not a DLC-8 CONTROL returns ::MOTOR_R_BAD_LENGTH and does
 * not write @p reply_id, @p reply_data, or @p reply. Any other structural
 * failure fills a REJECTED reply and leaves the lifecycle untouched.
 *
 * When @p in is non-NULL this call also samples the clock (@p rx_ms is the
 * frame's receive time) and the bridge. A fresh motion is answered after
 * those rules, so the reply can already say STARTING. ARM, DISABLE, and
 * rejects are answered at the verdict; the watchdog may still move the
 * state afterwards, which is how a late ARM is applied and then disarms.
 * NULL @p in does not sample, so an accepted motion stays READY and is APPLIED.
 *
 * ::MOTOR_R_NONE means the reply outcome is APPLIED or DUPLICATE.
 * ::MOTOR_R_STARTING means INHIBITED: the motion was recorded and the wheel
 * is not RUNNING yet.
 *
 * @param m          state
 * @param id         11-bit CAN id
 * @param data       payload
 * @param dlc        payload length, bytes
 * @param rx_ms      receive time of this frame, ms; recorded on ARM and on a fresh accept
 * @param in         clock and bridge sample; NULL skips the watchdog and the start
 * @param act        out: bridge requests; NULL if the caller does not need them
 * @param reply_id   out: CONTROL_REPLY id for @c cfg.select
 * @param reply_data out: 8 reply bytes
 * @param reply      out: the same reply; may be NULL
 * @return reason carried in the reply, or ::MOTOR_R_BAD_LENGTH when none is sent
 */
enum motor_reason motor_cmd_control(struct motor_cmd *m, uint32_t id, const uint8_t *data,
                                    uint32_t dlc, uint32_t rx_ms,
                                    const struct motor_cmd_inputs *in,
                                    struct motor_cmd_actions *act, uint32_t *reply_id,
                                    uint8_t reply_data[MOTOR_PROTOCOL_DLC],
                                    struct motor_reply *reply);

/**
 * @brief Advance the clock and the start rules with no frame.
 *
 * Same watchdog and READY / STARTING / RUNNING / start-timeout rules as a
 * sampled ::motor_cmd_control. @p in NULL, or @p m NULL, leaves the state
 * alone (actions are still cleared when @p act is non-NULL).
 *
 * @param m   state
 * @param in  clock and bridge sample
 * @param act out: bridge requests; may be NULL
 */
void motor_cmd_tick(struct motor_cmd *m, const struct motor_cmd_inputs *in,
                    struct motor_cmd_actions *act);

#endif
