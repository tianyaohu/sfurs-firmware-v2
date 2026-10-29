#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

/** @file motor_cmd.h
 *  @brief One-wheel CONTROL lifecycle slice: arm, then record a setpoint.
 *
 *  Host logic only. It calls the frame codec and does not start a motor.
 *  Specification: architecture/command-lifecycle.md.
 */

#include "motor_protocol.h"

#include <stdint.h>

/** @brief Limits and the wheel this state answers for. Copied at init. */
struct motor_cmd_cfg {
    uint32_t select;           /**< one wheel select bit; CONTROL_REPLY uses it */
    int16_t max_rpm;           /**< largest |SPEED setpoint| accepted, RPM */
    int16_t max_ma;            /**< largest |CURRENT setpoint| and |feed-forward| accepted, mA */
    uint8_t current_supported; /**< 0 rejects CURRENT with ::MOTOR_R_UNSUPPORTED */
};

/**
 * @brief Lifecycle memory for one wheel.
 *
 * The applied setpoint is host-visible because this slice never leaves
 * READY for STARTING. FAULTED is not entered here: a caller that has latched
 * a cause sets @c lifecycle and @c latched, and ARM then reports @c latched.
 */
struct motor_cmd {
    struct motor_cmd_cfg cfg;
    uint8_t lifecycle;   /**< ::motor_lifecycle; DISARMED, READY, or a caller-set FAULTED */
    uint8_t session;     /**< session byte this wheel accepts */
    uint8_t last_seq;    /**< last ARM, or the last accepted motion sequence */
    uint8_t applied_seq; /**< sequence of the recorded motion; 0 when none is recorded */
    uint8_t last_valid;  /**< @c last_frame is an accepted motion payload */
    uint8_t latched;     /**< ::motor_reason while FAULTED */
    uint8_t mode;        /**< mode of the recorded target */
    int16_t setpoint;    /**< recorded target: RPM (SPEED) or q-axis mA (CURRENT) */
    int16_t ff_ma;       /**< recorded feed-forward, q-axis mA */
    struct motor_control last_frame; /**< accepted motion payload, for a duplicate check */
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
 * ::MOTOR_R_NONE means the reply outcome is APPLIED or DUPLICATE.
 *
 * @param m          state
 * @param id         11-bit CAN id
 * @param data       payload
 * @param dlc        payload length, bytes
 * @param reply_id   out: CONTROL_REPLY id for @c cfg.select
 * @param reply_data out: 8 reply bytes
 * @param reply      out: the same reply; may be NULL
 * @return reason carried in the reply, or ::MOTOR_R_BAD_LENGTH when none is sent
 */
enum motor_reason motor_cmd_control(struct motor_cmd *m, uint32_t id, const uint8_t *data,
                                    uint32_t dlc, uint32_t *reply_id,
                                    uint8_t reply_data[MOTOR_PROTOCOL_DLC],
                                    struct motor_reply *reply);

#endif
