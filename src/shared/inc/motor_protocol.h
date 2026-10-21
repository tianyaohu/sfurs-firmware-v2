#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file motor_protocol.h
 *  @brief Motor command protocol v1: the only encoder/decoder of CONTROL frames.
 *
 *  Compiled into the ESC, the Nucleo and the PC tools, so the byte layout exists
 *  once. Specification: technical-development-plans/esc-command-v2/A-protocol.md
 *  (moves to architecture/ when the ESC uses it). These functions are stateless
 *  except where noted; the ESC applies its own range limits and the session and
 *  sequence state on top of them. */

#define MOTOR_PROTOCOL_VERSION 1U    /**< value of CONTROL byte 0 bits 7-4 */
#define MOTOR_PROTOCOL_DLC 8U        /**< every frame of this protocol carries exactly 8 bytes */
#define MOTOR_PROTOCOL_PERIOD_MS 20U /**< Nucleo CONTROL period, ms; the ESC interpolates the speed reference over it */
#define MOTOR_WATCHDOG_MS 100U       /**< no accepted fresh motion frame for this long -> output off and disarm, ms */
#define MOTOR_SEQ_WINDOW 16          /**< largest accepted forward sequence step (up to 15 lost frames) */

/** CAN ID bases; OR one wheel select bit (or several, for a broadcast DISABLE). */
#define MOTOR_CAN_ID_CONTROL 0x010U       /**< Nucleo -> ESC, highest bus priority */
#define MOTOR_CAN_ID_CONFIG 0x020U        /**< Nucleo -> ESC (Stage 3) */
#define MOTOR_CAN_ID_CONTROL_REPLY 0x410U /**< ESC -> Nucleo, one per CONTROL frame */
#define MOTOR_CAN_ID_CONFIG_REPLY 0x420U  /**< ESC -> Nucleo (Stage 3) */
#define MOTOR_CAN_ID_BASE_MASK 0x7F0U     /**< slot and sub-ID bits of an 11-bit ID */
#define MOTOR_CAN_SELECT_MASK 0x00FU      /**< wheel select bits of an 11-bit ID */

/** @brief CONTROL mode (byte 0 bits 3-0). */
enum motor_mode {
  MOTOR_MODE_DISABLE = 0, /**< output off; with a flag: ARM or CLEAR_FAULT */
  MOTOR_MODE_SPEED = 1,   /**< setpoint = motor RPM, ff = q-axis mA */
  MOTOR_MODE_CURRENT = 2  /**< setpoint = q-axis mA */
};

#define MOTOR_FLAG_ARM 0x01U         /**< DISABLE only: arm with the session in byte 7 */
#define MOTOR_FLAG_CLEAR_FAULT 0x02U /**< DISABLE only: clear a latched fault */
#define MOTOR_FLAG_RESERVED 0xFCU    /**< must be 0 */

/** @brief ESC command lifecycle (reply byte 2 bits 2-0); see B-lifecycle.md. */
enum motor_lifecycle {
  MOTOR_LC_DISARMED = 0, /**< motion rejected until ARM */
  MOTOR_LC_READY = 1,    /**< armed, output off */
  MOTOR_LC_STARTING = 2, /**< MCSDK start in progress */
  MOTOR_LC_RUNNING = 3,  /**< regulating */
  MOTOR_LC_HOLD = 4,     /**< parked at zero speed: output on, Iq = 0 */
  MOTOR_LC_FAULTED = 5   /**< output off, cause latched */
};

/** @brief What the ESC did with one CONTROL frame (reply byte 2 bits 5-3). */
enum motor_outcome {
  MOTOR_OUT_APPLIED = 0,    /**< the controller uses this frame's target from now on */
  MOTOR_OUT_INHIBITED = 1,  /**< applied, but the output cannot follow (reason says why) */
  MOTOR_OUT_SUPERSEDED = 2, /**< accepted, replaced by a newer frame before it was applied */
  MOTOR_OUT_REJECTED = 3,   /**< not used; nothing changed, watchdog not refreshed */
  MOTOR_OUT_DUPLICATE = 4   /**< same seq and payload as the last accepted frame; ignored */
};

/** @brief Reason codes (reply byte 3). Codes >= 32 are latched fault causes. */
enum motor_reason {
  MOTOR_R_NONE = 0,             /**< no reason */
  MOTOR_R_BAD_VERSION = 1,      /**< byte 0 version is not ::MOTOR_PROTOCOL_VERSION */
  MOTOR_R_BAD_MODE = 2,         /**< mode is not a ::motor_mode */
  MOTOR_R_BAD_FLAGS = 3,        /**< ARM/CLEAR_FAULT outside DISABLE, or both at once */
  MOTOR_R_RESERVED_NONZERO = 4, /**< a reserved bit or a field unused by the mode is not 0 */
  MOTOR_R_BAD_SELECT = 5,       /**< no select bit, or several for a frame that is not a plain DISABLE */
  MOTOR_R_OUT_OF_RANGE = 6,     /**< beyond the ESC's hard limits; rejected, never clamped */
  MOTOR_R_NOT_ARMED = 7,        /**< motion while DISARMED */
  MOTOR_R_BAD_SESSION = 8,      /**< byte 7 is not the ESC's current session */
  MOTOR_R_STALE = 9,            /**< sequence older than the last accepted */
  MOTOR_R_SEQ_AMBIGUOUS = 10,   /**< sequence exactly 128 away */
  MOTOR_R_SEQ_JUMP = 11,        /**< sequence more than ::MOTOR_SEQ_WINDOW ahead */
  MOTOR_R_NOT_ALIGNED = 12,     /**< Hall phase shift unknown */
  MOTOR_R_BUSY = 13,            /**< Hall alignment or capture running */
  MOTOR_R_MODE_CHANGE = 14,     /**< mode differs from the latched one; DISABLE first */
  MOTOR_R_NOT_IDLE = 15,        /**< CLEAR_FAULT while the motor is not stopped or the cause persists */
  MOTOR_R_LIMITED = 16,         /**< INHIBITED: effective current limit below the request */
  MOTOR_R_STARTING = 17,        /**< INHIBITED: MCSDK start in progress */
  MOTOR_R_FF_UNUSED = 18,       /**< INHIBITED: feed-forward not applied by the active loop */
  MOTOR_R_CONFLICT = 19,        /**< same sequence as the last accepted frame, different payload */
  MOTOR_R_BAD_LENGTH = 20,      /**< DLC is not 8: counted, never replied to */
  MOTOR_R_UNSUPPORTED = 21,     /**< valid mode this firmware does not implement yet */
  MOTOR_R_FAULT_MCSDK = 32,     /**< MCSDK protection (over-current, voltage, temperature, start timeout) */
  MOTOR_R_FAULT_STALL = 33,     /**< sustained current with no Hall progress */
  MOTOR_R_FAULT_HALL_LOST = 34, /**< speed estimate lost after valid motion */
  MOTOR_R_TIMEOUT = 35,         /**< command watchdog expired */
  MOTOR_R_LIFECYCLE_OVERFLOW = 36 /**< more than 4 lifecycle frames pending */
};

/** @brief Decoded CONTROL frame. */
struct motor_control {
  uint8_t mode;     /**< ::motor_mode */
  uint8_t flags;    /**< MOTOR_FLAG_* */
  int16_t setpoint; /**< SPEED: motor RPM; CURRENT: q-axis mA; DISABLE: 0 */
  int16_t ff_ma;    /**< SPEED: friction feed-forward, q-axis mA; otherwise 0 */
  uint8_t seq;      /**< per-wheel sequence */
  uint8_t session;  /**< session announced by the ESC */
};

/** @brief Decoded CONTROL_REPLY frame. */
struct motor_reply {
  uint8_t request_seq; /**< seq of the frame answered */
  uint8_t applied_seq; /**< seq of the motion frame the controller uses now (0 after a disarm) */
  uint8_t lifecycle;   /**< ::motor_lifecycle */
  uint8_t outcome;     /**< ::motor_outcome */
  uint8_t first_edge;  /**< 1 once a Hall edge along the command was seen since the current start */
  uint8_t reason;      /**< ::motor_reason */
  uint8_t session;     /**< session the ESC accepts now (the challenge while DISARMED) */
  int16_t rpm;         /**< measured speed, RPM; 0 when not valid */
  int16_t iq_ma;       /**< measured q-axis current, mA, carried in 50 mA steps (+-6350 mA) */
};

/** @brief Result of comparing a received sequence with the last accepted one. */
enum motor_seq_class {
  MOTOR_SEQ_NEXT = 0,      /**< 1..::MOTOR_SEQ_WINDOW ahead: fresh */
  MOTOR_SEQ_DUPLICATE = 1, /**< equal */
  MOTOR_SEQ_STALE = 2,     /**< 1..127 behind */
  MOTOR_SEQ_AMBIGUOUS = 3, /**< exactly 128 apart */
  MOTOR_SEQ_JUMP = 4       /**< more than ::MOTOR_SEQ_WINDOW ahead */
};

/**
 * @brief Structural check of a CONTROL frame (no ESC limits, no session or sequence state).
 * @param c      decoded fields
 * @param select wheel select bits of the frame's CAN ID
 * @return ::MOTOR_R_NONE or the first violated rule, in the order of A-protocol.md
 */
enum motor_reason motor_control_check(const struct motor_control *c, uint32_t select);

/**
 * @brief Decode and check a received CONTROL frame.
 * @param id   11-bit CAN ID (must be ::MOTOR_CAN_ID_CONTROL | select)
 * @param data payload bytes
 * @param dlc  payload length, bytes
 * @param out  decoded fields; filled whenever dlc is 8 and the ID base is CONTROL, even if the check
 *             fails (the reply needs seq); untouched otherwise (such a frame gets no reply)
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH for dlc != 8 or a wrong ID base, or a ::motor_control_check reason
 */
enum motor_reason motor_control_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                       struct motor_control *out);

/**
 * @brief Encode a CONTROL frame; refuses a frame the receiver would reject structurally.
 * @param c      fields to send
 * @param select wheel select bit(s)
 * @param id     out: 11-bit CAN ID
 * @param data   out: 8 payload bytes
 * @return ::MOTOR_R_NONE, or the ::motor_control_check reason (id and data then untouched)
 */
enum motor_reason motor_control_encode(const struct motor_control *c, uint32_t select,
                                       uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC]);

/**
 * @brief Encode a CONTROL_REPLY payload. Iq is rounded to 50 mA and saturated to +-6350 mA.
 * @param r    fields (lifecycle <= 5, outcome <= 4)
 * @param data out: 8 payload bytes
 */
void motor_reply_encode(const struct motor_reply *r, uint8_t data[MOTOR_PROTOCOL_DLC]);

/**
 * @brief Decode a CONTROL_REPLY payload.
 * @param data payload bytes
 * @param dlc  payload length, bytes
 * @param out  decoded fields
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or ::MOTOR_R_RESERVED_NONZERO (bit 7 of byte 2, or a
 *         lifecycle/outcome value outside its enum)
 */
enum motor_reason motor_reply_decode(const uint8_t *data, uint32_t dlc, struct motor_reply *out);

/**
 * @brief Classify a received sequence against the last accepted one (signed 8-bit difference).
 * @param last last accepted sequence
 * @param seq  received sequence
 */
enum motor_seq_class motor_seq_classify(uint8_t last, uint8_t seq);

/**
 * @brief Next session byte: never 0 and never equal to @p prev.
 * @param prev    current session (0 at boot)
 * @param entropy any changing value, e.g. the 1 kHz tick count
 */
uint8_t motor_session_next(uint8_t prev, uint32_t entropy);

#ifdef __cplusplus
}
#endif

#endif
