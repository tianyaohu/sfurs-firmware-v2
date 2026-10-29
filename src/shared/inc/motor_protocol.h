#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

/** @file motor_protocol.h
 *  @brief Encode and decode CONTROL v1, CONTROL_REPLY, STATUS, and DIAG.
 *
 *  Stateless wire codec, plus pure session and sequence helpers. It does
 *  not apply current or speed limits, and it does not remember a wheel or
 *  apply a command. Specification: architecture/protocol.md.
 */

#include <stdint.h>

/** @brief Value of CONTROL byte 0, bits 7-4. */
#define MOTOR_PROTOCOL_VERSION 1u

/** @brief CONTROL, STATUS reply, and DIAG reply payload length, bytes. */
#define MOTOR_PROTOCOL_DLC 8u

/** @brief STATUS_READ and DIAG_READ payload length, bytes. The bytes are ignored. */
#define MOTOR_POLL_DLC 4u

/** @brief Wheel select bits in an 11-bit id. FR, BR, BL, FL. */
#define MOTOR_SELECT_FR 0x1u
#define MOTOR_SELECT_BR 0x2u
#define MOTOR_SELECT_BL 0x4u
#define MOTOR_SELECT_FL 0x8u
#define MOTOR_SELECT_MASK 0x0Fu

/**
 * @brief CONTROL id base. OR one select bit, or several for a plain DISABLE.
 *
 * Bits 10-4 are the base (0x010). Bits 3-0 are the select. CONTROL_REPLY
 * uses the same split, base 0x410. Service commands use bits 10-7 only.
 */
#define MOTOR_CAN_ID_CONTROL 0x010u
#define MOTOR_CAN_ID_CONTROL_REPLY 0x410u
#define MOTOR_CAN_ID_BASE_MASK 0x7F0u

/** @brief Largest forward sequence step classified as fresh (15 lost frames). */
#define MOTOR_SEQ_WINDOW 16u

/**
 * @brief CONTROL_REPLY byte 7, mA per step.
 *
 * Signed on purpose: it is multiplied by a negative current. 127 steps is
 * 6.35 A; a larger magnitude saturates.
 */
#define MOTOR_REPLY_IQ_STEP_MA 50

/** @brief Service commands. OR select bits in the low 4; bits 6-4 stay 0. */
#define MOTOR_CAN_ID_STATUS_READ 0x100u
#define MOTOR_CAN_ID_STATUS 0x280u
#define MOTOR_CAN_ID_DIAG_READ 0x300u
#define MOTOR_CAN_ID_DIAG 0x380u
#define MOTOR_CAN_SERVICE_MASK 0x780u

/**
 * @brief STATUS q-axis current unit.
 *
 * 544 counts is 1 A of motor phase current, not battery current.
 */
#define MOTOR_COUNTS_PER_AMP 544

/** @brief CONTROL byte 0, bits 3-0. */
enum motor_mode {
    MOTOR_MODE_DISABLE = 0, /**< output off; flags may ask to arm or clear a fault */
    MOTOR_MODE_SPEED = 1,   /**< setpoint is mechanical RPM; ff is q-axis mA */
    MOTOR_MODE_CURRENT = 2  /**< setpoint is q-axis mA */
};

#define MOTOR_FLAG_ARM 0x01u         /**< DISABLE only: arm on the session in byte 7 */
#define MOTOR_FLAG_CLEAR_FAULT 0x02u /**< DISABLE only: clear a latched fault */
#define MOTOR_FLAG_RESERVED 0xFCu    /**< CONTROL byte 1, bits 7-2; must be 0 */

/** @brief STATUS byte 7, bits 3-0. Values 6 and 7 are retired and still pass through. */
enum motor_stop_reason {
    MOTOR_STOP_NONE = 0,
    MOTOR_STOP_DISABLE = 1,
    MOTOR_STOP_TIMEOUT = 2,
    MOTOR_STOP_FAULT = 3,
    MOTOR_STOP_ALIGN = 4,
    MOTOR_STOP_CAPTURE = 5,
    MOTOR_STOP_PARKED = 8
};

/** @brief MCSDK `RUN` in the vendored `MCI_State_t` (`mc_interface.h`). The codec does not interpret the byte. */
#define MOTOR_MCSDK_STATE_RUN 6u

#define MOTOR_STATUS_OUTPUT_ON 0x10u  /**< bridge on */
#define MOTOR_STATUS_DERATED 0x20u    /**< I2t limit is in force */
#define MOTOR_STATUS_ALIGNING 0x40u   /**< forced-angle alignment or capture */
#define MOTOR_STATUS_HALL_KNOWN 0x80u /**< Hall shift is known */

/** @brief CONTROL_REPLY byte 2, bits 2-0. The codec does not advance these. */
enum motor_lifecycle {
    MOTOR_LC_DISARMED = 0,
    MOTOR_LC_READY = 1,
    MOTOR_LC_STARTING = 2,
    MOTOR_LC_RUNNING = 3,
    MOTOR_LC_HOLD = 4,
    MOTOR_LC_FAULTED = 5
};

/** @brief CONTROL_REPLY byte 2, bits 5-3. */
enum motor_outcome {
    MOTOR_OUT_APPLIED = 0,
    MOTOR_OUT_INHIBITED = 1,
    MOTOR_OUT_SUPERSEDED = 2,
    MOTOR_OUT_REJECTED = 3,
    MOTOR_OUT_DUPLICATE = 4
};

/** @brief Where a new sequence sits relative to the last accepted one. */
enum motor_seq_class {
    MOTOR_SEQ_NEXT = 0,      /**< 1..::MOTOR_SEQ_WINDOW ahead */
    MOTOR_SEQ_DUPLICATE = 1, /**< equal; payload compare is not done here */
    MOTOR_SEQ_STALE = 2,     /**< 1..127 behind */
    MOTOR_SEQ_AMBIGUOUS = 3, /**< exactly 128 apart */
    MOTOR_SEQ_JUMP = 4       /**< more than ::MOTOR_SEQ_WINDOW ahead */
};

/** @brief DIAG byte 6. */
enum motor_align_result {
    MOTOR_ALIGN_NONE = 0,
    MOTOR_ALIGN_OK = 1,
    MOTOR_ALIGN_REVERSED = 2,
    MOTOR_ALIGN_NO_EDGES = 3,
    MOTOR_ALIGN_ABORTED = 4,
    MOTOR_ALIGN_RUNNING = 5
};

/**
 * @brief Why a frame was refused, or why a reply says the output did not follow.
 *
 * CONTROL decode returns only the structural codes and ::MOTOR_R_BAD_LENGTH.
 * From ::MOTOR_R_OUT_OF_RANGE upward the values are named so they stay put;
 * the ESC lifecycle that would return them is not in this codec. A
 * CONTROL_REPLY carries the reason byte and does not interpret it.
 */
enum motor_reason {
    MOTOR_R_NONE = 0,
    MOTOR_R_BAD_VERSION = 1,      /**< CONTROL version nibble is not 1 */
    MOTOR_R_BAD_MODE = 2,         /**< CONTROL mode is not a ::motor_mode */
    MOTOR_R_BAD_FLAGS = 3,        /**< ARM or CLEAR_FAULT used together or outside DISABLE */
    MOTOR_R_RESERVED_NONZERO = 4, /**< a reserved bit, or a field this mode does not use */
    MOTOR_R_BAD_SELECT = 5,       /**< select bits do not name the wheels this frame may address */
    MOTOR_R_OUT_OF_RANGE = 6,     /**< beyond the ESC hard limit; not clamped, not returned here */
    MOTOR_R_NOT_ARMED = 7,        /**< motion while disarmed */
    MOTOR_R_BAD_SESSION = 8,      /**< session byte is not the one the ESC holds */
    MOTOR_R_STALE = 9,            /**< sequence behind the last accepted */
    MOTOR_R_SEQ_AMBIGUOUS = 10,   /**< sequence exactly 128 away from the last accepted */
    MOTOR_R_SEQ_JUMP = 11,        /**< sequence more than ::MOTOR_SEQ_WINDOW ahead */
    MOTOR_R_NOT_ALIGNED = 12,     /**< Hall phase shift unknown */
    MOTOR_R_BUSY = 13,            /**< alignment or capture in progress */
    MOTOR_R_MODE_CHANGE = 14,     /**< mode differs from the one that is running */
    MOTOR_R_NOT_IDLE = 15,        /**< CLEAR_FAULT while the motor is not stopped */
    MOTOR_R_LIMITED = 16,         /**< applied, but the effective current limit is lower */
    MOTOR_R_STARTING = 17,        /**< applied, but the start is still in progress */
    MOTOR_R_FF_UNUSED = 18,       /**< retired; not sent, not reused */
    MOTOR_R_CONFLICT = 19,        /**< same sequence as the last accepted frame, different payload */
    MOTOR_R_BAD_LENGTH = 20,      /**< NULL, wrong DLC, or an id this function does not decode */
    MOTOR_R_UNSUPPORTED = 21,     /**< mode this firmware does not implement */
    MOTOR_R_STALLED = 22,         /**< applied, but the wheel is in a stall rest */
    MOTOR_R_FAULT_MCSDK = 32,     /**< latched MCSDK protection */
    MOTOR_R_FAULT_STALL = 33,     /**< retired; a stall rests as ::MOTOR_R_STALLED */
    MOTOR_R_FAULT_HALL_LOST = 34, /**< latched Hall sensing loss */
    MOTOR_R_TIMEOUT = 35,         /**< command watchdog expired */
    MOTOR_R_LIFECYCLE_OVERFLOW = 36 /**< more lifecycle frames pending than the queue holds */
};

/** @brief Decoded CONTROL payload. */
struct motor_control {
    uint8_t mode;     /**< ::motor_mode */
    uint8_t flags;    /**< MOTOR_FLAG_ARM, MOTOR_FLAG_CLEAR_FAULT, or 0 */
    int16_t setpoint; /**< SPEED: RPM; CURRENT: q-axis mA; DISABLE: 0 */
    int16_t ff_ma;    /**< SPEED: q-axis feed-forward, mA; otherwise 0 */
    uint8_t seq;      /**< per-wheel sequence chosen by the sender */
    uint8_t session;  /**< session byte; the codec does not interpret it */
};

/** @brief Decoded CONTROL_REPLY. Current is mA, not the on-wire step. */
struct motor_reply {
    uint8_t request_seq; /**< sequence of the CONTROL frame this answers */
    uint8_t applied_seq; /**< sequence the controller is using; 0 after a disarm */
    uint8_t lifecycle;   /**< ::motor_lifecycle */
    uint8_t outcome;     /**< ::motor_outcome */
    uint8_t first_edge;  /**< 1 after a Hall edge along the command since this start */
    uint8_t reason;      /**< ::motor_reason, carried as a byte */
    uint8_t session;     /**< session the ESC accepts now */
    int16_t rpm;         /**< measured mechanical RPM; 0 when not valid */
    int16_t iq_ma;       /**< measured q-axis current, mA, in ::MOTOR_REPLY_IQ_STEP_MA steps */
};

/** @brief Decoded STATUS reply. */
struct motor_status {
    int16_t rpm;           /**< mechanical RPM */
    int16_t iq_counts;     /**< measured q-axis current, ::MOTOR_COUNTS_PER_AMP per ampere */
    int16_t iq_ref_counts; /**< q-axis current reference, same unit */
    uint8_t mcsdk_state;   /**< MCSDK state byte; ::MOTOR_MCSDK_STATE_RUN is 6 */
    uint8_t flags;         /**< low nibble ::motor_stop_reason, high nibble MOTOR_STATUS_* */
};

/** @brief Decoded DIAG reply. */
struct motor_diag {
    uint16_t vbus_cv;       /**< DC bus, centivolts */
    uint16_t faults;        /**< latched MCSDK fault word */
    int16_t hall_shift_deg; /**< Hall phase shift, electrical degrees */
    uint8_t align_result;   /**< ::motor_align_result */
    uint8_t i2t_percent;    /**< I2t load, percent */
};

/**
 * @brief Structural check of a CONTROL payload and its select bits.
 *
 * @param frame  fields to send or just decoded
 * @param select wheel select bits from the id (not a full CAN id)
 * @return ::MOTOR_R_NONE, or the first failed rule in architecture/protocol.md
 */
enum motor_reason motor_control_check(const struct motor_control *frame, uint32_t select);

/**
 * @brief Decode one CONTROL frame.
 *
 * @param id    11-bit CAN id
 * @param data  payload; ignored when @p dlc is not 8
 * @param dlc   payload length, bytes
 * @param out   filled when the id is CONTROL and @p dlc is 8, including when
 *              the structural check fails (byte 6 is still the sequence)
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or a ::motor_control_check reason.
 *         ::MOTOR_R_BAD_VERSION is reported before the other field checks.
 */
enum motor_reason motor_control_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                       struct motor_control *out);

/**
 * @brief Encode one CONTROL frame the structural check would accept.
 *
 * @param frame  fields
 * @param select wheel select bit, or several for a plain DISABLE
 * @param id     out: 11-bit CAN id; untouched on failure
 * @param data   out: 8 bytes; untouched on failure
 * @return ::MOTOR_R_NONE, or the ::motor_control_check reason
 */
enum motor_reason motor_control_encode(const struct motor_control *frame, uint32_t select,
                                       uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC]);

/**
 * @brief Encode one CONTROL_REPLY.
 *
 * Iq is rounded to ::MOTOR_REPLY_IQ_STEP_MA, halves away from zero, and
 * saturated to +-127 steps. A nonzero @c first_edge sets bit 6.
 *
 * @param frame  fields. lifecycle at most ::MOTOR_LC_FAULTED, outcome at most
 *               ::MOTOR_OUT_DUPLICATE
 * @param select one wheel select bit
 * @param id     out: 11-bit CAN id; untouched on failure
 * @param data   out: 8 bytes; untouched on failure
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, ::MOTOR_R_RESERVED_NONZERO,
 *         or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_reply_encode(const struct motor_reply *frame, uint32_t select,
                                     uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC]);

/**
 * @brief Decode one CONTROL_REPLY.
 *
 * @param id    11-bit CAN id, ::MOTOR_CAN_ID_CONTROL_REPLY with one wheel select bit
 * @param data  8 payload bytes
 * @param dlc   payload length, bytes
 * @param out   filled when the id base, length, and byte 2 are legal, including
 *              when the select is not one wheel. Untouched otherwise.
 *              @c iq_ma is the on-wire step times ::MOTOR_REPLY_IQ_STEP_MA.
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, ::MOTOR_R_RESERVED_NONZERO
 *         (bit 7 of byte 2, or a lifecycle or outcome outside its enum),
 *         or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_reply_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                     struct motor_reply *out);

/**
 * @brief Decode one STATUS reply.
 *
 * @param id    11-bit CAN id, ::MOTOR_CAN_ID_STATUS with one wheel select bit
 * @param data  8 payload bytes
 * @param dlc   payload length, bytes
 * @param out   filled when the id base and length match, even if the select is illegal
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_status_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                      struct motor_status *out);

/**
 * @brief Encode one STATUS reply.
 *
 * @param frame  measurements
 * @param select one wheel select bit
 * @param id     out: 11-bit CAN id; untouched on failure
 * @param data   out: 8 bytes; untouched on failure
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_status_encode(const struct motor_status *frame, uint32_t select,
                                      uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC]);

/**
 * @brief Decode one DIAG reply.
 *
 * @param id    11-bit CAN id, ::MOTOR_CAN_ID_DIAG with one wheel select bit
 * @param data  8 payload bytes
 * @param dlc   payload length, bytes
 * @param out   filled when the id base and length match, even if the select is illegal
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_diag_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                    struct motor_diag *out);

/**
 * @brief Encode one DIAG reply.
 *
 * @param frame  measurements
 * @param select one wheel select bit
 * @param id     out: 11-bit CAN id; untouched on failure
 * @param data   out: 8 bytes; untouched on failure
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_diag_encode(const struct motor_diag *frame, uint32_t select,
                                    uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC]);

/**
 * @brief Decode a STATUS_READ or DIAG_READ poll.
 *
 * @param id       11-bit CAN id
 * @param data     payload; contents are not inspected
 * @param dlc      payload length, bytes; must be 4
 * @param command  out: ::MOTOR_CAN_ID_STATUS_READ or ::MOTOR_CAN_ID_DIAG_READ;
 *                 untouched on failure
 * @param select   out: wheel select bits; untouched on failure
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH, or ::MOTOR_R_BAD_SELECT.
 *         A poll may name any non-empty subset of the four wheels.
 */
enum motor_reason motor_poll_decode(uint32_t id, const uint8_t *data, uint32_t dlc,
                                    uint32_t *command, uint32_t *select);

/**
 * @brief Encode a STATUS_READ or DIAG_READ poll with a zero payload.
 *
 * @param command ::MOTOR_CAN_ID_STATUS_READ or ::MOTOR_CAN_ID_DIAG_READ
 * @param select  one or more wheel select bits
 * @param id      out: 11-bit CAN id; untouched on failure
 * @param data    out: 4 zero bytes; untouched on failure
 * @return ::MOTOR_R_NONE, ::MOTOR_R_BAD_LENGTH for a command that is not a poll,
 *         or ::MOTOR_R_BAD_SELECT
 */
enum motor_reason motor_poll_encode(uint32_t command, uint32_t select, uint32_t *id,
                                    uint8_t data[MOTOR_POLL_DLC]);

/**
 * @brief Classify @p seq against the last accepted sequence.
 *
 * The distance is the signed 8-bit difference. An equal sequence is
 * ::MOTOR_SEQ_DUPLICATE whether or not the payload matches; ::MOTOR_R_CONFLICT
 * is an ESC check and is not returned here.
 *
 * @param last_seq last accepted sequence
 * @param seq      sequence just received
 */
enum motor_seq_class motor_seq_classify(uint8_t last_seq, uint8_t seq);

/**
 * @brief Next session byte. Never 0, and never equal to @p prev.
 *
 * Boot (@p prev 0) is `1 + (entropy mod 255)`, so the wheel select bit as
 * entropy yields `1 + select`. Any other @p prev steps
 * `1 + (prev + (entropy mod 254)) mod 255`.
 *
 * @param prev    session held now; 0 at boot
 * @param entropy changing value, such as a tick count or, at boot, the select bit
 */
uint8_t motor_session_next(uint8_t prev, uint32_t entropy);

#endif
