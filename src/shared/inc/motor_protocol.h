#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

/** @file motor_protocol.h
 *  @brief Encode and decode CONTROL v1, STATUS, and DIAG frames.
 *
 *  Stateless wire codec. It does not apply current or speed limits, and it
 *  does not track session or sequence: those need the ESC's own state.
 *  Specification of this slice: architecture/protocol.md.
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
 * Bits 10-4 are the base (0x010). Bits 3-0 are the select. This is not the
 * service-command layout: those commands use bits 10-7 only.
 */
#define MOTOR_CAN_ID_CONTROL 0x010u
#define MOTOR_CAN_ID_BASE_MASK 0x7F0u

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
 * @brief Why a frame was refused.
 *
 * Range, session, and sequence reasons are not produced here.
 */
enum motor_reason {
    MOTOR_R_NONE = 0,
    MOTOR_R_BAD_VERSION = 1,      /**< CONTROL version nibble is not 1 */
    MOTOR_R_BAD_MODE = 2,         /**< CONTROL mode is not a ::motor_mode */
    MOTOR_R_BAD_FLAGS = 3,        /**< ARM or CLEAR_FAULT used together or outside DISABLE */
    MOTOR_R_RESERVED_NONZERO = 4, /**< a reserved flag bit, or a field this mode does not use */
    MOTOR_R_BAD_SELECT = 5,       /**< select bits do not name the wheels this frame may address */
    /* 6..19 are ESC state (range, session, sequence). Not produced here, and not reused. */
    MOTOR_R_BAD_LENGTH = 20       /**< NULL, wrong DLC, or an id this function does not decode */
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

#endif
