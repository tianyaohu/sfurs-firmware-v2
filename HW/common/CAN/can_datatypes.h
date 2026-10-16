/*
 * can_datatypes.h
 * CAN protocol shared by the motor controllers (B-G431B-ESC1) and the Nucleo.
 *
 * 11-bit ID = command (top 4 bits) | device select bits (bottom 7 bits).
 * Example: 0b000 1000 1000 is SPEED_WRITE to MOTOR_NW; 0x08F addresses all.
 * Frames are CAN FD format without BRS. Commands carry 4 bytes; replies 8.
 * All multi-byte fields are little endian.
 */
#ifndef CAN_DATATYPES_H
#define CAN_DATATYPES_H

#define CAN_U32_FROM_PAYLOAD(p) \
  ( ((uint32_t)((p)[0]))        | \
    ((uint32_t)((p)[1]) << 8)   | \
    ((uint32_t)((p)[2]) << 16)  | \
    ((uint32_t)((p)[3]) << 24) )

#define CAN_PAYLOAD_FROM_U32(p, v)         \
  do                                       \
  {                                        \
    (p)[0] = (uint8_t)((v) & 0xFFU);       \
    (p)[1] = (uint8_t)(((v) >> 8) & 0xFFU);  \
    (p)[2] = (uint8_t)(((v) >> 16) & 0xFFU); \
    (p)[3] = (uint8_t)(((v) >> 24) & 0xFFU); \
  } while (0)

#define CAN_COMMAND_FROM_ID(p) ((uint32_t)(p) & 0b11110000000)

/* Device select bits */
#define NUCLEO_CAN_ID   (0U)
#define MOTOR_NE_CAN_ID (1U << 0)
#define MOTOR_SE_CAN_ID (1U << 1)
#define MOTOR_SW_CAN_ID (1U << 2)
#define MOTOR_NW_CAN_ID (1U << 3)

/* SPEED_WRITE: int16 target motor RPM, uint16 ramp ms. Starts the motor from
 * IDLE and updates it while running. The motor stops itself if no
 * SPEED_WRITE arrives for ESC_COMMAND_TIMEOUT_MS, so senders stream it.
 * Ignored until the Hall phase shift is known (ESC_FLAG_ALIGNED): a wrong
 * shift reverses torque and the speed loop would run away. */
#define SPEED_WRITE (1U << 7)
#define ESC_COMMAND_TIMEOUT_MS 250U

/* STATUS_READ (any payload) -> STATUS_REPLY:
 * int16 RPM, int16 Iq, int16 Iq reference (SDK counts, ESC_COUNTS_PER_AMP),
 * uint8 MCSDK state, uint8 flags (ESC_FLAG_*, low nibble = last ESC_STOP_*). */
#define STATUS_READ  (2U << 7)
#define STATUS_REPLY (5U << 7)
/* DIAG_READ (any payload) -> DIAG_REPLY:
 * uint16 Vbus centivolts, uint16 occurred MCSDK faults,
 * int16 Hall phase shift degrees, uint8 align result (ESC_ALIGN_*),
 * uint8 I2T load percent. */
#define DIAG_READ    (6U << 7)
#define DIAG_REPLY   (7U << 7)
/* HALL_ALIGN: payload "HALN". From IDLE only, the wheel turns ~1/4 rev
 * forward and back under forced-angle current, then the measured Hall phase
 * shift is applied in RAM (lost on reset). Read the result with DIAG_READ. */
#define HALL_ALIGN   (9U << 7)
#define HALL_ALIGN_MAGIC 0x4E4C4148U /* "HALN" */
/* CONFIG_SET: uint16 key (ESC_CFG_*), int16 value. IDLE only, RAM only. */
#define CONFIG_SET   (10U << 7)
#define MOTOR_STOP   (15U << 7)

#define ESC_COUNTS_PER_AMP 544

enum { ESC_CFG_SPEED_KP = 1, ESC_CFG_SPEED_KI = 2, ESC_CFG_HALL_SHIFT_DEG = 3,
       ESC_CFG_IQ_MAX_MA = 4 };
enum { ESC_STOP_NONE = 0, ESC_STOP_COMMAND = 1, ESC_STOP_TIMEOUT = 2,
       ESC_STOP_FAULT = 3, ESC_STOP_ALIGN = 4 };
#define ESC_FLAG_ACTIVE  0x10U /* streaming speed commands accepted */
#define ESC_FLAG_DERATED 0x20U /* I2T limit reached, current derated */
#define ESC_FLAG_ALIGN   0x40U /* Hall alignment running */
#define ESC_FLAG_ALIGNED 0x80U /* Hall shift known: built in or measured this boot */
enum { ESC_ALIGN_NONE = 0, ESC_ALIGN_OK = 1, ESC_ALIGN_REVERSED = 2,
       ESC_ALIGN_NO_EDGES = 3, ESC_ALIGN_ABORTED = 4, ESC_ALIGN_RUNNING = 5 };

#endif
