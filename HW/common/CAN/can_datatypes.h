/*
 * can_datatypes.h
 * Holds the datatypes common to both the motor controllers and the nucleo.
 * 
 * HOW CAN MESSAGE ID IS DETERMINED
 * In classic CAN, each message ID is 11 bits.
 * We are using the bottom 7 bits as enable bits for CAN devices
 * and the top 4 bits as command bits. For example:
 * 0b000 1000 1000
 * is SPEED_WRITE to MOTOR_NW.
 * When sending a command, the message ID is (COMMAND || (SELECT BITS))
 * example:
 * 0b000 1000 1111
 * sends a SPEED_WRITE to ALL motors
 */
#ifndef CAN_DATATYPES_H
#define CAN_DATATYPES_H

/*
 * Helper Macros
 */
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

/*
 * CAN DEVICE IDs
 */
#define NUCLEO_CAN_ID   (0U)
#define MOTOR_NE_CAN_ID (1U << 0)
#define MOTOR_SE_CAN_ID (1U << 1)
#define MOTOR_SW_CAN_ID (1U << 2)
#define MOTOR_NW_CAN_ID (1U << 3)

/*
 * CAN COMMAND IDs
 */

/*
 * for speed, top 16 bits are time
 * bottom 16 bits are final speed
 */
#define SPEED_WRITE (1U << 7)
#define SPEED_READ  (2U << 7)
#define SPEED_READ_REPLY (5U << 7)
#define TEST_CURRENT_READ (6U << 7)
#define TEST_CURRENT_REPLY (7U << 7)
#define TEST_ARM (8U << 7)
#define TEST_CAL_REQ (9U << 7)
/* Slot 10 (0x500 base) is ESC_TEST_TORQUE_COMMAND in torque test profiles.
 * In RPM2 builds compiled with ESC_TUNE_GAIN_SET it is GAIN_SET instead:
 * payload int16 LE Kp numerator, int16 LE Ki numerator (both negative),
 * addressed per wheel like TEST_ARM. No reply frame; acceptance is proven
 * by ESC RAM readback (test_gain_set_result / test_speed_kp_gain). */
#define TEST_GAIN_SET (10U << 7)
/* Bench profile only: CURRENT_REPLY = i16 iq, i16 id, u16 flags/faults,
 * u16 peak(|iq|+|id|), all little endian SDK current units.
 * flags: bit15 profile, bit14 armed, bit13 used, bit12 active,
 * bit11 offsets calibrated; low11 faults. iq/id are valid only during RUN.
 * TEST_ARM requires the explicit 4-byte magic 0x54455354. */
/* SPEED_READ: 4-byte request. Reply: 8 bytes, little endian:
 * [0..1] signed RPM, [2] motor state, [3] raw Hall bits H1/H2/H3,
 * [4..7] observed Hall-state changes since boot (polling, not an encoder). */
#define TORQUE_WRITE (3U << 7)
#define MOTOR_STOP   (15U << 7)

/*
 * CAN Wrapper Functions
 */
HAL_StatusTypeDef can_motor_write_speed_ramp(FDCAN_HandleTypeDef* can_pointer,
		FDCAN_TxHeaderTypeDef* tx_header,
		uint8_t* tx_data,
		uint8_t* motor_id,
		int32_t speed,
		uint32_t time);

#endif
