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
  ( ((uint32_t)((p)[0]) << 24) | \
    ((uint32_t)((p)[1]) << 16) | \
    ((uint32_t)((p)[2]) << 8)  | \
    ((uint32_t)((p)[3])) )

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
#define TORQUE_WRITE (3U << 7)
#define MOTOR_STOP   (15U << 7)

#endif
