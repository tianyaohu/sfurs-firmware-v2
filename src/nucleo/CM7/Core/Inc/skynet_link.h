#ifndef SKYNET_LINK_H
#define SKYNET_LINK_H
#include <stdint.h>

/**
 * @file skynet_link.h
 * @brief skynet's serial robot commands (its "base station" ControlWire stream) into the drive mailbox.
 *
 * skynet (sfurs/software, src/network/protocol/ControlWire.cpp) writes one batch per send: for every robot
 * of its team a 7-byte record, CRC-16/CCITT-FALSE of the payload (little-endian) followed by the 5-byte
 * payload. There is no frame marker, so records are found by their CRC and reserved bits; after one valid
 * record the stream stays aligned. Payload:
 *   byte 0: bits 0-3 robot ID, bit 4 dribbler, bits 5-7 zero
 *   byte 1: bit 0 chipper, bits 1-4 kick power, bits 5-7 zero
 *   bytes 2-4: forward, left, CCW rotational velocity, int8, 127 = ::SKYNET_LINK_FULL_SCALE (m/s, rad/s)
 *
 * The record whose ID is ::skynet_link_robot_id becomes the body twist: forward -> +vy, left -> -vx
 * (firmware body frame: x right, y forward), rotation -> omega. Each such record sets ik_drive_enable and
 * bumps ik_drive_heartbeat, so ik_drive.c's own heartbeat timeout stops the robot when skynet goes quiet.
 * After ::SKYNET_LINK_TIMEOUT_MS without a matching record the link zeroes the twist and clears enable
 * once (which also acknowledges a latched drive fault), so a restarted stream drives again. Only one
 * command source at a time: the link writes the mailbox only while records for this robot arrive.
 *
 * Bytes come from the UART interrupt through skynet_link_isr_byte() (single producer) and are decoded in
 * the main loop by skynet_link_poll() (single consumer), next to ik_drive_tick().
 */

#define SKYNET_LINK_ABI 0x534B0001U   /**< ABI tag the host checks before reading the counters */
#define SKYNET_LINK_BAUD 250000U      /**< skynet's default robot/baud_rate */
#define SKYNET_LINK_RECORD 7U         /**< CRC (2) + payload (5) */
#define SKYNET_LINK_FULL_SCALE 6.0f   /**< skynet's kMaxVelocityPhysical: int8 127 = 6 m/s or 6 rad/s */
#define SKYNET_LINK_TIMEOUT_MS 300U   /**< = IK_DRIVE_HEARTBEAT_MS; no record this long ends link control */
#define SKYNET_LINK_RING 512U         /**< ISR-to-main byte ring, power of two (> 4 batches of 6 robots) */

/** @brief ABI tag, equals ::SKYNET_LINK_ABI. */
extern volatile const uint32_t skynet_link_abi;
extern volatile uint32_t skynet_link_robot_id;  /**< this robot's skynet ID (0-15); records for others are ignored */
extern volatile uint32_t skynet_link_bytes;     /**< bytes received */
extern volatile uint32_t skynet_link_records;   /**< valid records, any robot */
extern volatile uint32_t skynet_link_commands;  /**< records for this robot applied to the mailbox */
extern volatile uint32_t skynet_link_skipped;   /**< bytes discarded while searching for a valid record */
extern volatile uint32_t skynet_link_overruns;  /**< bytes lost: ring full or UART overrun */
extern volatile uint32_t skynet_link_active;    /**< 1 while the link is driving the mailbox */
extern volatile uint32_t skynet_link_last_ms;   /**< Nucleo time of the last applied record, ms */

/** @brief One received byte; interrupt context, the only producer. */
void skynet_link_isr_byte(uint8_t b);

/** @brief Count a byte lost in hardware (UART overrun); interrupt context. */
void skynet_link_isr_overrun(void);

/** @brief Decode queued bytes and apply this robot's latest command; main loop, before ik_drive_tick(). */
void skynet_link_poll(uint32_t now);

/** @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), as skynet computes it. */
uint16_t skynet_link_crc16(const uint8_t *data, uint32_t len);
#endif
