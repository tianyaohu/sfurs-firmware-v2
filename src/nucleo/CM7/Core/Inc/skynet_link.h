#ifndef SKYNET_LINK_H
#define SKYNET_LINK_H
#include <stdint.h>

/**
 * @file skynet_link.h
 * @brief skynet's serial robot commands (its "base station" ControlWire stream) into the drive mailbox.
 *
 * skynet (sfurs/software, src/network/protocol/ControlWire.cpp) writes one batch per send: for every robot
 * of its team, in ID order 0..N-1 (NetMaster's setRobotId(i, i)), a 7-byte record: CRC-16/CCITT-FALSE of the
 * payload (little-endian) followed by the 5-byte payload
 *   byte 0: bits 0-3 robot ID, bit 4 dribbler, bits 5-7 zero
 *   byte 1: bit 0 chipper, bits 1-4 kick power, bits 5-7 zero
 *   bytes 2-4: forward, left, CCW rotational velocity, int8, 127 = ::SKYNET_LINK_FULL_SCALE (m/s, rad/s)
 * There is no frame marker, and a dropped byte can make a window of two neighbouring records pass the CRC.
 * So a command is applied only from a complete batch: a contiguous run of valid records whose IDs count up
 * from 0, closed by the 0 that starts the next batch. Any invalid window, out-of-order ID or receive gap of
 * ::SKYNET_LINK_GAP_MS discards the batch in progress. The ISR stamps every byte; bytes older than
 * ::SKYNET_LINK_TIMEOUT_MS when decoded are dropped.
 *
 * The record whose ID is ::skynet_link_robot_id becomes the body twist: forward -> +vy, left -> -vx
 * (firmware body frame: x right, y forward), rotation -> omega, and sets ik_drive_enable and bumps
 * ik_drive_heartbeat, so ik_drive.c's arming, slew, fault and heartbeat stop all apply.
 *
 * Takeover: the link starts driving only while the drive is IK_STOPPED in speed mode, so it never inherits a
 * current-mode diagnostic or an SWD host's session (::skynet_link_refused counts the refusals).
 * Stop: ::SKYNET_LINK_TIMEOUT_MS without a command zeroes the twist and releases the drive (enable 0).
 * Faults: a drive fault while the link drives latches ::skynet_link_fault and is left latched in ik_drive
 * (enable stays 1, so the robot stays stopped and the fault visible) until an operator acknowledges it:
 * the Nucleo user button or a bump of ::skynet_link_ack_seq over SWD. Silence never acknowledges a fault.
 *
 * Bytes come from the UART interrupt through skynet_link_isr_byte() (single producer) and are decoded in the
 * main loop by skynet_link_poll() (single consumer), before ik_drive_tick().
 */

#define SKYNET_LINK_ABI 0x534B0002U   /**< ABI tag the host checks before reading the counters */
#define SKYNET_LINK_BAUD 250000U      /**< skynet's default robot/baud_rate */
#define SKYNET_LINK_RECORD 7U         /**< CRC (2) + payload (5) */
#define SKYNET_LINK_MAX_ROBOTS 16U    /**< 4-bit robot ID */
#define SKYNET_LINK_FULL_SCALE 6.0f   /**< skynet's kMaxVelocityPhysical: int8 127 = 6 m/s or 6 rad/s */
#define SKYNET_LINK_TIMEOUT_MS 300U   /**< = IK_DRIVE_HEARTBEAT_MS; no command this long ends link control */
#define SKYNET_LINK_GAP_MS 20U        /**< silence that ends a batch: one 16-robot batch is 4.5 ms at 250 kBd */
#define SKYNET_LINK_RING 512U         /**< ISR-to-main byte ring, power of two (> 4 batches of 16 robots) */

/** @brief ABI tag, equals ::SKYNET_LINK_ABI. */
extern volatile const uint32_t skynet_link_abi;
extern volatile uint32_t skynet_link_robot_id;  /**< this robot's skynet ID (0-15); records for others are ignored */
extern volatile uint32_t skynet_link_ack_seq;   /**< bump over SWD to acknowledge ::skynet_link_fault */
extern volatile uint32_t skynet_link_bytes;     /**< bytes decoded */
extern volatile uint32_t skynet_link_records;   /**< CRC-valid records, any robot */
extern volatile uint32_t skynet_link_batches;   /**< complete batches */
extern volatile uint32_t skynet_link_commands;  /**< commands for this robot applied to the mailbox */
extern volatile uint32_t skynet_link_skipped;   /**< windows discarded while searching for a valid record */
extern volatile uint32_t skynet_link_broken;    /**< batches discarded: invalid window, ID out of order or gap */
extern volatile uint32_t skynet_link_stale;     /**< bytes dropped as older than ::SKYNET_LINK_TIMEOUT_MS */
extern volatile uint32_t skynet_link_overruns;  /**< bytes lost: ring full or UART overrun */
extern volatile uint32_t skynet_link_refused;   /**< commands not applied: drive busy or not in speed mode */
extern volatile uint32_t skynet_link_active;    /**< 1 while the link drives the mailbox */
extern volatile uint32_t skynet_link_fault;     /**< ik_drive_fault bits latched while the link drove; 0 = none */
extern volatile uint32_t skynet_link_last_ms;   /**< Nucleo time of the last applied command, ms */

/** @brief One received byte and its arrival time (HAL tick, ms); interrupt context, the only producer. */
void skynet_link_isr_byte(uint8_t b, uint32_t now);

/** @brief Count a byte lost in hardware (UART overrun); interrupt context. */
void skynet_link_isr_overrun(void);

/**
 * @brief Decode queued bytes and apply this robot's latest complete command; main loop, before ik_drive_tick().
 * @param now     Nucleo time, ms
 * @param ack     nonzero while the operator's acknowledge input (user button) is pressed
 */
void skynet_link_poll(uint32_t now, int ack);

/** @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), as skynet computes it. */
uint16_t skynet_link_crc16(const uint8_t *data, uint32_t len);
#endif
