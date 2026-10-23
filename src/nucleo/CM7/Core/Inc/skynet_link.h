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
 * from 0, closed by the 0 that starts the next batch or by ::SKYNET_LINK_GAP_MS of silence on a record
 * boundary, and as long as the previous batch (skynet's team size is fixed for a run). Any invalid window,
 * out-of-order ID, gap inside a record or UART/ring overrun discards the batch in progress. The ISR stamps
 * every byte; a byte or command older than ::SKYNET_LINK_MAX_AGE_MS is dropped, and the link timeout counts
 * from the command's arrival, not from when the main loop got to it.
 *
 * The record whose ID is ::skynet_link_robot_id becomes the body twist: forward -> +vy, left -> -vx
 * (firmware body frame: x right, y forward), rotation -> omega, and sets ik_drive_enable and bumps
 * ik_drive_heartbeat, so ik_drive.c's arming, slew, fault and heartbeat stop all apply.
 *
 * Ownership: the link takes over only an IK_STOPPED, released (enable 0) drive in speed mode, so it never
 * inherits a current-mode diagnostic or another host's session (::skynet_link_refused counts refusals).
 * While it owns the drive it checks that the mailbox still holds what it wrote; any other writer (twist,
 * enable or heartbeat) gets the drive and ::SKYNET_LINK_FAULT_EXTERNAL latches the link out.
 * Stop: ::SKYNET_LINK_TIMEOUT_MS without a command zeroes the twist and clears enable; the link keeps
 * ownership until the drive is IK_STOPPED, so a skynet that resumes meanwhile drives again at once.
 * Faults: a drive fault while the link owns the drive latches ::skynet_link_fault and is left latched in
 * ik_drive (enable stays 1, so the robot stays stopped and the fault visible) until an operator
 * acknowledges it after the fault: the Nucleo user button (debounced) or a bump of ::skynet_link_ack_seq
 * over SWD. Silence never acknowledges a fault.
 *
 * Bytes come from the UART interrupt through skynet_link_isr_byte() (single producer) and are decoded in the
 * main loop by skynet_link_poll() (single consumer), before ik_drive_tick().
 */

#define SKYNET_LINK_ABI 0x534B0003U   /**< ABI tag the host checks before reading the counters */
#define SKYNET_LINK_BAUD 250000U      /**< skynet's default robot/baud_rate */
#define SKYNET_LINK_RECORD 7U         /**< CRC (2) + payload (5) */
#define SKYNET_LINK_MAX_ROBOTS 16U    /**< 4-bit robot ID */
#define SKYNET_LINK_FULL_SCALE 6.0f   /**< skynet's kMaxVelocityPhysical: int8 127 = 6 m/s or 6 rad/s */
#define SKYNET_LINK_TIMEOUT_MS 300U   /**< = IK_DRIVE_HEARTBEAT_MS; no command this long ends link control */
#define SKYNET_LINK_GAP_MS 20U        /**< silence that ends a batch: one 16-robot batch is 4.5 ms at 250 kBd */
#define SKYNET_LINK_MAX_AGE_MS 50U    /**< oldest byte or command applied: gap + a batch, with margin */
#define SKYNET_LINK_DEBOUNCE_MS 20U   /**< B1 must be stable this long released, then pressed */
#define SKYNET_LINK_FAULT_EXTERNAL 0x80000000U /**< ::skynet_link_fault bit: another host wrote the mailbox */
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
extern volatile uint32_t skynet_link_broken;    /**< batches discarded: damaged, cut, or length unlike the last */
extern volatile uint32_t skynet_link_stale;     /**< bytes or commands dropped as older than ::SKYNET_LINK_MAX_AGE_MS */
extern volatile uint32_t skynet_link_overruns;  /**< bytes lost: ring full or UART overrun */
extern volatile uint32_t skynet_link_refused;   /**< commands not applied: drive held elsewhere or not in speed mode */
extern volatile uint32_t skynet_link_active;    /**< 1 while the link drives the mailbox */
extern volatile uint32_t skynet_link_fault;     /**< ik_drive_fault bits latched while the link drove, plus
                                                     ::SKYNET_LINK_FAULT_EXTERNAL; 0 = none */
extern volatile uint32_t skynet_link_last_ms;   /**< arrival time of the last applied command, ms */

/** @brief One received byte and its arrival time (HAL tick, ms); interrupt context, the only producer. */
void skynet_link_isr_byte(uint8_t b, uint32_t now);

/** @brief Count a byte lost in hardware (UART overrun); interrupt context. */
void skynet_link_isr_overrun(void);

/**
 * @brief Decode queued bytes and apply this robot's latest complete command; main loop, before ik_drive_tick().
 * @param now     Nucleo time, ms
 * @param button  raw level of the operator's acknowledge input (B1), nonzero while pressed; debounced here
 */
void skynet_link_poll(uint32_t now, int button);

/** @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), as skynet computes it. */
uint16_t skynet_link_crc16(const uint8_t *data, uint32_t len);
#endif
