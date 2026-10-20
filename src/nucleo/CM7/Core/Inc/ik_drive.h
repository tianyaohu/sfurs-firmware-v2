#ifndef IK_DRIVE_H
#define IK_DRIVE_H
#include <stdint.h>

/**
 * @file ik_drive.h
 * @brief Streaming omni drive: body twist from the host to four ESCs.
 *
 * The host (src/tools/ik_link.py over SWD) writes a body twist and bumps
 * heartbeat at >=10 Hz. Every ::IK_DRIVE_PERIOD_MS the Nucleo runs the IK,
 * slews each wheel toward its target at a common acceleration limit (keeping
 * the twist direction), and sends SPEED_WRITE to all four ESCs.
 * enable == 0 or a heartbeat older than ::IK_DRIVE_HEARTBEAT_MS decelerates to
 * zero, holds briefly, then broadcasts STOP. ESCs stop on their own if the
 * stream stops (ESC_COMMAND_TIMEOUT_MS). All words are 32 bit for SWD access.
 */

#define IK_DRIVE_ABI 0x494B0004U         /**< ABI tag the host checks before talking */
#define IK_DRIVE_PERIOD_MS 20U           /**< drive loop period, ms */
#define IK_DRIVE_HEARTBEAT_MS 300U       /**< heartbeat age that counts as lost, ms */
#define IK_DRIVE_HOLD_MS 200U            /**< time at zero before STOP is sent, ms */
#define IK_DRIVE_STOP_FRAMES 3U          /**< STOP broadcasts per stop */
#define IK_DRIVE_DEFAULT_ACCEL_RPMPS 900 /**< wheel acceleration limit, RPM/s */
#define IK_DRIVE_MAX_LINEAR_MMPS 5000    /**< accepted |vx|, |vy|, mm/s */
#define IK_DRIVE_MAX_OMEGA_MRADPS 6000   /**< accepted |omega|, mrad/s */
/** @brief Values of ::ik_drive_state. */
enum { IK_STOPPED, IK_DRIVING, IK_STOPPING };

/** @brief ABI tag, equals ::IK_DRIVE_ABI. */
extern volatile const uint32_t ik_drive_abi;

/** @name Host to firmware */
/**@{*/
extern volatile int32_t ik_drive_vx_mmps;      /**< body velocity x (right), mm/s */
extern volatile int32_t ik_drive_vy_mmps;      /**< body velocity y (forward), mm/s */
extern volatile int32_t ik_drive_omega_mradps; /**< yaw rate, CCW positive, mrad/s */
extern volatile uint32_t ik_drive_enable;      /**< nonzero allows driving */
extern volatile uint32_t ik_drive_heartbeat;   /**< host bumps this to show it is alive */
extern volatile int32_t ik_drive_accel_rpmps;  /**< wheel acceleration limit, RPM/s; <= 0 selects the default */
/* One-shot raw 4-byte frame (HALL_ALIGN, CONFIG_SET, ...): write id and data,
 * then bump raw_seq. Sent only while IK_STOPPED; raw_done echoes raw_seq. */
extern volatile uint32_t ik_drive_raw_id;   /**< raw frame CAN ID (11 bit) */
extern volatile uint32_t ik_drive_raw_data; /**< raw frame payload as one little-endian word */
extern volatile uint32_t ik_drive_raw_seq;  /**< bump to send; sent only while ::IK_STOPPED, echoed by raw_done */
/**@}*/

/** @name Firmware to host */
/**@{*/
extern volatile uint32_t ik_drive_state;       /**< IK_STOPPED, IK_DRIVING or IK_STOPPING */
extern volatile uint32_t ik_drive_raw_done;    /**< last raw_seq sent */
extern volatile int32_t ik_drive_target_rpm[4]; /**< FR/BR/BL/FL raw motor RPM requested */
extern volatile int32_t ik_drive_rpm[4];       /**< slewed RPM, last sent */
extern volatile uint32_t ik_drive_tx_count;    /**< CAN frames queued */
extern volatile uint32_t ik_drive_tx_fail;     /**< CAN frames that failed to queue */
extern volatile uint32_t ik_drive_bad_input;   /**< twists rejected as out of range */
/**@}*/

/**
 * @brief One step of the drive loop; call every main-loop pass.
 * @param now  Nucleo time, ms.
 * @param send queues one 4-byte CAN frame; returns nonzero on success.
 */
void ik_drive_tick(uint32_t now, int (*send)(uint32_t id, const uint8_t data[4]));
#endif
