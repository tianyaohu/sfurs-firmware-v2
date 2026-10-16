#ifndef IK_DRIVE_H
#define IK_DRIVE_H
#include <stdint.h>

/* Streaming omni drive. The host (scripts/drive.py over SWD) writes a body
 * twist and bumps heartbeat at >=10 Hz. Every IK_DRIVE_PERIOD_MS the Nucleo
 * runs the IK, slews each wheel toward its target at a common acceleration
 * limit (keeping the twist direction), and sends SPEED_WRITE to all four ESCs.
 * enable == 0 or a heartbeat older than IK_DRIVE_HEARTBEAT_MS decelerates to
 * zero, holds briefly, then broadcasts STOP. ESCs stop on their own if the
 * stream stops (ESC_COMMAND_TIMEOUT_MS). All words are 32 bit for SWD access. */
#define IK_DRIVE_ABI 0x494B0003U
#define IK_DRIVE_PERIOD_MS 20U
#define IK_DRIVE_HEARTBEAT_MS 300U
#define IK_DRIVE_HOLD_MS 200U
#define IK_DRIVE_STOP_FRAMES 3U
#define IK_DRIVE_DEFAULT_ACCEL_RPMPS 900
#define IK_DRIVE_MAX_LINEAR_MMPS 5000
#define IK_DRIVE_MAX_OMEGA_MRADPS 6000
enum { IK_STOPPED, IK_DRIVING, IK_STOPPING };

extern volatile const uint32_t ik_drive_abi;
/* Host -> firmware */
extern volatile int32_t ik_drive_vx_mmps, ik_drive_vy_mmps, ik_drive_omega_mradps;
extern volatile uint32_t ik_drive_enable, ik_drive_heartbeat;
extern volatile int32_t ik_drive_accel_rpmps; /* <= 0 selects the default */
/* One-shot raw 4-byte frame (HALL_ALIGN, CONFIG_SET, ...): write id and data,
 * then bump raw_seq. Sent only while IK_STOPPED; raw_done echoes raw_seq. */
extern volatile uint32_t ik_drive_raw_id, ik_drive_raw_data, ik_drive_raw_seq;
/* Firmware -> host */
extern volatile uint32_t ik_drive_state, ik_drive_raw_done;
extern volatile int32_t ik_drive_target_rpm[4]; /* FR/BR/BL/FL raw motor RPM */
extern volatile int32_t ik_drive_rpm[4];        /* slewed, last sent */
extern volatile uint32_t ik_drive_tx_count, ik_drive_tx_fail, ik_drive_bad_input;

/* send() queues one 4-byte frame; returns nonzero on success. */
void ik_drive_tick(uint32_t now, int (*send)(uint32_t id, const uint8_t data[4]));
#endif
