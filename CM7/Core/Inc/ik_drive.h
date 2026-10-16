#ifndef IK_DRIVE_H
#define IK_DRIVE_H
#include <stdint.h>

#define IK_DRIVE_ABI 0x494B0002U
#define IK_DRIVE_MAX_LINEAR_MMPS 5000
enum { IK_PREPARE = 1, IK_START = 2, IK_STOP = 3 };
enum { IK_IDLE, IK_BUSY, IK_RUNNING, IK_STOPPING, IK_FAULT };
enum { IK_NONE, IK_SENT, IK_REJECTED, IK_TX_FAULT, IK_BAD_TARGET };

/* Exclusive IK build only: no concurrent wheel_bench/CAL/FWU pumps.
 * Write target (mm/s, mrad/s), request, then sequence LAST (commit word).
 * Firmware never clears request; a new nonzero sequence publishes a command.
 * Exactly one outstanding command; STOP may supersede it.
 * completed == sequence acknowledges completion; result is bus delivery,
 * NOT ESC acceptance or motion. All words are 32 bit for debugger access.
 * PREPARE sends ARM-only for existing auto-CAL; never starts a motor.
 * START computes IK and sends ARM then nonzero SPEED per wheel, from IDLE
 * only. Running updates are deliberately rejected (ESCs require IDLE).
 * STOP uses existing 3 broadcast attempts; there is no new watchdog/lease. */
extern volatile const uint32_t ik_drive_abi;
extern volatile int32_t ik_drive_vx_mmps, ik_drive_vy_mmps, ik_drive_omega_mradps;
extern volatile uint32_t ik_drive_request, ik_drive_sequence, ik_drive_completed;
extern volatile uint32_t ik_drive_state, ik_drive_result;
extern volatile int32_t ik_drive_rpm[4]; /* raw CAN motor RPM FR/BR/BL/FL */
extern volatile uint32_t ik_drive_tx_count, ik_drive_ack_count, ik_drive_fault;
void ik_drive_tick(uint32_t now);
/* For read-only telemetry requests: false while drive TX, a newly committed
 * command, ARM/SPEED burst or STOP drain owns the transport. */
int ik_drive_transport_idle(void);
#endif
