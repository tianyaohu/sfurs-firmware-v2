#ifndef IK_TELEMETRY_H
#define IK_TELEMETRY_H
#include <stdint.h>

/* Debugger ABI: four rows in FR/BR/BL/FL (selectors 1/2/4/8) order.
 * All fields exactly 32 bits. Snapshot generation before/after a block read;
 * accept only equal, even generations. Counts == 0 means no reply received.
 * RPM/state and current have independent timestamps; they are not synchronous.
 * Flags: 0x8000 profile, 0x4000 armed, 0x2000 ever started, 0x1000 active,
 * 0x0800 CAL valid; low 0x07ff current faults. Raw currents are SDK counts.
 * Existing ESC profile scale is 544 counts/A; validity/scaling is a separate
 * interpretation, not proof of calibrated physical phase or battery current.
 * No voltage, Iq-reference or stop-reason field exists in these CAN packets. */
struct ik_telemetry_wheel {
  uint32_t generation;
  int32_t rpm;
  uint32_t state, hall, hall_changes, speed_tick;
  int32_t iq_counts, id_counts;
  uint32_t flags, peak_l1_counts, current_tick, speed_count, current_count;
};
extern volatile struct ik_telemetry_wheel ik_drive_telem[4];
extern volatile uint32_t ik_drive_telem_now;

/* Call every loop. Poll requests are round-robin, one per >=20 ms, only
 * when allowed (drive owns no TX and is not BUSY/STOPPING). IDs returned are
 * SPEED_READ 0x100|selector and TEST_CURRENT_READ 0x300|selector, alternating
 * per wheel; 4 zero bytes. Return 0 when not due. No drive commands/retries. */
uint32_t ik_telemetry_poll_due(uint32_t now, int allowed);
/* Caller checks standard ID, data frame, FD no BRS and exact DLC 8 first.
 * Returns 1 for accepted speed/current reply, 0 for unknown/wrong selectors. */
int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now);
#endif
