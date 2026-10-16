#ifndef IK_TELEMETRY_H
#define IK_TELEMETRY_H
#include <stdint.h>

/* Debugger ABI: four rows in FR/BR/BL/FL (selectors 1/2/4/8) order, every
 * field 32 bits. Read a row between two equal, even generation values.
 * Status and diag replies (can_datatypes.h) refresh separately, each with
 * its own tick and count; count == 0 means that reply never arrived. */
struct ik_telemetry_wheel {
  uint32_t generation;
  int32_t rpm, iq_counts, iq_ref_counts;
  uint32_t state, flags, status_tick, status_count;
  uint32_t vbus_centivolts, faults;
  int32_t hall_shift_deg;
  uint32_t align_result, i2t_percent, diag_tick, diag_count;
};
#define IK_TELEMETRY_WORDS 15U
extern volatile struct ik_telemetry_wheel ik_drive_telem[4];
extern volatile uint32_t ik_drive_telem_now;

/* Call every loop. Returns a STATUS_READ or DIAG_READ id (send 4 zero bytes)
 * once per IK_TELEMETRY_POLL_MS, or 0. Each wheel gets three status polls
 * per diag poll. */
#define IK_TELEMETRY_POLL_MS 5U
uint32_t ik_telemetry_poll_due(uint32_t now);
/* Caller checks standard ID, data frame and DLC 8. Returns 1 if consumed. */
int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now);
#endif
