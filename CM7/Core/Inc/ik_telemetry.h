#ifndef IK_TELEMETRY_H
#define IK_TELEMETRY_H
#include <stdint.h>

/* Debugger ABI: four rows in FR/BR/BL/FL (selectors 1/2/4/8) order, every
 * field 32 bits. Read a row between two equal, even generation values.
 * Status, diag and ident replies (can_datatypes.h) refresh separately, each
 * with its own tick and count; count == 0 means that reply never arrived. */
struct ik_telemetry_wheel {
  uint32_t generation;
  int32_t rpm, iq_counts, iq_ref_counts;
  uint32_t state, flags, status_tick, status_count;
  uint32_t vbus_centivolts, faults;
  int32_t hall_shift_deg;
  uint32_t align_result, i2t_percent, diag_tick, diag_count;
  uint32_t app_crc, app_size, ident_tick, ident_count;
};
#define IK_TELEMETRY_WORDS 19U
extern volatile struct ik_telemetry_wheel ik_drive_telem[4];
extern volatile uint32_t ik_drive_telem_now;

/* CAPTURE_REPLY frames from any wheel land here by sample index; the host
 * requests one wheel's readout at a time (raw CAPTURE frames) and counts
 * frames to know when its chunk has arrived. */
#define IK_CAPTURE_SAMPLES 512U
struct ik_capture {
  uint32_t frames;  /* CAPTURE_REPLY frames stored */
  uint32_t source;  /* selector of the last frame */
  uint32_t status, vbus_centivolts, samples; /* info bank */
  int16_t sample[IK_CAPTURE_SAMPLES][6]; /* Id, Iq, Vd, Vq, Id ref, angle */
};
extern volatile struct ik_capture ik_drive_capture;

/* Call every loop. Returns a STATUS_READ, DIAG_READ or IDENT_READ id (send 4
 * zero bytes) once per IK_TELEMETRY_POLL_MS, or 0. A 17-slot cycle: three
 * status polls and one diag poll per wheel, then one ident poll for the next
 * wheel in turn. */
#define IK_TELEMETRY_POLL_MS 5U
#define IK_TELEMETRY_SLOTS 17U
uint32_t ik_telemetry_poll_due(uint32_t now);
/* Caller checks standard ID, data frame and DLC 8. Returns 1 if consumed. */
int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now);
#endif
