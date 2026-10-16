#ifndef DRIVE_APP_H
#define DRIVE_APP_H
#include <stdint.h>

/* Wheel drive application on top of MCSDK (protocol: can_datatypes.h).
 *
 * Contexts: drive_app_can_rx() runs in the FDCAN ISR and only latches
 * commands. All motor-control calls happen in the 1 kHz MCSDK medium
 * frequency hook. drive_app_poll() runs in the main loop and sends replies.
 *
 * Protections, in addition to MCSDK hardware over-current, over/under
 * voltage and over-temperature:
 *  - command watchdog: no SPEED_WRITE for ESC_COMMAND_TIMEOUT_MS -> stop;
 *  - I2T: sustained current above nominal derates the current limit;
 *  - a fault while driving latches off until STOP or a command gap. */

struct drive_app_state {
  uint32_t now_ms, last_command_ms;
  uint32_t active;          /* motor started by SPEED_WRITE, not yet stopped */
  uint32_t aligned;         /* Hall shift built in or measured: driving allowed */
  uint32_t fault_latched;
  uint32_t stop_reason;     /* ESC_STOP_* */
  uint32_t derated;
  float i2t_a2s;
  int32_t iq_limit_counts;  /* current speed-loop output limit */
  int32_t iq_max_counts;    /* configured (non-derated) limit */
  uint32_t align_phase, align_result;
  uint32_t align_edges_fwd, align_edges_back, align_wrong_dir;
  int32_t align_error_mean; /* s16 degrees added to the previous shift */
  uint32_t capture_phase, capture_status; /* ESC_CAPTURE_* */
  uint32_t speed_commands, status_replies, rejected;
};
extern volatile struct drive_app_state drive_app;

/* Forced electrical angle for Hall alignment and capture, read by
 * FOC_CurrControllerM1. */
extern volatile uint8_t drive_align_force;
extern volatile int16_t drive_align_angle;

/* Current-loop capture, called from FOC_CurrControllerM1 every FOC period:
 * _ref may replace the Id reference before the PI controllers, _store records
 * the period (measured Iqd, applied Vqd after circle limitation). */
void drive_capture_isr_ref(int16_t *id_ref);
void drive_capture_isr_store(int16_t id, int16_t iq, int16_t vd, int16_t vq,
                             int16_t id_ref, int16_t el_angle);

void drive_app_init(void);
void drive_app_can_rx(uint32_t id, const uint8_t *data, uint32_t length);
/* send() returns nonzero once the 8-byte frame is queued. */
void drive_app_poll(int (*send)(uint32_t id, const uint8_t data[8]));
void drive_app_tick(void); /* 1 kHz, from MC_APP_PostMediumFrequencyHook_M1 */
int drive_app_idle(void);  /* nothing running or queued: safe for boot entry */
#endif
