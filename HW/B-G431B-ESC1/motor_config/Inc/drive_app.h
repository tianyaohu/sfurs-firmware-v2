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
 *  - a fault while driving latches off until STOP or a command gap.
 *
 * Parking: a wheel commanded to 0 RPM that has been at standstill (below
 * PARK_RPM) for PARK_MS is switched off (coasts, no current) instead of holding zero speed;
 * at standstill the Hall speed estimate is too coarse to regulate on and the
 * PI only buzzes. The next non-zero SPEED_WRITE starts it again (~30 ms).
 *
 * Friction feed-forward: in speed mode the Iq reference is the speed PI
 * output plus sign(speed ref) * ff, where ff is ff_static at standstill and
 * fades linearly to ff_kinetic as the wheel first reaches FRICTION_FADE_RPM,
 * then stays kinetic until a stop command or a stall; clamped to the current
 * limit. It supplies the breakaway current up front so the integrator does
 * not have to wind up to it. */

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
  int32_t ff_static_counts, ff_kinetic_counts;
  int32_t command_rpm;      /* last SPEED_WRITE target */
  uint32_t park_ms;         /* time at zero command and standstill */
  uint32_t speed_commands, status_replies, rejected;
  uint32_t start_failed;    /* ESC_STOP_START_FAILED latch */
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

/* Speed-mode Iq reference = PI output + friction feed-forward. Called from
 * FOC_CalcCurrRef (medium-frequency task) with the ramped speed reference and
 * the measured speed. */
int16_t drive_speed_iq_ref(int16_t pi_out, int16_t ref_rpm, int16_t rpm);

/* Speed loop for STC_CalcTorqueReference: PI_Controller, or with the startup
 * loop enabled (ESC_CFG_DSC_ENABLE) the complete Iq reference from dsc_step().
 * drive_dsc_enabled() tells FOC_CalcCurrRef to skip drive_speed_iq_ref. */
#include "drive_speed_ctrl.h"
int16_t drive_speed_pi(PID_Handle_t *pid, int16_t ref_rpm, int16_t rpm);
int drive_dsc_enabled(void);
extern dsc_cfg_t drive_dsc_cfg;
extern dsc_state_t drive_dsc;
extern dsc_rev_t drive_rev;
extern uint8_t drive_rev_enable; /* ESC_CFG_SPEED_REV_FILTER */
extern uint8_t drive_edge_speed; /* ESC_CFG_SPEED_EDGE_EST */

/* Speed-loop recorder (ESC_RECORD_START), from FOC_CalcCurrRef every speed
 * tick with the speed-loop output and the final Iq reference. */
void drive_record_tick(int16_t pi_out, int16_t iq_ref);

void drive_app_init(void);
void drive_app_can_rx(uint32_t id, const uint8_t *data, uint32_t length);
/* send() returns nonzero once the 8-byte frame is queued. */
void drive_app_poll(int (*send)(uint32_t id, const uint8_t data[8]));
void drive_app_tick(void); /* 1 kHz, from MC_APP_PostMediumFrequencyHook_M1 */
int drive_app_idle(void);  /* nothing running or queued: safe for boot entry */
#endif
