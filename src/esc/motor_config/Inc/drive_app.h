#ifndef DRIVE_APP_H
#define DRIVE_APP_H
#include <stdint.h>

/** @file drive_app.h
 * @brief Wheel drive application on top of MCSDK (protocol: can_datatypes.h).
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

/** @brief Application state, readable over SWD for diagnostics. */
struct drive_app_state {
  uint32_t now_ms;          /**< application time, ms */
  uint32_t last_command_ms; /**< time of the last SPEED_WRITE, ms */
  uint32_t active;          /**< motor started by SPEED_WRITE, not yet stopped */
  uint32_t aligned;         /**< Hall shift built in or measured: driving allowed */
  uint32_t fault_latched;   /**< a fault while driving latched the motor off */
  uint32_t stop_reason;     /**< ESC_STOP_* */
  uint32_t derated;         /**< I2T has lowered the current limit */
  float i2t_a2s;            /**< I2T accumulator, A^2*s */
  int32_t iq_limit_counts;  /**< current speed-loop output limit */
  int32_t iq_max_counts;    /**< configured (non-derated) limit */
  uint32_t align_phase;     /**< Hall alignment sequencer phase */
  uint32_t align_result;    /**< Hall alignment result code */
  uint32_t align_edges_fwd; /**< Hall edges counted in the forward sweep */
  uint32_t align_edges_back; /**< Hall edges counted in the backward sweep */
  uint32_t align_wrong_dir; /**< edges that arrived in the wrong direction */
  int32_t align_error_mean; /**< s16 degrees added to the previous shift */
  uint32_t capture_phase;   /**< current-loop capture sequencer phase */
  uint32_t capture_status;  /**< ESC_CAPTURE_* */
  int32_t ff_static_counts; /**< friction feed-forward at standstill, Iq counts */
  int32_t ff_kinetic_counts; /**< friction feed-forward once moving, Iq counts */
  int32_t command_rpm;      /**< last SPEED_WRITE target */
  uint32_t park_ms;         /**< time at zero command and standstill */
  uint32_t speed_commands;  /**< SPEED_WRITE frames accepted */
  uint32_t status_replies;  /**< status replies sent */
  uint32_t rejected;        /**< commands rejected as invalid or not allowed */
  uint32_t start_failed;    /**< ESC_STOP_START_FAILED latch */
};
/** @brief The application state. */
extern volatile struct drive_app_state drive_app;

/** Forced electrical angle for Hall alignment and capture, read by
 * FOC_CurrControllerM1: nonzero @c drive_align_force overrides the Hall angle. */
extern volatile uint8_t drive_align_force;
extern volatile int16_t drive_align_angle; /**< forced electrical angle, s16 degrees */

/** @brief Current-loop capture hook, called from FOC_CurrControllerM1 every FOC period.
 *  @param[in,out] id_ref Id reference; may be replaced before the PI controllers. */
void drive_capture_isr_ref(int16_t *id_ref);
/** @brief Record one FOC period: measured Id/Iq, applied Vd/Vq after circle
 *  limitation, the Id reference and the electrical angle. */
void drive_capture_isr_store(int16_t id, int16_t iq, int16_t vd, int16_t vq,
                             int16_t id_ref, int16_t el_angle);

/** @brief Speed-mode Iq reference = PI output + friction feed-forward.
 *
 * Called from FOC_CalcCurrRef (medium-frequency task).
 * @param pi_out  speed PI output, Iq counts
 * @param ref_rpm ramped speed reference, RPM
 * @param rpm     measured speed, RPM
 * @return Iq reference, counts, clamped to the current limit */
int16_t drive_speed_iq_ref(int16_t pi_out, int16_t ref_rpm, int16_t rpm);

/** @brief Cap an implausible measured speed.
 *
 * Called from FOC_CalcCurrRef before the speed PI: while the Hall speed FIFO
 * refills after standstill or a direction change, the estimate is one edge
 * interval, and a single noisy edge reads 4-5x the true speed.
 * @param[in,out] rpm measured speed, capped in place (also what telemetry reports)
 * @param ref_rpm     speed reference, RPM */
void drive_speed_plausible(int16_t *rpm, int16_t ref_rpm);
#include "drive_speed_ctrl.h"
/** @brief Speed loop for STC_CalcTorqueReference.
 *
 * PI_Controller, or with the startup loop enabled (ESC_CFG_DSC_ENABLE) the
 * complete Iq reference from dsc_step().
 * @return Iq reference, counts */
int16_t drive_speed_pi(PID_Handle_t *pid, int16_t ref_rpm, int16_t rpm);
/** @brief Nonzero when the startup loop is on; FOC_CalcCurrRef then skips ::drive_speed_iq_ref. */
int drive_dsc_enabled(void);
extern dsc_cfg_t drive_dsc_cfg;  /**< startup-loop configuration */
extern dsc_state_t drive_dsc;    /**< startup-loop state */
extern dsc_rev_t drive_rev;      /**< speed reversal filter state */
extern uint8_t drive_rev_enable; /**< ESC_CFG_SPEED_REV_FILTER */
extern uint8_t drive_edge_speed; /**< ESC_CFG_SPEED_EDGE_EST */

/** @brief Speed-loop recorder (ESC_RECORD_START), called from FOC_CalcCurrRef
 * every speed tick with the speed-loop output and the final Iq reference. */
void drive_record_tick(int16_t pi_out, int16_t iq_ref);

/** @brief Initialise the application state. */
void drive_app_init(void);
/** @brief FDCAN ISR entry: latch one received command frame. */
void drive_app_can_rx(uint32_t id, const uint8_t *data, uint32_t length);
/** @brief Main-loop entry: send pending replies.
 *  @param send queues an 8-byte frame; returns nonzero once queued. */
void drive_app_poll(int (*send)(uint32_t id, const uint8_t data[8]));
/** @brief 1 kHz tick, from MC_APP_PostMediumFrequencyHook_M1. */
void drive_app_tick(void);
/** @brief Nonzero when nothing is running or queued: safe for boot entry. */
int drive_app_idle(void);
#endif
