#ifndef DRIVE_APP_H
#define DRIVE_APP_H
#include <stdint.h>
#include "hall_angle.h"
#include "motor_cmd.h"
#include "motor_speed.h"

/** @file drive_app.h
 * @brief Wheel drive application on top of MCSDK (protocols: motor_protocol.h, can_datatypes.h).
 *
 * Contexts: drive_app_can_rx() runs in the FDCAN ISR and only queues frames.
 * All motor-control calls happen in the 1 kHz MCSDK medium frequency hook.
 * drive_app_poll() runs in the main loop and sends replies.
 *
 * Motion is commanded by protocol v1 CONTROL frames. Their lifecycle (arming,
 * session and sequence checks, watchdog, faults, HOLD) lives in ::drive_cmd
 * (motor_cmd.c) and the speed loop in ::drive_speed (motor_speed.c); this file
 * executes their decisions on MCSDK, in torque mode: the speed loop's q-current
 * goes to MC_SetCurrentReferenceMotor1 every 1 ms. Protections, in addition to MCSDK hardware over-current,
 * over/under voltage and over-temperature:
 *  - command watchdog (motor_cmd): no fresh CONTROL frame for MOTOR_WATCHDOG_MS -> off, disarmed;
 *  - I2T: sustained current above nominal derates the current limit;
 *  - stall (motor_speed): high current without Hall progress -> FAULTED(STALL);
 *  - Hall loss: invalid Hall code past the stale window, or MCSDK's speed-feedback fault -> FAULTED(HALL_LOST);
 *  - any fault latches the wheel off until DISABLE + CLEAR_FAULT and a new ARM. */

/** @brief Application state, readable over SWD for diagnostics. */
struct drive_app_state {
  uint32_t now_ms;          /**< application time, ms (first field: tools resolve it by symbol) */
  uint32_t active;          /**< output on under the command lifecycle (STARTING, RUNNING, HOLD) */
  uint32_t aligned;         /**< Hall shift built in or measured: driving allowed */
  uint32_t stop_reason;     /**< ESC_STOP_* of the last switch-off */
  uint32_t derated;         /**< I2T has lowered the current limit */
  float i2t_a2s;            /**< I2T accumulator, A^2*s */
  int32_t iq_limit_counts;  /**< speed-loop current limit L (configured or I2T-derated), counts */
  int32_t iq_max_counts;    /**< configured (non-derated) limit, counts */
  int32_t iq_max_ma;        /**< configured (non-derated) limit as set over CONFIG, mA */
  uint32_t align_phase;     /**< Hall alignment sequencer phase */
  uint32_t align_result;    /**< Hall alignment result code */
  uint32_t align_edges_fwd; /**< Hall edges counted in the forward sweep */
  uint32_t align_edges_back; /**< Hall edges counted in the backward sweep */
  uint32_t align_wrong_dir; /**< edges that arrived in the wrong direction */
  int32_t align_error_mean; /**< s16 degrees added to the previous shift */
  uint32_t capture_phase;   /**< current-loop capture sequencer phase */
  uint32_t capture_status;  /**< ESC_CAPTURE_* */
  uint32_t status_replies;  /**< status replies sent */
  uint32_t rejected;        /**< non-CONTROL commands rejected (CONTROL rejects: drive_cmd.count) */
};
/** @brief The application state. */
extern volatile struct drive_app_state drive_app;
/** @brief Protocol v1 command lifecycle of this wheel (readable over SWD). */
extern struct motor_cmd drive_cmd;

/** Forced electrical angle for Hall alignment and capture, read by
 * FOC_CurrControllerM1: nonzero @c drive_align_force overrides the Hall angle. */
extern volatile uint8_t drive_align_force;
extern volatile int16_t drive_align_angle; /**< forced electrical angle, s16 degrees */

/** @brief Current-loop capture hook, called from FOC_CurrControllerM1 every FOC period.
 *  @param[in,out] id_ref Id reference; may be replaced before the PI controllers. */
void drive_capture_isr_ref(int16_t *id_ref);
/** @brief Record one FOC period: measured Id/Iq, applied Vd/Vq after circle
 *  limitation, the Id reference and the electrical angle.
 *  @param id      measured d-axis current, counts (ESC_COUNTS_PER_AMP per A)
 *  @param iq      measured q-axis current, counts
 *  @param vd      applied d-axis voltage, s16V (32767 = Vbus / sqrt(3))
 *  @param vq      applied q-axis voltage, s16V
 *  @param id_ref  d-axis current reference, counts
 *  @param el_angle electrical angle, s16 degrees (65536 = 360 degrees) */
void drive_capture_isr_store(int16_t id, int16_t iq, int16_t vd, int16_t vq,
                             int16_t id_ref, int16_t el_angle);

/** @brief Speed loop of this wheel (readable over SWD). */
extern struct motor_speed drive_speed;
/** @brief Rotor angle state behind the HALL_CalcElAngle override (hall_angle.c). */
extern struct hall_angle drive_hall_angle;

/** @brief Initialise the application state. */
void drive_app_init(void);
/** @brief FDCAN ISR entry: latch one received command frame.
 *  @param id     11-bit CAN ID
 *  @param data   payload bytes
 *  @param length payload length, bytes (decoded from the DLC) */
void drive_app_can_rx(uint32_t id, const uint8_t *data, uint32_t length);
/** @brief Main-loop entry: send pending replies.
 *  @param send queues an 8-byte frame; returns nonzero once queued. */
void drive_app_poll(int (*send)(uint32_t id, const uint8_t data[8]));
/** @brief 1 kHz tick, from MC_APP_PostMediumFrequencyHook_M1. */
void drive_app_tick(void);
/** @brief Nonzero when nothing is running or queued: safe for boot entry. */
int drive_app_idle(void);
#endif
