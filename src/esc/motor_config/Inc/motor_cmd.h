#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include "motor_protocol.h"
#include <stdatomic.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @file motor_cmd.h
 *  @brief ESC command lifecycle for protocol v1: inbox, session and sequence, watchdog, replies.
 *
 *  Pure logic: no MCSDK or HAL calls. The caller samples the motor-control state into
 *  ::motor_cmd_inputs, calls ::motor_cmd_tick every 1 ms, and executes the returned
 *  ::motor_cmd_actions. Specification: architecture/protocol.md (wire), architecture/command-lifecycle.md (states and transitions).
 *
 *  Contexts. ::motor_cmd_rx runs in the CAN receive ISR and is the only writer of the inbox;
 *  ::motor_cmd_tick is its only reader and the only writer of the reply ring;
 *  ::motor_cmd_reply_peek / ::motor_cmd_reply_pop run in the main loop and are the ring's only
 *  reader. Both rings are single-producer/single-consumer with C11 release/acquire indices, so
 *  they need no interrupt masking whichever context preempts the other. */

#define MOTOR_CMD_INBOX 8U          /**< received frames buffered between two ticks */
#define MOTOR_CMD_REPLIES 4U        /**< replies buffered between two main-loop polls */
#define MOTOR_CMD_START_TIMEOUT_MS 500U /**< STARTING without MCSDK RUN for this long -> FAULTED, ms */
/* HOLD. The Nucleo slews to 0, so by the time the target is 0 the wheel is nearly stopped.
 * Holding zero speed on the Hall estimate only buzzed (0.2-0.6 A average, peaks 3.9 A), so at
 * standstill the speed loop is replaced by a zero-current hold with the bridge on: no buzz, and
 * the next motion needs no MCSDK start. Standstill is the speed loop's call (motor_speed.h). */
#define MOTOR_CMD_HOLD_MS 150U      /**< zero target at standstill for this long -> HOLD, ms */

/** @brief Motor-control state as the lifecycle needs it (the caller maps MCSDK states onto this). */
enum motor_cmd_mc {
  MOTOR_CMD_MC_IDLE = 0,     /**< bridge off, ready to start */
  MOTOR_CMD_MC_STARTING = 1, /**< start-up states before RUN (boot-cap charge, offset calibration) */
  MOTOR_CMD_MC_RUN = 2,      /**< regulating */
  MOTOR_CMD_MC_STOPPING = 3, /**< stop in progress */
  MOTOR_CMD_MC_FAULT = 4     /**< MCSDK fault state (FAULT_NOW or FAULT_OVER) */
};

/** @brief Why the output was switched off (mapped to the STATUS stop reason by the caller). */
enum motor_cmd_stop {
  MOTOR_CMD_STOP_NONE = 0,
  MOTOR_CMD_STOP_DISABLE,     /**< DISABLE frame */
  MOTOR_CMD_STOP_TIMEOUT,     /**< command watchdog */
  MOTOR_CMD_STOP_FAULT,       /**< MCSDK fault, start timeout, stall or Hall loss */
  MOTOR_CMD_STOP_OVERFLOW     /**< inbox overflow (faulty sender) */
};

/** @brief Fixed limits and capabilities, set once by ::motor_cmd_init. */
struct motor_cmd_config {
  uint32_t select;      /**< this wheel's CAN select bit (replies use it) */
  int16_t max_rpm;      /**< |SPEED setpoint| limit, RPM; larger is rejected */
  int16_t max_ma;       /**< |CURRENT setpoint| and |ff| limit, mA; larger is rejected */
  uint8_t current_supported; /**< 0: CURRENT frames are rejected (UNSUPPORTED) */
};

/** @brief State sampled by the caller before each tick. */
struct motor_cmd_inputs {
  uint32_t now_ms;      /**< 1 kHz time, the same clock passed to ::motor_cmd_rx */
  uint8_t mc;           /**< ::motor_cmd_mc */
  uint8_t mc_fault_present; /**< a fault cause is still active (MCSDK current faults != 0) */
  uint8_t aligned;      /**< Hall phase shift known */
  uint8_t busy;         /**< Hall alignment or current capture running */
  uint8_t standstill;   /**< the speed loop sees the wheel at rest (motor_speed_outputs.standstill) */
  uint8_t stall_fault;  /**< the speed loop's stall timer expired (motor_speed_outputs.stall_fault) */
  uint8_t hall_lost;    /**< Hall sensing lost: invalid Hall code past the stale window, or MCSDK's
                             speed-feedback fault (with mc FAULT it names the fault HALL_LOST) */
  int16_t rpm;          /**< speed estimate, RPM */
  int16_t iq_ma;        /**< measured q-axis current, mA */
  uint16_t edges_pos;   /**< Hall edges counted in + direction (wrapping) */
  uint16_t edges_neg;   /**< Hall edges counted in - direction (wrapping) */
};

/** @brief What the caller must do after a tick. Several may be set; execute in field order. */
struct motor_cmd_actions {
  uint8_t abort_busy;  /**< abort Hall alignment or capture */
  uint8_t disabled;    /**< a DISABLE was processed: also drop align/capture requests not started yet */
  uint8_t stop;        /**< switch the output off (MC_StopMotor1) */
  uint8_t stop_cause;  /**< ::motor_cmd_stop of this stop; ::MOTOR_CMD_STOP_NONE when stop is 0 */
  uint8_t ack_fault;   /**< acknowledge the MCSDK fault (MC_AcknowledgeFaultMotor1) */
  uint8_t set_target;  /**< (re)program the target below */
  uint8_t start;       /**< start the motor (MC_StartMotor1), after programming the target */
  uint8_t mode;        /**< ::motor_mode of the target */
  int16_t setpoint;    /**< target: RPM (SPEED) or mA (CURRENT) */
  int16_t ff_ma;       /**< target feed-forward, mA */
};

/** @brief One received frame on its way from the ISR to the tick. */
struct motor_cmd_rx_entry {
  struct motor_control c; /**< decoded fields */
  uint8_t reason;         /**< structural verdict, ::motor_reason */
  uint32_t rx_ms;         /**< receive time */
};

/** @brief Counters for diagnosis (read over SWD). */
struct motor_cmd_counters {
  uint32_t bad_length;  /**< frames with DLC != 8 or another ID base */
  uint32_t overflow;    /**< inbox overflows (each forces a disarm) */
  uint32_t rejected;    /**< frames answered REJECTED */
  uint32_t replies_dropped; /**< replies lost to a full reply ring */
  uint8_t last_reject;  /**< ::motor_reason of the most recent rejection */
};

/** @brief Complete lifecycle state; one per wheel controller, statically allocated. */
struct motor_cmd {
  struct motor_cmd_config cfg;                        /**< limits, set at init */
  struct motor_cmd_rx_entry inbox[MOTOR_CMD_INBOX];   /**< ISR -> tick ring */
  atomic_uint inbox_head;                             /**< written by the ISR only */
  atomic_uint inbox_tail;                             /**< written by the tick only */
  atomic_uint inbox_overflow;                         /**< set by the ISR, cleared by the tick */
  uint8_t replies[MOTOR_CMD_REPLIES][MOTOR_PROTOCOL_DLC]; /**< tick -> main-loop ring */
  atomic_uint reply_head;                             /**< written by the tick only */
  atomic_uint reply_tail;                             /**< written by the main loop only */
  uint8_t lifecycle;       /**< ::motor_lifecycle */
  uint8_t latched;         /**< ::motor_reason of the fault while FAULTED, else 0 */
  uint8_t session;         /**< current session (the challenge while DISARMED) */
  uint8_t tick_session;    /**< session when this tick began: frames drained in the tick of a session
                                change were sent before anyone could know the new one */
  uint8_t last_seq;        /**< sequence baseline (ARM or last accepted motion frame) */
  uint8_t applied_seq;     /**< seq of the motion frame the target came from; 0 after a disarm */
  uint8_t last_valid;      /**< last_frame holds an accepted motion frame of this session */
  struct motor_control last_frame; /**< last accepted motion frame, for duplicate detection */
  uint8_t mode;            /**< mode latched at READY -> STARTING */
  int16_t setpoint;        /**< applied target */
  int16_t ff_ma;           /**< applied feed-forward */
  uint32_t watch_ms;       /**< ARM time or receive time of the last fresh motion frame */
  uint32_t state_ms;       /**< entry time of STARTING */
  uint32_t hold_since;     /**< time the wheel came to standstill at a zero target, ms */
  uint8_t holding;         /**< hold_since is running */
  uint16_t start_pos;      /**< Hall edge counters when the target last left zero or reversed */
  uint16_t start_neg;      /**< see start_pos */
  int8_t start_dir;        /**< sign of the target at that moment */
  uint8_t first_edge;      /**< a Hall edge along the target since then */
  uint32_t now_ms;         /**< time of the current tick */
  uint8_t disarm_cause;    /**< ::motor_cmd_stop of the last disarm (TIMEOUT, OVERFLOW; NONE after a clear),
                                kept until the next ARM so a DISARMED wheel shows why over SWD */
  struct motor_cmd_counters count; /**< diagnosis */
};

/**
 * @brief Initialise: DISARMED, a boot session from @p entropy, empty rings.
 * @param m       state
 * @param cfg     limits and capabilities (copied)
 * @param entropy any changing value (e.g. a timer) for the first session
 */
void motor_cmd_init(struct motor_cmd *m, const struct motor_cmd_config *cfg, uint32_t entropy);

/**
 * @brief ISR: queue one received frame of the CONTROL ID base.
 * @param m      state
 * @param id     11-bit CAN ID
 * @param data   payload
 * @param dlc    payload length, bytes
 * @param now_ms receive time, ms (same clock as ::motor_cmd_inputs.now_ms)
 */
void motor_cmd_rx(struct motor_cmd *m, uint32_t id, const uint8_t *data, uint32_t dlc, uint32_t now_ms);

/**
 * @brief 1 kHz tick: fault check, drain the inbox in order, watchdog, start, fault and HOLD rules.
 * @param m   state
 * @param in  sampled motor-control state
 * @param out actions for the caller; cleared first
 */
void motor_cmd_tick(struct motor_cmd *m, const struct motor_cmd_inputs *in, struct motor_cmd_actions *out);

/**
 * @brief Main loop: oldest pending reply, without removing it.
 * @param m    state
 * @param id   out: 11-bit CAN ID
 * @param data out: 8 payload bytes
 * @return 1 when a reply was copied, 0 when none is pending
 */
int motor_cmd_reply_peek(struct motor_cmd *m, uint32_t *id, uint8_t data[MOTOR_PROTOCOL_DLC]);

/** @brief Main loop: remove the reply returned by the last successful ::motor_cmd_reply_peek. */
void motor_cmd_reply_pop(struct motor_cmd *m);

/** @brief Nonzero when the output is off and no motion is possible without a new frame
 *  (lifecycle DISARMED, READY or FAULTED): CONFIG, alignment and capture are allowed. */
int motor_cmd_output_off(const struct motor_cmd *m);

/** @brief Nonzero while the output is on under this lifecycle (STARTING, RUNNING, HOLD). */
int motor_cmd_output_on(const struct motor_cmd *m);

#ifdef __cplusplus
}
#endif

#endif
