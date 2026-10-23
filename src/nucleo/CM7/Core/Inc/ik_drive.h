#ifndef IK_DRIVE_H
#define IK_DRIVE_H
#include <stdint.h>

/**
 * @file ik_drive.h
 * @brief Streaming omni drive: body twist from the host to four ESCs over protocol v1.
 *
 * The host (src/tools/lib/ik_link.c over SWD) writes a body twist and bumps
 * heartbeat at >=10 Hz. Every ::IK_DRIVE_PERIOD_MS the Nucleo runs the IK,
 * slews each wheel toward its target at a common acceleration limit (keeping
 * the twist direction), and sends one CONTROL frame (motor_protocol.h) to each ESC.
 *
 * Link per wheel: a drive session starts by probing every ESC with an ARM that
 * carries session 0 (always rejected; the reply announces the ESC's session), then
 * arming it with that session. Motion starts only when all four wheels are armed,
 * so they start together. Every frame is answered; a wheel that misses
 * ::IK_DRIVE_LOST_PERIODS replies in a row is LOST. A FAULTED or LOST wheel stops
 * the whole robot (broadcast DISABLE) and latches ::ik_drive_fault until the host
 * sets enable to 0; a fault on the ESC itself is cleared by bumping clear_seq while
 * stopped. enable == 0 or a heartbeat older than ::IK_DRIVE_HEARTBEAT_MS decelerates
 * to zero, holds briefly, then broadcasts DISABLE. The ESCs disarm on their own
 * ::MOTOR_WATCHDOG_MS after the stream stops. All words are 32 bit for SWD access.
 */

#define IK_DRIVE_ABI 0x494B0009U         /**< ABI tag the host checks before talking */
#define IK_DRIVE_PERIOD_MS 20U           /**< drive loop period, ms (= MOTOR_PROTOCOL_PERIOD_MS) */
#define IK_DRIVE_HEARTBEAT_MS 300U       /**< heartbeat age that counts as lost, ms */
#define IK_DRIVE_HOLD_MS 200U            /**< time at zero before DISABLE is sent, ms */
#define IK_DRIVE_STOP_FRAMES 3U          /**< DISABLE broadcasts per stop */
#define IK_DRIVE_LOST_PERIODS 3U         /**< periods in a row without a reply -> wheel LOST */
#define IK_DRIVE_DEFAULT_ACCEL_RPMPS 900 /**< wheel acceleration limit, RPM/s */
#define IK_DRIVE_MAX_LINEAR_MMPS 5000    /**< accepted |vx|, |vy|, mm/s */
#define IK_DRIVE_MAX_OMEGA_MRADPS 6000   /**< accepted |omega|, mrad/s */
#define IK_DRIVE_FF_MAX_MA 6000          /**< |feed-forward| sent, mA (the ESC's hard limit) */
#define IK_DRIVE_MAX_CURRENT_MA 6000     /**< |current setpoint| sent in current mode, mA (the ESC's hard limit) */
#define IK_DRIVE_BREAKAWAY_RPM 30        /**< a wheel this fast along its target has broken free, RPM */
#define IK_DRIVE_BREAKAWAY_WHEELS 2U     /**< wheels broken free that end the breakaway boost */
#define IK_LINK_NONE 0xFFU               /**< ik_drive_link.lifecycle before the first reply */
#define IK_DRIVE_LAT_BINS 16U            /**< reply latency histogram bins per wheel */
#define IK_DRIVE_LAT_BIN_US 250U         /**< histogram bin width, us; the last bin holds everything above */
/** @brief Values of ::ik_drive_state. */
enum { IK_STOPPED, IK_DRIVING, IK_STOPPING };
/** @brief Values of ::ik_drive_mode. */
enum { IK_MODE_SPEED = 0, IK_MODE_CURRENT = 1 };

/** @brief Protocol link to one wheel; a seqlocked row the host reads like the telemetry rows. */
struct ik_drive_link {
  uint32_t generation;     /**< even = stable, odd = being written */
  uint32_t lifecycle;      /**< motor_lifecycle of the last reply, ::IK_LINK_NONE before any */
  uint32_t outcome;        /**< motor_outcome of the last reply */
  uint32_t reason;         /**< motor_reason of the last reply */
  uint32_t session;        /**< session the ESC announced in the last reply */
  uint32_t request_seq;    /**< seq the last reply answered */
  uint32_t applied_seq;    /**< seq the ESC's controller is using */
  uint32_t first_edge;     /**< a Hall edge along the target since the current start */
  int32_t rpm;             /**< measured speed, RPM (0 with the output off) */
  int32_t iq_ma;           /**< measured q-current, mA, 50 mA steps */
  uint32_t reply_count;    /**< replies received */
  uint32_t reply_us;       /**< Nucleo microsecond clock at the last reply */
  uint32_t misses;         /**< periods in a row whose frame got no reply */
  uint32_t rejects;        /**< REJECTED replies */
  uint32_t start_count;    /**< starts (target leaving zero or reversing) in this drive session */
  uint32_t start_reply_us; /**< Nucleo microsecond clock of the reply to the first frame of the latest start */
  uint32_t first_start_reply_us; /**< same, for the session's first start; move_test records that start */
  uint32_t first_start_valid; /**< 1 once first_start_reply_us belongs to this session (its reply arrived) */
  uint32_t fault_reason;   /**< motor_reason latched with this wheel's ::ik_drive_fault bit (LOST: 0); kept
                                until enable goes 0, while later replies overwrite reason */
  uint32_t cfg_count;      /**< CONFIG_REPLY frames received (the host sends CONFIG through the raw mailbox) */
  uint32_t cfg_seq;        /**< config seq of the last CONFIG_REPLY */
  uint32_t cfg_status;     /**< its status: 0 OK, a motor_reason, or MOTOR_CFG_UNKNOWN_KEY / BAD_VALUE */
  uint32_t cfg_key;        /**< its key */
  int32_t cfg_stored;      /**< value the ESC holds */
  int32_t cfg_effective;   /**< value in force (the derated limit for IQ_LIMIT_MA) */
};
/** @brief Words per ::ik_drive_link row. */
#define IK_DRIVE_LINK_WORDS 25U

/** @brief ABI tag, equals ::IK_DRIVE_ABI. */
extern volatile const uint32_t ik_drive_abi;

/** @name Host to firmware */
/**@{*/
extern volatile int32_t ik_drive_vx_mmps;      /**< body velocity x (right), mm/s */
extern volatile int32_t ik_drive_vy_mmps;      /**< body velocity y (forward), mm/s */
extern volatile int32_t ik_drive_omega_mradps; /**< yaw rate, CCW positive, mrad/s */
extern volatile uint32_t ik_drive_enable;      /**< nonzero allows driving; 0 also acknowledges ::ik_drive_fault */
extern volatile uint32_t ik_drive_heartbeat;   /**< host bumps this to show it is alive */
extern volatile int32_t ik_drive_accel_rpmps;  /**< wheel acceleration limit, RPM/s; <= 0 selects the default */
/* One-shot raw frame (HALL_ALIGN, CONFIG, a CONTROL frame for a test, ...): write id, data,
 * data_hi and len, then bump raw_seq. Sent only while IK_STOPPED; raw_done echoes raw_seq. */
extern volatile uint32_t ik_drive_raw_id;      /**< raw frame CAN ID (11 bit) */
extern volatile uint32_t ik_drive_raw_data;    /**< raw payload bytes 0-3 as one little-endian word */
extern volatile uint32_t ik_drive_raw_data_hi; /**< raw payload bytes 4-7 as one little-endian word */
extern volatile uint32_t ik_drive_raw_len;     /**< raw payload length: 4 or 8 bytes (anything else: 4) */
extern volatile uint32_t ik_drive_raw_seq;     /**< bump to send; sent only while ::IK_STOPPED, echoed by raw_done */
extern volatile uint32_t ik_drive_clear_seq;   /**< bump to send CLEAR_FAULT to every wheel; only while ::IK_STOPPED */
/* Body-frame friction model (friction_ff.h); every term 0 = off. Breakaway boost: while the twist is nonzero
 * and fewer than IK_DRIVE_BREAKAWAY_WHEELS wheels turn along their targets, the breakaway levels replace the
 * Coulomb levels for the whole robot, for at most ik_drive_ff_breakaway_ms per stuck episode. */
extern volatile uint32_t ik_drive_ff_enable;                  /**< nonzero: send the model's current as ff */
extern volatile int32_t ik_drive_ff_coulomb_x_mn;             /**< x (strafe) Coulomb force, mN */
extern volatile int32_t ik_drive_ff_coulomb_y_mn;             /**< y (forward) Coulomb force, mN */
extern volatile int32_t ik_drive_ff_viscous_xy_mn_per_mps;    /**< translation viscous force, mN per m/s */
extern volatile int32_t ik_drive_ff_coulomb_w_mnm;            /**< yaw Coulomb torque, mN*m */
extern volatile int32_t ik_drive_ff_viscous_w_mnm_per_radps;  /**< yaw viscous torque, mN*m per rad/s */
extern volatile int32_t ik_drive_ff_breakaway_x_mn;           /**< x force while breaking away, mN */
extern volatile int32_t ik_drive_ff_breakaway_y_mn;           /**< y force while breaking away, mN */
extern volatile int32_t ik_drive_ff_breakaway_w_mnm;          /**< yaw torque while breaking away, mN*m */
extern volatile int32_t ik_drive_ff_breakaway_ms;             /**< longest boost per stuck episode, ms; <= 0: no boost */
extern volatile int32_t ik_drive_ff_a_per_nm_milli;           /**< motor current per N*m (1/Kt), mA per N*m */
extern volatile int32_t ik_drive_ff_scale_pct[4];             /**< per-wheel trim, % (100 = nominal) */
/* Current mode (friction and motor-constant measurement): each wheel gets its own q-current setpoint instead
 * of the IK speed; no slew, no feed-forward. Latched when a drive session starts. */
extern volatile uint32_t ik_drive_mode;       /**< IK_MODE_SPEED (twist) or IK_MODE_CURRENT (ik_drive_current_ma) */
extern volatile int32_t ik_drive_current_ma[4]; /**< FR/BR/BL/FL q-current, mA, clamped to +-IK_DRIVE_MAX_CURRENT_MA */
/**@}*/

/** @name Firmware to host */
/**@{*/
extern volatile uint32_t ik_drive_state;       /**< IK_STOPPED, IK_DRIVING or IK_STOPPING */
extern volatile uint32_t ik_drive_raw_done;    /**< last raw_seq sent */
extern volatile uint32_t ik_drive_clear_done;  /**< last clear_seq sent */
extern volatile uint32_t ik_drive_fault;       /**< bit w: wheel w FAULTED, bit 4+w: wheel w LOST; latched until enable == 0 */
extern volatile uint32_t ik_drive_armed;       /**< bit w: wheel w reported an armed lifecycle in its last reply */
extern volatile int32_t ik_drive_target_rpm[4]; /**< FR/BR/BL/FL raw motor RPM requested */
extern volatile int32_t ik_drive_rpm[4];       /**< slewed RPM, last sent */
extern volatile int32_t ik_drive_ff_ma[4];     /**< feed-forward last sent, mA */
extern volatile uint32_t ik_drive_ff_boost;    /**< 1 while the last frames carried the breakaway boost */
extern volatile struct ik_drive_link ik_drive_link[4]; /**< per-wheel protocol link, FR/BR/BL/FL */
extern volatile uint32_t ik_drive_tx_count;    /**< CAN frames queued */
extern volatile uint32_t ik_drive_tx_fail;     /**< CAN frames that failed to queue */
extern volatile uint32_t ik_drive_bad_input;   /**< twists rejected as out of range */
/** Reply latency per wheel since boot: Nucleo microseconds from the drive tick that queued a
 *  wheel's frame to the arrival of the reply answering it (first reply only). Counts per
 *  ::IK_DRIVE_LAT_BIN_US bin; the host takes differences between two reads. */
extern volatile uint32_t ik_drive_reply_lat_hist[4][IK_DRIVE_LAT_BINS];
extern volatile uint32_t ik_drive_reply_lat_max_us[4]; /**< largest such latency since boot, us */
/**@}*/

/** @brief Queue one CAN FD frame (no BRS); returns nonzero on success. */
typedef int (*ik_send_fn)(uint32_t id, const uint8_t *data, uint32_t len);

/**
 * @brief One step of the drive loop; call every main-loop pass.
 * @param now    Nucleo time, ms.
 * @param now_us Nucleo microsecond clock (wraps), same clock as ik_drive_rx's; times reply latency.
 * @param send   queues one frame of 4 or 8 bytes.
 */
void ik_drive_tick(uint32_t now, uint32_t now_us, ik_send_fn send);

/**
 * @brief Take a received CAN frame if it is a CONTROL_REPLY or CONFIG_REPLY; call for every received frame.
 * @param id     11-bit CAN ID
 * @param data   payload
 * @param len    payload length, bytes
 * @param now_us Nucleo microsecond clock at reception (wraps)
 * @return 1 if the frame was one of those (consumed), else 0
 */
int ik_drive_rx(uint32_t id, const uint8_t *data, uint32_t len, uint32_t now_us);
#endif
