#include "drive_app.h"
#include "can_datatypes.h"
#include "motor_controller_can.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_config_common.h"
#include "parameters_conversion.h"
#include <string.h>

/* I2T thermal model, values from the previous engineer's ea7b1a0 "Add I2T". */
#define I2T_LIMIT_A2S 30.0f
#define I2T_NOMINAL_A 3.26f
#define I2T_DERATED_A 3.0f
#define AMPS_PER_COUNT (1.0f / (float)CURRENT_CONV_FACTOR)

/* Hall alignment: hold the rotor with d-axis current at a forced angle, sweep
 * the angle slowly forward then back, and compare it with the angle the Hall
 * decoder assigns at each edge. Averaging both directions cancels rotor lag. */
#define ALIGN_CURRENT_A 2.0f
#define ALIGN_START_TIMEOUT_MS 1000U
#define ALIGN_RAMP_MS 300U
#define ALIGN_HOLD_MS 500U
#define ALIGN_SWEEP_MS 2000U   /* per direction */
#define ALIGN_SWEEP_TURNS 2U   /* electrical revolutions per direction */
#define ALIGN_MIN_EDGES 9U     /* of 12 expected per direction */
enum { ALIGN_IDLE, ALIGN_START, ALIGN_RAMP, ALIGN_HOLD, ALIGN_FWD, ALIGN_BACK,
       ALIGN_STOPPING };

/* Current-loop capture: the same forced-angle hold as alignment but at a
 * fixed angle, so the rotor settles on the d axis and an Id step makes no
 * torque. The FOC ISR applies the step and records every period. */
#define CAPTURE_START_TIMEOUT_MS 1000U
#define CAPTURE_RAMP_MS 200U
#define CAPTURE_HOLD_MS 300U
#define CAPTURE_RUN_TIMEOUT_MS 200U /* 512 samples at 16 kHz take 32 ms */
#define CAPTURE_READ_MAX 32U
enum { CAPTURE_IDLE, CAPTURE_STARTING, CAPTURE_RAMP, CAPTURE_HOLD, CAPTURE_RUN,
       CAPTURE_STOPPING };

/* Bootloader validity record (can_fwu_validity.h): magic, ~magic, size, crc. */
#ifndef DRIVE_APP_VALIDITY
#define DRIVE_APP_VALIDITY ((const volatile uint32_t *)0x0801F800U)
#endif

_Static_assert(ESC_RECORD_TRIGGER_RPM == MOTOR_SPEED_MOVING_RPM, "the recorder starts where the loop calls it motion");

volatile struct drive_app_state drive_app;
struct motor_cmd drive_cmd;
volatile uint8_t drive_align_force;
volatile int16_t drive_align_angle;

/* ISR -> tick mailbox for the 4-byte service commands. Each payload is one
 * word, published before its count. CONTROL frames go to drive_cmd instead. */
static volatile uint32_t rx_align_count;
/* CONFIG frames (ISR -> tick) and their replies (tick -> main loop) pass through single-producer /
 * single-consumer rings with release/acquire indices, so a frame or reply is never read while it is written.
 * The sender waits for each reply; a second slot covers one overlapping request, more are dropped and counted. */
#define CFG_SLOTS 2U
static struct { uint8_t data[MOTOR_PROTOCOL_DLC]; uint32_t id; } cfg_in[CFG_SLOTS];
static atomic_uint cfg_in_head, cfg_in_tail;
static uint8_t cfg_out[CFG_SLOTS][MOTOR_PROTOCOL_DLC];
static atomic_uint cfg_out_head, cfg_out_tail;
static volatile uint32_t cfg_dropped; /* CONFIG frames or replies lost to a full ring */
static volatile uint32_t rx_capture_word, rx_capture_count;
static volatile uint32_t rx_read_word, rx_read_count;
static volatile uint8_t status_pending, diag_pending, ident_pending;
static uint32_t seen_align, seen_capture, seen_read;

/* FOC ISR <-> tick: the tick arms, the ISR records and disarms when full. */
/* Shared by the current-loop capture and the speed-loop recorder. */
static union {
  int16_t cur[ESC_CAPTURE_SAMPLES][6]; /**< current-loop capture */
  int16_t rec[ESC_RECORD_SAMPLES][8];  /**< speed-loop recording */
} capture_mem;
_Static_assert(sizeof(capture_mem.rec) == sizeof(capture_mem.cur), "recorder reuses capture RAM");
#define capture_buf capture_mem.cur
static volatile uint32_t rec_status, rec_index, rec_remaining, rec_delay, rec_trigger_index;
static uint32_t rec_pre, rec_prev_moving, rec_owner; /* rec_owner: buffer holds a recording */
static volatile uint32_t rx_record_word, rx_record_count;
static uint32_t seen_record;
static volatile uint32_t capture_armed, capture_index;
static volatile int16_t capture_step_counts;
static uint32_t capture_ms, capture_vbus_cv;
static uint32_t read_next, read_end, read_bank, read_ch67; /* poll context only */

struct motor_speed drive_speed;
static struct motor_speed_config speed_cfg;
static struct motor_speed_outputs speed_out; /* this tick's loop results, for replies and the recorder */

static uint32_t align_ms, align_prev_hall;
static int16_t align_saved_shift, align_ref;
static int32_t align_sum;

/* The speed loop reads the limit every tick, so a derate or a CONFIG change applies at once. */
static void set_iq_limit(int32_t counts)
{
  drive_app.iq_limit_counts = counts;
}

void drive_app_init(void)
{
  drive_app.iq_max_ma = (int32_t)(IQMAX_A * 1000);
  drive_app.iq_max_counts = (int32_t)IQMAX;
  set_iq_limit(drive_app.iq_max_counts);
  speed_cfg = (struct motor_speed_config){
    .kp = PID_SPEED_KP_DEFAULT, .ki = PID_SPEED_KI_DEFAULT,
    .stall_ramp_ma_per_s = SPEED_STALL_RAMP_MA_PER_S_DEFAULT, .stall_ms = SPEED_STALL_MS_DEFAULT };
  motor_speed_init(&drive_speed, &speed_cfg, HALL_M1.EdgesPos, HALL_M1.EdgesNeg);
#ifdef HALL_PHASE_SHIFT_OVERRIDE
  drive_app.aligned = 1U; /* measured shift compiled in by scripts/esc_build.sh --hall-shift */
#endif
  /* The boot session needs no randomness: after boot the wheel is DISARMED and only an
   * ARM sent in answer to its own reply (which carries the session) can arm it. */
  const struct motor_cmd_config cfg = {
    .select = MOTOR_CAN_ID, .max_rpm = MAX_APPLICATION_SPEED_RPM, .max_ma = (int16_t)(IQMAX_A * 1000),
    .current_supported = 1U };
  motor_cmd_init(&drive_cmd, &cfg, MOTOR_CAN_ID);
}

void drive_app_can_rx(uint32_t id, const uint8_t *data, uint32_t length)
{
  if ((id & MOTOR_CAN_ID) == 0U) return; /* the hardware filter does this too; keep it explicit */
  if ((id & MOTOR_CAN_ID_BASE_MASK) == MOTOR_CAN_ID_CONTROL) {
    motor_cmd_rx(&drive_cmd, id, data, length, drive_app.now_ms);
    return;
  }
  if ((id & MOTOR_CAN_ID_BASE_MASK) == MOTOR_CAN_ID_CONFIG) {
    if (length != MOTOR_PROTOCOL_DLC) return; /* no seq to answer */
    const unsigned head = atomic_load_explicit(&cfg_in_head, memory_order_relaxed);
    const unsigned tail = atomic_load_explicit(&cfg_in_tail, memory_order_acquire);
    if (head - tail >= CFG_SLOTS) { ++cfg_dropped; return; }
    for (uint32_t i = 0; i < MOTOR_PROTOCOL_DLC; ++i) cfg_in[head % CFG_SLOTS].data[i] = data[i];
    cfg_in[head % CFG_SLOTS].id = id;
    atomic_store_explicit(&cfg_in_head, head + 1U, memory_order_release);
    return;
  }
  if (length != 4U) return;
  const uint32_t word = CAN_U32_FROM_PAYLOAD(data);
  switch (CAN_COMMAND_FROM_ID(id)) {
    case STATUS_READ: status_pending = 1U; break;
    case DIAG_READ: diag_pending = 1U; break;
    case HALL_ALIGN: if (word == HALL_ALIGN_MAGIC) ++rx_align_count; break;
    case IDENT_READ: ident_pending = 1U; break;
    case CAPTURE:
      if ((word >> 24) == ESC_CAPTURE_START) { rx_capture_word = word; ++rx_capture_count; }
      else if ((word >> 24) == ESC_RECORD_START) { rx_record_word = word; ++rx_record_count; }
      else if ((word >> 24) == ESC_CAPTURE_READ) { rx_read_word = word; ++rx_read_count; }
      break;
    default: break;
  }
}

void drive_capture_isr_ref(int16_t *id_ref)
{
  if (capture_armed && capture_index >= ESC_CAPTURE_PRE) *id_ref = capture_step_counts;
}

void drive_capture_isr_store(int16_t id, int16_t iq, int16_t vd, int16_t vq,
                             int16_t id_ref, int16_t el_angle)
{
  if (!capture_armed) return;
  const uint32_t i = capture_index;
  int16_t *s = capture_buf[i];
  s[0] = id; s[1] = iq; s[2] = vd; s[3] = vq; s[4] = id_ref; s[5] = el_angle;
  capture_index = i + 1U;
  if (i + 1U >= ESC_CAPTURE_SAMPLES) capture_armed = 0U;
}

/* Measured q-axis current, SDK counts. MCSDK computes Iq from phase-current offsets that are only
 * calibrated on the first start, and keeps computing it with the bridge off, so outside the states
 * where the current loop drives the bridge it reads up to ~25 A of garbage (hardware 2026-10-21).
 * No current flows with the bridge off: report 0 there. */
static int16_t measured_iq(void)
{
  const int state = MC_GetSTMStateMotor1();
  return state == START || state == SWITCH_OVER || state == RUN ? MC_GetIqdMotor1().q : 0;
}

/* One recorder sample per drive_app_tick (1 kHz in every MCSDK state), so it sees the stopped
 * reference before a start and finishes after a stop. Loop values are this tick's motor_speed
 * results; they are 0 while the loop is off (IDLE, STOP, alignment, capture). */
static void record_sample(uint8_t speed_mode)
{
  const uint32_t status = rec_status;
  if (status != ESC_RECORD_ARMED && status != ESC_RECORD_TRIGGERED) return;
  const struct motor_speed_outputs *o = &speed_out;
  const uint32_t ran = speed_mode != MOTOR_SPEED_OFF;
  const uint32_t moving = o->ref_rpm >= ESC_RECORD_TRIGGER_RPM || o->ref_rpm <= -ESC_RECORD_TRIGGER_RPM;
  const uint32_t mode = speed_mode == MOTOR_SPEED_HOLD ? ESC_REC_MODE_HOLD
                        : o->stall_ramp ? ESC_REC_MODE_STALL_RAMP : ran ? ESC_REC_MODE_RUN : ESC_REC_MODE_OFF;
  int16_t *s = capture_mem.rec[rec_index % ESC_RECORD_SAMPLES];
  s[0] = o->ref_rpm;
  s[1] = o->w_hat_rpm;
  s[2] = o->p_counts;
  s[3] = o->i_counts;
  s[4] = o->ff_counts;
  s[5] = o->iq_ref_counts;
  s[6] = measured_iq();
  s[7] = (int16_t)(mode | ((o->valid & 1U) << 2) | ((o->saturated & 1U) << 3) | ((o->bleed & 1U) << 4) |
                   ((o->stall_timer & 1U) << 5) |
                   (((uint32_t)(HALL_M1.EdgesPos - HALL_M1.EdgesNeg) & 15U) << 8) |
                   (((uint32_t)HALL_M1.HallState & 7U) << 12) | (ran << 15));
  if (status == ESC_RECORD_ARMED) {
    if (rec_delay == UINT32_MAX) { /* waiting for the reference to leave zero */
      if (moving && !rec_prev_moving) rec_delay = (rx_record_word >> 9) & 0xFFFU;
    }
    if (rec_delay != UINT32_MAX && rec_delay-- == 0U) {
      rec_status = ESC_RECORD_TRIGGERED;
      rec_trigger_index = rec_index;
      rec_remaining = ESC_RECORD_SAMPLES - rec_pre;
    }
  }
  rec_prev_moving = moving;
  ++rec_index;
  if (rec_status == ESC_RECORD_TRIGGERED && --rec_remaining == 0U) rec_status = ESC_RECORD_DONE;
}

static void record_begin(uint32_t word)
{
  const uint32_t pre = word & 0x1FFU;
  if (drive_app.capture_phase != CAPTURE_IDLE || pre >= ESC_RECORD_SAMPLES) {
    rec_status = ESC_RECORD_REJECTED;
    ++drive_app.rejected;
    return;
  }
  rec_status = ESC_RECORD_NONE; /* the speed tick stops writing first */
  rec_owner = 1U;
  rec_pre = pre;
  rec_index = 0U;
  rec_trigger_index = 0U;
  rec_prev_moving = 1U; /* a start must follow a zero reference */
  rec_delay = UINT32_MAX;
  rec_status = ESC_RECORD_ARMED;
}

/* Oldest recorded sample: the window is the last ESC_RECORD_SAMPLES written. */
static uint32_t record_first(void)
{
  return rec_index > ESC_RECORD_SAMPLES ? rec_index - ESC_RECORD_SAMPLES : 0U;
}

static uint32_t vbus_centivolts(void)
{
  return (uint32_t)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super) *
         BusVoltageSensor_M1._Super.ConversionFactor * 100U / 65536U;
}

static int16_t s16_degrees(int32_t s16)
{
  const int32_t scaled = s16 * 360;
  return (int16_t)((scaled + (scaled < 0 ? -32768 : 32768)) / 65536);
}

static void align_finish(uint32_t result)
{
  drive_app.align_result = result;
  if (result == ESC_ALIGN_OK) {
    HALL_M1.PhaseShift = (int16_t)(align_saved_shift + drive_app.align_error_mean);
    drive_app.aligned = 1U;
  }
  (void)MC_StopMotor1();
  drive_app.stop_reason = ESC_STOP_ALIGN;
  drive_app.align_phase = ALIGN_STOPPING;
}

static void align_record_edge(int8_t expected_direction)
{
  const uint8_t hall = HALL_M1.HallState;
  if (hall == align_prev_hall) return;
  align_prev_hall = hall;
  if (HALL_M1.Direction != expected_direction) {
    ++drive_app.align_wrong_dir;
    return;
  }
  const int16_t error = (int16_t)(drive_align_angle - HALL_M1.MeasuredElAngle);
  const uint32_t edges = drive_app.align_edges_fwd + drive_app.align_edges_back;
  if (edges == 0U) align_ref = error;
  align_sum += (int16_t)(error - align_ref);
  if (expected_direction > 0) ++drive_app.align_edges_fwd;
  else ++drive_app.align_edges_back;
}

static void align_tick(uint16_t state)
{
  const uint32_t phase = drive_app.align_phase;
  if (phase == ALIGN_IDLE) return;
  if (phase == ALIGN_STOPPING) {
    if (state == IDLE || state == FAULT_OVER) {
      drive_align_force = 0U;
      drive_app.align_phase = ALIGN_IDLE;
    }
    return;
  }
  ++align_ms;
  if (state == FAULT_NOW || state == FAULT_OVER) {
    align_finish(ESC_ALIGN_ABORTED);
    return;
  }
  const uint32_t sweep = 65536U * ALIGN_SWEEP_TURNS;
  switch (phase) {
    case ALIGN_START:
      if (state == RUN) { drive_app.align_phase = ALIGN_RAMP; align_ms = 0U; }
      else if (align_ms > ALIGN_START_TIMEOUT_MS) align_finish(ESC_ALIGN_ABORTED);
      break;
    case ALIGN_RAMP: {
      const qd_t iqd = { .q = 0, .d = (int16_t)(ALIGN_CURRENT_A * CURRENT_CONV_FACTOR *
                                               (float)align_ms / (float)ALIGN_RAMP_MS) };
      MC_SetCurrentReferenceMotor1(iqd);
      if (align_ms >= ALIGN_RAMP_MS) { drive_app.align_phase = ALIGN_HOLD; align_ms = 0U; }
      break;
    }
    case ALIGN_HOLD:
      if (align_ms >= ALIGN_HOLD_MS) {
        drive_app.align_phase = ALIGN_FWD;
        align_ms = 0U;
        align_prev_hall = HALL_M1.HallState; /* the first edge is a real one */
      }
      break;
    case ALIGN_FWD:
      drive_align_angle = (int16_t)(uint16_t)(align_ms * sweep / ALIGN_SWEEP_MS);
      align_record_edge(1);
      if (align_ms >= ALIGN_SWEEP_MS) { drive_app.align_phase = ALIGN_BACK; align_ms = 0U; }
      break;
    case ALIGN_BACK:
      drive_align_angle = (int16_t)(uint16_t)(sweep - align_ms * sweep / ALIGN_SWEEP_MS);
      align_record_edge(-1);
      if (align_ms >= ALIGN_SWEEP_MS) {
        const uint32_t edges = drive_app.align_edges_fwd + drive_app.align_edges_back;
        if (drive_app.align_wrong_dir > edges) {
          align_finish(ESC_ALIGN_REVERSED);
        } else if (drive_app.align_edges_fwd < ALIGN_MIN_EDGES ||
                   drive_app.align_edges_back < ALIGN_MIN_EDGES) {
          align_finish(ESC_ALIGN_NO_EDGES);
        } else {
          drive_app.align_error_mean = (int16_t)(align_ref + align_sum / (int32_t)edges);
          align_finish(ESC_ALIGN_OK);
        }
      }
      break;
    default:
      break;
  }
}

static void align_begin(void)
{
  drive_app.align_edges_fwd = drive_app.align_edges_back = 0U;
  drive_app.align_wrong_dir = 0U;
  drive_app.align_error_mean = 0;
  align_sum = 0;
  align_ms = 0U;
  align_saved_shift = HALL_M1.PhaseShift;
  drive_align_angle = 0;
  drive_align_force = 1U;
  MC_SetCurrentReferenceMotor1((qd_t){ .q = 0, .d = 0 });
  drive_app.align_result = ESC_ALIGN_RUNNING;
  drive_app.align_phase = ALIGN_START;
  if (!MC_StartMotor1()) align_finish(ESC_ALIGN_ABORTED);
}

static void capture_finish(uint32_t status)
{
  capture_armed = 0U;
  drive_app.capture_status = status;
  (void)MC_StopMotor1();
  drive_app.stop_reason = ESC_STOP_CAPTURE;
  drive_app.capture_phase = CAPTURE_STOPPING;
}

static void capture_begin(uint32_t step_ma)
{
  if (step_ma > (uint32_t)(IQMAX_A * 1000)) {
    drive_app.capture_status = ESC_CAPTURE_REJECTED;
    ++drive_app.rejected;
    return;
  }
  rec_status = ESC_RECORD_NONE; /* the capture overwrites the recorder RAM */
  rec_owner = 0U;
  capture_step_counts = (int16_t)(step_ma * (uint32_t)CURRENT_CONV_FACTOR / 1000U);
  capture_armed = 0U;
  capture_index = 0U;
  capture_ms = 0U;
  drive_align_angle = 0;
  drive_align_force = 1U;
  MC_SetCurrentReferenceMotor1((qd_t){ .q = 0, .d = 0 });
  drive_app.capture_status = ESC_CAPTURE_RUNNING;
  drive_app.capture_phase = CAPTURE_STARTING;
  if (!MC_StartMotor1()) capture_finish(ESC_CAPTURE_ABORTED);
}

static void capture_tick(uint16_t state)
{
  const uint32_t phase = drive_app.capture_phase;
  if (phase == CAPTURE_IDLE) return;
  if (phase == CAPTURE_STOPPING) {
    if (state == IDLE || state == FAULT_OVER) {
      drive_align_force = 0U;
      drive_app.capture_phase = CAPTURE_IDLE;
    }
    return;
  }
  ++capture_ms;
  if (state == FAULT_NOW || state == FAULT_OVER) {
    capture_finish(ESC_CAPTURE_ABORTED);
    return;
  }
  switch (phase) {
    case CAPTURE_STARTING:
      if (state == RUN) { drive_app.capture_phase = CAPTURE_RAMP; capture_ms = 0U; }
      else if (capture_ms > CAPTURE_START_TIMEOUT_MS) capture_finish(ESC_CAPTURE_ABORTED);
      break;
    case CAPTURE_RAMP: {
      const qd_t iqd = { .q = 0, .d = (int16_t)(ESC_CAPTURE_HOLD_MA * CURRENT_CONV_FACTOR / 1000U *
                                               capture_ms / CAPTURE_RAMP_MS) };
      MC_SetCurrentReferenceMotor1(iqd);
      if (capture_ms >= CAPTURE_RAMP_MS) { drive_app.capture_phase = CAPTURE_HOLD; capture_ms = 0U; }
      break;
    }
    case CAPTURE_HOLD:
      if (capture_ms >= CAPTURE_HOLD_MS) {
        capture_vbus_cv = vbus_centivolts();
        capture_index = 0U;
        capture_armed = 1U; /* the ISR steps Id after ESC_CAPTURE_PRE samples */
        drive_app.capture_phase = CAPTURE_RUN;
        capture_ms = 0U;
      }
      break;
    case CAPTURE_RUN:
      if (!capture_armed) capture_finish(ESC_CAPTURE_DONE);
      else if (capture_ms > CAPTURE_RUN_TIMEOUT_MS) capture_finish(ESC_CAPTURE_ABORTED);
      break;
    default:
      break;
  }
}

/* Effective q-current limit, mA: the stored limit, or the I2T derate when lower. */
static int16_t effective_limit_ma(void)
{
  const int32_t derated = (int32_t)(I2T_DERATED_A * 1000.0f);
  return (int16_t)(drive_app.derated && derated < drive_app.iq_max_ma ? derated : drive_app.iq_max_ma);
}

/* Stored value of a key, or -1 for an unknown key (*out untouched). */
static int config_read(uint16_t key, int16_t *out)
{
  switch (key) {
    case MOTOR_CFG_IQ_LIMIT_MA: *out = (int16_t)drive_app.iq_max_ma; return 0;
    case MOTOR_CFG_HALL_SHIFT_DEG: *out = s16_degrees(HALL_M1.PhaseShift); return 0;
    case MOTOR_CFG_SPEED_KP: *out = speed_cfg.kp; return 0;
    case MOTOR_CFG_SPEED_KI: *out = speed_cfg.ki; return 0;
    case MOTOR_CFG_CURRENT_KP: *out = PID_GetKP(&PIDIqHandle_M1); return 0;
    case MOTOR_CFG_CURRENT_KI: *out = PID_GetKI(&PIDIqHandle_M1); return 0;
    case MOTOR_CFG_STALL_RAMP_MA_PER_S: *out = (int16_t)speed_cfg.stall_ramp_ma_per_s; return 0;
    case MOTOR_CFG_STALL_MS: *out = (int16_t)speed_cfg.stall_ms; return 0;
    default: return -1;
  }
}

/* Range check, then apply; MOTOR_CFG_BAD_VALUE leaves everything unchanged. */
static uint8_t config_write(uint16_t key, int16_t v)
{
  switch (key) {
    case MOTOR_CFG_IQ_LIMIT_MA:
      if (v < 1 || v > (int16_t)(IQMAX_A * 1000)) return MOTOR_CFG_BAD_VALUE;
      drive_app.iq_max_ma = v;
      drive_app.iq_max_counts = (int32_t)v * (int32_t)CURRENT_CONV_FACTOR / 1000;
      if (!drive_app.derated) {
        set_iq_limit(drive_app.iq_max_counts);
      } else { /* derated: the lower of the derate and the new limit, as effective_limit_ma() reports */
        const int32_t derated = (int32_t)(I2T_DERATED_A * CURRENT_CONV_FACTOR);
        set_iq_limit(derated < drive_app.iq_max_counts ? derated : drive_app.iq_max_counts);
      }
      return 0;
    case MOTOR_CFG_HALL_SHIFT_DEG:
      if (v < -359 || v > 359) return MOTOR_CFG_BAD_VALUE;
      HALL_M1.PhaseShift = (int16_t)((int32_t)v * 65536 / 360);
      drive_app.aligned = 1U;
      return 0;
    case MOTOR_CFG_SPEED_KP: case MOTOR_CFG_SPEED_KI:
      if (v < 0) return MOTOR_CFG_BAD_VALUE;
      if (key == MOTOR_CFG_SPEED_KP) speed_cfg.kp = v;
      else speed_cfg.ki = v;
      motor_speed_configure(&drive_speed, &speed_cfg);
      return 0;
    case MOTOR_CFG_CURRENT_KP: case MOTOR_CFG_CURRENT_KI:
      if (v < 0) return MOTOR_CFG_BAD_VALUE;
      if (key == MOTOR_CFG_CURRENT_KP) { PID_SetKP(&PIDIqHandle_M1, v); PID_SetKP(&PIDIdHandle_M1, v); }
      else { PID_SetKI(&PIDIqHandle_M1, v); PID_SetKI(&PIDIdHandle_M1, v); }
      return 0;
    case MOTOR_CFG_STALL_RAMP_MA_PER_S:
      if (v < 2000 || v > 30000) return MOTOR_CFG_BAD_VALUE;
      speed_cfg.stall_ramp_ma_per_s = (uint16_t)v;
      motor_speed_configure(&drive_speed, &speed_cfg);
      return 0;
    case MOTOR_CFG_STALL_MS:
      if (v < 200 || v > 2000) return MOTOR_CFG_BAD_VALUE;
      speed_cfg.stall_ms = (uint16_t)v;
      motor_speed_configure(&drive_speed, &speed_cfg);
      return 0;
    default:
      return MOTOR_CFG_UNKNOWN_KEY;
  }
}

/* One CONFIG frame -> one CONFIG_REPLY. Order of refusals: structure, not idle, unknown key, value. */
static void config_frame(const uint8_t data[MOTOR_PROTOCOL_DLC], uint32_t id, int idle)
{
  struct motor_config c = { 0 };
  struct motor_config_reply r = { 0 };
  const enum motor_reason why = motor_config_decode(id, data, MOTOR_PROTOCOL_DLC, &c);
  r.seq = c.seq;
  r.key = c.key;
  int16_t stored = 0;
  const int known = config_read(c.key, &stored) == 0;
  if (why != MOTOR_R_NONE) r.status = (uint8_t)why;
  else if (!idle) r.status = MOTOR_R_NOT_IDLE;
  else if (!known) r.status = MOTOR_CFG_UNKNOWN_KEY;
  else if (c.op == MOTOR_CONFIG_SET) r.status = config_write(c.key, c.value);
  if (r.status != 0U) ++drive_app.rejected;
  if (known) (void)config_read(c.key, &stored);
  r.stored = stored;
  r.effective = known && c.key == MOTOR_CFG_IQ_LIMIT_MA ? effective_limit_ma() : stored;
  const unsigned head = atomic_load_explicit(&cfg_out_head, memory_order_relaxed);
  const unsigned tail = atomic_load_explicit(&cfg_out_tail, memory_order_acquire);
  if (head - tail >= CFG_SLOTS) { ++cfg_dropped; return; }
  motor_config_reply_encode(&r, cfg_out[head % CFG_SLOTS]);
  atomic_store_explicit(&cfg_out_head, head + 1U, memory_order_release);
}

static uint8_t mc_state(uint16_t state)
{
  switch (state) {
    case IDLE: return MOTOR_CMD_MC_IDLE;
    case RUN: return MOTOR_CMD_MC_RUN;
    case STOP: case WAIT_STOP_MOTOR: return MOTOR_CMD_MC_STOPPING;
    case FAULT_NOW: case FAULT_OVER: return MOTOR_CMD_MC_FAULT;
    default: return MOTOR_CMD_MC_STARTING; /* boot-cap charge, offset calibration, start-up */
  }
}

static uint32_t esc_stop_reason(uint8_t cause)
{
  switch (cause) {
    case MOTOR_CMD_STOP_DISABLE: return ESC_STOP_COMMAND;
    case MOTOR_CMD_STOP_TIMEOUT: case MOTOR_CMD_STOP_OVERFLOW: return ESC_STOP_TIMEOUT;
    case MOTOR_CMD_STOP_FAULT: return ESC_STOP_FAULT;
    default: return ESC_STOP_NONE;
  }
}

static int busy(void)
{
  return drive_app.align_phase != ALIGN_IDLE || drive_app.capture_phase != CAPTURE_IDLE;
}

/* Run the command lifecycle for this tick and carry out what it decided, in its field order. */
static void command_tick(uint16_t state)
{
  const struct motor_cmd_inputs in = {
    .now_ms = drive_app.now_ms, .mc = mc_state(state),
    .mc_fault_present = state == FAULT_NOW || MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS,
    .aligned = drive_app.aligned != 0U, .busy = (uint8_t)busy(),
    /* the loop's verdicts are from the previous tick: it runs after the lifecycle decides its mode */
    .standstill = speed_out.standstill, .stall_fault = speed_out.stall_fault,
    .hall_lost = speed_out.hall_lost ||
                 ((MC_GetCurrentFaultsMotor1() | MC_GetOccurredFaultsMotor1()) & MC_SPEED_FDBK) != 0U,
    .rpm = speed_out.w_hat_rpm,
    .limit_ma = (int16_t)((drive_app.iq_limit_counts * 1000 + (int32_t)CURRENT_CONV_FACTOR / 2) /
                          (int32_t)CURRENT_CONV_FACTOR),
    .iq_ma = (int16_t)((int32_t)measured_iq() * 1000 / (int32_t)CURRENT_CONV_FACTOR),
    .edges_pos = HALL_M1.EdgesPos, .edges_neg = HALL_M1.EdgesNeg };
  struct motor_cmd_actions act;
  motor_cmd_tick(&drive_cmd, &in, &act);
  if (act.disabled) { /* service requests that arrived before the DISABLE never start */
    if (rx_align_count != seen_align) { seen_align = rx_align_count; ++drive_app.rejected; }
    if (rx_capture_count != seen_capture) {
      seen_capture = rx_capture_count;
      drive_app.capture_status = ESC_CAPTURE_REJECTED;
      ++drive_app.rejected;
    }
  }
  if (act.abort_busy) {
    if (drive_app.align_phase != ALIGN_IDLE && drive_app.align_phase != ALIGN_STOPPING)
      align_finish(ESC_ALIGN_ABORTED);
    else if (drive_app.capture_phase != CAPTURE_IDLE && drive_app.capture_phase != CAPTURE_STOPPING)
      capture_finish(ESC_CAPTURE_ABORTED);
  }
  if (act.stop) {
    (void)MC_StopMotor1();
    drive_app.stop_reason = esc_stop_reason(act.stop_cause);
  }
  if (act.ack_fault) (void)MC_AcknowledgeFaultMotor1();
  if (act.set_target && act.mode == MOTOR_MODE_SPEED) motor_speed_set(&drive_speed, act.setpoint, act.ff_ma);
  if (act.start) {
    drive_app.stop_reason = ESC_STOP_NONE;
    /* Torque mode from the first period: MCSDK holds Iq 0 until the speed loop takes over in RUN. */
    MC_SetCurrentReferenceMotor1((qd_t){ .q = 0, .d = 0 });
    (void)MC_StartMotor1(); /* refused only off IDLE, which motor_cmd waits for; its start timeout covers the rest */
  }
  drive_app.active = motor_cmd_output_on(&drive_cmd) ? 1U : 0U;
}

static void i2t_tick(uint16_t state)
{
  float amps2 = 0.0f;
  if (state == RUN) {
    const qd_t iqd = MC_GetIqdMotor1();
    const float q = (float)iqd.q * AMPS_PER_COUNT, d = (float)iqd.d * AMPS_PER_COUNT;
    amps2 = q * q + d * d;
  }
  float i2t = drive_app.i2t_a2s + (amps2 - I2T_NOMINAL_A * I2T_NOMINAL_A) * 0.001f;
  if (i2t < 0.0f) i2t = 0.0f;
  drive_app.i2t_a2s = i2t;
  if (!drive_app.derated && i2t >= I2T_LIMIT_A2S) {
    drive_app.derated = 1U;
    const int32_t derated = (int32_t)(I2T_DERATED_A * CURRENT_CONV_FACTOR);
    set_iq_limit(derated < drive_app.iq_max_counts ? derated : drive_app.iq_max_counts);
  } else if (drive_app.derated && i2t < 0.9f * I2T_LIMIT_A2S) {
    drive_app.derated = 0U;
    set_iq_limit(drive_app.iq_max_counts);
  }
}

/* The lifecycle owns the running bridge in RUNNING and HOLD; alignment and capture own it otherwise and set
 * their own references. In SPEED mode the speed loop makes the q-current; in CURRENT mode it is the
 * setpoint itself, clamped to the effective limit, and the loop only keeps estimating (Hall loss still trips). */
static uint8_t speed_tick(uint16_t state)
{
  const uint8_t lc = drive_cmd.lifecycle;
  const int owned = state == RUN && (lc == MOTOR_LC_RUNNING || lc == MOTOR_LC_HOLD);
  const int current = owned && drive_cmd.mode == MOTOR_MODE_CURRENT;
  const uint8_t mode = !owned || current ? MOTOR_SPEED_OFF
                       : lc == MOTOR_LC_RUNNING ? MOTOR_SPEED_RUNNING : MOTOR_SPEED_HOLD;
  const uint8_t hall = HALL_M1.HallState;
  const struct motor_speed_inputs in = {
    .mode = mode, .limit_counts = (int16_t)drive_app.iq_limit_counts,
    .edges_pos = HALL_M1.EdgesPos, .edges_neg = HALL_M1.EdgesNeg,
    .mcsdk_avg_rpm = (int16_t)SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1()),
    .mcsdk_buffer_full = HALL_M1.BufferFilled >= HALL_M1.SpeedBufferSize,
    .hall_code_invalid = hall == 0U || hall == 7U };
  motor_speed_tick(&drive_speed, &in, &speed_out);
  if (mode != MOTOR_SPEED_OFF) MC_SetCurrentReferenceMotor1((qd_t){ .q = speed_out.iq_ref_counts, .d = 0 });
  if (current) {
    const int32_t limit = drive_app.iq_limit_counts;
    int32_t q = (int32_t)drive_cmd.setpoint * (int32_t)CURRENT_CONV_FACTOR / 1000;
    if (q > limit) q = limit;
    if (q < -limit) q = -limit;
    MC_SetCurrentReferenceMotor1((qd_t){ .q = (int16_t)q, .d = 0 });
  }
  return mode;
}

/* Service commands (align, capture, config) need the motor off and not latched in a fault. */
static int service_allowed(uint16_t state)
{
  return !busy() && motor_cmd_output_off(&drive_cmd) && drive_cmd.lifecycle != MOTOR_LC_FAULTED &&
         state == IDLE;
}

void drive_app_tick(void)
{
  ++drive_app.now_ms;
  const uint16_t state = (uint16_t)MC_GetSTMStateMotor1();
  command_tick(state); /* first: a DISABLE in it cancels service requests queued before it */
  const uint8_t speed_mode = speed_tick(state);
  const uint32_t align_count = rx_align_count;
  const uint32_t capture_count = rx_capture_count, record_count = rx_record_count;
  align_tick(state);
  capture_tick(state);

  if (align_count != seen_align) {
    seen_align = align_count;
    if (service_allowed(state)) align_begin();
    else ++drive_app.rejected;
  }
  if (capture_count != seen_capture) {
    seen_capture = capture_count;
    if (service_allowed(state) && rec_status != ESC_RECORD_ARMED && rec_status != ESC_RECORD_TRIGGERED) {
      capture_begin(rx_capture_word & 0xFFFFU);
    } else {
      drive_app.capture_status = ESC_CAPTURE_REJECTED;
      ++drive_app.rejected;
    }
  }
  if (record_count != seen_record) {
    seen_record = record_count;
    record_begin(rx_record_word);
  }
  for (unsigned tail = atomic_load_explicit(&cfg_in_tail, memory_order_relaxed);
       tail != atomic_load_explicit(&cfg_in_head, memory_order_acquire); ++tail) {
    uint8_t frame[MOTOR_PROTOCOL_DLC];
    memcpy(frame, cfg_in[tail % CFG_SLOTS].data, sizeof frame);
    const uint32_t id = cfg_in[tail % CFG_SLOTS].id;
    atomic_store_explicit(&cfg_in_tail, tail + 1U, memory_order_release);
    config_frame(frame, id, service_allowed(state));
  }
  i2t_tick(state);
  record_sample(speed_mode);
}

int drive_app_idle(void)
{
  return motor_cmd_output_off(&drive_cmd) && !busy() &&
         rx_align_count == seen_align && rx_capture_count == seen_capture;
}

static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }

/* One CAPTURE_REPLY for sample i of bank read_bank. */
static void capture_frame(uint8_t data[8], uint32_t i)
{
  put16(data, (read_bank << 12) | i);
  if (read_bank == ESC_CAPTURE_INFO_BANK && i == ESC_RECORD_INFO_SAMPLE) {
    const uint32_t first = record_first(), done = rec_status == ESC_RECORD_DONE;
    put16(data + 2, rec_status);
    put16(data + 4, rec_index - first);
    put16(data + 6, done && rec_trigger_index >= first ? rec_trigger_index - first : 0U);
    return;
  }
  if (read_bank == ESC_CAPTURE_INFO_BANK) {
    put16(data + 2, drive_app.capture_status);
    put16(data + 4, capture_vbus_cv);
    put16(data + 6, capture_index);
    return;
  }
  if (rec_owner) {
    const int16_t *r = capture_mem.rec[(record_first() + i) % ESC_RECORD_SAMPLES];
    const uint32_t c = read_ch67 ? 6U : read_bank * 3U;
    put16(data + 2, (uint16_t)r[c]);
    put16(data + 4, (uint16_t)r[c + 1U]);
    put16(data + 6, c + 2U < 8U ? (uint16_t)r[c + 2U] : 0U);
    return;
  }
  const int16_t *s = &capture_buf[i][read_bank == 0U ? 0 : 3];
  put16(data + 2, (uint16_t)s[0]);
  put16(data + 4, (uint16_t)s[1]);
  put16(data + 6, (uint16_t)s[2]);
}

void drive_app_poll(int (*send)(uint32_t id, const uint8_t data[8]))
{
  uint8_t data[8];
  uint32_t reply_id;
  /* CONTROL replies first: the Nucleo times command application by them. A full
   * TX FIFO leaves the reply queued for the next poll. */
  while (motor_cmd_reply_peek(&drive_cmd, &reply_id, data)) {
    if (!send(reply_id, data)) break;
    motor_cmd_reply_pop(&drive_cmd);
  }
  for (unsigned tail = atomic_load_explicit(&cfg_out_tail, memory_order_relaxed);
       tail != atomic_load_explicit(&cfg_out_head, memory_order_acquire); ++tail) {
    if (!send(MOTOR_CAN_ID_CONFIG_REPLY | MOTOR_CAN_ID, cfg_out[tail % CFG_SLOTS])) break; /* FIFO full: next poll */
    atomic_store_explicit(&cfg_out_tail, tail + 1U, memory_order_release);
  }
  if (status_pending) {
    const int16_t rpm = speed_out.w_hat_rpm;
    const int16_t iq = measured_iq();
    const qd_t ref = MC_GetIqdrefMotor1();
    const uint8_t flags = (uint8_t)((drive_app.stop_reason & 0x0FU) |
        (drive_app.active ? ESC_FLAG_ACTIVE : 0U) |
        (drive_app.derated ? ESC_FLAG_DERATED : 0U) |
        (drive_app.align_phase != ALIGN_IDLE || drive_app.capture_phase != CAPTURE_IDLE
             ? ESC_FLAG_ALIGN : 0U) |
        (drive_app.aligned ? ESC_FLAG_ALIGNED : 0U));
    data[0] = (uint8_t)rpm; data[1] = (uint8_t)((uint16_t)rpm >> 8);
    data[2] = (uint8_t)iq; data[3] = (uint8_t)((uint16_t)iq >> 8);
    data[4] = (uint8_t)ref.q; data[5] = (uint8_t)((uint16_t)ref.q >> 8);
    data[6] = (uint8_t)MC_GetSTMStateMotor1();
    data[7] = flags;
    if (send(STATUS_REPLY | MOTOR_CAN_ID, data)) {
      status_pending = 0U;
      ++drive_app.status_replies;
    }
  }
  if (diag_pending) {
    const uint32_t centivolts = vbus_centivolts();
    const uint16_t faults = MC_GetOccurredFaultsMotor1();
    const int16_t shift = s16_degrees(HALL_M1.PhaseShift);
    uint32_t load = (uint32_t)(drive_app.i2t_a2s * 100.0f / I2T_LIMIT_A2S);
    if (load > 255U) load = 255U;
    data[0] = (uint8_t)centivolts; data[1] = (uint8_t)(centivolts >> 8);
    data[2] = (uint8_t)faults; data[3] = (uint8_t)(faults >> 8);
    data[4] = (uint8_t)shift; data[5] = (uint8_t)((uint16_t)shift >> 8);
    data[6] = (uint8_t)drive_app.align_result;
    data[7] = (uint8_t)load;
    if (send(DIAG_REPLY | MOTOR_CAN_ID, data)) diag_pending = 0U;
  }
  if (ident_pending) {
    const volatile uint32_t *validity = DRIVE_APP_VALIDITY;
    put32(data, validity[3]);     /* crc */
    put32(data + 4, validity[2]); /* size */
    if (send(IDENT_REPLY | MOTOR_CAN_ID, data)) ident_pending = 0U;
  }
  const uint32_t read_count = rx_read_count;
  if (read_count != seen_read) {
    seen_read = read_count;
    const uint32_t word = rx_read_word;
    read_bank = (word >> 16) & 0x3U;
    read_ch67 = (word & ESC_RECORD_READ_CH67) != 0U && read_bank == 0U;
    if (read_bank == ESC_CAPTURE_INFO_BANK) {
      read_next = (word & 0x3FFU) == ESC_RECORD_INFO_SAMPLE ? ESC_RECORD_INFO_SAMPLE : 0U;
      read_end = read_next + 1U;
    } else {
      const uint32_t samples = rec_owner ? ESC_RECORD_SAMPLES : ESC_CAPTURE_SAMPLES;
      uint32_t count = (word >> 10) & 0x3FU;
      if (count > CAPTURE_READ_MAX) count = CAPTURE_READ_MAX;
      read_next = word & 0x3FFU;
      read_end = read_next + count;
      if (read_end > samples) read_end = samples;
      if (read_bank == 2U) read_end = read_next; /* no bank 2 */
    }
  }
  while (read_next < read_end) {
    capture_frame(data, read_next);
    if (!send(CAPTURE_REPLY | MOTOR_CAN_ID, data)) break; /* FIFO full: next poll */
    ++read_next;
  }
}

void MC_APP_PostMediumFrequencyHook_M1(void)
{
  drive_app_tick();
}
