#include "drive_app.h"
#include "can_datatypes.h"
#include "motor_controller_can.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_config_common.h"
#include "parameters_conversion.h"

/* I2T thermal model, values from the previous engineer's ea7b1a0 "Add I2T". */
#define I2T_LIMIT_A2S 30.0f
#define I2T_NOMINAL_A 3.26f
#define I2T_DERATED_A 3.0f
#define AMPS_PER_COUNT (1.0f / (float)CURRENT_CONV_FACTOR)

/* Friction feed-forward: the static part fades out as the wheel first speeds
 * up to FRICTION_FADE_RPM and then stays off (kinetic only), because the Hall
 * estimate scatters ~+-25 RPM at low speed and would otherwise toggle it. It
 * re-arms on a stop command or a stall (no forward speed for
 * FRICTION_STALL_MS). */
#define FRICTION_FADE_RPM 30
#define FRICTION_MIN_REF_RPM 2 /* below this the command means "stop" */
#define FRICTION_STALL_MS 100U

/* Parking (drive_app.h). The Nucleo slews to 0, so by the time the command is
 * 0 the wheel is nearly stopped; |speed| below PARK_RPM counts as standstill. */
#define PARK_MS 150U
#define PARK_RPM 10

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

volatile struct drive_app_state drive_app;
volatile uint8_t drive_align_force;
volatile int16_t drive_align_angle;

/* ISR -> tick mailbox. Each payload is one word, published before its count. */
static volatile uint32_t rx_speed_word, rx_speed_count, rx_stop_count;
static volatile uint32_t rx_align_count, rx_config_word, rx_config_count;
static volatile uint32_t rx_capture_word, rx_capture_count;
static volatile uint32_t rx_read_word, rx_read_count;
static volatile uint8_t status_pending, diag_pending, ident_pending;
static uint32_t seen_speed, seen_stop, seen_align, seen_config, seen_capture, seen_read;

/* FOC ISR <-> tick: the tick arms, the ISR records and disarms when full. */
static int16_t capture_buf[ESC_CAPTURE_SAMPLES][6];
static volatile uint32_t capture_armed, capture_index;
static volatile int16_t capture_step_counts;
static uint32_t capture_ms, capture_vbus_cv;
static uint32_t read_next, read_end, read_bank; /* poll context only */

static uint32_t ff_broken_free, ff_stall_ms;
static int32_t ff_peak; /* highest forward speed since the start, <= FRICTION_FADE_RPM */

static uint32_t align_ms, align_prev_hall;
static int16_t align_saved_shift, align_ref;
static int32_t align_sum;

static void set_iq_limit(int32_t counts)
{
  const int32_t integral = counts * (int32_t)PID_GetKIDivisor(&PIDSpeedHandle_M1);
  PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, (int16_t)counts);
  PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, (int16_t)-counts);
  PID_SetUpperIntegralTermLimit(&PIDSpeedHandle_M1, integral);
  PID_SetLowerIntegralTermLimit(&PIDSpeedHandle_M1, -integral);
  drive_app.iq_limit_counts = counts;
}

void drive_app_init(void)
{
  drive_app.iq_max_counts = (int32_t)IQMAX;
  set_iq_limit(drive_app.iq_max_counts);
  drive_app.ff_static_counts = FRICTION_FF_STATIC_MA_DEFAULT * (int32_t)CURRENT_CONV_FACTOR / 1000;
  drive_app.ff_kinetic_counts = FRICTION_FF_KINETIC_MA_DEFAULT * (int32_t)CURRENT_CONV_FACTOR / 1000;
#ifdef HALL_PHASE_SHIFT_OVERRIDE
  drive_app.aligned = 1U; /* measured shift compiled in by build_cli.py --hall-shift */
#endif
}

void drive_app_can_rx(uint32_t id, const uint8_t *data, uint32_t length)
{
  if (length != 4U || (id & MOTOR_CAN_ID) == 0U) return;
  const uint32_t word = CAN_U32_FROM_PAYLOAD(data);
  switch (CAN_COMMAND_FROM_ID(id)) {
    case SPEED_WRITE: rx_speed_word = word; ++rx_speed_count; break;
    case MOTOR_STOP: ++rx_stop_count; break;
    case STATUS_READ: status_pending = 1U; break;
    case DIAG_READ: diag_pending = 1U; break;
    case HALL_ALIGN: if (word == HALL_ALIGN_MAGIC) ++rx_align_count; break;
    case CONFIG_SET: rx_config_word = word; ++rx_config_count; break;
    case IDENT_READ: ident_pending = 1U; break;
    case CAPTURE:
      if ((word >> 24) == ESC_CAPTURE_START) { rx_capture_word = word; ++rx_capture_count; }
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

int16_t drive_speed_iq_ref(int16_t pi_out, int16_t ref_rpm, int16_t rpm)
{
  int32_t ff = 0;
  if (drive_app.active && (ref_rpm >= FRICTION_MIN_REF_RPM || ref_rpm <= -FRICTION_MIN_REF_RPM)) {
    const int32_t dir = ref_rpm > 0 ? 1 : -1;
    const int32_t along = dir * rpm; /* speed in the commanded direction */
    if (along > 0) ff_stall_ms = 0U;
    else if (++ff_stall_ms >= FRICTION_STALL_MS) { ff_broken_free = 0U; ff_peak = 0; }
    if (!ff_broken_free && along > ff_peak) ff_peak = along;
    if (ff_peak >= FRICTION_FADE_RPM) ff_broken_free = 1U;
    const int32_t st = drive_app.ff_static_counts, kin = drive_app.ff_kinetic_counts;
    ff = ff_broken_free ? kin : kin + (st - kin) * (FRICTION_FADE_RPM - ff_peak) / FRICTION_FADE_RPM;
    ff *= dir;
  } else {
    ff_broken_free = 0U;
    ff_peak = 0;
    ff_stall_ms = 0U;
  }
  int32_t out = (int32_t)pi_out + ff;
  const int32_t limit = drive_app.iq_limit_counts;
  if (out > limit) out = limit;
  if (out < -limit) out = -limit;
  return (int16_t)out;
}

static uint32_t vbus_centivolts(void)
{
  return (uint32_t)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super) *
         BusVoltageSensor_M1._Super.ConversionFactor * 100U / 65536U;
}

static void stop(uint32_t reason)
{
  (void)MC_StopMotor1();
  drive_app.active = 0U;
  drive_app.stop_reason = reason;
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

static void apply_config(uint32_t word)
{
  const int16_t value = (int16_t)(word >> 16);
  switch (word & 0xFFFFU) {
    case ESC_CFG_SPEED_KP: PID_SetKP(&PIDSpeedHandle_M1, value); break;
    case ESC_CFG_SPEED_KI: PID_SetKI(&PIDSpeedHandle_M1, value); break;
    case ESC_CFG_HALL_SHIFT_DEG:
      HALL_M1.PhaseShift = (int16_t)((int32_t)value * 65536 / 360);
      drive_app.aligned = 1U;
      break;
    case ESC_CFG_IQ_MAX_MA:
      if (value <= 0 || value > (int16_t)(IQMAX_A * 1000)) { ++drive_app.rejected; return; }
      drive_app.iq_max_counts = (int32_t)value * (int32_t)CURRENT_CONV_FACTOR / 1000;
      if (!drive_app.derated) set_iq_limit(drive_app.iq_max_counts);
      break;
    case ESC_CFG_CURRENT_KP:
      if (value < 0) { ++drive_app.rejected; return; }
      PID_SetKP(&PIDIqHandle_M1, value);
      PID_SetKP(&PIDIdHandle_M1, value);
      break;
    case ESC_CFG_CURRENT_KI:
      if (value < 0) { ++drive_app.rejected; return; }
      PID_SetKI(&PIDIqHandle_M1, value);
      PID_SetKI(&PIDIdHandle_M1, value);
      break;
    case ESC_CFG_FF_STATIC_MA:
    case ESC_CFG_FF_KINETIC_MA: {
      if (value < 0 || value > (int16_t)(IQMAX_A * 1000)) { ++drive_app.rejected; return; }
      const int32_t counts = (int32_t)value * (int32_t)CURRENT_CONV_FACTOR / 1000;
      if ((word & 0xFFFFU) == ESC_CFG_FF_STATIC_MA) drive_app.ff_static_counts = counts;
      else drive_app.ff_kinetic_counts = counts;
      break;
    }
    default: ++drive_app.rejected; break;
  }
}

static void speed_command(uint32_t word, uint16_t state)
{
  int32_t rpm = (int16_t)(word & 0xFFFFU);
  const uint16_t ramp_ms = (uint16_t)(word >> 16);
  if (rpm > MAX_APPLICATION_SPEED_RPM) rpm = MAX_APPLICATION_SPEED_RPM;
  if (rpm < -MAX_APPLICATION_SPEED_RPM) rpm = -MAX_APPLICATION_SPEED_RPM;
  ++drive_app.speed_commands;
  drive_app.last_command_ms = drive_app.now_ms;
  drive_app.command_rpm = rpm;
  if (drive_app.fault_latched) return;
  if (!drive_app.aligned) { ++drive_app.rejected; return; }
  if (drive_app.active) {
    MC_ProgramSpeedRampMotor1((int16_t)RPM_2_SPEED_UNIT(rpm), ramp_ms);
    return;
  }
  if (state == FAULT_OVER && MC_GetCurrentFaultsMotor1() == MC_NO_FAULTS) {
    (void)MC_AcknowledgeFaultMotor1();
    state = MC_GetSTMStateMotor1();
  }
  if (state != IDLE || rpm == 0) return;
  MC_ProgramSpeedRampMotor1((int16_t)RPM_2_SPEED_UNIT(rpm), ramp_ms);
  if (MC_StartMotor1()) {
    drive_app.active = 1U;
    drive_app.stop_reason = ESC_STOP_NONE;
  } else {
    ++drive_app.rejected;
  }
}

static void park_tick(uint16_t state)
{
  const int32_t rpm = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
  if (!drive_app.active || state != RUN || drive_app.command_rpm != 0 ||
      rpm > PARK_RPM || rpm < -PARK_RPM) {
    drive_app.park_ms = 0U;
    return;
  }
  if (++drive_app.park_ms >= PARK_MS) {
    drive_app.park_ms = 0U;
    stop(ESC_STOP_PARKED);
  }
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

void drive_app_tick(void)
{
  const uint32_t now = ++drive_app.now_ms;
  uint16_t state = (uint16_t)MC_GetSTMStateMotor1();
  const uint32_t speed_count = rx_speed_count, stop_count = rx_stop_count;
  const uint32_t align_count = rx_align_count, config_count = rx_config_count;
  const uint32_t capture_count = rx_capture_count;
  const int stop_received = stop_count != seen_stop;
  const int speed_received = speed_count != seen_speed;
  seen_stop = stop_count;
  seen_speed = speed_count;

  if (drive_app.active && (state == FAULT_NOW || state == FAULT_OVER || state == IDLE)) {
    drive_app.active = 0U;
    drive_app.fault_latched = 1U;
    drive_app.stop_reason = ESC_STOP_FAULT;
  }
  if (stop_received) {
    drive_app.fault_latched = 0U;
    if (drive_app.align_phase != ALIGN_IDLE && drive_app.align_phase != ALIGN_STOPPING) {
      align_finish(ESC_ALIGN_ABORTED);
    } else if (drive_app.capture_phase != CAPTURE_IDLE &&
               drive_app.capture_phase != CAPTURE_STOPPING) {
      capture_finish(ESC_CAPTURE_ABORTED);
    } else if (state == FAULT_OVER) {
      /* Clear a fault whose cause is gone (e.g. undervolt latched on USB power
       * before the battery came on); align and boot entry need IDLE. */
      if (MC_GetCurrentFaultsMotor1() == MC_NO_FAULTS) (void)MC_AcknowledgeFaultMotor1();
      state = (uint16_t)MC_GetSTMStateMotor1();
    } else if (state != IDLE) {
      stop(ESC_STOP_COMMAND);
    }
    drive_app.active = 0U;
  }
  align_tick(state);
  capture_tick(state);

  const int idle = drive_app.align_phase == ALIGN_IDLE &&
                   drive_app.capture_phase == CAPTURE_IDLE && !drive_app.active;
  if (align_count != seen_align) {
    seen_align = align_count;
    if (idle && state == IDLE && !stop_received) align_begin();
    else ++drive_app.rejected;
  }
  if (capture_count != seen_capture) {
    seen_capture = capture_count;
    if (idle && state == IDLE && !stop_received && drive_app.align_phase == ALIGN_IDLE) {
      capture_begin(rx_capture_word & 0xFFFFU);
    } else {
      drive_app.capture_status = ESC_CAPTURE_REJECTED;
      ++drive_app.rejected;
    }
  }
  if (config_count != seen_config) {
    seen_config = config_count;
    if (idle && state == IDLE) apply_config(rx_config_word);
    else ++drive_app.rejected;
  }
  if (speed_received && drive_app.align_phase == ALIGN_IDLE &&
      drive_app.capture_phase == CAPTURE_IDLE && !stop_received)
    speed_command(rx_speed_word, state);

  if ((uint32_t)(now - drive_app.last_command_ms) >= ESC_COMMAND_TIMEOUT_MS) {
    drive_app.fault_latched = 0U;
    if (drive_app.active) stop(ESC_STOP_TIMEOUT);
  }
  park_tick(state);
  i2t_tick(state);
}

int drive_app_idle(void)
{
  return !drive_app.active && drive_app.align_phase == ALIGN_IDLE &&
         drive_app.capture_phase == CAPTURE_IDLE && rx_speed_count == seen_speed &&
         rx_align_count == seen_align && rx_capture_count == seen_capture;
}

static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }

/* One CAPTURE_REPLY for sample i of bank read_bank. */
static void capture_frame(uint8_t data[8], uint32_t i)
{
  put16(data, (read_bank << 12) | i);
  if (read_bank == ESC_CAPTURE_INFO_BANK) {
    put16(data + 2, drive_app.capture_status);
    put16(data + 4, capture_vbus_cv);
    put16(data + 6, capture_index);
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
  if (status_pending) {
    const int16_t rpm = (int16_t)SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
    const qd_t iqd = MC_GetIqdMotor1(), ref = MC_GetIqdrefMotor1();
    const uint8_t flags = (uint8_t)((drive_app.stop_reason & 0x0FU) |
        (drive_app.active ? ESC_FLAG_ACTIVE : 0U) |
        (drive_app.derated ? ESC_FLAG_DERATED : 0U) |
        (drive_app.align_phase != ALIGN_IDLE || drive_app.capture_phase != CAPTURE_IDLE
             ? ESC_FLAG_ALIGN : 0U) |
        (drive_app.aligned ? ESC_FLAG_ALIGNED : 0U));
    data[0] = (uint8_t)rpm; data[1] = (uint8_t)((uint16_t)rpm >> 8);
    data[2] = (uint8_t)iqd.q; data[3] = (uint8_t)((uint16_t)iqd.q >> 8);
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
    read_next = read_bank == ESC_CAPTURE_INFO_BANK ? 0U : (word & 0x3FFU);
    uint32_t count = read_bank == ESC_CAPTURE_INFO_BANK ? 1U : ((word >> 10) & 0x3FU);
    if (count > CAPTURE_READ_MAX) count = CAPTURE_READ_MAX;
    read_end = read_next + count;
    if (read_end > ESC_CAPTURE_SAMPLES) read_end = ESC_CAPTURE_SAMPLES;
    if (read_bank == 2U) read_end = read_next; /* no bank 2 */
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
