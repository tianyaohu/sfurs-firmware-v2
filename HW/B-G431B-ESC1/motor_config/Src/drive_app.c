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

volatile struct drive_app_state drive_app;
volatile uint8_t drive_align_force;
volatile int16_t drive_align_angle;

/* ISR -> tick mailbox. Each payload is one word, published before its count. */
static volatile uint32_t rx_speed_word, rx_speed_count, rx_stop_count;
static volatile uint32_t rx_align_count, rx_config_word, rx_config_count;
static volatile uint8_t status_pending, diag_pending;
static uint32_t seen_speed, seen_stop, seen_align, seen_config;

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
    default: break;
  }
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

  const int idle = drive_app.align_phase == ALIGN_IDLE && !drive_app.active;
  if (align_count != seen_align) {
    seen_align = align_count;
    if (idle && state == IDLE && !stop_received) align_begin();
    else ++drive_app.rejected;
  }
  if (config_count != seen_config) {
    seen_config = config_count;
    if (idle && state == IDLE) apply_config(rx_config_word);
    else ++drive_app.rejected;
  }
  if (speed_received && drive_app.align_phase == ALIGN_IDLE && !stop_received)
    speed_command(rx_speed_word, state);

  if ((uint32_t)(now - drive_app.last_command_ms) >= ESC_COMMAND_TIMEOUT_MS) {
    drive_app.fault_latched = 0U;
    if (drive_app.active) stop(ESC_STOP_TIMEOUT);
  }
  i2t_tick(state);
}

int drive_app_idle(void)
{
  return !drive_app.active && drive_app.align_phase == ALIGN_IDLE &&
         rx_speed_count == seen_speed && rx_align_count == seen_align;
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
        (drive_app.align_phase != ALIGN_IDLE ? ESC_FLAG_ALIGN : 0U) |
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
    const uint32_t centivolts = (uint32_t)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super) *
        BusVoltageSensor_M1._Super.ConversionFactor * 100U / 65536U;
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
}

void MC_APP_PostMediumFrequencyHook_M1(void)
{
  drive_app_tick();
}
