/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_datatypes.h"
#include "motor_controller_can.h"
#include "mc_api.h"
#ifdef ESC_BOOT_OFFSET_APP
#include "boot_entry.h"
#include "mc_config.h"
#endif
#ifdef ESC_LOW_CURRENT_TEST
#include "parameters_conversion.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if (MOTOR_CAN_ID == 0U)
#error "Define exactly one motor macro: MOTOR_NE, MOTOR_SE, MOTOR_SW, or MOTOR_NW"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac3;

FDCAN_HandleTypeDef hfdcan1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

FDCAN_TxHeaderTypeDef can_tx_header;
FDCAN_RxHeaderTypeDef can_rx_header;
FDCAN_FilterTypeDef   can_filter_config;
uint8_t             can_tx_data[8];
uint8_t             can_rx_data[8];
#ifdef ESC_BOOT_OFFSET_APP
/* HAL copies up to 64 bytes before the callback can inspect DLC. */
static uint8_t boot_can_rx_data[64];
static volatile uint8_t boot_entry_pending;
#endif
static volatile uint8_t telemetry_pending = 0;
static uint8_t telemetry_hall_state = 0;
static uint32_t telemetry_hall_changes = 0;
volatile uint32_t telemetry_reply_count = 0;
#ifdef ESC_LOW_CURRENT_TEST
/* Deliberately disarmed at boot. Arming is a separate, explicit debugger action. */
volatile uint32_t test_drive_armed = 0;
volatile uint32_t test_drive_run_count = 0; /* Completed start count, diagnostics only. */
volatile uint32_t test_drive_active = 0;
volatile uint32_t test_drive_start_tick = 0;
volatile uint32_t test_drive_stop_reason = 0; /* TEST_STOP_* above. */
volatile int32_t test_drive_iq = 0;
volatile int32_t test_drive_id = 0;
volatile uint32_t test_drive_peak_l1 = 0;
volatile uint32_t test_calibration_request = 0; /* Explicit CAL1 (TEST_REQ_CAL1). */
volatile uint32_t test_calibration_result = 0; /* 0=none,1=pending,2=done,3=rejected,4=failed */
volatile uint32_t test_calibration_valid = 0;
volatile uint32_t test_drive_current_valid = 0;
/* Opt-in torque diagnostic only: DARM, DRUN, DSTP. Never set automatically. */
volatile uint32_t test_direct_request = 0;
volatile uint32_t test_direct_result = 0; /* 1=armed,2=rejected,3=started,4=start rejected,5=STOP */
volatile uint32_t test_direct_last_request = 0;
volatile int32_t test_drive_iq_reference = 0;
volatile int32_t test_drive_id_reference = 0;
volatile int32_t test_drive_measured_rpm = 0;
volatile int32_t test_drive_hall_net_edges = 0;
volatile uint32_t test_drive_hall_edges = 0;
volatile uint32_t test_drive_hall_invalid = 0;
static volatile uint32_t test_drive_hall_last_tick = 0;
volatile uint32_t test_profile_reference_limit_ma = (uint32_t)(ESC_TEST_REFERENCE_A * 1000);
static volatile uint32_t test_calibration_tick = 0;
static volatile uint32_t test_drive_arm_tick = 0;
static volatile uint8_t test_current_pending = 0;
static volatile uint8_t test_monitor_ready = 0;
static volatile uint32_t test_drive_start_permit = 0;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_CORDIC_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef ESC_BOOT_OFFSET_APP
static int BootEntrySafe(void)
{
  /* Inspect pending commands without consuming MCSDK command-state flags.
   * Ordinary firmware has no persistent arm latch: IDLE + no direct/buffered
   * start/drive/calibration command is its disarmed state. */
  if (MC_GetSTMStateMotor1() != IDLE ||
      MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS ||
      MC_GetOccurredFaultsMotor1() != MC_NO_FAULTS ||
      Mci[0].DirectCommand != MCI_NO_COMMAND ||
      Mci[0].CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED ||
      MC_GetMecSpeedAverageMotor1() != 0) return 0;
#ifdef ESC_LOW_CURRENT_TEST
  if (test_drive_armed || test_drive_active || test_drive_start_permit ||
      test_direct_request || test_calibration_request ||
      test_calibration_result == 1U) return 0;
#endif
  return 1;
}

static void BootEntryPoll(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (boot_entry_pending) {
    boot_entry_pending = 0U;
    if (BootEntrySafe()) {
      esc_boot_mailbox_write(ESC_BOOT_MAILBOX, MOTOR_CAN_ID);
      __DSB();
      NVIC_SystemReset();
    }
  }
  __set_PRIMASK(primask);
}
#endif
#ifdef ESC_LOW_CURRENT_TEST
/* Explicit diagnostic request words (ASCII): CAL1, DARM, DRUN, DSTP. */
#define TEST_REQ_CAL1 0x43414C31U
#define TEST_REQ_DARM 0x4441524DU
#define TEST_REQ_DRUN 0x4452554EU
#define TEST_REQ_DSTP 0x44535450U
/* Stop reasons: 1=timeout, 2=CAN/direct stop, 3=start rejected,
 * 4=sampled current, 5=sampled speed, 6=fault/calibration loss,
 * 7=invalid Hall, 8=turn-count target, 9=Hall stall. */
#define TEST_STOP_TIMEOUT 1U
#define TEST_STOP_REQUESTED 2U
#define TEST_STOP_START_REJECTED 3U
#define TEST_STOP_CURRENT 4U
#define TEST_STOP_SPEED 5U
#define TEST_STOP_FAULT 6U
#define TEST_STOP_HALL_INVALID 7U
#define TEST_STOP_COUNT 8U
#define TEST_STOP_STALL 9U
bool TestDriveConsumeStartPermit(void)
{
  if (test_drive_start_permit == 0U) return false;
  test_drive_start_permit = 0U;
  return true;
}

/* IRQ-fed Hall sector counts, not polling-derived revolutions. Positive order
 * matches the SDK Hall decoder. Physical pole pairs/counting need validation. */
void TestDriveHallTransition(uint8_t previous, uint8_t next)
{
  static const uint8_t positive_next[7] = {0, 3, 6, 2, 5, 1, 4};
  if ((!ESC_TEST_RPM_PROFILE && !ESC_TEST_COUNT_PROFILE) || test_drive_active == 0U) return;
  if (previous < 1U || previous > 6U || next < 1U || next > 6U ||
      (positive_next[previous] != next && positive_next[next] != previous)) {
    ++test_drive_hall_invalid;
    return;
  }
  ++test_drive_hall_edges;
  test_drive_hall_last_tick = HAL_GetTick();
  test_drive_hall_net_edges += positive_next[previous] == next ? 1 : -1;
}

bool TestDriveArm(uint32_t magic)
{
  if (magic != ESC_TEST_ARM_MAGIC ||
      test_drive_active != 0U || MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS)
    return false;
  if (MC_GetSTMStateMotor1() == FAULT_OVER) (void)MC_AcknowledgeFaultMotor1();
  if (MC_GetSTMStateMotor1() != IDLE) return false;
  /* Auto-CAL (ESC_TEST_AUTO_CAL_ON_ARM==1, default on all wheels): first ARM
   * with no valid offsets starts the same RAM-only measurement as manual
   * CAL1 (result 1=pending). Retry ARM after pending->done (result 2/valid
   * 1). Never starts the motor here; faults/non-IDLE/active above still
   * hard-reject. */
  if (test_calibration_valid == 0U || test_calibration_result != 2U)
  {
    if (ESC_TEST_AUTO_CAL_ON_ARM != 0U && test_calibration_result != 1U)
    {
      test_calibration_valid = 0U;
      test_calibration_tick = HAL_GetTick();
      test_calibration_result = 1U;
      if (!MC_StartPolarizationOffsetsMeasurementMotor1()) test_calibration_result = 3U;
    }
    return false;
  }
  test_drive_arm_tick = HAL_GetTick();
  test_drive_armed = ESC_TEST_ARM_MAGIC;
  return true;
}

/* Shared by CAN and direct diagnostics: 0=preflight rejected, 1=started,
 * 2=SDK start rejected. Every accepted start consumes ARM and increments the
 * run counter; repeats need a fresh ARM after standstill/cooldown. */
uint32_t TestDriveStart(int16_t value, uint16_t ramp_ms)
{
  if (test_drive_armed != ESC_TEST_ARM_MAGIC ||
      test_calibration_result != 2U || test_calibration_valid == 0U ||
      MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS || MC_GetSTMStateMotor1() != IDLE)
    return 0U;
  if (ESC_TEST_TORQUE_PROFILE)
  {
    if (value <= 0 || value > (int16_t)(ESC_TEST_REFERENCE_A * CURRENT_CONV_FACTOR) ||
        ramp_ms < 200U || ramp_ms > 500U) return 0U;
  }
  else if (ESC_TEST_RPM_PROFILE) {
    /* Negated-gain profile: bounded bidirectional range. Reverse was
     * validated; forward is stabilized by the same sign argument but must
     * be proven on hardware starting at +60. Exact 0 rejected (no motion,
     * would only stall-trip). Ramp stays exactly 500 ms. */
    if (value == 0 || value < ESC_TEST_RPM_MIN || value > ESC_TEST_RPM_MAX ||
        ramp_ms != 500U) return 0U;
  }
  else if (value < 12 || value > 60 || ramp_ms < 500U || ramp_ms > 1000U) return 0U;
  test_drive_armed = 0U;
  ++test_drive_run_count;
  test_drive_peak_l1 = 0U;
  test_drive_hall_net_edges = 0;
  test_drive_hall_edges = test_drive_hall_invalid = 0U;
  test_drive_start_tick = HAL_GetTick();
  test_drive_hall_last_tick = test_drive_start_tick;
  if (ESC_TEST_TORQUE_PROFILE) MC_ProgramTorqueRampMotor1(value, ramp_ms);
  else MC_ProgramSpeedRampMotor1(RPM_2_SPEED_UNIT(value), ramp_ms);
  test_drive_active = 1U;
  test_drive_start_permit = 1U;
  bool started = MC_StartMotor1();
  test_drive_start_permit = 0U;
  if (!started)
  {
    test_drive_active = 0U;
    test_drive_stop_reason = TEST_STOP_START_REJECTED;
    return 2U;
  }
  return 1U;
}

void TestDirectService(void)
{
  uint32_t request = test_direct_request;
  if (request == 0U) return;
  test_direct_request = 0U;
  test_direct_last_request = request;
  if (!ESC_TEST_GUARDED_MOTION) { test_direct_result = 2U; return; }
  if (request == TEST_REQ_DSTP)
  {
    test_drive_armed = test_drive_active = 0U;
    test_drive_stop_reason = TEST_STOP_REQUESTED;
    (void)MC_StopMotor1();
    test_direct_result = 5U;
  }
  else if (request == TEST_REQ_DARM) /* DARM */
    test_direct_result = TestDriveArm(ESC_TEST_ARM_MAGIC) ? 1U : 2U;
  else if (request == TEST_REQ_DRUN) /* DRUN: fixed profile reference */
  {
    uint32_t started = ESC_TEST_RPM_PROFILE ? TestDriveStart(ESC_TEST_TARGET_RPM, 500U) :
        TestDriveStart((int16_t)(ESC_TEST_REFERENCE_A * CURRENT_CONV_FACTOR), 200U);
    test_direct_result = started == 1U ? 3U : started == 2U ? 4U : 2U;
  }
  else test_direct_result = 2U;
}

/* Single-responsibility guard helpers: each checks one stop condition and
 * stops the drive with its TEST_STOP_* reason. TestDriveSafetyTick keeps the
 * original evaluation order so stop-reason priority is unchanged. */
static int TestDriveGuardFault(void)
{
  if (!(ESC_TEST_GUARDED_MOTION && test_drive_active != 0U &&
      (test_calibration_valid == 0U || MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS)))
    return 0;
  test_drive_active = test_drive_armed = 0U;
  test_drive_stop_reason = TEST_STOP_FAULT;
  (void)MC_StopMotor1();
  return 1;
}

static int TestDriveGuardSpeed(void)
{
  /* Bidirectional RPM2 accepts both command endpoints. Equality must not
   * immediately trip a legal +/-900 RPM target; other profiles keep their
   * existing at-limit trip behavior. No overspeed margin is added. */
  const int overspeed = ESC_TEST_RPM_PROFILE && ESC_TEST_RPM_MIN < 0 ?
      (test_drive_measured_rpm > ESC_TEST_MAX_MOTOR_RPM ||
       test_drive_measured_rpm < -ESC_TEST_MAX_MOTOR_RPM) :
      (test_drive_measured_rpm >= ESC_TEST_MAX_MOTOR_RPM ||
       test_drive_measured_rpm <= -ESC_TEST_MAX_MOTOR_RPM);
  if (!(ESC_TEST_GUARDED_MOTION && test_drive_active != 0U &&
      overspeed))
    return 0;
  test_drive_active = 0U;
  test_drive_stop_reason = TEST_STOP_SPEED;
  (void)MC_StopMotor1();
  return 1;
}

static int TestDriveGuardHallInvalid(void)
{
  if (!((ESC_TEST_RPM_PROFILE || ESC_TEST_COUNT_PROFILE) && test_drive_active != 0U &&
      test_drive_hall_invalid != 0U))
    return 0;
  test_drive_active = test_drive_armed = 0U;
  test_drive_stop_reason = TEST_STOP_HALL_INVALID;
  (void)MC_StopMotor1();
  return 1;
}

static int TestDriveGuardStall(void)
{
  if (!((ESC_TEST_RPM_PROFILE || ESC_TEST_COUNT_PROFILE) && test_drive_active != 0U &&
      (uint32_t)(HAL_GetTick() - test_drive_hall_last_tick) >= ESC_TEST_HALL_STALL_MS))
    return 0;
  test_drive_active = test_drive_armed = 0U;
  test_drive_stop_reason = TEST_STOP_STALL; /* No valid Hall progress, including startup. */
  (void)MC_StopMotor1();
  return 1;
}

static int TestDriveGuardCount(void)
{
  if (!(ESC_TEST_COUNT_PROFILE && test_drive_active != 0U &&
      (test_drive_hall_net_edges >= (int32_t)ESC_TEST_TARGET_NET_EDGES ||
      test_drive_hall_net_edges <= -(int32_t)ESC_TEST_TARGET_NET_EDGES)))
    return 0;
  test_drive_active = test_drive_armed = 0U;
  test_drive_stop_reason = TEST_STOP_COUNT; /* Turn-count target reached. */
  (void)MC_StopMotor1();
  return 1;
}

static int TestDriveGuardCurrent(qd_t measured)
{
  uint32_t l1;
  if (!(test_drive_active != 0U && test_drive_current_valid != 0U))
    return 0;
  l1 = (uint32_t)(measured.q < 0 ? -(int32_t)measured.q : measured.q) +
       (uint32_t)(measured.d < 0 ? -(int32_t)measured.d : measured.d);
  if (l1 > test_drive_peak_l1) test_drive_peak_l1 = l1;
  /* Conservative L1 magnitude, sampled at SysTick rate; not peak protection. */
  if (l1 < (uint32_t)(ESC_TEST_SAMPLED_STOP_A * CURRENT_CONV_FACTOR))
    return 0;
  test_drive_active = 0U;
  test_drive_stop_reason = TEST_STOP_CURRENT;
  (void)MC_StopMotor1();
  return 1;
}

static int TestDriveGuardTimeout(void)
{
  if (!(test_drive_active != 0U &&
      (uint32_t)(HAL_GetTick() - test_drive_start_tick) >= ESC_TEST_TIMEOUT_MS))
    return 0;
  test_drive_active = 0U;
  test_drive_stop_reason = TEST_STOP_TIMEOUT;
  (void)MC_StopMotor1();
  return 1;
}

/* Called from SysTick, independent of CAN reception and the application loop. */
void TestDriveSafetyTick(void)
{
  if (test_monitor_ready == 0U) return;
  PolarizationOffsets_t offsets;
  test_calibration_valid = MC_GetPolarizationOffsetsMotor1(&offsets) ? 1U : 0U;
  test_drive_current_valid = test_calibration_valid != 0U &&
      MC_GetSTMStateMotor1() == RUN && MC_GetCurrentFaultsMotor1() == MC_NO_FAULTS;
  if (test_drive_armed != 0U &&
      (uint32_t)(HAL_GetTick() - test_drive_arm_tick) >= 5000U)
    test_drive_armed = 0U;
  qd_t measured = {0};
  if (test_drive_current_valid != 0U) measured = MC_GetIqdMotor1();
  test_drive_iq = measured.q;
  test_drive_id = measured.d;
  qd_t reference = MC_GetIqdrefMotor1();
  test_drive_iq_reference = reference.q;
  test_drive_id_reference = reference.d;
  test_drive_measured_rpm = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
  (void)TestDriveGuardFault();
  (void)TestDriveGuardSpeed();
  (void)TestDriveGuardHallInvalid();
  (void)TestDriveGuardStall();
  (void)TestDriveGuardCount();
  (void)TestDriveGuardCurrent(measured);
  (void)TestDriveGuardTimeout();
  if (test_calibration_result == 1U &&
      (uint32_t)(HAL_GetTick() - test_calibration_tick) >= 1000U)
  {
    test_calibration_result = 4U;
    (void)MC_StopMotor1();
  }
}

/* Calibration-only SDK path returns to IDLE, never grants a start permit. */
void TestCalibrationService(void)
{
  uint32_t request = test_calibration_request;
  if (request != 0U)
  {
    test_calibration_request = 0U;
    if (request != TEST_REQ_CAL1 || test_drive_active != 0U ||
        test_drive_armed != 0U ||
        test_calibration_result == 1U || MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS)
    {
      if (test_calibration_result != 1U) test_calibration_result = 3U;
      return;
    }
    if (MC_GetSTMStateMotor1() == FAULT_OVER) (void)MC_AcknowledgeFaultMotor1();
    if (MC_GetSTMStateMotor1() != IDLE)
    {
      test_calibration_result = 3U;
      return;
    }
    test_calibration_valid = 0U;
    test_calibration_tick = HAL_GetTick();
    test_calibration_result = 1U;
    if (!MC_StartPolarizationOffsetsMeasurementMotor1()) test_calibration_result = 3U;
    return;
  }
  if (test_calibration_result == 1U)
  {
    if (MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS)
    {
      test_calibration_result = 4U;
      (void)MC_StopMotor1();
    }
    else if (MC_GetSTMStateMotor1() == IDLE && test_calibration_valid != 0U)
      test_calibration_result = 2U;
  }
}

#if defined(ESC_TUNE_GAIN_SET)
/* Runtime RAM-only speed-PI numerator update (CAN GAIN_SET, 0x500|dev) for
 * staged auto-tune sweeps. Only hKpGain/hKiGain change; SP_KPDIV/SP_KIDIV
 * stay at the compiled values so the integral accumulator limit (scaled by
 * SP_KIDIV at PID init) never changes. Requires armed + IDLE + no faults,
 * does NOT consume or refresh the 5 s armed window, and never starts the
 * motor. RPM2 negated-gain convention: both numerators must stay negative;
 * magnitude floors reject near-zero gains that would only stall-trip.
 * RAM-only: any reset restores the compiled ESC_TEST_SPEED_KP/KI defaults.
 * result: 0=none, 1=applied, 2=state rejected, 3=bounds rejected. */
volatile int32_t test_speed_kp_gain = ESC_TEST_SPEED_KP;
volatile int32_t test_speed_ki_gain = ESC_TEST_SPEED_KI;
volatile uint32_t test_gain_set_result = 0, test_gain_set_count = 0;
uint32_t TestDriveGainSet(uint32_t packed)
{
  int16_t kp = (int16_t)(packed & 0xFFFFU);
  int16_t ki = (int16_t)(packed >> 16);
  ++test_gain_set_count;
  if (test_drive_armed != ESC_TEST_ARM_MAGIC || test_drive_active != 0U ||
      MC_GetCurrentFaultsMotor1() != MC_NO_FAULTS || MC_GetSTMStateMotor1() != IDLE)
  {
    test_gain_set_result = 2U;
    return 2U;
  }
  if (kp == (int16_t)-32768 || ki == (int16_t)-32768 || kp > -1000 || ki > -1)
  {
    test_gain_set_result = 3U;
    return 3U;
  }
  PID_SetKP(&PIDSpeedHandle_M1, kp);
  PID_SetKI(&PIDSpeedHandle_M1, ki);
  test_speed_kp_gain = kp;
  test_speed_ki_gain = ki;
  test_gain_set_result = 1U;
  return 1U;
}
#endif
/* HARNESS HELPERS END */
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_CORDIC_Init();
  MX_DAC3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_MotorControl_Init();
  MX_FDCAN1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
#ifdef ESC_LOW_CURRENT_TEST
  test_monitor_ready = 1U;
#endif

  /* Telemetry replies never issue a motor start or setpoint command. */
  can_tx_header.IdType = FDCAN_STANDARD_ID;
  can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_header.DataLength = FDCAN_DLC_BYTES_4;
  can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  can_tx_header.FDFormat = FDCAN_FD_CAN;
  can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  can_tx_header.MessageMarker = 0x00;

  can_filter_config.IdType = FDCAN_STANDARD_ID;
  can_filter_config.FilterIndex = 0U;
  can_filter_config.FilterType = FDCAN_FILTER_MASK;
  can_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  can_filter_config.FilterID1 = MOTOR_CAN_ID;
  can_filter_config.FilterID2 = MOTOR_CAN_ID;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
	  Error_Handler();
  }

  #ifdef MOTOR_NE
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  #endif

  #ifdef MOTOR_SE
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  #endif

  #ifdef MOTOR_SW
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  #endif

  #ifdef MOTOR_NW
  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef ESC_BOOT_OFFSET_APP
    BootEntryPoll();
#endif
#ifdef ESC_LOW_CURRENT_TEST
    TestCalibrationService();
    TestDirectService();
#endif
    uint8_t hall =
        (HAL_GPIO_ReadPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin) == GPIO_PIN_SET ? 1U : 0U) |
        (HAL_GPIO_ReadPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin) == GPIO_PIN_SET ? 2U : 0U) |
        (HAL_GPIO_ReadPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin) == GPIO_PIN_SET ? 4U : 0U);
    static uint8_t hall_initialized = 0;
    if (hall_initialized != 0U && hall != telemetry_hall_state)
    {
      ++telemetry_hall_changes;
    }
    hall_initialized = 1U;
    telemetry_hall_state = hall;

#ifdef ESC_LOW_CURRENT_TEST
    if (test_current_pending != 0U && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0U)
    {
      FDCAN_TxHeaderTypeDef reply = can_tx_header;
      uint8_t data[8];
      uint16_t flags = (uint16_t)(MC_GetCurrentFaultsMotor1() | 0x8000U |
          (test_drive_armed != 0U ? 0x4000U : 0U) |
          (test_drive_run_count != 0U ? 0x2000U : 0U) |
          (test_drive_active != 0U ? 0x1000U : 0U) |
          (test_calibration_valid != 0U ? 0x0800U : 0U));
      reply.Identifier = TEST_CURRENT_REPLY | MOTOR_CAN_ID;
      reply.DataLength = FDCAN_DLC_BYTES_8;
      CAN_PAYLOAD_FROM_U32(data, (uint32_t)(uint16_t)test_drive_iq |
                               ((uint32_t)(uint16_t)test_drive_id << 16));
      uint32_t peak = test_drive_peak_l1 > 65535U ? 65535U : test_drive_peak_l1;
      CAN_PAYLOAD_FROM_U32(&data[4], (uint32_t)flags | (peak << 16));
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &reply, data) == HAL_OK)
        test_current_pending = 0U;
    }
#endif

    /* Send outside the CAN ISR. A full TX FIFO defers the reply. */
    if (telemetry_pending != 0U && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0U)
    {
      FDCAN_TxHeaderTypeDef reply = can_tx_header;
      uint8_t data[8];
      int16_t rpm = SPEED_UNIT_2_RPM(MC_GetMecSpeedAverageMotor1());
      reply.Identifier = SPEED_READ_REPLY | MOTOR_CAN_ID;
      reply.DataLength = FDCAN_DLC_BYTES_8;
      data[0] = (uint8_t)((uint16_t)rpm & 0xFFU);
      data[1] = (uint8_t)((uint16_t)rpm >> 8);
      data[2] = (uint8_t)MC_GetSTMStateMotor1();
      data[3] = telemetry_hall_state;
      CAN_PAYLOAD_FROM_U32(&data[4], telemetry_hall_changes);
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &reply, data) == HAL_OK)
      {
        telemetry_pending = 0U;
        ++telemetry_reply_count;
      }
    }
	MCI_State_t s = MC_GetSTMStateMotor1();
	if ((s == FAULT_OVER) && (MC_GetCurrentFaultsMotor1() == MC_NO_FAULTS))
	{
#ifndef ESC_LOW_CURRENT_TEST
	  (void)MC_AcknowledgeFaultMotor1();
#endif
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP3_ADC2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 17;
  hfdcan1.Init.NominalTimeSeg1 = 152;
  hfdcan1.Init.NominalTimeSeg2 = 17;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 9;
  hfdcan1.Init.DataTimeSeg1 = 10;
  hfdcan1.Init.DataTimeSeg2 = 9;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 3;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 3;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = M1_HALL_TIM_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_HALL_IC_FILTER;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1843200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
	  return;
  }

#ifdef ESC_BOOT_OFFSET_APP
  uint8_t *rx_payload = boot_can_rx_data;
#else
  uint8_t *rx_payload = can_rx_data;
#endif
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can_rx_header, rx_payload) != HAL_OK)
  {
	  return;
  }

#ifdef ESC_BOOT_OFFSET_APP
  if (boot_entry_pending) return; /* No drive command may race queued reset. */
  if (can_rx_header.IdType == FDCAN_STANDARD_ID &&
      can_rx_header.RxFrameType == FDCAN_DATA_FRAME &&
      can_rx_header.FDFormat == FDCAN_FD_CAN &&
      can_rx_header.BitRateSwitch == FDCAN_BRS_OFF &&
      can_rx_header.DataLength == FDCAN_DLC_BYTES_8 &&
      esc_boot_request_valid(can_rx_header.Identifier, rx_payload, MOTOR_CAN_ID)) {
    if (BootEntrySafe()) boot_entry_pending = 1U;
    return;
  }
#endif
  if ((can_rx_header.IdType != FDCAN_STANDARD_ID) ||
      (can_rx_header.RxFrameType != FDCAN_DATA_FRAME) ||
      (can_rx_header.DataLength != FDCAN_DLC_BYTES_4))
  {
	  return;
  }

  if ((can_rx_header.Identifier & MOTOR_CAN_ID) == 0U)
  {
	  return;
  }

#ifdef ESC_BOOT_OFFSET_APP
  for (unsigned i = 0; i < 4U; ++i) can_rx_data[i] = rx_payload[i];
#endif

  if (CAN_COMMAND_FROM_ID(can_rx_header.Identifier) == SPEED_READ)
  {
      telemetry_pending = 1U;
      return;
  }

#ifdef ESC_LOW_CURRENT_TEST
  if (can_rx_header.Identifier == (TEST_CURRENT_READ | MOTOR_CAN_ID))
  {
    test_current_pending = 1U;
    return;
  }
  /* CAN CAL1 (0x480|dev, payload "CAL1"): request the RAM-only offset
   * measurement via TestCalibrationService. Accepted only when MCI is IDLE
   * with no faults; otherwise ignored. The service keeps its own gating
   * (active/armed/pending rejections) untouched. */
  if (can_rx_header.Identifier == (TEST_CAL_REQ | MOTOR_CAN_ID))
  {
    if (CAN_U32_FROM_PAYLOAD(can_rx_data) == TEST_REQ_CAL1 &&
        MC_GetSTMStateMotor1() == IDLE && MC_GetCurrentFaultsMotor1() == MC_NO_FAULTS)
      test_calibration_request = TEST_REQ_CAL1;
    return;
  }
  /* Explicit arming may acknowledge a cleared USB-only undervoltage fault.
   * Never arm through a present fault. Accepted starts are counted, never latched. */
  if (can_rx_header.Identifier == (TEST_ARM | MOTOR_CAN_ID))
  {
    (void)TestDriveArm(CAN_U32_FROM_PAYLOAD(can_rx_data));
    return;
  }
#if defined(ESC_TUNE_GAIN_SET)
  /* GAIN_SET (0x500|dev): armed+IDLE speed-PI numerator update; acceptance
   * is only proven by test_gain_set_result/test_speed_kp_gain readback. */
  if (can_rx_header.Identifier == (TEST_GAIN_SET | MOTOR_CAN_ID))
  {
    (void)TestDriveGainSet(CAN_U32_FROM_PAYLOAD(can_rx_data));
    return;
  }
#endif
  /* Stop is always accepted for an addressed motor; never refresh the run deadline. */
  if (CAN_COMMAND_FROM_ID(can_rx_header.Identifier) == MOTOR_STOP)
  {
    test_drive_armed = 0U;
    test_drive_active = 0U;
    test_drive_stop_reason = 2U;
    (void)MC_StopMotor1();
    return;
  }
  /* Diagnostic torque uses a distinct command; legacy speed/button frames are
   * ignored. Default old 0.25 A profile retains its bounded speed command. */
  uint32_t test_command = ESC_TEST_TORQUE_PROFILE ? ESC_TEST_TORQUE_COMMAND : SPEED_WRITE;
  if (can_rx_header.Identifier != (test_command | MOTOR_CAN_ID)) return;
  uint32_t test_data = CAN_U32_FROM_PAYLOAD(can_rx_data);
  int16_t test_rpm = (int16_t)(test_data & 0xFFFFU);
  uint16_t test_ramp_ms = (uint16_t)(test_data >> 16);
  (void)TestDriveStart(test_rpm, test_ramp_ms);
  return;
/* HARNESS COMMANDS END */
#endif

  if (CAN_COMMAND_FROM_ID(can_rx_header.Identifier) == SPEED_WRITE)
  {
	  uint32_t rx_data = CAN_U32_FROM_PAYLOAD(can_rx_data);
	  int16_t final_speed_rpm = (int16_t)(rx_data & 0xFFFFU);
	  uint16_t duration_ms = (uint16_t)((rx_data >> 16) & 0xFFFFU);
	  MC_ProgramSpeedRampMotor1(RPM_2_SPEED_UNIT(final_speed_rpm), duration_ms);

	  if (MC_GetSTMStateMotor1() == IDLE)
	  {
		  (void)MC_StartMotor1();
	  }
  }
  else if (CAN_COMMAND_FROM_ID(can_rx_header.Identifier) == TORQUE_WRITE)
  {
	  uint32_t rx_data = CAN_U32_FROM_PAYLOAD(can_rx_data);
	  int16_t final_torque = (int16_t)(rx_data & 0xFFFFU);
	  uint16_t duration_ms = (uint16_t)((rx_data >> 16) & 0xFFFFU);
      MC_ProgramTorqueRampMotor1(final_torque, duration_ms);

	  if (MC_GetSTMStateMotor1() == IDLE)
	  {
		  (void)MC_StartMotor1();
	  }
  }
  else if (CAN_COMMAND_FROM_ID(can_rx_header.Identifier) == MOTOR_STOP)
  {
	  MC_StopMotor1();
  } else {
	  // just do nothing if we do not recognize the command
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
