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
#include "wheel_bench.h"
#include "can_fwu_host.h"
#include "fwu_test_image.h"
#include "boot_entry.h"
#include "can_cal_sender.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_BENCH_MODE 1U
#define FWU_RX_DIAGNOSTIC 0U
#define FWU_BLINK_BENCH_ONLY 1U /* Never emit motor ARM/RUN from this test build. */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */


// Only Running CM7 for now. 
// #define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
/* Debugger-visible receive count and latest periodic transmit result.
 * TX status: 0 = waiting, 1 = pending, 2 = acknowledged, 3 = failed. */
volatile uint32_t can_rx_frame_count = 0;
volatile uint32_t can_tx_test_status = 0;
volatile uint32_t can_tx_success_count = 0;
volatile uint32_t can_tx_failure_count = 0;
/* Matches firmware-v2 HW/common/CAN/can_datatypes.h, front-right MOTOR_NE. */
volatile uint32_t esc_telemetry_reply_count = 0;
volatile int32_t esc_ne_rpm = 0;
volatile uint32_t esc_ne_motor_state = 0;
volatile uint32_t esc_ne_hall_state = 0;
volatile uint32_t esc_ne_hall_changes = 0;
volatile uint32_t esc_ne_last_reply_tick = 0;
volatile uint32_t esc_current_reply_count = 0;
volatile int32_t esc_ne_iq_ma = 0;
volatile int32_t esc_ne_id_ma = 0;
volatile uint32_t esc_ne_peak_l1_ma = 0;
volatile uint32_t esc_ne_test_flags = 0;
volatile uint32_t esc_ne_current_tick = 0;
/* Explicit debugger triggers only: ARM1=0x41524D31, RUN1=0x52554E31,
 * STOP=0x53544F50. Never populated automatically or from a button. */
volatile uint32_t wheel_test_request = 0;
/* Result: 0=none, 1=queued, 2=rejected, 3=ACKed, 4=TX failed. ACK != motor start. */
volatile uint32_t wheel_test_result = 0;
volatile uint32_t wheel_test_last_request = 0;
volatile uint32_t wheel_test_run_attempted = 0;
/* P4/P5: CAN-FWU updater state. can_fwu_request is debugger-only and
 * consumed once; no transfer starts without it AND bench IDLE. Result:
 * 0=none, 1=session active, 2=rejected (busy/no image), 3=done, 4=failed. */
volatile uint32_t can_fwu_request = 0;
volatile uint32_t can_fwu_dev = 1;
volatile uint32_t can_fwu_result = 0;
/* Explicit remote-entry trigger, separate from destructive update START.
 * Result: 0=none, 1=queued (NOT boot-confirmed), 2=rejected, 3=TX failed. */
volatile uint32_t can_fwu_enter_boot = 0;
volatile uint32_t can_fwu_boot_result = 0;
/* Counts received FWU ACK frames (0x680|dev) for LED + SWD diagnosis. */
volatile uint32_t can_fwu_rx_cnt = 0;
/* Raw receive snapshot BEFORE application validation. Payload is two LE
 * words; header: ID, ID type, frame type, FD format, BRS, DLC. */
volatile uint32_t can_diag_rx[9] = {0};
/* Debugger one-shot: consume a valid ENTER ACK and queue one ERASE probe.
 * NE diagnostic firmware rejects ERASE without accessing flash. */
volatile uint32_t can_diag_advance = 0;
volatile uint32_t can_diag_erase_reply[3] = {0}; /* seen, status, detail */
volatile uint32_t can_diag_timing[3] = {0}; /* begin ms, ACK state, ACK age ms */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t bench_tx_mask;
volatile uint32_t wheel_bench_can_ecr, wheel_bench_can_psr;
volatile uint32_t wheel_bench_can_txbto, wheel_bench_can_txbcf;
volatile uint32_t wheel_bench_can_enqueue_error;

int wheel_bench_send(uint32_t id, const uint8_t data[4])
{
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_4;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) != HAL_OK) {
    wheel_bench_can_enqueue_error = hfdcan1.ErrorCode;
    return 0;
  }
  bench_tx_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  return 1;
}

int wheel_bench_poll(void)
{
  /* PSR/ECR reads acknowledge some diagnostic fields; capture only during TX. */
  wheel_bench_can_ecr = hfdcan1.Instance->ECR;
  wheel_bench_can_psr = hfdcan1.Instance->PSR;
  wheel_bench_can_txbto = hfdcan1.Instance->TXBTO;
  wheel_bench_can_txbcf = hfdcan1.Instance->TXBCF;
  return wheel_bench_tx_status(wheel_bench_can_txbto,
                               wheel_bench_can_txbcf, bench_tx_mask);
}

void wheel_bench_abort(void)
{
  /* A failed cancellation latches transport failure in the bench service;
   * do not enter Error_Handler and strand a potentially running ESC. */
  (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, bench_tx_mask);
}

/* P4: FWU transport on the same FDCAN1, 8-byte frames. Dormant unless a
 * session is explicitly requested via can_fwu_request (debugger only). */
static struct can_fwu_host fwu_host;
static uint32_t fwu_tx_mask;
static int fwu_transport_ready;

static int fwu_can_send(uint32_t id, const uint8_t data[8])
{
  if (FWU_RX_DIAGNOSTIC != 0U &&
      id != can_fwu_id(CAN_FWU_ENTER_BASE, can_fwu_dev) &&
      id != can_fwu_id(CAN_FWU_ERASE_BASE, can_fwu_dev)) {
    return 0; /* Diagnostic transport cannot send DATA/VERIFY/JUMP. */
  }
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) != HAL_OK) {
    return 0;
  }
  fwu_tx_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  return 1;
}

static int fwu_can_poll(void)
{
  return wheel_bench_tx_status(hfdcan1.Instance->TXBTO,
                               hfdcan1.Instance->TXBCF, fwu_tx_mask);
}

static void fwu_can_abort(void)
{
  (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, fwu_tx_mask);
}

/* CAL1 sender transport: reuse the bench FDCAN trio (4-byte frames).
 * Dormant unless the debugger writes can_cal_request; see can_cal_tick. */
int can_cal_send(uint32_t id, const uint8_t data[4])
{
  return wheel_bench_send(id, data);
}

int can_cal_poll(void)
{
  /* CAN_CAL_TX_* codes match BENCH_TX_* by design. */
  return wheel_bench_poll();
}

void can_cal_abort(void)
{
  wheel_bench_abort();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  FDCAN_RxHeaderTypeDef can_rx_header;
  uint8_t can_rx_data[64];
  uint32_t heartbeat_tick = 0;
  uint32_t last_can_rx_tick = 0;
  uint32_t activity_tick = 0;
  uint8_t can_activity_seen = 0;
  uint32_t last_tx_attempt_tick = 0;
  uint32_t tx_request_tick = 0;
  uint32_t tx_buffer_mask = 0;
  uint8_t tx_last_failed = 0;
  uint8_t tx_is_wheel_command = 0;
  uint8_t request_current_next = 0;
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /* NUCLEO-H755ZI-Q LD1 (green) is active high on PB0. */
  GPIO_InitTypeDef led_gpio = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  led_gpio.Pin = GPIO_PIN_0;
  led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
  led_gpio.Pull = GPIO_NOPULL;
  led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &led_gpio);
  /* LD3 (red), PB14: transmit attempt failed or timed out. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  led_gpio.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &led_gpio);
  /* LD2 (yellow), PE1: pulses while complete CAN frames are arriving. */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
  led_gpio.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOE, &led_gpio);

  /* Accept standard/extended data frames, but reject remote requests. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                  FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_REJECT_REMOTE,
                                  FDCAN_REJECT_REMOTE) != HAL_OK ||
      HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  heartbeat_tick = HAL_GetTick();
  last_tx_attempt_tick = heartbeat_tick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if (WHEEL_BENCH_MODE != 0U && FWU_BLINK_BENCH_ONLY == 0U) {
      wheel_bench_button(now, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
      wheel_bench_tick(now);
    }
    /* CAL1 sender pump. can_cal_request boots 0 (debugger-only, consumed
     * once inside); refused unless the bench is IDLE, so this sends
     * nothing by default. */
    can_cal_tick(now, wheel_bench_state == BENCH_IDLE);
    /* Embedded NE blink test: explicit request only, bench IDLE required.
     * Unanswered ENTER requests repeat every 300ms until answered; later
     * failures latch and require a NEW operator request. No automatic
     * erase/data retries and no periodic drive commands. */
    static uint32_t fwu_last_try;
    static int fwu_armed;
    static int fwu_fresh_request;
    static int fwu_boot_pending;
    static uint32_t fwu_boot_tick;
    /* CAL1 sender pump. can_cal_request boots 0 (debugger-only, consumed
     * once inside); refused unless the bench is IDLE, so this sends
     * nothing by default. */
    can_cal_tick(now, wheel_bench_state == BENCH_IDLE);
    if (!fwu_transport_ready) {
      struct can_fwu_transport t = { fwu_can_send, fwu_can_poll,
                                     fwu_can_abort };
      can_fwu_host_init(&fwu_host, &t);
      fwu_transport_ready = 1;
    }
    if (can_fwu_enter_boot != 0U) {
      uint8_t payload[8];
      can_fwu_enter_boot = 0U;
      if (!fwu_armed && !fwu_boot_pending && wheel_bench_state == BENCH_IDLE &&
          (fwu_host.state == FWU_H_IDLE || fwu_host.state == FWU_H_DONE ||
           fwu_host.state == FWU_H_FAULT) &&
          esc_boot_request_pack(payload, can_fwu_dev)) {
        if (fwu_can_send(ESC_BOOT_REQUEST_BASE | can_fwu_dev, payload)) {
          fwu_boot_pending = 1;
          fwu_boot_tick = HAL_GetTick();
          can_fwu_boot_result = 1U;
        } else {
          can_fwu_boot_result = 3U;
        }
      } else {
        can_fwu_boot_result = 2U;
      }
    }
    if (fwu_boot_pending) {
      int status = fwu_can_poll();
      if (status == BENCH_TX_FAILED ||
          (uint32_t)(HAL_GetTick() - fwu_boot_tick) >= 100U) {
        fwu_can_abort();
        fwu_boot_pending = 0;
        can_fwu_boot_result = 3U;
      } else if (status == BENCH_TX_OK) {
        fwu_boot_pending = 0;
      }
    }
    if (can_fwu_request != 0U) {
      can_fwu_request = 0U;
      if (!fwu_boot_pending && wheel_bench_state == BENCH_IDLE &&
          (fwu_host.state == FWU_H_IDLE || fwu_host.state == FWU_H_FAULT ||
           fwu_host.state == FWU_H_DONE)) {
        fwu_armed = 1;
        fwu_fresh_request = 1;
        fwu_last_try = 0U;
        can_diag_erase_reply[0] = 0U;
      } else {
        can_fwu_result = 2U;
      }
    }
    /* Cable-free arm: B1 held continuously since power-up for 3 s arms the
     * update (bench ignores startup-held buttons, so no conflict). Either
     * a release or a completed 3 s hold ends the gesture for this boot. */
    static uint32_t fwu_hold_start;
    static int fwu_hold_timing;
    static int fwu_power_gesture_done;
    if (!fwu_power_gesture_done) {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
        if (!fwu_hold_timing) {
          fwu_hold_timing = 1;
          fwu_hold_start = now;
        } else if ((uint32_t)(now - fwu_hold_start) >= 3000U &&
                   wheel_bench_state == BENCH_IDLE &&
                   fwu_host.state == FWU_H_IDLE && !fwu_armed) {
          fwu_armed = 1;
          fwu_fresh_request = 1;
          fwu_last_try = 0U;
          fwu_power_gesture_done = 1;
        }
      } else {
        fwu_power_gesture_done = 1;
      }
    }
    if (fwu_armed && can_diag_erase_reply[0] == 0U &&
        (fwu_fresh_request || fwu_host.state == FWU_H_IDLE ||
         fwu_host.state == FWU_H_DONE ||
         (fwu_host.state == FWU_H_FAULT &&
          fwu_host.tx_id == can_fwu_id(CAN_FWU_ENTER_BASE, can_fwu_dev))) &&
        (uint32_t)(now - fwu_last_try) >= 300U) {
      /* ENTER retry is deliberately unbounded: the ESC answers from any
       * state, so calling forever is safe and removes all timing pressure. */
      fwu_last_try = now;
      fwu_fresh_request = 0;
      uint32_t diag_begin_tick = HAL_GetTick();
      if (wheel_bench_state == BENCH_IDLE &&
                 can_fwu_host_begin(&fwu_host, can_fwu_dev, fwu_test_image,
                                    FWU_TEST_IMAGE_SIZE, 1, now)) {
        can_fwu_result = 1U;
      } else {
        fwu_armed = 0;
        can_fwu_result = 2U;
      }
      can_diag_timing[0] = HAL_GetTick() - diag_begin_tick;
    }
    if (fwu_armed && fwu_host.state == FWU_H_FAULT &&
        fwu_host.tx_id != can_fwu_id(CAN_FWU_ENTER_BASE, can_fwu_dev)) {
      /* Progress was made then lost: latch, needs a fresh arm. */
      fwu_armed = 0;
      can_fwu_result = 4U;
    }
    if (fwu_host.state != FWU_H_IDLE && fwu_host.state != FWU_H_DONE &&
        fwu_host.state != FWU_H_FAULT) {
      /* begin() CRCs the embedded image synchronously (~191 ms in Debug).
       * Never stamp enqueue with the pre-CRC loop timestamp: that consumes
       * the entire ACK budget before the frame has even been sent. */
      now = HAL_GetTick();
      can_fwu_host_poll(&fwu_host, now);
      can_fwu_result = can_fwu_host_done(&fwu_host) ? 3U : 1U;
      if (fwu_host.state == FWU_H_FAULT) {
        if (fwu_host.tx_id == can_fwu_id(CAN_FWU_ENTER_BASE, can_fwu_dev)) {
          /* ENTER never acked: keep armed, retry above. */
          can_fwu_result = 1U;
        } else {
          fwu_armed = 0;
          can_fwu_result = 4U;
        }
      }
      if (can_fwu_host_done(&fwu_host)) {
        fwu_armed = 0;
      }
    }
    /* Alternate speed/current telemetry at 10 Hz each. Drive frames require
     * explicit debugger requests and fresh bench-profile telemetry. */
    /* Legacy guarded NE sender retained for reference/tests, but disabled in
     * this original-firmware bench build. No unsolicited telemetry or ARM1. */
    if (WHEEL_BENCH_MODE == 0U && can_tx_test_status != 1U &&
        (wheel_test_request != 0U || (uint32_t)(now - last_tx_attempt_tick) >= 50U) &&
        (tx_buffer_mask == 0U ||
         HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, tx_buffer_mask) == 0U))
    {
      last_tx_attempt_tick = now;
      FDCAN_TxHeaderTypeDef tx_header = {0};
      uint8_t tx_data[4] = {0};
      tx_header.Identifier = request_current_next != 0U ? 0x301U : 0x101U;
      request_current_next ^= 1U;
      tx_is_wheel_command = 0U;
      uint32_t requested = wheel_test_request;
      if (requested != 0U)
      {
        wheel_test_request = 0U;
        wheel_test_last_request = requested;
        uint8_t fresh_profile = esc_current_reply_count != 0U &&
            (uint32_t)(now - esc_ne_current_tick) <= 300U &&
            (esc_ne_test_flags & 0xAFFFU) == 0x8800U;
        if (requested == 0x53544F50U)
        {
          tx_header.Identifier = 0x781U; /* MOTOR_STOP | MOTOR_NE */
          tx_is_wheel_command = 1U;
        }
        else if (requested == 0x41524D31U && fresh_profile &&
                 wheel_test_run_attempted == 0U)
        {
          tx_header.Identifier = 0x401U;
          tx_data[0] = 0x54U; tx_data[1] = 0x53U;
          tx_data[2] = 0x45U; tx_data[3] = 0x54U; /* 0x54455354 */
          tx_is_wheel_command = 1U;
        }
        else if (requested == 0x52554E31U && fresh_profile &&
                 (esc_ne_test_flags & 0x5000U) == 0x4000U &&
                 wheel_test_run_attempted == 0U &&
                 esc_telemetry_reply_count != 0U && esc_ne_motor_state == 0U &&
                 (uint32_t)(now - esc_ne_last_reply_tick) <= 300U)
        {
          wheel_test_run_attempted = 1U; /* Never retry a possibly delivered start. */
          tx_header.Identifier = 0x081U;
          tx_data[0] = 60U; tx_data[1] = 0U;
          tx_data[2] = 0xE8U; tx_data[3] = 0x03U; /* 1000 ms ramp */
          tx_is_wheel_command = 1U;
        }
        else
        {
          wheel_test_result = 2U; /* Rejected request; send telemetry only. */
        }
      }
      tx_header.IdType = FDCAN_STANDARD_ID;
      tx_header.TxFrameType = FDCAN_DATA_FRAME;
      tx_header.DataLength = FDCAN_DLC_BYTES_4;
      tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      tx_header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_header.FDFormat = FDCAN_FD_CAN;
      tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK)
      {
        tx_buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
        tx_request_tick = now;
        can_tx_test_status = 1U;
        if (tx_is_wheel_command != 0U) wheel_test_result = 1U;
      }
      else
      {
        can_tx_test_status = 3U;
        ++can_tx_failure_count;
        tx_last_failed = 1U;
        if (tx_is_wheel_command != 0U) wheel_test_result = 4U;
      }
    }

    if (can_tx_test_status == 1U)
    {
      /* TXBTO is set only on successful transmission, not merely enqueue. */
      if ((hfdcan1.Instance->TXBTO & tx_buffer_mask) != 0U)
      {
        can_tx_test_status = 2U;
        ++can_tx_success_count;
        tx_last_failed = 0U;
        if (tx_is_wheel_command != 0U) wheel_test_result = 3U;
      }
      else if (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, tx_buffer_mask) == 0U ||
               (uint32_t)(now - tx_request_tick) >= 250U)
      {
        if (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, tx_buffer_mask) != 0U)
        {
          if (HAL_FDCAN_AbortTxRequest(&hfdcan1, tx_buffer_mask) != HAL_OK)
          {
            Error_Handler();
          }
        }
        can_tx_test_status = 3U;
        ++can_tx_failure_count;
        tx_last_failed = 1U;
        if (tx_is_wheel_command != 0U) wheel_test_result = 4U;
      }
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,
                     tx_last_failed != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if ((uint32_t)(now - heartbeat_tick) >= 500U)
    {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      heartbeat_tick = now;
    }

    /* Bound each polling pass so bus traffic cannot starve the heartbeat. */
    for (uint32_t i = 0; i < 8U; ++i)
    {
      if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U)
      {
        break;
      }
      for (uint32_t j = 0; j < sizeof(can_rx_data); ++j) can_rx_data[j] = 0U;
      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                &can_rx_header, can_rx_data) != HAL_OK)
      {
        Error_Handler();
      }
      ++can_rx_frame_count;
      can_diag_rx[1] = can_rx_header.Identifier;
      can_diag_rx[2] = can_rx_header.IdType;
      can_diag_rx[3] = can_rx_header.RxFrameType;
      can_diag_rx[4] = can_rx_header.FDFormat;
      can_diag_rx[5] = can_rx_header.BitRateSwitch;
      can_diag_rx[6] = can_rx_header.DataLength;
      can_diag_rx[7] = can_fwu_get_u32(can_rx_data, 0);
      can_diag_rx[8] = can_fwu_get_u32(can_rx_data, 1);
      ++can_diag_rx[0];
      if (can_rx_header.IdType == FDCAN_STANDARD_ID &&
          can_rx_header.RxFrameType == FDCAN_DATA_FRAME &&
          can_rx_header.FDFormat == FDCAN_FD_CAN &&
          can_rx_header.BitRateSwitch == FDCAN_BRS_OFF &&
          can_rx_header.Identifier == 0x381U &&
          can_rx_header.DataLength == FDCAN_DLC_BYTES_8)
      {
        uint16_t iq = (uint16_t)can_rx_data[0] | ((uint16_t)can_rx_data[1] << 8);
        uint16_t id = (uint16_t)can_rx_data[2] | ((uint16_t)can_rx_data[3] << 8);
        /* Existing B-G431B-ESC1 profile: CURRENT_CONV_FACTOR=544 counts/A. */
        esc_ne_iq_ma = ((iq & 0x8000U ? (int32_t)iq - 65536 : (int32_t)iq) * 1000) / 544;
        esc_ne_id_ma = ((id & 0x8000U ? (int32_t)id - 65536 : (int32_t)id) * 1000) / 544;
        esc_ne_test_flags = (uint16_t)can_rx_data[4] | ((uint16_t)can_rx_data[5] << 8);
        esc_ne_peak_l1_ma = (((uint32_t)can_rx_data[6] |
                            ((uint32_t)can_rx_data[7] << 8)) * 1000U) / 544U;
        esc_ne_current_tick = now;
        ++esc_current_reply_count;
      }
      if (can_rx_header.IdType == FDCAN_STANDARD_ID &&
          can_rx_header.RxFrameType == FDCAN_DATA_FRAME &&
          can_rx_header.FDFormat == FDCAN_FD_CAN &&
          can_rx_header.BitRateSwitch == FDCAN_BRS_OFF &&
          can_rx_header.Identifier == 0x281U &&
          can_rx_header.DataLength == FDCAN_DLC_BYTES_8)
      {
        uint16_t rpm_bits = (uint16_t)can_rx_data[0] |
                            ((uint16_t)can_rx_data[1] << 8);
        esc_ne_rpm = (rpm_bits & 0x8000U) != 0U ?
                     (int32_t)rpm_bits - 65536 : (int32_t)rpm_bits;
        esc_ne_motor_state = can_rx_data[2];
        esc_ne_hall_state = can_rx_data[3];
        esc_ne_hall_changes = (uint32_t)can_rx_data[4] |
                             ((uint32_t)can_rx_data[5] << 8) |
                             ((uint32_t)can_rx_data[6] << 16) |
                             ((uint32_t)can_rx_data[7] << 24);
        esc_ne_last_reply_tick = now;
        ++esc_telemetry_reply_count;
      }
      /* P4: FWU target ACKs (0x680|dev). Live dispatch, but the host
       * session ignores them unless a transfer was explicitly started. */
      if (can_rx_header.IdType == FDCAN_STANDARD_ID &&
          can_rx_header.RxFrameType == FDCAN_DATA_FRAME &&
          can_rx_header.FDFormat == FDCAN_FD_CAN &&
          can_rx_header.BitRateSwitch == FDCAN_BRS_OFF &&
          (can_rx_header.Identifier & ~0xFU) == CAN_FWU_ACK_BASE &&
          can_rx_header.DataLength == FDCAN_DLC_BYTES_8)
      {
        ++can_fwu_rx_cnt;
        can_diag_timing[1] = (uint32_t)fwu_host.state;
        can_diag_timing[2] = HAL_GetTick() - fwu_host.tx_tick;
        if (FWU_RX_DIAGNOSTIC == 0U) {
          can_fwu_host_ack(&fwu_host, can_rx_header.Identifier,
                           can_rx_data, now);
        } else if (can_diag_advance != 0U &&
                   fwu_host.state == FWU_H_ENTER_SENT &&
                   can_rx_header.Identifier ==
                       can_fwu_id(CAN_FWU_ACK_BASE, can_fwu_dev) &&
                   can_fwu_ack_status(can_rx_data) == CAN_FWU_OK) {
          can_diag_advance = 0U;
          can_fwu_host_ack(&fwu_host, can_rx_header.Identifier,
                           can_rx_data, now);
        } else if (fwu_host.state == FWU_H_ERASE_SENT) {
          /* Capture rejection via the actual host state machine. An
           * unexpected OK still cannot send DATA (transport guard). */
          can_diag_erase_reply[1] = can_fwu_ack_status(can_rx_data);
          can_diag_erase_reply[2] = can_fwu_ack_detail(can_rx_data);
          can_diag_erase_reply[0] = 1U;
          can_fwu_host_ack(&fwu_host, can_rx_header.Identifier,
                           can_rx_data, now);
        }
      }
      last_can_rx_tick = now;
      if (can_activity_seen == 0U)
      {
        can_activity_seen = 1U;
        activity_tick = now;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
      }
    }

    if (can_fwu_host_done(&fwu_host)) {
      can_fwu_result = 3U;
      fwu_armed = 0;
    }
    /* LD2 indicates received data, never a TX acknowledgement. */
    if (can_activity_seen != 0U)
    {
      if ((uint32_t)(now - last_can_rx_tick) >= 200U)
      {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
        can_activity_seen = 0U;
      }
      else if ((uint32_t)(now - activity_tick) >= 100U)
      {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
        activity_tick = now;
      }
    }
    if (WHEEL_BENCH_MODE != 0U) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,
                       wheel_bench_state == BENCH_ARMED ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,
                       wheel_bench_state == BENCH_FAULT ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    /* FWU session LEDs take precedence while active: LD2 tracks received
     * FWU ACKs (parity blink ~4 Hz when they arrive, solid when none yet),
     * LD3 on fault. */
    if (fwu_host.state != FWU_H_IDLE && fwu_host.state != FWU_H_DONE &&
        fwu_host.state != FWU_H_FAULT) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,
                        (can_fwu_rx_cnt == 0U ||
                         (can_fwu_rx_cnt & 1U) != 0U)
                            ? GPIO_PIN_SET
                            : GPIO_PIN_RESET);
    }
    if (fwu_host.state == FWU_H_FAULT) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
   /* HSI 64 MHz / 4 * 20 / 8 = 40 MHz on PLL1Q (FDCAN). */
   RCC_OscInitStruct.PLL.PLLM = 4;
   RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
   RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
   RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
   hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
   hfdcan1.Init.NominalPrescaler = 4;
   hfdcan1.Init.NominalSyncJumpWidth = 2;
   hfdcan1.Init.NominalTimeSeg1 = 17;
   hfdcan1.Init.NominalTimeSeg2 = 2;
   /* BRS is disabled; keep data timing valid and equal to nominal timing. */
   hfdcan1.Init.DataPrescaler = 4;
   hfdcan1.Init.DataSyncJumpWidth = 2;
   hfdcan1.Init.DataTimeSeg1 = 17;
   hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
   hfdcan1.Init.RxFifo0ElmtsNbr = 8;
   hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
   hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
   hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* NUCLEO-H755ZI-Q B1 blue USER button: PC13, pressed high (ST BSP).
   * Polled/debounced; no interrupt or automatic start at power-up. */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef button_gpio = {0};
  button_gpio.Pin = GPIO_PIN_13;
  button_gpio.Mode = GPIO_MODE_INPUT;
  button_gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &button_gpio);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
