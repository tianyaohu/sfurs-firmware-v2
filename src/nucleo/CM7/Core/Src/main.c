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
#include "can_tx_status.h"
#if NUCLEO_DRIVE
#include "ik_drive.h"
#include "ik_telemetry.h"
#include "skynet_link.h"
#include "skynet_uart.h"
#else
#include "fwu_updater.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** NUCLEO_DRIVE (CMake): 1 = streaming omni drive image, 0 = ESC updater
 * image carrying one embedded ESC application (FWU_IMAGE_FILE). */
#ifndef NUCLEO_DRIVE
#define NUCLEO_DRIVE 0
#endif

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

/* NUCLEO-H755ZI-Q LEDs, active high: LD1 green PB0 (alive), LD2 yellow PE1
 * (CAN activity / drive running), LD3 red PB14 (fault). */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
/** Debugger-visible image identity: 1 = drive, 2 = ESC updater. */
volatile const uint32_t nucleo_image_kind = NUCLEO_DRIVE ? 1U : 2U;
/** CAN frames received, read over SWD. */
volatile uint32_t can_rx_frame_count;
/** CAN bus-off recoveries, read over SWD. */
volatile uint32_t can_bus_off_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t last_tx_mask;

static int can_send(uint32_t id, const uint8_t *data, uint32_t dlc)
{
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, (uint8_t *)data) != HAL_OK) return 0;
  last_tx_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  return 1;
}

#if NUCLEO_DRIVE
static int can_send4(uint32_t id, const uint8_t data[4])
{
  return can_send(id, data, FDCAN_DLC_BYTES_4);
}

/* ik_drive's sender: 4-byte service frames and 8-byte protocol frames. */
static int can_send_len(uint32_t id, const uint8_t *data, uint32_t len)
{
  return can_send(id, data, len == 8U ? FDCAN_DLC_BYTES_8 : FDCAN_DLC_BYTES_4);
}

/* Microsecond clock from the core cycle counter, for timing ESC replies (the
 * synchronization metric needs better than the 1 ms HAL tick). Accumulated per
 * call so it wraps at 2^32 us, not when CYCCNT wraps; the main loop calls it far
 * more often than the 26 s CYCCNT period at 160 MHz. */
static uint32_t now_us(void)
{
  static uint32_t last_cycles, rem_cycles, us;
  const uint32_t per_us = SystemCoreClock / 1000000U;
  const uint32_t cycles = DWT->CYCCNT;
  rem_cycles += cycles - last_cycles;
  last_cycles = cycles;
  us += rem_cycles / per_us;
  rem_cycles %= per_us;
  return us;
}
#else
static int can_send8(uint32_t id, const uint8_t data[8])
{
  return can_send(id, data, FDCAN_DLC_BYTES_8);
}

static int can_poll_last(void)
{
  return can_tx_status(hfdcan1.Instance->TXBTO, hfdcan1.Instance->TXBCF, last_tx_mask);
}

static void can_abort_last(void)
{
  (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, last_tx_mask);
}
#endif

static void led_init(void)
{
  GPIO_InitTypeDef led = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
  led.Mode = GPIO_MODE_OUTPUT_PP;
  led.Pull = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  led.Pin = GPIO_PIN_0 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &led);
  led.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOE, &led);
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
  uint32_t alive_tick = 0;
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
  led_init();
  /* Accept all standard/extended data frames into FIFO0; reject remote. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
      HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
#if !NUCLEO_DRIVE
  const struct can_fwu_transport fwu_io = { can_send8, can_poll_last, can_abort_last };
  fwu_updater_init(&fwu_io, HAL_GetTick);
#else
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* cycle counter for now_us() */
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  skynet_uart_init();
#endif
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    (void)nucleo_image_kind; /* keep the identity word through linker GC */
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - alive_tick) >= 500U) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      alive_tick = now;
    }
#if NUCLEO_DRIVE
    (void)ik_drive_abi; /* keep the debugger-visible ABI word through linker GC */
    /* every pass also keeps the microsecond clock accumulating while the bus is quiet */
    (void)skynet_link_abi;
    skynet_link_poll(now); /* skynet commands land in the mailbox before this pass reads it */
    ik_drive_tick(now, now_us(), can_send_len);
    const uint32_t poll_id = ik_telemetry_poll_due(now);
    if (poll_id != 0U) {
      const uint8_t zero[4] = {0};
      (void)can_send4(poll_id, zero);
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, ik_drive_state == IK_DRIVING ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, ik_drive_tx_fail != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    fwu_updater_tick(now, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, fwu_updater_active() && (can_fwu_rx_cnt & 1U) == 0U
                      ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, fwu_host.state == FWU_H_FAULT ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    /* Bus-off (e.g. a connector jolted while driving) sets CCCR.INIT and the
     * controller stays off the bus until software clears it; it then rejoins
     * after 128 x 11 recessive bits. Without this every later frame fails. */
    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) && (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT)) {
      CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
      ++can_bus_off_count;
    }
    /* Bounded RX pass so bus traffic cannot starve the loop. */
    for (uint32_t i = 0; i < 8U && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U; ++i) {
      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data) != HAL_OK) break;
      ++can_rx_frame_count;
      if (can_rx_header.IdType != FDCAN_STANDARD_ID || can_rx_header.RxFrameType != FDCAN_DATA_FRAME ||
          can_rx_header.DataLength != FDCAN_DLC_BYTES_8) continue;
#if NUCLEO_DRIVE
      if (!ik_drive_rx(can_rx_header.Identifier, can_rx_data, 8U, now_us()))
        (void)ik_telemetry_rx(can_rx_header.Identifier, can_rx_data, now);
#else
      fwu_updater_rx(can_rx_header.Identifier, can_rx_data, HAL_GetTick());
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
