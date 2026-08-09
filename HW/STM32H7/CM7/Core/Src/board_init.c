/**
 ******************************************************************************
 * @file           : board_init.c
 * @brief          : Initialize board-level peripherals and application state for robot
 * soccer RTOS
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "board_init.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "../../Common_SPI/spi_datatypes.h"
#include "../../IMU/sensor_board.h"


// Private Variables
COM_InitTypeDef BspCOMInit;
FDCAN_HandleTypeDef hfdcan1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi6;

static sensor_board_t sensor_board_ctx;

/**
 * @brief Initialize all necessary hardware peripherals (GPIO, FDCAN, etc) for the robot
 * soccer RTOS.
 * @retval None
 */

void Board_Init(void)
{
    SystemClock_Config();
    MX_GPIO_Init();
    // MX_FDCAN1_Init();
    // Peripherals_Init();
    MX_SPI2_Init();
    MX_SPI6_Init();

    if (spi_optical_flow_init(&hspi2) != HAL_OK) {
        Error_Handler();
    }

    sensor_board_ctx.spi = &hspi6;
    sensor_board_ctx.imu_cs_port = nIMU_CS_GPIO_Port;
    sensor_board_ctx.imu_cs_pin = nIMU_CS_Pin;
    sensor_board_ctx.imu_reset_port = NULL;
    sensor_board_ctx.imu_reset_pin = 0;
    sensor_board_ctx.imu_int1_port = IMU_INT1_GPIO_Port;
    sensor_board_ctx.imu_int1_pin = IMU_INT1_Pin;
    sensor_board_ctx.imu_int2_port = IMU_INT2_GPIO_Port;
    sensor_board_ctx.imu_int2_pin = IMU_INT2_Pin;
    sensor_board_ctx.spi_timeout_ms = 10;

    sensor_board_init(&sensor_board_ctx);
}

/**
 * @brief Initialize application peripherals and state.
 * @retval None
 */
void Peripherals_Init(void)
{
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
        Error_Handler();
    }

    if (BSP_LED_Init(LED_GREEN) != BSP_ERROR_NONE)
        Error_Handler();
    if (BSP_LED_Init(LED_YELLOW) != BSP_ERROR_NONE)
        Error_Handler();
    if (BSP_LED_Init(LED_RED) != BSP_ERROR_NONE)
        Error_Handler();

    if (BSP_LED_Off(LED_GREEN) != BSP_ERROR_NONE)
        Error_Handler();
    if (BSP_LED_Off(LED_YELLOW) != BSP_ERROR_NONE)
        Error_Handler();
    if (BSP_LED_Off(LED_RED) != BSP_ERROR_NONE)
        Error_Handler();
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 9;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 3;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOMEDIUM;
    RCC_OscInitStruct.PLL.PLLFRACN        = 3072;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_FD_NO_BRS;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 2;
    hfdcan1.Init.NominalSyncJumpWidth = 5;
    hfdcan1.Init.NominalTimeSeg1      = 19;
    hfdcan1.Init.NominalTimeSeg2      = 5;
    hfdcan1.Init.DataPrescaler        = 1;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 1;
    hfdcan1.Init.DataTimeSeg2         = 1;
    hfdcan1.Init.MessageRAMOffset     = 0;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr      = 1;
    hfdcan1.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr      = 0;
    hfdcan1.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr         = 0;
    hfdcan1.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr          = 0;
    hfdcan1.Init.TxBuffersNbr         = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr  = 1;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(nIMU_CS_GPIO_Port, nIMU_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : PC1 PC4 PC5 */
    GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA1 PA2 PA7 */
    GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB13 */
    GPIO_InitStruct.Pin       = GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PA8 PA11 PA12 */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PG11 PG13 */
    GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : PG10 */
    GPIO_InitStruct.Pin   = nIMU_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(nIMU_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB4 */
    GPIO_InitStruct.Pin   = IMU_INT1_Pin | IMU_INT2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : SPI2_CS_Pin */
    GPIO_InitStruct.Pin   = SPI2_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
void MX_SPI2_Init(void)
{

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance                        = SPI2;
    hspi2.Init.Mode                       = SPI_MODE_MASTER;
    hspi2.Init.Direction                  = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize                   = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity                = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase                   = SPI_PHASE_1EDGE;
    hspi2.Init.NSS                        = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode                     = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial              = 0x0;
    hspi2.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
    hspi2.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi2.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief SPI6 Initialization Function
 * @param None
 * @retval None
 */
void MX_SPI6_Init(void)
{

    /* USER CODE BEGIN SPI6_Init 0 */

    /* USER CODE END SPI6_Init 0 */

    /* USER CODE BEGIN SPI6_Init 1 */

    /* USER CODE END SPI6_Init 1 */
    /* SPI6 parameter configuration*/
    hspi6.Instance                        = SPI6;
    hspi6.Init.Mode                       = SPI_MODE_MASTER;
    hspi6.Init.Direction                  = SPI_DIRECTION_2LINES;
    hspi6.Init.DataSize                   = SPI_DATASIZE_8BIT;
    hspi6.Init.CLKPolarity                = SPI_POLARITY_LOW;
    hspi6.Init.CLKPhase                   = SPI_PHASE_1EDGE;
    hspi6.Init.NSS                        = SPI_NSS_SOFT;
    hspi6.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;
    hspi6.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
    hspi6.Init.TIMode                     = SPI_TIMODE_DISABLE;
    hspi6.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
    hspi6.Init.CRCPolynomial              = 0x0;
    hspi6.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
    hspi6.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    hspi6.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi6.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi6.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi6.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi6.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi6.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi6) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI6_Init 2 */

    /* USER CODE END SPI6_Init 2 */
}


