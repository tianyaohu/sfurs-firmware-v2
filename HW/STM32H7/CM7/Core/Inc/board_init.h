/**
  ******************************************************************************
  * @file           : board_init.h
  * @brief          : Initialize board-level peripherals and application state for robot soccer RTOS
  ******************************************************************************
*/

#ifndef __BOARD_INIT_H
#define __BOARD_INIT_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/**
  * @brief Initialize all board-level peripherals here (GPIO, FDCAN, etc).
  * @retval None
  */
void Board_Init(void);

/**
  * @brief Initialize application peripherals and state.
  * @retval None
  */
void Peripherals_Init(void);

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void);

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_FDCAN1_Init(void);

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void);

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void);


#ifdef __cplusplus
}
#endif
#endif /* __BOARD_INIT_H */