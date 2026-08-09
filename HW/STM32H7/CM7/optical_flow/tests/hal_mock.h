#pragma once
#include "stm32h7xx_hal.h" // Fake Mock HAL
#include <gmock/gmock.h>

/* Defines the linker between fake HAL and sensor_board.c*/
/* Points at the mock the currently running test*/

class MockHal {
  public:
    MOCK_METHOD(void, HAL_GPIO_WritePin,
                (struct __GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin,
                 GPIO_PinState pinState));

    MOCK_METHOD(HAL_StatusTypeDef, HAL_SPI_Transmit,
                (struct __SPI_HandleTypeDef * hspi, uint8_t *pData, uint16_t Size,
                 uint32_t Timeout));

    MOCK_METHOD(HAL_StatusTypeDef, HAL_SPI_TransmitReceive,
                (struct __SPI_HandleTypeDef * hspi, uint8_t *pTxData, uint8_t *pRxData,
                 uint16_t Size, uint32_t Timeout));
};

extern MockHal *g_hal;
