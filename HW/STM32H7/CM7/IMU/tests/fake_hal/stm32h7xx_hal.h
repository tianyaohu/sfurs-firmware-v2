#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

typedef struct {
    uint32_t dummy;
} SPI_HandleTypeDef;

typedef struct {
    uint32_t dummy;
} GPIO_TypeDef;

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;

HAL_StatusTypeDef HAL_SPI_Transmit(
    SPI_HandleTypeDef *hspi,
    uint8_t *pData,
    uint16_t Size,
    uint32_t Timeout
);

HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef *hspi,
    uint8_t *pTxData,
    uint8_t *pRxData,
    uint16_t Size,
    uint32_t Timeout
);

void HAL_GPIO_WritePin(
    GPIO_TypeDef *GPIOx,
    uint16_t GPIO_Pin,
    GPIO_PinState PinState
);

#endif