/*
Declares what sensor_board.c actually uses from the ST HAL layer.
*/
#ifndef STM32H7xx_HAL_H
#define STM32H7xx_HAL_H
#include <stdint.h>

typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET  = 0,
    GPIO_PIN_SET    = 1
} GPIO_PinState;

// Decalre the structs
struct __GPIO_TypeDef;
struct __SPI_HandleTypeDef;

#ifdef __cplusplus
    extern "C" {
#endif

void HAL_GPIO_WritePin(struct __GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

HAL_StatusTypeDef HAL_SPI_Transmit(struct __SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_SPI_TransmitReceive(struct __SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, 
                                          uint16_t Size, uint32_t Timeout);


#ifdef __cplusplus
    }
#endif

#endif