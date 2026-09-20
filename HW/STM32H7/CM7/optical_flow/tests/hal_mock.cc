#include "hal_mock.h"

MockHal *g_hal = nullptr;

extern "C" void HAL_GPIO_WritePin(struct __GPIO_TypeDef *GPIOx,
                                  uint16_t GPIO_Pin, GPIO_PinState PinState) // C name
{
    g_hal->HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState); // C++ body 
}

extern "C" HAL_StatusTypeDef HAL_SPI_Transmit(struct __SPI_HandleTypeDef *hspi,
                                              uint8_t *pData,
                                              uint16_t Size, uint32_t Timeout)
{
    return g_hal->HAL_SPI_Transmit(hspi, pData, Size, Timeout);
}

extern "C" HAL_StatusTypeDef HAL_SPI_TransmitReceive(struct __SPI_HandleTypeDef *hspi,
                                                     uint8_t *pTxData, uint8_t *pRxData,
                                                     uint16_t Size, uint32_t Timeout)
{
    return g_hal->HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout);
}