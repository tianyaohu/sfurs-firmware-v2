#include "hal_mock.h"
#include "stm32h7xx_hal_tim.h"

MockHal *g_hal = nullptr;

extern "C" HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim,
                                                   uint32_t Channel)
{
    return g_hal->HAL_TIM_Encoder_Start(htim, Channel);
}

extern "C" HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim,
                                                  uint32_t Channel)
{
    return g_hal->HAL_TIM_Encoder_Stop(htim, Channel);
}

