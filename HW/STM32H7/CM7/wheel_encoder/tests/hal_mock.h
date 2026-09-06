#pragma once

#include "stm32h7xx_hal.h"
#include <gmock/gmock.h>

/* Points at the mock for the currently running test. */
class MockHal {
public:
    MOCK_METHOD(HAL_StatusTypeDef, HAL_TIM_Encoder_Start,
                (TIM_HandleTypeDef *htim,
                 uint32_t Channel));

    MOCK_METHOD(HAL_StatusTypeDef, HAL_TIM_Encoder_Stop,
                (TIM_HandleTypeDef *htim,
                 uint32_t Channel));
};

extern MockHal *g_hal;