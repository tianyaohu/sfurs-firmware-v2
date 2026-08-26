#include "wheel_encoder.h"


HAL_StatusTypeDef Encoder_Start(TIM_HandleTypeDef *tim_handle)
{
    return HAL_TIM_Encoder_Start(tim_handle, TIM_CHANNEL_ALL);
}

HAL_StatusTypeDef Encoder_Stop(TIM_HandleTypeDef *tim_handle)
{
    return HAL_TIM_Encoder_Stop(tim_handle, TIM_CHANNEL_ALL);
}

int32_t Encoder_GetCount(TIM_HandleTypeDef *tim_handle) {
    return __HAL_TIM_GET_COUNTER(tim_handle);
}