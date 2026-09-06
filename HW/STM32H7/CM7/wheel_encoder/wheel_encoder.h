/*
 * wheel_encoder.h
 * Driver interface for the AS5047P wheel encoder sensor.
 */

#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include "stm32h7xx_hal_tim.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef Encoder_Start(TIM_HandleTypeDef *tim_handle);

HAL_StatusTypeDef Encoder_Stop(TIM_HandleTypeDef *tim_handle);

int32_t Encoder_GetCount(TIM_HandleTypeDef *tim_handle);

#ifdef __cplusplus
}
#endif

#endif
