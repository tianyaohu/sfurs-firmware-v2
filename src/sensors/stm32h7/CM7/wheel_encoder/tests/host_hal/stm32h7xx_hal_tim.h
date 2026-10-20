#ifndef HOST_STM32H7XX_HAL_TIM_H
#define HOST_STM32H7XX_HAL_TIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct {
    volatile uint32_t CNT;
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_ALL 0x0000FFFFU
#define __HAL_TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim,
                                        uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim,
                                       uint32_t Channel);

#ifdef __cplusplus
}
#endif

#endif
