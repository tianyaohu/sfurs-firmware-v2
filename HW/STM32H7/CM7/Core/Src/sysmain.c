/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for robot soccer RTOS skeleton
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 ****************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "board_init.h"
#include "cmsis_os.h"
#include "main.h"
#include "optical_flow.h"

/* Private define ------------------------------------------------------------*/
#define DEFAULT_TASK_STACK_SIZE (128U * 4U)
#define DEFAULT_TASK_PRIORITY   ((osPriority_t)osPriorityNormal)

/* Private variables ---------------------------------------------------------*/

// Task handles
osThreadId_t defaultTaskHandle;

/* Add application task handles here as needed
osThreadId_t loraTaskHandle;
osThreadId_t kickTaskHandle;
osThreadId_t motorTaskHandle;
*/

/* External task entry points prob declared in separate modules:
   extern void StartLoraTask(void *argument);
   extern void StartKickerTask(void *argument);
   extern void StartMotorTask(void *argument);
*/

// Function Prototypes
void StartDefaultTask(void *argument);

/**
 * @brief Create RTOS tasks and initialize scheduling.
 * @retval None
 */
static void Tasks_Init(void)
{
    const osThreadAttr_t defaultTask_attributes = {
        .name       = "defaultTask",
        .stack_size = DEFAULT_TASK_STACK_SIZE,
        .priority   = DEFAULT_TASK_PRIORITY,
    };

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    if (defaultTaskHandle == NULL) {
        Error_Handler();
    }

    /* Application task creation placeholders. We will add more as we go prob
       osThreadNew(StartLoraTask, NULL, &loraTask_attributes);
       osThreadNew(StartKickerTask, NULL, &kickTask_attributes);
       osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
    */
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    HAL_Init();

    Board_Init();


    // osKernelInitialize();

    // Tasks_Init();

    // osKernelStart();

    /* Should never get here */
    HAL_StatusTypeDef optical_flow_status;
    SPI_OPTICAL_FLOW_DELTA_DATA_t data;

    int32_t dx = 0;
    int32_t dy = 0;
    while (1) {
        optical_flow_status = spi_optical_flow_get_delta(&hspi2, &data);

        if (optical_flow_status == HAL_OK) {
            dx += data.delta_x;
            dy += data.delta_y;
            printf("delta x: %ld, delta y: %ld\r\n", (long)dx, (long)dy);
        } else {
            printf("Optical-flow read failed: %d\r\n", (int)optical_flow_status);
        }

        HAL_Delay(200);
    }
}

void StartDefaultTask(void *argument)
{
    // extern SPI_HandleTypeDef hspi2;
    for (;;) {
        osDelay(1);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
