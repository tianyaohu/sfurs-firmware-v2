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
#include "spi_datatypes.h"

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
    uint8_t product_id;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(50);


    HAL_StatusTypeDef status = spi_optical_flow_read_register(
        &hspi2, SPI_OPTICAL_FLOW_REGISTER_PRODUCT_ID, &product_id);
    while (1) {
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
