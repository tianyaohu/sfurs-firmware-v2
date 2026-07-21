/*
 * optical_flow.h
 * Driver interface for the PAA5100JE-Q optical flow sensor.
 */

#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * SPI CHIP SELECT PINS
 */
#define SPI_OPTICAL_FLOW_CHIP_SELECT_PIN  GPIO_PIN_9
#define SPI_OPTICAL_FLOW_CHIP_SELECT_GPIO GPIOB

typedef struct {
    int32_t delta_x;
    int32_t delta_y;
} SPI_OPTICAL_FLOW_DELTA_DATA_t;

/*
 * Initializes the optical flow sensor for delta reads.
 * Resets the sensor, clears motion registers, applies the vendor config
 * sequence, then reads and validates the sensor identifiers.
 * Returns HAL_OK on successful initialization.
 */
HAL_StatusTypeDef spi_optical_flow_init(SPI_HandleTypeDef *spi_pointer);

/*
 * Reads the latest optical-flow delta.
 * Returns HAL_OK when communication succeeds. If no valid motion is
 * available, delta_x and delta_y are set to zero.
 * Returns HAL_ERROR for invalid arguments or communication failures.
 */
HAL_StatusTypeDef spi_optical_flow_get_delta(SPI_HandleTypeDef *spi_pointer,
                                             SPI_OPTICAL_FLOW_DELTA_DATA_t *delta_data);

#endif
