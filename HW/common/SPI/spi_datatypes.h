/*
 * spi_datatypes.h
 * Driver interface for the PAA5100JE-Q optical flow sensor.
 */

#ifndef SPI_DATATYPES_H
#define SPI_DATATYPES_H

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
 * Reads the latest optical flow delta.
 * Populates delta_data with the measured X and Y motion.
 * Returns HAL_OK when valid motion data is available
 */
HAL_StatusTypeDef spi_optical_flow_get_delta(SPI_HandleTypeDef *spi_pointer,
                                             SPI_OPTICAL_FLOW_DELTA_DATA_t *delta_data);

#endif
