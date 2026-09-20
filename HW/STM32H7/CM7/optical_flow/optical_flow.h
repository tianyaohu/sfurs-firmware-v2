/*
 * optical_flow.h
 * Driver interface for the PAA5100JE-Q optical flow sensor.
 */

#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/**
 * @brief Stores X and Y motion deltas reported by the PAA5100JE sensor.
 */
typedef struct {
    int32_t delta_x; /* Relative X-axis motion since the last read. */
    int32_t delta_y; /* Relative Y-axis motion since the last read. */
} PAA5100_DELTA_t;

/**
 * @brief Initializes the PAA5100JE optical flow sensor for motion tracking.
 *
 * Resets the sensor, clears the motion registers, applies the vendor
 * configuration sequence, and verifies communication by reading and
 * validating the sensor identifiers.
 *
 * @param spi_handle Pointer to the SPI peripheral used to communicate with
 *                   the optical flow sensor.
 * @return HAL_OK if the initialization succeeds, otherwise an appropriate HAL error code.
 */
HAL_StatusTypeDef paa5100_init(SPI_HandleTypeDef *spi_handle);

/**
 * @brief Reads the latest motion delta from the PAA5100JE optical flow sensor.
 *
 * Reads the X and Y motion deltas reported by the sensor.
 *
 * @param spi_handle Pointer to the SPI peripheral used to communicate with the
 *                   optical flow sensor.
 * @param delta Pointer to a structure that receives the measured X and Y
 *              motion deltas.
 * @return HAL_OK if the read succeeds, otherwise an appropriate HAL error code.
 */
HAL_StatusTypeDef paa5100_get_delta(SPI_HandleTypeDef *spi_handle,
                                    PAA5100_DELTA_t *delta);

#endif
