#ifndef SPI_DATATYPES_H
#define SPI_DATATYPES_H

#include <stdint.h>
#include "main.h"

#define SPI_OPTICAL_FLOW_CHIP_SELECT_GPIO SPI2_CS_GPIO_Port
#define SPI_OPTICAL_FLOW_CHIP_SELECT_PIN  SPI2_CS_Pin

typedef struct {
    int32_t delta_x;
    int32_t delta_y;
} SPI_OPTICAL_FLOW_DELTA_DATA_t;

HAL_StatusTypeDef spi_optical_flow_read_register(SPI_HandleTypeDef *spi_pointer,
                                                 uint8_t reg, uint8_t *value);
HAL_StatusTypeDef spi_optical_flow_init(SPI_HandleTypeDef *spi_pointer);
HAL_StatusTypeDef spi_optical_flow_get_delta(SPI_HandleTypeDef *spi_pointer,
                                             SPI_OPTICAL_FLOW_DELTA_DATA_t *delta_data);

#endif