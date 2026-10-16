/**
 * @file            sensor_board.c
 * @brief           Source code for all sensor_board related functions
 */

/************************************************************************
 * Copyright (c) 2023 Bandits-FC SFURS                                  *
 *                                                                      *
 * Permission is hereby granted, free of charge, to any person          *
 * obtaining a copy of this software and associated documentation       *
 * files (the "Software"), to deal in the Software without restriction, *
 * including without limitation the rights to use, copy, modify, merge, *
 * publish, distribute, sublicense, and/or sell copies of the Software, *
 * and to permit persons to whom the Software is furnished to do so,    *
 * subject to the following conditions:                                 *
 *                                                                      *
 * The above copyright notice and this permission notice shall be       *
 * included in all copies or substantial portions of the Software.      *
 *                                                                      *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      *
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES      *
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE                 *
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT      *
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,         *
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING         *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR        *
 * OTHER DEALINGS IN THE SOFTWARE.                                      *
 *                                                                      *
 * This file is part of sensor_board.                                   *
 *                                                                      *
 * Author:        Bandits-FC SFURS <sfurs@sfu.ca>                     *
 ************************************************************************/

#include "stm32h7xx_hal.h"
#include "sensor_board.h"

/*
 * The sensor board layer is IMU-only on the STM32H7 target. The board code is
 * expected to populate this context with the shared SPI bus and IMU control
 * pins before calling sensor_board_init().
 */

#define SENSOR_BOARD_SPI_READ_MASK   0x80U
#define SENSOR_BOARD_LSM6_WHO_AM_I   0x6CU
#define SENSOR_BOARD_SPI_TIMEOUT_MS   10U

sensor_board_t *sb = NULL;

int8_t sensor_board_rev = 0;
sensor_board_imu_t sensor_board_imu = IMU_UNSUPPORTED;

static uint8_t sensor_board_has_bus(void)
{
    if (sb == NULL || sb->spi == NULL || sb->imu_cs_port == NULL || sb->imu_cs_pin == 0U) {
        return 0U;
    }

    return 1U;
}

static uint8_t sensor_board_has_lsm6(void)
{
    return (sensor_board_has_bus() != 0U && sensor_board_imu == IMU_LSM6) ? 1U : 0U;
}

static uint32_t sensor_board_timeout(void)
{
    if (sb != NULL && sb->spi_timeout_ms != 0U) {
        return sb->spi_timeout_ms;
    }

    return SENSOR_BOARD_SPI_TIMEOUT_MS;
}

static void sensor_board_select_imu(void)
{
    if (sensor_board_has_bus() == 0U) {
        return;
    }

    HAL_GPIO_WritePin((GPIO_TypeDef *)sb->imu_cs_port, sb->imu_cs_pin, GPIO_PIN_RESET);
}

static void sensor_board_deselect_imu(void)
{
    if (sensor_board_has_bus() == 0U) {
        return;
    }

    HAL_GPIO_WritePin((GPIO_TypeDef *)sb->imu_cs_port, sb->imu_cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef sensor_board_read_reg(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t address = (uint8_t)(reg | SENSOR_BOARD_SPI_READ_MASK);
    uint8_t dummy = 0U;

    if (value == NULL || sensor_board_has_bus() == 0U) {
        return HAL_ERROR;
    }

    sensor_board_select_imu();
    status = HAL_SPI_Transmit((SPI_HandleTypeDef *)sb->spi, &address, 1U,
                              sensor_board_timeout());

    if (status == HAL_OK) {
        status = HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)sb->spi, &dummy, value, 1U,
                                         sensor_board_timeout());
    }

    sensor_board_deselect_imu();
    return status;
}

static HAL_StatusTypeDef sensor_board_write_reg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t payload[2] = {(uint8_t)(reg & 0x7FU), value};

    if (sensor_board_has_bus() == 0U) {
        return HAL_ERROR;
    }

    sensor_board_select_imu();
    status = HAL_SPI_Transmit((SPI_HandleTypeDef *)sb->spi, payload, 2U,
                              sensor_board_timeout());
    sensor_board_deselect_imu();

    return status;
}

static HAL_StatusTypeDef sensor_board_update_reg_bits(uint8_t reg, uint8_t clear_mask,
                                                      uint8_t set_mask)
{
    uint8_t value = 0U;

    if (sensor_board_read_reg(reg, &value) != HAL_OK) {
        return HAL_ERROR;
    }

    value &= clear_mask;
    value |= set_mask;

    return sensor_board_write_reg(reg, value);
}

static HAL_StatusTypeDef sensor_board_read_axis_pair(uint8_t low_reg, int16_t *value)
{
    uint8_t low = 0U;
    uint8_t high = 0U;

    if (value == NULL) {
        return HAL_ERROR;
    }

    if (sensor_board_read_reg(low_reg, &low) != HAL_OK) {
        return HAL_ERROR;
    }

    if (sensor_board_read_reg((uint8_t)(low_reg + 1U), &high) != HAL_OK) {
        return HAL_ERROR;
    }

    *value = (int16_t)((((uint16_t)high) << 8) | low);
    return HAL_OK;
}

static sensor_board_imu_t sensor_board_probe_imu(void)
{
    uint8_t who_am_i = 0U;

    if (sensor_board_read_reg(LSM6_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
        return IMU_UNSUPPORTED;
    }

    if (who_am_i == SENSOR_BOARD_LSM6_WHO_AM_I) {
        return IMU_LSM6;
    }

    return IMU_UNSUPPORTED;
}

static void sensor_board_configure_lsm6(void)
{
    /* Preserve the original default startup profile for the IMU path. */
    xl_set_fs(LSM6_XL_FS_2G);
    xl_set_odr(LSM6_XL_ODR_208HZ);
    gyr_set_fs(LSM6_GYR_FS_250DPS);
    gyr_set_odr(LSM6_GYR_ODR_208HZ);
}

void sensor_board_init(sensor_board_t *s)
{
    sb = s;
    sensor_board_rev = 0;
    sensor_board_imu = IMU_UNSUPPORTED;

    if (sb == NULL || sensor_board_has_bus() == 0U) {
        return;
    }

    sensor_board_deselect_imu();
    sensor_board_imu = sensor_board_probe_imu();

    if (sensor_board_imu == IMU_LSM6) {
        sensor_board_configure_lsm6();
    }
}

void xl_set_fs(uint8_t fs)
{
    if (sensor_board_has_lsm6() == 0U) {
        return;
    }

    (void)sensor_board_update_reg_bits(LSM6_REG_CTRL1_XL, 0xF3U,
                                       (uint8_t)((fs & 0x03U) << 2U));
}

void xl_set_odr(uint8_t odr)
{
    if (sensor_board_has_lsm6() == 0U) {
        return;
    }

    (void)sensor_board_update_reg_bits(LSM6_REG_CTRL1_XL, 0x0FU,
                                       (uint8_t)((odr & 0x0FU) << 4U));
}

void gyr_set_fs(uint8_t fs)
{
    if (sensor_board_has_lsm6() == 0U) {
        return;
    }

    (void)sensor_board_update_reg_bits(LSM6_REG_CTRL2_G, 0xF3U,
                                       (uint8_t)((fs & 0x03U) << 2U));
}

void gyr_set_odr(uint8_t odr)
{
    if (sensor_board_has_lsm6() == 0U) {
        return;
    }

    (void)sensor_board_update_reg_bits(LSM6_REG_CTRL2_G, 0x0FU,
                                       (uint8_t)((odr & 0x0FU) << 4U));
}

int16_t xl_get_x(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUTX_L_A, &value) != HAL_OK) {
        return 0;
    }

    return value;
}

int16_t xl_get_y(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUTY_L_A, &value) != HAL_OK) {
        return 0;
    }

    return value;
}

int16_t xl_get_z(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUTZ_L_A, &value) != HAL_OK) {
        return 0;
    }

    return value;
}

int16_t gyr_get_x(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUTX_L_G, &value) != HAL_OK) {
        return 0;
    }

    return value;
}

int16_t gyr_get_y(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUTY_L_G, &value) != HAL_OK) {
        return 0;
    }

    return value;
}

int16_t gyr_get_z(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUTZ_L_G, &value) != HAL_OK) {
        return 0;
    }

    return value;
}

int16_t get_temp(void)
{
    int16_t value = 0;

    if (sensor_board_has_lsm6() == 0U) {
        return 0;
    }

    if (sensor_board_read_axis_pair(LSM6_REG_OUT_TEMP_L, &value) != HAL_OK) {
        return 0;
    }

    return value;
}