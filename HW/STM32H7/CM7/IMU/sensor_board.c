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

#include "sensor_board.h"

sensor_board_t *sb = NULL;

int8_t sensor_board_rev = 0;
int8_t sensor_board_imu = IMU_UNSUPPORTED;

void sensor_board_init(sensor_board_t *s){


    if (s!= NULL) sb = s;

}

void x1_set_fs(uint8_t fs){
    if (sb == NULL) return;
}

void x1_set_odr(uint8_t odr){
    if (sb == NULL) return;
}

void gyr_set_fs(uint8_t fs){
    if (sb == NULL) return;
}

void gyr_set_odr(uint8_t odr){
    if (sb == NULL) return;
}

int16_t x1_get_x(void){
    if (sb == NULL) return 0;
}

int16_t x1_get_y(void){
    if (sb == NULL) return 0;
}

int16_t x1_get_z(void){
    if (sb == NULL) return 0;
}

int16_t gyr_get_x(void){
    if (sb == NULL) return 0;
}

int16_t gyr_get_y(void){
    if (sb == NULL) return 0;
}

int16_t gyr_get_z(void){
    if (sb == NULL) return 0;
}

int16_t get_temp(void){
    if (sb == NULL) return 0;
}