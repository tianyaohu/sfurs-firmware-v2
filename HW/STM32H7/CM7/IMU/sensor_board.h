#ifndef SENSOR_BOARD_H
#define SENSOR_BOARD_H

#include <stdio.h>
#include <stdlib.h>

// Various pin-compatible IMUs that could be implemented on the sensor board.
#define IMU_UNSUPPORTED                         0
#define IMU_BMI_270                             1     
#define IMU_LSM6                                2
#define IMU_ICM_42670                           3

// Register definitions for LSM6 IMU
#define LSM6_REG_FUNC_CFG_ACCESS                0x01
#define LSM6_REG_PIN_CTRL                       0x02
#define LSM6_REG_FIFO_CTRL1                     0x07
#define LSM6_REG_FIFO_CTRL2                     0x08
#define LSM6_REG_FIFO_CTRL3                     0x09
#define LSM6_REG_FIFO_CTRL4                     0x0A
#define LSM6_REG_COUNTER_BDR_REG1               0x0B
#define LSM6_REG_COUNTER_BDR_REG2               0x0C
#define LSM6_REG_INT1_CTRL                      0x0D
#define LSM6_REG_INT2_CTRL                      0x0E
#define LSM6_REG_WHO_AM_I                       0x0F
#define LSM6_REG_CTRL1_XL                       0x10
#define LSM6_REG_CTRL2_G                        0x11
#define LSM6_REG_CTRL3_C                        0x12
#define LSM6_REG_CTRL4_C                        0x13
#define LSM6_REG_CTRL5_C                        0x14
#define LSM6_REG_CTRL6_C                        0x15
#define LSM6_REG_CTRL7_G                        0x16
#define LSM6_REG_CTRL8_XL                       0x17
#define LSM6_REG_CTRL9_XL                       0x18
#define LSM6_REG_CTRL_IO_C                      0x19
#define LSM6_REG_ALL_INT_SRC                    0x1A
#define LSM6_REG_WAKE_UP_SRC                    0x1B
#define LSM6_REG_TAP_SRC                        0x1C
#define LSM6_REG_D6D_SRC                        0x1D
#define LSM6_REG_STATUS_REG                     0x1E
#define LSM6_REG_OUT_TEMP_L                     0x20
#define LSM6_REG_OUT_TEMP_H                     0x21
#define LSM6_REG_OUTX_L_G                       0x22
#define LSM6_REG_OUTX_H_G                       0x23
#define LSM6_REG_OUTY_L_G                       0x24
#define LSM6_REG_OUTY_H_G                       0x25
#define LSM6_REG_OUTZ_L_G                       0x26
#define LSM6_REG_OUTZ_H_G                       0x27
#define LSM6_REG_OUTX_L_A                       0x28
#define LSM6_REG_OUTX_H_A                       0x29
#define LSM6_REG_OUTY_L_A                       0x2A
#define LSM6_REG_OUTY_H_A                       0x2B
#define LSM6_REG_OUTZ_L_A                       0x2C
#define LSM6_REG_OUTZ_H_A                       0x2D
#define LSM6_REG_EMB_FUNC_STATUS_MAINPAGE       0x35
#define LSM6_REG_FSM_STATUS_A_MAINPAGE          0x36
#define LSM6_REG_FSM_STATUS_B_MAINPAGE          0x37
#define LSM6_REG_STATUS_MASTER_MAINPAGE         0x39
#define LSM6_REG_FIFO_STATUS1                   0x3A
#define LSM6_REG_FIFO_STATUS2                   0x3B
#define LSM6_REG_TIMESTAMP0                     0x40
#define LSM6_REG_TIMESTAMP1                     0x41
#define LSM6_REG_TIMESTAMP2                     0x42
#define LSM6_REG_TIMESTAMP3                     0x43
#define LSM6_REG_TAP_CFG0                       0x56
#define LSM6_REG_TAP_CFG1                       0x57
#define LSM6_REG_TAP_CFG2                       0x58
#define LSM6_REG_TAP_THS_6D                     0x59
#define LSM6_REG_INT_DUR2                       0x5A
#define LSM6_REG_WAKE_UP_THS                    0x5B
#define LSM6_REG_WAKE_UP_DUR                    0x5C
#define LSM6_REG_FREE_FALL                      0x5D
#define LSM6_REG_MD1_CFG                        0x5E
#define LSM6_REG_MD2_CFG                        0x5F
#define LSM6_REG_I3C_BUS_AVB                    0x62
#define LSM6_REG_INTERNAL_FREQ_FINE             0x63
#define LSM6_REG_INT_OIS                        0x6F
#define LSM6_REG_CTRL1_OIS                      0x70
#define LSM6_REG_CTRL2_OIS                      0x71
#define LSM6_REG_CTRL3_OIS                      0x72
#define LSM6_REG_X_OFS_USR                      0x73
#define LSM6_REG_Y_OFS_USR                      0x74
#define LSM6_REG_Z_OFS_USR                      0x75
#define LSM6_REG_FIFO_DATA_OUT_TAG              0x78
#define LSM6_REG_FIFO_DATA_OUT_X_L              0x79
#define LSM6_REG_FIFO_DATA_OUT_X_H              0x7A
#define LSM6_REG_FIFO_DATA_OUT_Y_L              0x7B
#define LSM6_REG_FIFO_DATA_OUT_Y_H              0x7C
#define LSM6_REG_FIFO_DATA_OUT_Z_L              0x7D
#define LSM6_REG_FIFO_DATA_OUT_Z_H              0X7E

#define LSM6_XL_FS_2G                           0b00
#define LSM6_XL_FS_4G                           0b10
#define LSM6_XL_FS_8G                           0b11
#define LSM6_XL_FS_16G                          0b01

#define LSM6_XL_ODR_OFF                         0b0000
#define LSM6_XL_ODR_1HZ6                        0b1011
#define LSM6_XL_ODR_12ZH5                       0b0001
#define LSM6_XL_ODR_26HZ                        0b0010
#define LSM6_XL_ODR_52HZ                        0b0011
#define LSM6_XL_ODR_104HZ                       0b0100
#define LSM6_XL_ODR_208HZ                       0b0101
#define LSM6_XL_ODR_416HZ                       0b0110
#define LSM6_XL_ODR_833HZ                       0b0111
#define LSM6_XL_ODR_1660HZ                      0b1000
#define LSM6_XL_ODR_3330HZ                      0b1001
#define LSM6_XL_ODR_6660HZ                      0b1010

#define LSM6_GYR_FS_250DPS                      0b00
#define LSM6_GYR_FS_500DPS                      0b01
#define LSM6_GYR_FS_1000DPS                     0b10
#define LSM6_GYR_FS_2000DPS                     0b11

#define LSM6_GYR_ODR_OFF                        0b0000
#define LSM6_GYR_ODR_12HZ5                      0b0001
#define LSM6_GYR_ODR_26HZ                       0b0010
#define LSM6_GYR_ODR_52HZ                       0b0011
#define LSM6_GYR_ODR_104HZ                      0b0100
#define LSM6_GYR_ODR_208HZ                      0b0101
#define LSM6_GYR_ODR_416HZ                      0b0110
#define LSM6_GYR_ODR_833HZ                      0b0111
#define LSM6_GYR_ODR_1660HZ                     0b1000
#define LSM6_GYR_ODR_3330HZ                     0b1001
#define LSM6_GYR_ODR_6660HZ                     0b1010


type_def struct {

}sensor_board_t;

extern sensor_board_t *sb;

extern int8_t sensor_board_rev;
extern int8_t sensor_board_imu;

void sensor_board_init(sensor_board_t *s);

void x1_set_fs(uint8_t fs);

void x1_set_odr(uint8_t odr);

void gyr_set_fs(uint8_t fs);

void gyr_set_odr(uint8_t odr);

int16_t x1_get_x(void);

int16_t x1_get_y(void);

int16_t x1_get_z(void);

int16_t gyr_get_x(void);

int16_t gyr_get_y(void);

int16_t gyr_get_z(void);


int16_t get_temp(void);

#endif