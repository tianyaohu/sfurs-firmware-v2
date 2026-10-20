#ifndef SENSOR_BOARD_H
#define SENSOR_BOARD_H

#include <stdint.h>

typedef enum {
	IMU_UNSUPPORTED = 0,
	IMU_LSM6        = 1,
} sensor_board_imu_t;

#define LSM6_REG_WHO_AM_I        0x0FU
#define LSM6_REG_CTRL1_XL        0x10U
#define LSM6_REG_CTRL2_G         0x11U
#define LSM6_REG_CTRL3_C         0x12U
#define LSM6_REG_OUT_TEMP_L      0x20U
#define LSM6_REG_OUTX_L_G        0x22U
#define LSM6_REG_OUTY_L_G        0x24U
#define LSM6_REG_OUTZ_L_G        0x26U
#define LSM6_REG_OUTX_L_A        0x28U
#define LSM6_REG_OUTY_L_A        0x2AU
#define LSM6_REG_OUTZ_L_A        0x2CU

#define LSM6_XL_FS_2G            0b00
#define LSM6_XL_FS_4G            0b10
#define LSM6_XL_FS_8G            0b11
#define LSM6_XL_FS_16G           0b01

#define LSM6_XL_ODR_OFF          0b0000
#define LSM6_XL_ODR_12ZH5        0b0001
#define LSM6_XL_ODR_26HZ         0b0010
#define LSM6_XL_ODR_52HZ         0b0011
#define LSM6_XL_ODR_104HZ        0b0100
#define LSM6_XL_ODR_208HZ        0b0101
#define LSM6_XL_ODR_416HZ        0b0110
#define LSM6_XL_ODR_833HZ        0b0111
#define LSM6_XL_ODR_1660HZ       0b1000
#define LSM6_XL_ODR_3330HZ       0b1001
#define LSM6_XL_ODR_6660HZ       0b1010

#define LSM6_GYR_FS_250DPS       0b00
#define LSM6_GYR_FS_500DPS       0b01
#define LSM6_GYR_FS_1000DPS      0b10
#define LSM6_GYR_FS_2000DPS      0b11

#define LSM6_GYR_ODR_OFF         0b0000
#define LSM6_GYR_ODR_12HZ5       0b0001
#define LSM6_GYR_ODR_26HZ        0b0010
#define LSM6_GYR_ODR_52HZ        0b0011
#define LSM6_GYR_ODR_104HZ       0b0100
#define LSM6_GYR_ODR_208HZ       0b0101
#define LSM6_GYR_ODR_416HZ       0b0110
#define LSM6_GYR_ODR_833HZ       0b0111
#define LSM6_GYR_ODR_1660HZ      0b1000
#define LSM6_GYR_ODR_3330HZ      0b1001
#define LSM6_GYR_ODR_6660HZ      0b1010

typedef struct {
	void *spi;
	void *imu_cs_port;
	uint16_t imu_cs_pin;
	void *imu_reset_port;
	uint16_t imu_reset_pin;
	void *imu_int1_port;
	uint16_t imu_int1_pin;
	void *imu_int2_port;
	uint16_t imu_int2_pin;
	unsigned long spi_timeout_ms;
} sensor_board_t;

extern sensor_board_t *sb;
extern int8_t sensor_board_rev;
extern sensor_board_imu_t sensor_board_imu;

void sensor_board_init(sensor_board_t *s);

void xl_set_fs(uint8_t fs);
void xl_set_odr(uint8_t odr);

void gyr_set_fs(uint8_t fs);
void gyr_set_odr(uint8_t odr);

int16_t xl_get_x(void);
int16_t xl_get_y(void);
int16_t xl_get_z(void);

int16_t gyr_get_x(void);
int16_t gyr_get_y(void);
int16_t gyr_get_z(void);

int16_t get_temp(void);

#endif