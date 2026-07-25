/**
 ******************************************************************************
 * @file    icm42688.h
 * @author  Geng LX (ported from Legacy by Geng LX)
 * @brief   ICM42688 6-axis IMU SPI driver header
 *
 * @note    Uses MSPM0 DL SPI API. Register definitions based on
 *          DS-ICM-42688v1.2 datasheet.
 ******************************************************************************
 */

#ifndef __BSP_ICM42688_H__
#define __BSP_ICM42688_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Register Map (Bank 0, unless noted otherwise)
 * ═══════════════════════════════════════════════════════════════════════ */

// Bank 0
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_INT_STATUS2               0x37
#define ICM42688_INT_STATUS3               0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76

// Bank 1
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

// Bank 2
#define ICM42688_ACCEL_CONFIG_STATIC2      0x03
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04
#define ICM42688_ACCEL_CONFIG_STATIC4      0x05

// Bank 4
#define ICM42688_GYRO_ON_OFF_CONFIG        0x0E
#define ICM42688_OFFSET_USER0              0x77
#define ICM42688_OFFSET_USER1              0x78
#define ICM42688_OFFSET_USER2              0x79
#define ICM42688_OFFSET_USER3              0x7A
#define ICM42688_OFFSET_USER4              0x7B
#define ICM42688_OFFSET_USER5              0x7C
#define ICM42688_OFFSET_USER6              0x7D
#define ICM42688_OFFSET_USER7              0x7E
#define ICM42688_OFFSET_USER8              0x7F

/* ═══════════════════════════════════════════════════════════════════════
 * Configuration Constants
 * ═══════════════════════════════════════════════════════════════════════ */

/* Full-scale range options */
#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00

#define GFS_2000DPS   0x00
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_125DPS 0x07

/* ODR options */
#define AODR_1000Hz  0x06
#define AODR_200Hz   0x07
#define AODR_100Hz   0x08
#define AODR_50Hz    0x09

#define GODR_1000Hz  0x06
#define GODR_200Hz   0x07
#define GODR_100Hz   0x08
#define GODR_50Hz    0x09

/* Sensitivity constants (Legacy verified values) */
#define ICM_GYRO_SENSITIVITY_2000   0.0610352f     /* 2000/32768 [dps/LSB] */
#define ICM_ACCEL_SENSITIVITY_16G   0.0004882813f  /* 16/32768 [g/LSB]   */
#define pidivide180                 0.0174532925f  /* pi/180             */
#define g_gravity                   9.8f           /* [m/s²]             */

/* Sensor data burst read */
#define ICM_BURST_START             0x1D
#define ICM_BURST_LEN               14
#define ICM_SPI_READ                0x80

/* WHO_AM_I expected value */
#define ICM42688_ID                 0x47

/* ═══════════════════════════════════════════════════════════════════════
 * Type Definitions
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float x;
    float y;
    float z;
} icm42688RealData_t;

/* ═══════════════════════════════════════════════════════════════════════
 * API Functions
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize ICM42688 (register config + gyro bias calibration)
 * @retval 0 = success, -1 = WHO_AM_I mismatch
 */
int8_t bsp_Icm42688Init(void);

/**
 * @brief  Read temperature
 * @param  pTemp  [out] temperature in °C / 100 (int16_t)
 * @retval 0 = success
 */
int8_t bsp_IcmGetTemperature(int16_t *pTemp);

/**
 * @brief  Read accelerometer + gyroscope data in one burst
 * @param  accData  [out] acceleration in m/s²
 * @param  gyroData [out] angular velocity in rad/s
 * @retval 0 = success
 */
int8_t bsp_IcmGetRawData(icm42688RealData_t *accData, icm42688RealData_t *gyroData);

/**
 * @brief  Subtract calibrated gyro bias from readings
 * @param  gyro [in/out] gyro rad/s array, corrected in-place
 */
void icm42688_correct_gyro_bias(float *gyro);

#endif /* __BSP_ICM42688_H__ */
