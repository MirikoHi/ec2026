#ifndef __BSP_ICM42688_IMPROVED_H__
#define __BSP_ICM42688_IMPROVED_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define ICM42688_IMP_DEVICE_CONFIG       0x11
#define ICM42688_IMP_TEMP_DATA1          0x1D
#define ICM42688_IMP_PWR_MGMT0           0x4E
#define ICM42688_IMP_GYRO_CONFIG0        0x4F
#define ICM42688_IMP_ACCEL_CONFIG0       0x50
#define ICM42688_IMP_GYRO_ACCEL_CONFIG0  0x52
#define ICM42688_IMP_WHO_AM_I            0x75
#define ICM42688_IMP_REG_BANK_SEL        0x76

#define ICM42688_IMP_AFS_16G             0x00
#define ICM42688_IMP_AFS_4G              0x02
#define ICM42688_IMP_GFS_2000DPS         0x00
#define ICM42688_IMP_GFS_1000DPS         0x01
#define ICM42688_IMP_AODR_1000HZ         0x06
#define ICM42688_IMP_AODR_200HZ          0x07
#define ICM42688_IMP_GODR_1000HZ         0x06
#define ICM42688_IMP_GODR_200HZ          0x07

#define ICM42688_IMP_GYRO_SENS_2000      0.0610352f
#define ICM42688_IMP_GYRO_SENS_1000      0.0305176f
#define ICM42688_IMP_ACCEL_SENS_16G      0.0004882813f
#define ICM42688_IMP_ACCEL_SENS_4G       0.0001220703f
#define ICM42688_IMP_PI_DIV_180          0.0174532925f
#define ICM42688_IMP_GRAVITY             9.8f
#define ICM42688_IMP_BURST_START         0x1D
#define ICM42688_IMP_BURST_LEN           14
#define ICM42688_IMP_ID                  0x47

typedef struct {
    float x;
    float y;
    float z;
} icm42688_imp_data_t;

int8_t ICM42688_Improved_Init(void);
int8_t ICM42688_Improved_GetTemperature(int16_t *temp);
int8_t ICM42688_Improved_GetRawData(icm42688_imp_data_t *acc,
                                    icm42688_imp_data_t *gyro);
void ICM42688_Improved_CorrectGyroBias(float *gyro);
uint8_t ICM42688_Improved_IsGyroCalibrated(void);

#endif
