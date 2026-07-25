#ifndef __BMI088_H__
#define __BMI088_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ═══════════════════════════════════════════════════════════════════════
   BMI088 寄存器定义
   ═══════════════════════════════════════════════════════════════════════ */
/* ── 加速度计寄存器 ── */
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03
#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E
#define BMI088_ACC_SOFTRESET_VALUE  0xB6

/* ── 陀螺仪寄存器 ── */
#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F
#define BMI088_GYRO_RATE_X_LSB      0x02
#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

/* 数据就绪标志位 */
#define BMI088_ACCEL_DRDY           (1 << 7)  /* ACC_STATUS bit7 */
#define BMI088_GYRO_DRDY            (1 << 7)  /* GYRO_INT_STAT_1 bit7 */

/* 灵敏度定义 (对应 ±12g 与 ±2000dps) */
#define BMI088_ACCEL_SENSITIVITY_G  0.000366210938f  /* g per LSB */
#define BMI088_GYRO_SENSITIVITY_DPS 0.06103515625f   /* deg/s per LSB */

typedef struct {
    float x;
    float y;
    float z;
} bmi088RealData_t;

/* ═══════════════════════════════════════════════════════════════════════
   根据 ti_msp_dl_config.h 生成的 CS 片选宏定义
   ═══════════════════════════════════════════════════════════════════════ */
// CS1: 加速度计片选 (GPIOA.29)
#define BMI088_ACC_CS(x)  ((x) ? DL_GPIO_setPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN) : DL_GPIO_clearPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN))

// CS2: 陀螺仪片选 (GPIOB.23)
#define BMI088_GYRO_CS(x) ((x) ? DL_GPIO_setPins(BMI088_CS2_PORT, BMI088_CS2_CS2_PIN) : DL_GPIO_clearPins(BMI088_CS2_PORT, BMI088_CS2_CS2_PIN))

// SPI 外设映射 (使用 SPI0)
#define BMI088_SPI_INST   ICM42688_INST

/* API 接口 */
int8_t bsp_Bmi088Init(void);
void bsp_Bmi088GetRawData(bmi088RealData_t* accData, bmi088RealData_t* GyroData);

#endif /* __BMI088_H__ */