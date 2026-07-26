//
// Created by gengcheese on 2026/7/10.
//

#ifndef DART_G_ICM42688_H
#define DART_G_ICM42688_H

#include "main.h"
#include <stdint.h>

/* 传感器数据起始地址 (TEMP_DATA1 = 0x1D) */
#define ICM_BURST_START             0x1D
#define ICM_BURST_LEN               14
#define ICM_SPI_READ                0x80

/* 灵敏度常量 */
#define ICM_GYRO_SENSITIVITY_2000   0.0610352f //见p1，2000dps，16g
#define ICM_ACCEL_SENSITIVITY_16G   0.0004882813f
#define pidivide180 0.0174532925f
#define g_gravity 9.8f
/* CS 宏 */
#define ICM_CS_LOW()    HAL_GPIO_WritePin(SPI1_CSB1_GPIO_Port, SPI1_CSB1_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()   HAL_GPIO_WritePin(SPI1_CSB1_GPIO_Port, SPI1_CSB1_Pin, GPIO_PIN_SET)

/* 初始化 ICM（配置寄存器 + 陀螺零偏校准） */
int  icm42688_init(SPI_HandleTypeDef *hspi);

/* 解析 14 字节原始数据 → acc(m/s²), gyro(rad/s), temp(°C, 可传NULL) */
void icm42688_parse(const uint8_t *buf, float *acc, float *gyro, float *temp);

/* 减去零偏 */
void icm42688_correct_gyro_bias(float *gyro);

#endif //DART_G_ICM42688_H