/**
 * @file crc8.h
 * @brief CRC8 校验工具
 * @note  移植自 Hero__2026_chasisis 项目，适配 MSPM0 平台
 */

#ifndef BSP_CRC8_H
#define BSP_CRC8_H

#include <stddef.h>

#include "stdint.h"

#define CRC_START_8 0x00

uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint8_t update_crc_8(uint8_t crc, uint8_t val);

/* CRC-8/MAXIM (Dallas 1-Wire): 右移, 反射多项式 0x8C, 初值 0x00 */
uint8_t crc8_maxim(const uint8_t *data, size_t len);

#endif // BSP_CRC8_H
