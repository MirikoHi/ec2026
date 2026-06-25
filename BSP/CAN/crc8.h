/**
 * @file crc8.h
 * @brief CRC8 校验工具
 * @note  移植自 Hero__2026_chasisis 项目，适配 MSPM0 平台
 */

#ifndef BSP_CRC8_H
#define BSP_CRC8_H

#include "stdint.h"

#define CRC_START_8 0x00

uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif // BSP_CRC8_H
