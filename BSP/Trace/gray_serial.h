/**
 ******************************************************************************
 * @file    gray_serial.h
 * @brief   八路灰度模块 GPIO 模拟串口驱动头文件
 * @note    通过 PA26(DAT) / PA27(CLK) 两线协议读取八路灰度二值数据
 *          取消注释 USE_GRAY_SERIAL 宏以启用串口模式，
 *          注释掉则使用原有的模拟 ADC 多路复用模式
 ******************************************************************************
 */

#ifndef _GRAY_SERIAL_H_
#define _GRAY_SERIAL_H_

#include "ti_msp_dl_config.h"

// ============================================================
// 取消注释以启用串口模式灰度传感器
// 注释掉则使用原有的模拟 ADC 多路复用模式
// ============================================================
#define USE_GRAY_SERIAL

void Gray_Serial_Init(void);
uint8_t Gray_Serial_Read(void);

#endif /* _GRAY_SERIAL_H_ */
