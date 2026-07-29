/**
 * @file    TJC.h
 * @brief   TJC 串口屏驱动 — 通过 UART_2 接收 HMI 按键数据，解析参数修改命令
 * @note    协议详见 BSP/Comm/TJC_PROTOCOL.md
 */

#ifndef BSP_COMM_TJC_H
#define BSP_COMM_TJC_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/**
 * @brief 初始化 TJC 串口屏驱动
 * @note  需要在 UART_2 硬件初始化 (SYSCFG_DL_init) 之后调用
 * @note  启用 UART_2 RX 中断，开始接收屏幕数据
 */
void TJC_Init(void);

/**
 * @brief TJC 主处理函数，在 RobotCmdTask (200Hz) 中调用
 * @note  检查是否有完整帧到达，解析命令并修改参数
 * @note  不会阻塞 —— 帧不完整或没有新帧时立即返回
 */
void TJC_Process(void);

/**
 * @brief 向 TJC 接收状态机喂一个字节
 * @param byte 从 UART_2 RX FIFO 读取的一个字节
 * @note  由 UART_2 ISR 调用 (ISR 上下文)
 */
void TJC_FeedByte(uint8_t byte);

#endif /* BSP_COMM_TJC_H */
