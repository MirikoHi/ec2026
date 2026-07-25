/**
 * @file lichee_rec.h
 * @brief LiChee 识别模块通信接口 (UART2)
 * @note  帧格式: 帧头(0xA5) + cmd_id(uint8_t) + data(float) + crc8
 *        使用 UART_2 (UART0外设, PA0/PA1, 9600bps)
 */

#ifndef BSP_LICHEE_REC_H
#define BSP_LICHEE_REC_H

#include "stdint.h"

#define LICHEE_REC_FRAME_HEADER  0xA5U
#define LICHEE_REC_DATA_SIZE     5U     /* cmd_id(1) + float(4) */
#define LICHEE_REC_FRAME_SIZE    7U     /* header(1) + data(5) + crc8(1) */

void    LicheeRec_Init(void);
void    LicheeRec_Send(uint8_t cmd_id, float data);
void    LicheeRec_ReceiveByte(uint8_t data);
uint8_t LicheeRec_GetCmdId(void);
float   LicheeRec_GetData(void);
uint8_t LicheeRec_IsFrameReceived(void);
void    LicheeRec_ClearFrameReceived(void);

#endif /* BSP_LICHEE_REC_H */
