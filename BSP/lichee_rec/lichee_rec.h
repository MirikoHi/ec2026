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


/* ── UART 实例映射 ─────────────────────────────────────────── */
#define LICHEE_REC_UART      Licheervnano_INST           /* UART0 peripheral */
#define LICHEE_REC_UART_IRQN Licheervnano_INST_INT_IRQN   /* UART0_INT_IRQn  */

/* ── float / uint8_t 联合体 ────────────────────────────────── */
typedef union {
    float   f;
    uint8_t bytes[4];
} FloatBytes_u;
/*  无线通信数据帧  */
typedef struct {
    uint8_t cmdid;
    float data;
} Licheervnano_Frame;

/* ── 接收状态机状态 ────────────────────────────────────────── */
typedef enum {
    STATE_HEADER = 0,
    STATE_CMD_ID,
    STATE_DATA0,
    STATE_DATA1,
    STATE_DATA2,
    STATE_DATA3,
    STATE_CRC
} RxState_e;

// 上位机状态
typedef enum
{
    OFFLINE = 0,
    ONLINE  = 1
} LicheervnanoStatus_t;


void    LicheeRec_Init(void);
void    LicheeRec_Send(uint8_t cmd_id, float data);
void    LicheeRec_ReceiveByte(uint8_t data);
uint8_t LicheeRec_GetCmdId(void);
float   LicheeRec_GetData(void);
uint8_t LicheeRec_IsFrameReceived(void);
void    LicheeRec_ClearFrameReceived(void);
LicheervnanoStatus_t Licheervnano_CheckOnline(uint8_t cmdid, float data);

#endif /* BSP_LICHEE_REC_H */
