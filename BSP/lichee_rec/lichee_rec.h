/**
 * @file lichee_rec.h
 * @brief LiChee 识别模块通信接口 (UART2)
 * @note  帧格式: 帧头(0xA5) + cmd_id(0x0A) + slider_length_cm(float32_LE) + relative_position(float32_LE) + crc8
 *        CRC8 计算范围: cmd_id + slider(4B) + position(4B) = 9字节
 *        使用 UART_2 (UART0外设, PA0/PA1, 115200bps)
 */

#ifndef BSP_LICHEE_REC_H
#define BSP_LICHEE_REC_H

#include "stdint.h"
#include <stdbool.h>

#define LICHEE_REC_FRAME_HEADER  0xA5U
#define LICHEE_REC_CMD_ID        0x0AU     /* 固定命令ID */
#define LICHEE_REC_DATA_SIZE     9U        /* cmd_id(1) + slider(4) + position(4) */
#define LICHEE_REC_FRAME_SIZE    11U       /* header(1) + data(9) + crc8(1) */


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
    float slider_length_cm;    /* 滑槽长度 (cm) */
    float relative_position;   /* 相对位置 */
} Licheervnano_Frame;

/* ── 接收状态机状态 ────────────────────────────────────────── */
typedef enum {
    STATE_HEADER = 0,
    STATE_CMD_ID,
    STATE_DATA0,
    STATE_DATA1,
    STATE_DATA2,
    STATE_DATA3,
    STATE_DATA4,
    STATE_DATA5,
    STATE_DATA6,
    STATE_DATA7,
    STATE_CRC
} RxState_e;

// 上位机状态
typedef enum
{
    OFFLINE = 0,
    ONLINE  = 1
} LicheervnanoStatus_t;


void    LicheeRec_Init(void);
void    LicheeRec_Send(float slider_length_cm, float relative_position);
void    LicheeRec_ReceiveByte(uint8_t data);
uint8_t LicheeRec_IsFrameReceived(void);
void    LicheeRec_ClearFrameReceived(void);
LicheervnanoStatus_t Licheervnano_CheckOnline(float slider_length_cm, float relative_position);
Licheervnano_Frame LicheeRec_GetFrame(void);

#endif /* BSP_LICHEE_REC_H */
