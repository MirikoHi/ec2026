/**
 * @file lichee_rec.c
 * @brief LiChee 识别模块通信实现
 * @note  帧格式: 帧头(0xA5) + cmd_id(uint8_t) + data(float, 小端) + crc8
 *        CRC8 计算范围: cmd_id + float data (5字节)
 *        使用 UART_2 (UART0外设, PA0-TX / PA1-RX, 9600bps)
 */

#include "lichee_rec.h"
#include "ti_msp_dl_config.h"
#include "crc8.h"
#include <string.h>

/* ── UART 实例映射 ─────────────────────────────────────────── */
#define LICHEE_REC_UART      Licheervnano_INST           /* UART0 peripheral */
#define LICHEE_REC_UART_IRQN Licheervnano_INST_INT_IRQN   /* UART0_INT_IRQn  */

/* ── float / uint8_t 联合体 ────────────────────────────────── */
typedef union {
    float   f;
    uint8_t bytes[4];
} FloatBytes_u;

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

/* ── 静态变量 ──────────────────────────────────────────────── */
static uint8_t       rx_buf[LICHEE_REC_DATA_SIZE];   /* CRC计算缓冲区     */
static uint8_t       rx_state;                        /* 状态机当前状态     */
static uint8_t       rx_data_index;                   /* CRC缓冲写入索引   */
static uint8_t       rx_cmd_id;                      /* 当前帧的cmd_id    */
static FloatBytes_u  rx_float_data;                   /* 当前帧的float数据  */

static volatile uint8_t frame_received;               /* 帧接收标志        */
static volatile uint8_t last_cmd_id;                  /* 最新有效cmd_id    */
static volatile float   last_data;                    /* 最新有效float数据  */

/* ── 公开接口 ──────────────────────────────────────────────── */

/**
 * @brief 初始化 LiChee 通信模块
 * @note  使能 UART RX 中断, 清除FIFO, 注册中断服务函数
 */
void LicheeRec_Init(void)
{
    rx_state      = STATE_HEADER;
    rx_data_index = 0;
    frame_received = 0;
    last_cmd_id   = 0;
    last_data     = 0.0f;

    /* 清空 RX FIFO */
    while (!DL_UART_isRXFIFOEmpty(LICHEE_REC_UART))
    {
        (void)DL_UART_receiveData(LICHEE_REC_UART);
    }

    /* 使能 UART RX 中断 (syscfg 默认为 UART_2 关闭了中断) */
    DL_UART_Main_enableInterrupt(LICHEE_REC_UART, DL_UART_MAIN_INTERRUPT_RX);

    /* 清除并打开 NVIC 中断 */
    DL_UART_clearInterruptStatus(LICHEE_REC_UART, DL_UART_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(LICHEE_REC_UART_IRQN);
    NVIC_EnableIRQ(LICHEE_REC_UART_IRQN);
}

/**
 * @brief 发送一帧数据
 * @param cmd_id 命令ID
 * @param data   浮点数据
 * @note  帧格式: [0xA5][cmd_id][float(4B)][crc8]
 */
void LicheeRec_Send(uint8_t cmd_id, float data)
{
    uint8_t     buf[LICHEE_REC_FRAME_SIZE];
    FloatBytes_u fb;
    uint8_t     crc;

    /* 组装帧 */
    buf[0] = LICHEE_REC_FRAME_HEADER;
    buf[1] = cmd_id;

    fb.f    = data;
    buf[2]  = fb.bytes[0];
    buf[3]  = fb.bytes[1];
    buf[4]  = fb.bytes[2];
    buf[5]  = fb.bytes[3];

    /* CRC8 校验: 计算 cmd_id + float 数据 (5字节) */
    crc     = crc_8(&buf[1], LICHEE_REC_DATA_SIZE);
    buf[6]  = crc;

    /* 阻塞发送 */
    for (uint8_t i = 0; i < LICHEE_REC_FRAME_SIZE; i++)
    {
        DL_UART_transmitDataBlocking(LICHEE_REC_UART, buf[i]);
    }
}

/**
 * @brief 向接收状态机喂入一个字节 (由 ISR 调用)
 * @param data 接收到的字节
 */
void LicheeRec_ReceiveByte(uint8_t data)
{
    switch (rx_state)
    {
        case STATE_HEADER:
            if (data == LICHEE_REC_FRAME_HEADER)
            {
                rx_state      = STATE_CMD_ID;
                rx_data_index = 0;
            }
            break;

        case STATE_CMD_ID:
            rx_cmd_id              = data;
            rx_buf[rx_data_index++] = data;
            rx_state               = STATE_DATA0;
            break;

        case STATE_DATA0:
            rx_float_data.bytes[0] = data;
            rx_buf[rx_data_index++] = data;
            rx_state               = STATE_DATA1;
            break;

        case STATE_DATA1:
            rx_float_data.bytes[1] = data;
            rx_buf[rx_data_index++] = data;
            rx_state               = STATE_DATA2;
            break;

        case STATE_DATA2:
            rx_float_data.bytes[2] = data;
            rx_buf[rx_data_index++] = data;
            rx_state               = STATE_DATA3;
            break;

        case STATE_DATA3:
            rx_float_data.bytes[3] = data;
            rx_buf[rx_data_index++] = data;
            rx_state               = STATE_CRC;
            break;

        case STATE_CRC:
        {
            uint8_t calc_crc = crc_8(rx_buf, LICHEE_REC_DATA_SIZE);
            if (calc_crc == data)
            {
                last_cmd_id    = rx_cmd_id;
                last_data      = rx_float_data.f;
                frame_received = 1;
            }
            rx_state = STATE_HEADER;
            break;
        }

        default:
            rx_state = STATE_HEADER;
            break;
    }
}

/**
 * @brief 获取最近一次成功接收的 cmd_id
 */
uint8_t LicheeRec_GetCmdId(void)
{
    return last_cmd_id;
}

/**
 * @brief 获取最近一次成功接收的 float 数据
 */
float LicheeRec_GetData(void)
{
    return last_data;
}

/**
 * @brief 查询是否有新帧接收
 * @return 1 = 有新帧, 0 = 无
 */
uint8_t LicheeRec_IsFrameReceived(void)
{
    return frame_received;
}

/**
 * @brief 清除帧接收标志
 */
void LicheeRec_ClearFrameReceived(void)
{
    frame_received = 0;
}

/* ── UART2 中断服务函数 ────────────────────────────────────── */

/**
 * @brief UART_2 (UART0外设) 接收中断服务函数
 * @note  中断向量表: UART0_IRQHandler, 中断号: UART0_INT_IRQn
 */
void UART0_IRQHandler(void)
{
    while (!DL_UART_isRXFIFOEmpty(LICHEE_REC_UART))
    {
        LicheeRec_ReceiveByte(DL_UART_receiveData(LICHEE_REC_UART));
    }

    DL_UART_clearInterruptStatus(LICHEE_REC_UART, DL_UART_INTERRUPT_RX);
}
