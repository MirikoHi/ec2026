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
#include <stdint.h>
#include <stdbool.h>



// licheervnano当前状态
LicheervnanoStatus_t host_status = OFFLINE;

/* ── 静态变量 ──────────────────────────────────────────────── */
static uint8_t       rx_buf[LICHEE_REC_DATA_SIZE];   /* CRC计算缓冲区     */
static uint8_t       rx_state;                        /* 状态机当前状态     */
static uint8_t       rx_data_index;                   /* CRC缓冲写入索引   */
static uint8_t       rx_cmd_id;                      /* 当前帧的cmd_id    */
static volatile FloatBytes_u  rx_float_data;                   /* 当前帧的float数据  */

static volatile uint8_t frame_received;               /* 帧接收标志        */
static volatile Licheervnano_Frame LicheeRec_Frame;    /* CRC 校验通过的有效帧 */
static volatile uint32_t crc_pass_cnt;                  /* 调试：CRC通过计数  */
static volatile uint32_t crc_fail_cnt;                  /* 调试：CRC失败计数  */
static volatile uint8_t  dbg_calc_crc;                  /* 调试：接收端计算的CRC */
static volatile uint8_t  dbg_recv_crc;                  /* 调试：发送端传来的CRC */
static volatile uint8_t  dbg_rx_buf[LICHEE_REC_DATA_SIZE]; /* 调试：CRC计算的5字节 */

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
    LicheeRec_Frame.cmdid = 0;
    LicheeRec_Frame.data = 0.0f;

    /* 清空 RX FIFO（限定最多清 16 次，防止死循环） */
    for (int _i = 0; _i < 16; _i++)
    {
        if (DL_UART_isRXFIFOEmpty(LICHEE_REC_UART))
            break;
        (void)DL_UART_receiveData(LICHEE_REC_UART);
    }
    // DL_GPIO_setInternalResistor(GPIOA, DL_GPIO_PIN_1, DL_GPIO_RESISTOR_PULL_UP);
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
    crc     = crc8_maxim(&buf[1], LICHEE_REC_DATA_SIZE);
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
            uint8_t calc_crc = crc8_maxim(rx_buf, LICHEE_REC_DATA_SIZE);

            /* 调试：保存最近一次 CRC 校验的详细信息 */
            dbg_calc_crc = calc_crc;
            dbg_recv_crc = data;
            dbg_rx_buf[0] = rx_buf[0];
            dbg_rx_buf[1] = rx_buf[1];
            dbg_rx_buf[2] = rx_buf[2];
            dbg_rx_buf[3] = rx_buf[3];
            dbg_rx_buf[4] = rx_buf[4];

            if (calc_crc == data)
            {
                frame_received = 1;
                LicheeRec_Frame.cmdid = rx_cmd_id ;
                LicheeRec_Frame.data = rx_float_data.f;
                crc_pass_cnt++;
            }
            else
            {
                crc_fail_cnt++;
            }
            rx_state = STATE_HEADER;
            break;
        }

        default:
            rx_state = STATE_HEADER;
            break;
    }
}


Licheervnano_Frame LicheeRec_GetFrame(void) {
    /* 返回 CRC 校验通过后存储的有效帧，而非状态机中间值 */
    return LicheeRec_Frame;
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

/**
 * @brief  接收上位机命令，判断在线状态
 * @param  cmdid: 命令ID
 * @param  data : 数据
 * @return 当前上位机状态
 */
LicheervnanoStatus_t Licheervnano_CheckOnline(uint8_t cmdid, float data)
{
    if(cmdid == 1 && data == 0)
    {
        // 上位机上线
        host_status = ONLINE;
    }
    else if(cmdid == 0 && data == 0)
    {
        // 上位机下线
        host_status = OFFLINE;
    }

    return host_status;
}


/* ── UART2 中断服务函数 ────────────────────────────────────── */

/**
 * @brief UART_2 (UART0外设) 接收中断服务函数
 * @note  中断向量表: UART0_IRQHandler, 中断号: UART0_INT_IRQn
 */
void UART0_IRQHandler(void)
{
    uint8_t recByte = DL_UART_receiveData(LICHEE_REC_UART);
    LicheeRec_ReceiveByte(recByte);

    DL_UART_clearInterruptStatus(LICHEE_REC_UART, DL_UART_INTERRUPT_RX);
}
