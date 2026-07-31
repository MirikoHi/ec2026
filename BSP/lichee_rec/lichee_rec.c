/**
 * @file lichee_rec.c
 * @brief LiChee 识别模块通信实现
 * @note  帧格式: 帧头(0xA5) + cmd_id(0x0A) + slider_length_cm(float32_LE) + relative_position(float32_LE) + crc8
 *        CRC8 计算范围: cmd_id + slider(4B) + position(4B) = 9字节
 *        使用 UART_2 (UART0外设, PA0-TX / PA1-RX, 115200bps)
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
static uint8_t       rx_buf[LICHEE_REC_DATA_SIZE];     /* CRC计算缓冲区 (9B)   */
static uint8_t       rx_state;                         /* 状态机当前状态       */
static uint8_t       rx_data_index;                    /* CRC缓冲写入索引      */
static FloatBytes_u  rx_slider;                        /* 滑槽长度 float 数据   */
static FloatBytes_u  rx_position;                      /* 相对位置 float 数据   */

static volatile uint8_t frame_received;                /* 帧接收标志           */
static volatile Licheervnano_Frame LicheeRec_Frame;     /* CRC 校验通过的有效帧  */
static volatile uint32_t crc_pass_cnt;                  /* 调试：CRC通过计数     */
static volatile uint32_t crc_fail_cnt;                  /* 调试：CRC失败计数     */
static volatile uint8_t  dbg_calc_crc;                  /* 调试：接收端计算的CRC */
static volatile uint8_t  dbg_recv_crc;                  /* 调试：发送端传来的CRC */
static volatile uint8_t  dbg_rx_buf[LICHEE_REC_DATA_SIZE]; /* 调试：CRC计算的9字节 */

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
    LicheeRec_Frame.slider_length_cm = 0.0f;
    LicheeRec_Frame.relative_position = 0.0f;

    /* 清空 RX FIFO（限定最多清 16 次，防止死循环） */
    for (int _i = 0; _i < 16; _i++)
    {
        if (DL_UART_isRXFIFOEmpty(LICHEE_REC_UART))
            break;
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
 * @param slider_length_cm  滑槽长度 (cm)
 * @param relative_position 相对位置
 * @note  帧格式: [0xA5][0x0A][slider(4B_LE)][position(4B_LE)][crc8]
 */
void LicheeRec_Send(float slider_length_cm, float relative_position)
{
    uint8_t     buf[LICHEE_REC_FRAME_SIZE];
    FloatBytes_u fb_slider;
    FloatBytes_u fb_pos;
    uint8_t     crc;

    /* 组装帧 */
    buf[0] = LICHEE_REC_FRAME_HEADER;           /* 帧头 0xA5 */
    buf[1] = LICHEE_REC_CMD_ID;                 /* 固定 cmdid=0x0A */

    fb_slider.f = slider_length_cm;
    buf[2] = fb_slider.bytes[0];
    buf[3] = fb_slider.bytes[1];
    buf[4] = fb_slider.bytes[2];
    buf[5] = fb_slider.bytes[3];

    fb_pos.f = relative_position;
    buf[6] = fb_pos.bytes[0];
    buf[7] = fb_pos.bytes[1];
    buf[8] = fb_pos.bytes[2];
    buf[9] = fb_pos.bytes[3];

    /* CRC8 校验: 计算 cmd_id + slider(4B) + position(4B) (9字节) */
    crc     = crc8_maxim(&buf[1], LICHEE_REC_DATA_SIZE);
    buf[10] = crc;

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
            rx_buf[rx_data_index++] = data;        /* rx_buf[0] = cmd_id */
            rx_state               = STATE_DATA0;
            break;

        case STATE_DATA0:
            rx_slider.bytes[0] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[1] */
            rx_state               = STATE_DATA1;
            break;

        case STATE_DATA1:
            rx_slider.bytes[1] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[2] */
            rx_state               = STATE_DATA2;
            break;

        case STATE_DATA2:
            rx_slider.bytes[2] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[3] */
            rx_state               = STATE_DATA3;
            break;

        case STATE_DATA3:
            rx_slider.bytes[3] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[4] */
            rx_state               = STATE_DATA4;
            break;

        case STATE_DATA4:
            rx_position.bytes[0] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[5] */
            rx_state               = STATE_DATA5;
            break;

        case STATE_DATA5:
            rx_position.bytes[1] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[6] */
            rx_state               = STATE_DATA6;
            break;

        case STATE_DATA6:
            rx_position.bytes[2] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[7] */
            rx_state               = STATE_DATA7;
            break;

        case STATE_DATA7:
            rx_position.bytes[3] = data;
            rx_buf[rx_data_index++] = data;        /* rx_buf[8] */
            rx_state               = STATE_CRC;
            break;

        case STATE_CRC:
        {
            uint8_t calc_crc = crc8_maxim(rx_buf, LICHEE_REC_DATA_SIZE);

            /* 调试：保存最近一次 CRC 校验的详细信息 */
            dbg_calc_crc = calc_crc;
            dbg_recv_crc = data;
            for (uint8_t k = 0; k < LICHEE_REC_DATA_SIZE; k++) {
                dbg_rx_buf[k] = rx_buf[k];
            }

            if (calc_crc == data)
            {
                frame_received = 1;
                LicheeRec_Frame.slider_length_cm = rx_slider.f;
                LicheeRec_Frame.relative_position = rx_position.f;
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
 * @param  slider_length_cm  滑槽长度 (cm)
 * @param  relative_position 相对位置
 * @return 当前上位机状态
 */
LicheervnanoStatus_t Licheervnano_CheckOnline(float slider_length_cm, float relative_position)
{
    if (slider_length_cm > 0.0f || relative_position != 0.0f)
    {
        /* 收到有效非零数据 → 上位机在线 */
        host_status = ONLINE;
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
