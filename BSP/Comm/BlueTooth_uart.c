//
// Created by xiaofangxing on 2026/7/27.
//

#include "BlueTooth_uart.h"
#include "ti_msp_dl_config.h"
#include "daemon.h"
#include "../Algorithm/crc8.h"
#include <string.h>

// ===== 编译期缓冲区尺寸计算 =====
#define BT_TX_DATA_LEN    ((uint8_t)sizeof(BlueTooth_Tx_t))
#define BT_TX_BUF_LEN     (BT_TX_DATA_LEN + BLUETOOTH_UART_OFFSET_BYTES)
#define BT_RX_DATA_LEN    ((uint8_t)sizeof(BlueTooth_Rx_t))
#define BT_RX_BUF_LEN     (BT_RX_DATA_LEN + BLUETOOTH_UART_OFFSET_BYTES)

// ===== 编译期结构体大小校验 =====
_Static_assert(BT_TX_DATA_LEN <= BLUETOOTH_UART_MAX_BUFFSIZE, "BlueTooth TX struct too large");
_Static_assert(BT_RX_DATA_LEN <= BLUETOOTH_UART_MAX_BUFFSIZE, "BlueTooth RX struct too large");

// ===== 中断标志位掩码 =====
#define BT_UART_ERROR_INTERRUPTS   (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
                                    DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | \
                                    DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
                                    DL_UART_MAIN_INTERRUPT_BREAK_ERROR | \
                                    DL_UART_MAIN_INTERRUPT_NOISE_ERROR)
#define BT_UART_RX_INTERRUPTS      (DL_UART_MAIN_INTERRUPT_RX)
#define BT_UART_HANDLED_INTERRUPTS (BT_UART_RX_INTERRUPTS | BT_UART_ERROR_INTERRUPTS)

#define BT_DAEMON_RELOAD_COUNT     50U

// ===== RX 状态机状态 =====
#define BT_RX_STATE_IDLE        0U
#define BT_RX_STATE_RECEIVING   1U

// ===== 静态单例 =====
static uint8_t  bt_tx_raw_buf[BT_TX_BUF_LEN];
static uint8_t  bt_rx_raw_buf[BT_RX_BUF_LEN];
static BlueTooth_Rx_t bt_rx_data;
static uint8_t  bt_rx_state;
static uint8_t  bt_rx_idx;
static volatile uint8_t  bt_frame_updated;
static volatile uint32_t bt_uart_error_count;
static DaemonInstance *bt_daemon;
static uint8_t  bt_tx_byte_idx;

//内部函数
static void BlueToothUart_ResetRx(void);
static void BlueToothUart_ClearFifo(void);
static void BlueToothUart_FeedByte(uint8_t byte);
static void BlueToothUart_LostCallback(void *ptr);

/**
 * @brief 初始化凌翔科技模块蓝牙，实质是用串口传输，俩模块目前已自动连接
 */
void BlueToothUart_Init(void)
{
    BlueToothUart_ResetRx();
    BlueToothUart_ClearFifo();

    // 预填 TX 帧头尾（数据部分在 Send 时填入）
    bt_tx_raw_buf[0] = (uint8_t)BLUETOOTH_UART_HEADER;
    bt_tx_raw_buf[1] = BT_TX_DATA_LEN;
    bt_tx_raw_buf[BT_TX_BUF_LEN - 1] = (uint8_t)BLUETOOTH_UART_TAIL;

    // 使能 UART 外设中断
    DL_UART_Main_enableInterrupt(UART_2_INST, BT_UART_HANDLED_INTERRUPTS);
    DL_UART_clearInterruptStatus(UART_2_INST, BT_UART_HANDLED_INTERRUPTS);

    // 注册 daemon 看门狗
    Daemon_Init_Config_s daemon_config = {
        .callback = BlueToothUart_LostCallback,
        .owner_id = NULL,
        .reload_count = BT_DAEMON_RELOAD_COUNT,
    };
    bt_daemon = DaemonRegister(&daemon_config);

    // 使能 NVIC 中断
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void BlueToothUart_Send(const BlueTooth_Tx_t *data)
{
    // 每次新帧开始时拷贝数据
    if (bt_tx_byte_idx == 0U)
    {
        memcpy(&bt_tx_raw_buf[2], data, BT_TX_DATA_LEN);
        bt_tx_raw_buf[2 + BT_TX_DATA_LEN] = crc_8((const uint8_t *)data, BT_TX_DATA_LEN);
    }

    // 非阻塞：能塞多少塞多少，FIFO满了下次Task再继续
    while (bt_tx_byte_idx < BT_TX_BUF_LEN)
    {
        if (!DL_UART_Main_transmitDataCheck(UART_2_INST, bt_tx_raw_buf[bt_tx_byte_idx]))
        {
            return;  // FIFO 满，下一轮继续
        }
        bt_tx_byte_idx++;
    }

    bt_tx_byte_idx = 0U;  // 一帧发送完毕，下次调用开始新帧
}

/**
 * @brief  获取串口数据，并置零update标志位代表已取走
 * @return 数据缓冲区的指针
 */
volatile BlueTooth_Rx_t *BlueToothUart_Get(void)
{
    if (bt_frame_updated != 0U)
    {
        bt_frame_updated = 0U;
        return &bt_rx_data;
    }
    return NULL;
}

uint8_t BlueToothUart_IsOnline(void)
{
    return (bt_daemon != NULL) ? DaemonIsOnline(bt_daemon) : 0U;
}

uint8_t BlueToothUart_IsFrameUpdated(void)
{
    return bt_frame_updated;
}

void BlueToothUart_ClearFrameUpdated(void)
{
    bt_frame_updated = 0U;
}

uint32_t BlueToothUart_GetErrorCount(void)
{
    return bt_uart_error_count;
}

// ===== 内部函数 =====

static void BlueToothUart_ResetRx(void)
{
    memset(bt_rx_raw_buf, 0, bt_rx_idx);
    bt_rx_state = BT_RX_STATE_IDLE;
    bt_rx_idx   = 0U;
}

static void BlueToothUart_ClearFifo(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_2_INST))
    {
        (void)DL_UART_receiveData(UART_2_INST);
    }
}

static void BlueToothUart_FeedByte(uint8_t byte)
{
    if (bt_rx_state == BT_RX_STATE_IDLE)
    {
        // 等待帧头 's'
        if (byte == (uint8_t)BLUETOOTH_UART_HEADER)
        {
            bt_rx_raw_buf[0] = byte;
            bt_rx_state = BT_RX_STATE_RECEIVING;
            bt_rx_idx   = 1U;
        }
        // 否则丢弃
        return;
    }

    // STATE = RECEIVING
    bt_rx_raw_buf[bt_rx_idx] = byte;
    bt_rx_idx++;

    // 溢出保护
    if (bt_rx_idx > BT_RX_BUF_LEN)
    {
        BlueToothUart_ResetRx();
        return;
    }

    // 收满一帧，开始校验
    if (bt_rx_idx == BT_RX_BUF_LEN)
    {
        uint8_t valid = 1U;

        // 校验帧头 's'
        if (bt_rx_raw_buf[0] != (uint8_t)BLUETOOTH_UART_HEADER)
        {
            valid = 0U;
        }
        // 校验帧尾 'e'
        else if (bt_rx_raw_buf[BT_RX_BUF_LEN - 1] != (uint8_t)BLUETOOTH_UART_TAIL)
        {
            valid = 0U;
        }
        // 校验数据长度
        else if (bt_rx_raw_buf[1] != BT_RX_DATA_LEN)
        {
            valid = 0U;
        }
        // 校验 CRC8
        else
        {
            uint8_t rx_crc  = bt_rx_raw_buf[BT_RX_BUF_LEN - 2];
            uint8_t cal_crc = crc_8(&bt_rx_raw_buf[2], BT_RX_DATA_LEN);
            if (rx_crc != cal_crc)
            {
                valid = 0U;
            }
        }

        if (valid != 0U)
        {
            // 提取数据
            memcpy((void *)&bt_rx_data, &bt_rx_raw_buf[2], BT_RX_DATA_LEN);
            bt_frame_updated = 1U;
            if (bt_daemon != NULL)
            {
                DaemonReload(bt_daemon);
            }
        }

        BlueToothUart_ResetRx();
    }
}

static void BlueToothUart_LostCallback(void *ptr)
{
    (void)ptr;
    BlueToothUart_ResetRx();
    BlueToothUart_ClearFifo();
}

// ===== ISR 中断服务函数 =====

// void UART_7_INST_IRQHandler(void)
// {
//     uint32_t status = DL_UART_getEnabledInterruptStatus(UART_2_INST, BT_UART_HANDLED_INTERRUPTS);

//     if (status != 0U)
//     {
//         DL_UART_clearInterruptStatus(UART_2_INST, status);
//     }

//     // 错误中断：计数、复位状态、清 FIFO
//     if ((status & BT_UART_ERROR_INTERRUPTS) != 0U)
//     {
//         bt_uart_error_count++;
//         BlueToothUart_ResetRx();
//         BlueToothUart_ClearFifo();
//     }

//     // RX 中断：排空 FIFO
//     if ((status & BT_UART_RX_INTERRUPTS) != 0U)
//     {
//         while (!DL_UART_isRXFIFOEmpty(UART_2_INST))
//         {
//             uint8_t byte = DL_UART_receiveData(UART_2_INST);
//             BlueToothUart_FeedByte(byte);
//         }
//     }
// }
