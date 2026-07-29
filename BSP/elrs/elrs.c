/**
 * @file    elrs.c
 * @brief   ELRS UART3 已改为 ZDT 电机通信 (115200 bps)
 *
 * 原 ELRS CRSF 协议已移除。UART3 现专用于 ZDT 步进电机串口通讯。
 * ISR 将接收字节喂入 ZDT_Emm RX FIFO，应用层通过 ZDT_Emm_GetResponse() 读取。
 *
 * ISR 实现参照 ZDT 原版例程的 usart.c (UART_0_INST_IRQHandler)
 */

#include "elrs.h"
#include "ZDT_Emm.h"
#include "ti_msp_dl_config.h"

/* ============================================================
 *  UART3 RX 中断服务函数
 *
 *  参照原版 ZDT 例程:
 *    if (DL_UART_getPendingInterrupt(inst) == DL_UART_IIDX_RX) {
 *        fifo_enQueue(DL_UART_Main_receiveData(inst));
 *    }
 *    DL_UART_clearInterruptStatus(inst, DL_UART_IIDX_RX);
 * ============================================================ */

void ELRS_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(ELRS_INST) == DL_UART_IIDX_RX) {
        /* 接收一个字节，推入 ZDT FIFO */
        ZDT_Emm_RxPushByte((uint8_t)DL_UART_Main_receiveData(ELRS_INST));
    }

    /* 清除接收中断标志 */
    DL_UART_clearInterruptStatus(ELRS_INST, DL_UART_IIDX_RX);
}

/* ============================================================
 *  兼容性桩函数 (保持 Robot_Cmd 等引用不报链接错误)
 * ============================================================ */

static ELRS_Data_s dummy_elrs_data;

void ELRS_Init(void)
{
    /* ELRS 已弃用，UART3 硬件由 SYSCFG_DL_ELRS_init() 初始化 */
    /* NVIC 中断由 Gimbal_Init() 使能 */
}

const volatile ELRS_Data_s *ELRS_GetData(void)
{
    return &dummy_elrs_data;
}

uint16_t ELRS_GetChannel(uint8_t index)
{
    (void)index;
    return 992U;
}

uint8_t ELRS_IsOnline(void)
{
    return 0U;
}

uint8_t ELRS_IsFrameUpdated(void)
{
    return 0U;
}

void ELRS_ClearFrameUpdated(void)
{
}

uint32_t ELRS_GetUartErrorCount(void)
{
    return 0U;
}
