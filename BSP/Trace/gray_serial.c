/**
 ******************************************************************************
 * @file    gray_serial.c
 * @brief   八路灰度模块 GPIO 模拟串口驱动实现
 * @note    移植自辅助板 MSPM0G3507 串行例程的 gw_gray_serial_read()
 *          引脚适配：DAT = PA26, CLK = PA27
 *          使用 DWT 周期计数器实现 5μs 延时
 ******************************************************************************
 */

#include "gray_serial.h"
#include "dwt.h"

/**
 * @brief  初始化灰度传感器串口接口
 * @note   GPIO 方向和功能已由 SYSCFG_DL_GPIO_init() 配置，
 *         此处仅确保 CLK 初始为低电平
 */
void Gray_Serial_Init(void)
{
    DL_GPIO_clearPins(Gray_Serial_PORT, Gray_Serial_CLK_PIN);
}

/**
 * @brief  通过 GPIO 模拟串口读取八路灰度数据
 * @note   协议：CLK 上升沿触发传感器更新数据，下降沿后 MCU 读取 DAT
 *         每 bit 约 5μs 高电平，LSB first（bit 0 = 通道 0）
 * @retval uint8_t 八路灰度二值数据（0 = 黑线, 1 = 白色）
 */
uint8_t Gray_Serial_Read(void)
{
    uint8_t ret = 0;
    uint8_t i;

    DL_GPIO_clearPins(Gray_Serial_PORT, Gray_Serial_CLK_PIN);
    for (i = 0; i < 8; ++i) {
        /* CLK 上升沿，让传感器更新当前通道数据 */
        DL_GPIO_setPins(Gray_Serial_PORT, Gray_Serial_CLK_PIN);
        /* 延时约 5μs，满足传感器数据建立时间 */
        DWT_Delay(0.000005f);
        /* CLK 下降沿，读取 DAT 引脚状态 */
        DL_GPIO_clearPins(Gray_Serial_PORT, Gray_Serial_CLK_PIN);
        /* 读取 DAT 并移位拼合（0 = 黑, 1 = 白） */
        ret |= (DL_GPIO_readPins(Gray_Serial_PORT, Gray_Serial_DAT_PIN) == 0 ? 0 : 1) << i;
    }

    return ret;
}

bool Gray_Is_Line(uint8_t trace)
{
    uint8_t black = ~trace;

    uint8_t count = 0;


    for(int i=0;i<8;i++)
    {
        if(black&(1<<i))
        {
            count++;
        }
    }


    return count>=6;
}

/**
 * @brief  检测停止线 — 全部 8 路灰度同时检测到黑线
 * @param  trace  灰度原始值 (0=黑, 1=白)
 * @return true=检测到停止线, false=未检测到
 * @note   停止线通常是一条横跨整个赛道的粗黑线,
 *         需要全部 8 个通道都输出 0 (黑) 才判定有效。
 *         threshold 可设为 7 或 8, 根据实际停止线宽度调整。
 */
bool Gray_Is_StopLine(uint8_t trace)
{
    uint8_t black = ~trace;  /* 按位取反: 黑(0) → 1, 白(1) → 0 */

    uint8_t count = 0;
    const uint8_t threshold = 3;  /* 3路见黑才判定停止线 */

    for (int i = 0; i < 8; i++) {
        if (black & (1 << i)) {
            count++;
        }
    }

    return count >= threshold;
}

/**
 * @brief  检测是否所有通道均为黑色 (用于停止线确定)
 */
bool Gray_Is_All_Black(uint8_t trace)
{
    return trace == 0x00;  /* 全部 8 位为 0 = 全黑 */
}
