/**
 * @file    TJC.c
 * @brief   TJC 串口屏驱动实现
 * @note    协议: ASCII 文本，\r\n 帧尾。详见 TJC_PROTOCOL.md
 */

#include "TJC.h"
#include "../Flash/flash_param_store.h"
#include "Chassis.h"
#include "trace.h"
#include "bsp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ===== 常量 ===== */
#define TJC_RX_BUF_SIZE      64U    /* 行缓冲最大字节数 */
#define TJC_FRAME_TAIL       '\n'   /* 帧尾字符 */

/* ===== 命令首字符 ===== */
#define TJC_CMD_SET          'S'    /* 设置参数 */
#define TJC_CMD_WRITE        'W'    /* 保存至 Flash */
#define TJC_CMD_RESTORE      'R'    /* 恢复默认 */

/* ===== S 命令字段数 ===== */
#define TJC_S_FIELD_COUNT    3      /* S + page + comp + value → sscanf 赋值 3 个变量 */

/* ===== 页面 ID ===== */
#define TJC_PAGE_TRACE_PID   1U     /* 循迹 PID */
#define TJC_PAGE_LENGTH      2U     /* 设定长度 */
#define TJC_PAGE_LINE_PID    4U     /* 直线 PID */
#define TJC_PAGE_TURN_PID    5U     /* 转向 PID */

/* ===== 静态变量 ===== */
static uint8_t  tjc_rx_buf[TJC_RX_BUF_SIZE];
static uint8_t  tjc_rx_idx;
static volatile uint8_t tjc_frame_ready;    /* ISR 置 1，Process 消费后清零 */
static FlashParam_Data_s tjc_working_params; /* RAM 工作副本，累积所有 S 修改，W 时写 Flash */

/* ===== 内部函数声明 ===== */
static void TJC_ResetRx(void);
static void TJC_ClearFifo(void);
static void TJC_Dispatch(const char *line);

/**
 * @brief 初始化 TJC 驱动
 */
void TJC_Init(void)
{
    TJC_ResetRx();
    TJC_ClearFifo();

    /* 使能 UART_2 RX 中断（硬件已由 SYSCFG_DL_init 初始化） */
    DL_UART_Main_enableInterrupt(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);

    DL_UART_clearInterruptStatus(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);

    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    /* 初始化 RAM 工作副本（启动时从 Flash 加载，或使用默认值） */
    tjc_working_params = *FlashParam_GetActive();

    LOGINFO("[tjc] init OK");
}

/**
 * @brief TJC 主循环处理 (200Hz)
 */
void TJC_Process(void)
{
    if (tjc_frame_ready == 0U)
    {
        return;
    }

    tjc_frame_ready = 0U;

    /* 确保字符串以 \0 结尾 */
    if (tjc_rx_idx < TJC_RX_BUF_SIZE)
    {
        tjc_rx_buf[tjc_rx_idx] = '\0';
    }
    else
    {
        tjc_rx_buf[TJC_RX_BUF_SIZE - 1] = '\0';
    }

    TJC_Dispatch((const char *)tjc_rx_buf);
    TJC_ResetRx();
}

/**
 * @brief ISR 逐字节喂入 (ISR 上下文)
 */
void TJC_FeedByte(uint8_t byte)
{
    /* 忽略 \r (TJC 用 printh 0D 0A 发送) 和 0x00 (TJC prints 自带的终止符) */
    if ((byte == '\r') || (byte == 0x00))
    {
        return;
    }

    if (byte == TJC_FRAME_TAIL)
    {
        if (tjc_rx_idx > 0U)
        {
            tjc_frame_ready = 1U;
        }
        return;
    }

    /* 溢出保护：丢弃当前帧 */
    if (tjc_rx_idx >= TJC_RX_BUF_SIZE)
    {
        TJC_ResetRx();
        return;
    }

    tjc_rx_buf[tjc_rx_idx++] = byte;
}

/* ===== 内部函数 ===== */

static void TJC_ResetRx(void)
{
    tjc_rx_idx = 0U;
    memset(tjc_rx_buf, 0, sizeof(tjc_rx_buf));
}

static void TJC_ClearFifo(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_2_INST))
    {
        (void)DL_UART_receiveData(UART_2_INST);
    }
}

/**
 * @brief 根据 page + comp 设置 pid_init_config_s 的对应字段
 * @param pid   目标 pid_init_config_s 指针
 * @param comp  控件 ID (0~5 对应 Kp/Ki/Kd/max_out/max_iout/deadzone)
 * @param value 新值
 */
static void TJC_SetPidField(pid_init_config_s *pid, uint8_t comp, float value)
{
    if (pid == NULL) return;

    switch (comp)
    {
    case 0: pid->Kp       = value; break;
    case 1: pid->Ki       = value; break;
    case 2: pid->Kd       = value; break;
    case 3: pid->max_out  = value; break;
    case 4: pid->max_iout = value; break;
    case 5: pid->deadzone = value; break;
    default: break;
    }
}

/**
 * @brief 处理 S 命令 — 修改参数值
 * @param line 完整的帧字符串 (不含 \r\n)
 */
static void TJC_HandleSet(const char *line)
{
    /* 手动解析 "S,<page>,<comp>,<value>" ，避免 newlib-nano 禁用 sscanf %f 的问题 */
    if ((line[0] != 'S') || (line[1] != ','))
    {
        LOGWARNING("[tjc] S cmd format err: %s", line);
        return;
    }

    const char *p = line + 2;  /* 跳过 "S," */

    char *end = NULL;
    unsigned long page_ul = strtoul(p, &end, 10);
    if ((end == p) || (*end != ','))
    {
        LOGWARNING("[tjc] S cmd parse fail: %s", line);
        return;
    }

    p = end + 1;  /* 跳过 "," */
    unsigned long comp_ul = strtoul(p, &end, 10);
    if ((end == p) || (*end != ','))
    {
        LOGWARNING("[tjc] S cmd parse fail: %s", line);
        return;
    }

    p = end + 1;  /* 跳过 "," */
    float value = strtof(p, &end);
    if (end == p)
    {
        LOGWARNING("[tjc] S cmd parse fail: %s", line);
        return;
    }

    unsigned int page = (unsigned int)page_ul;
    unsigned int comp = (unsigned int)comp_ul;

    /* 在工作副本上直接修改（累积所有 S 命令的改动） */
    FlashParam_Data_s *params = &tjc_working_params;

    switch (page)
    {
    case TJC_PAGE_TRACE_PID:
        /* 循迹 PID */
        TJC_SetPidField(&params->trace_pid, (uint8_t)comp, value);
        LOGINFO("[tjc] trace pid comp=%u val=%.6f", (unsigned)comp, (double)value);
        break;

    case TJC_PAGE_LENGTH:
        /* 设定长度：comp 1→idx0, 3→idx1, 5→idx2, 7→idx3 */
        {
            uint8_t idx = 0xFFU;
            switch (comp)
            {
            case 1U: idx = 0U; break;
            case 3U: idx = 1U; break;
            case 5U: idx = 2U; break;
            case 7U: idx = 3U; break;
            default: break;
            }
            if (idx < FLASH_PARAM_LINE_SEGMENT_COUNT)
            {
                params->line_distance_m[idx] = value;
                LOGINFO("[tjc] line_distance[%u]=%.3f", idx, (double)value);
            }
            else
            {
                LOGWARNING("[tjc] length bad comp=%u", (unsigned)comp);
                return;
            }
        }
        break;

    case TJC_PAGE_LINE_PID:
        /* 直线 PID */
        TJC_SetPidField(&params->line_yaw_pid, (uint8_t)comp, value);
        LOGINFO("[tjc] line yaw pid comp=%u val=%.6f", (unsigned)comp, (double)value);
        break;

    case TJC_PAGE_TURN_PID:
        /* 转向 PID */
        TJC_SetPidField(&params->turn_pid, (uint8_t)comp, value);
        LOGINFO("[tjc] turn pid comp=%u val=%.6f", (unsigned)comp, (double)value);
        break;

    default:
        LOGWARNING("[tjc] S cmd unknown page=%u", (unsigned)page);
        return;
    }

    /* 立即应用到底盘和循迹（不写 Flash，等 W 命令才写） */
    Chassis_ApplyParams(params);
    Trace_ApplyParams(params);
}

/**
 * @brief 处理 W 命令 — 保存到 Flash
 */
static void TJC_HandleWrite(void)
{
    FlashParam_Status_e status;

    /* 将 RAM 工作副本写入 Flash */
    status = FlashParam_SaveAndApply(&tjc_working_params);
    if (status == FLASH_PARAM_OK)
    {
        LOGINFO("[tjc] save to flash OK, seq=%u", (unsigned)FlashParam_GetSequence());
    }
    else
    {
        LOGERROR("[tjc] save to flash FAIL, err=%d", (int)status);
    }
}

/**
 * @brief 处理 R 命令 — 恢复默认
 */
static void TJC_HandleRestore(void)
{
    FlashParam_Data_s defaults;
    FlashParam_GetDefault(&defaults);

    FlashParam_Status_e status = FlashParam_SaveAndApply(&defaults);
    if (status == FLASH_PARAM_OK)
    {
        /* 同步 RAM 工作副本 */
        tjc_working_params = defaults;
        LOGINFO("[tjc] restore defaults OK, seq=%u", (unsigned)FlashParam_GetSequence());
    }
    else
    {
        LOGERROR("[tjc] restore defaults FAIL, err=%d", (int)status);
    }
}

/**
 * @brief 帧分发
 */
static void TJC_Dispatch(const char *line)
{
    if ((line == NULL) || (line[0] == '\0'))
    {
        return;
    }

    switch (line[0])
    {
    case TJC_CMD_SET:
        TJC_HandleSet(line);
        break;

    case TJC_CMD_WRITE:
        TJC_HandleWrite();
        break;

    case TJC_CMD_RESTORE:
        TJC_HandleRestore();
        break;

    default:
        LOGWARNING("[tjc] unknown cmd: %s", line);
        break;
    }
}
