#ifndef _TRACE_H_
#define _TRACE_H_

#include "ti_msp_dl_config.h"
#include "PID.h"

float Trace_task(void);
void Trace_Init(void);
void Trace_ResetLineError(void);
float raw_transform_easy(uint8_t current_trace);
float second_process(float raw_val);
uint8_t filter_raw(uint8_t raw);

/**
 * @brief 获取当前灰度传感器算出的线位置原始误差
 * @return 线位置误差，范围约 -7.0 ~ +7.0，0 表示居中，正值表示线偏右
 * @note 不含 PID 计算，用于弯道检测等上层逻辑
 */
float Trace_GetError(void);

/**
 * @brief 基于质心法的连续线位置计算（无状态残留版本）
 *
 * 与 raw_transform_easy 的区别：
 *  - 质心法输出连续值（如 -5.3, +3.1），而非离散整数（如 -4, +2）
 *  - 全白(0xFF)时直接返回 0.0f，不保持上一帧值
 *  - 内置一阶低通滤波平滑输出
 *
 * @param raw 8 路传感器数字量（bit=0 表示黑线）
 * @return 线位置误差，范围约 -7.0 ~ +7.0
 */
float Trace_CalcErrorContinuous(uint8_t raw);


typedef enum {
	TRACE_END = 0,
	TRACE_INLINE,
	TRACE_LOST,
}trace_state_e;

typedef enum {
	TRACE_NORMAL = 0,
	TRACE_LOST_DETECT ,
}trace_mode_e;


typedef struct {
		float pid_output;
}trace_fetch_data_q;

extern trace_state_e trace_state;
extern trace_mode_e trace_mode;

/**
 * @brief 完全复位巡线状态（含 CalcLineError 残留值和 Trace_PID）
 * @note 模式切换时调用，避免旧状态的误差和积分影响新模式
 */
void Trace_Reset(void);




#endif
