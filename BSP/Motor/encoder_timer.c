#include "encoder_timer.h"
#include "encoder.h"
#include "dwt.h"

/**
 * @brief 使能编码器定时器中断
 *
 * 编码器脉冲在 GPIO 中断里累计，定时器中断周期性调用 Encoder_Update()
 * 完成计数结算。
 */
void EncoderTimer_Init(void)
{
	NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
}

/**
 * @brief 编码器定时结算中断服务函数
 *
 * 在定时器到点时刷新编码器累计值，让速度和位置计算使用最新计数。
 */
void TIMER_TICK_INST_IRQHandler(void)
{
	
	if( DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) == DL_TIMER_IIDX_ZERO )
	{
		
		
		Encoder_Update();

	}
}