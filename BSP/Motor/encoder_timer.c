#include "encoder_timer.h"
#include "encoder.h"
#include "dwt.h"

void EncoderTimer_Init(void)
{
	NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
}

void TIMER_TICK_INST_IRQHandler(void)
{
	
	if( DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) == DL_TIMER_IIDX_ZERO )
	{
		
		
		Encoder_Update();

	}
}