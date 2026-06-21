#include "ZDT_Motor.h"
#include "stdlib.h"
#include "string.h"
#include "dwt.h"
#include "misc.h"
ZDT_Motor_t* ZDT_Motor_Instance[ZDT_MOTOR_MAX_NUM]={0};
uint8_t idx = 0;
void ZDT_TICK_Init(void)
{
	NVIC_ClearPendingIRQ(ZDT_MOTOR_TICK_INST_INT_IRQN);
	NVIC_EnableIRQ(ZDT_MOTOR_TICK_INST_INT_IRQN);
}
ZDT_Motor_t *ZDT_Motor_Init(ZDT_Motor_Config_s* config)
{
	if(config == NULL)return NULL;
	ZDT_Motor_t *instance =(ZDT_Motor_t*)malloc(sizeof (ZDT_Motor_t));
	memset(instance,0,sizeof(ZDT_Motor_t));
	instance->PortPin = config->PortPin;
	instance->delay_ms = config->delay_ms;
//	instance->state = ENABLE;
	ZDT_Motor_Instance[idx++]=instance;
	return instance;
}
void ZDT_Motor(void)
{
	ZDT_Motor_t *motor;
	ZDT_Motor_PortPin_s *PortPin;
	 for ( uint8_t i = 0;  i< idx; ++i) 
	{
//		if(motor->state == DISABLE)
//		{
//			continue;
//		}
		motor =ZDT_Motor_Instance[i];
		PortPin = &motor->PortPin;
		//方向判断
		switch(motor->dir)
		{
			case 0:
				DL_GPIO_clearPins(PortPin->Dir_PORT,PortPin->Dir_pin);
				break;
			case 1:
				DL_GPIO_setPins(PortPin->Dir_PORT,PortPin->Dir_pin);
				break;
			default:
				break;
		}
		//施加脉冲
		if((motor->clk>0)&&((DWT_GetTimeline_ms()-motor->last_pulse_ms)>motor->delay_ms))
		{
			DL_GPIO_togglePins(PortPin->Stp_PORT, PortPin->Stp_pin);
      motor->clk--;
			motor->now_angle+=(motor->dir==0?1:-1)*360.0f/3200; 
      motor->last_pulse_ms = DWT_GetTimeline_ms(); // 更新时间戳
		}
	}
	
}
void ZDT_Set_Position(ZDT_Motor_t *motor,float angle)
{
	while(motor->clk!=0)
	{
		return;
	}
	float delta_angle = angle-motor->now_angle;
	motor->dir = (delta_angle)>0?0:1; 
	motor->clk = 3200*abs_out(delta_angle)/360.0f;
}
void ZDT_MOTOR_TICK_INST_IRQHandler(void)
{
	
	if( DL_Timer_getPendingInterrupt(ZDT_MOTOR_TICK_INST) == DL_TIMER_IIDX_ZERO )
	{
		
		
		ZDT_Motor();
	}
}

