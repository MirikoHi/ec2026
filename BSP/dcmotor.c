#include "dcmotor.h"
#include "misc.h"
#include "stdlib.h"
#include "string.h"
#include "dwt.h"
DCMotorInstance dcmotor_instance[MOTOR_MAX_NUM] = {0};
uint8_t idx_dcmotor = 0;
static float DCMotor_Speed_Filter(Motor_Speed_Filter_e *filter,float speed);

uint16_t iiii;
DCMotorInstance* DCMotor_Init(DCMotorInitConfig_s *config)
{

	DCMotorInstance* instance=&dcmotor_instance[idx_dcmotor++];
	iiii=sizeof(DCMotor_PortPin_s);
	PID_init(&instance->speed_pid,&config->speed_pid_config);
	PID_init(&instance->position_pid,&config->position_pid_config);
	instance->encoder = encoder_init(&config->encoder_config);
	instance->PortPin = config->PortPin;
	instance->Input_Dir = config->Input_Dir;
	instance->Output_Dir = config->Output_Dir;
	instance->feedforward = config->feedforward;
	DCMotor_Cmd(instance,DISABLE);
	
	return instance;
}
static void set_motor(DCMotorInstance *motor)
{
	int16_t speed = (int16_t)(motor->speed_pid.out*motor->State);
	if(motor->State == DISABLE)
	{
		DL_GPIO_clearPins(motor->PortPin.EN_A_PORT,motor->PortPin.EN_A_pin);                                                   
		DL_GPIO_clearPins(motor->PortPin.EN_B_PORT,motor->PortPin.EN_B_pin);       
	}
	else
	{
		if(speed*(motor->Output_Dir == MOTOR_NORMAL? 1:-1)<0)                                                                                				  
		{                                                                                                      
			DL_GPIO_setPins(motor->PortPin.EN_A_PORT,motor->PortPin.EN_A_pin);                                                   
			DL_GPIO_clearPins(motor->PortPin.EN_B_PORT,motor->PortPin.EN_B_pin);                                                 
		}                                                                                                      
		else                                                                                               
		{                                                                                                      
			DL_GPIO_setPins(motor->PortPin.EN_B_PORT,motor->PortPin.EN_B_pin);                                                   
			DL_GPIO_clearPins(motor->PortPin.EN_A_PORT,motor->PortPin.EN_A_pin);                                                 
		}
	}
	    
	
	Abs(speed);
	DL_TimerG_setCaptureCompareValue(motor->PortPin.inst,speed,motor->PortPin.idx);
		
}

void DCMotor_SetPosition(DCMotorInstance *motor,float Position)
{
	motor->position_ref = Position;
}
void DCMotor_Cmd(DCMotorInstance* motor,State state)
{
		motor->State=state;
}
static float DCMotor_Speed_Filter(Motor_Speed_Filter_e *filter,float speed)
{
	float filtered_MotorSpeed = 0;
	filter->MotorSpeedBuf[filter->filter_index] = speed;
	filter->filter_index = (filter->filter_index + 1) % FILTER_NUM;
  if(filter->filter_count < FILTER_NUM) filter->filter_count++;
  for(uint8_t i = 0; i < filter->filter_count; i++) {
       filtered_MotorSpeed += filter->MotorSpeedBuf[i];
  }
  filtered_MotorSpeed /=filter->filter_count;
	filter->speed_filtered = filtered_MotorSpeed;
	return filtered_MotorSpeed;
}
void DCMotor_SetTraceCompensation(DCMotorInstance *motor,float compensation)
{
	motor->Trace_Compensation = compensation;
}
void Hw_Motor_Task(void)
{
	static float last_control_s=0;
	static float now_control_s=0;
	static float control_period=0;
	//encoder_update();
	now_control_s = DWT_GetTimeline_s();
	control_period = now_control_s-last_control_s;
	last_control_s = now_control_s;
	for(uint8_t i=0;i<idx_dcmotor;i++)
	{
		//计算位置
		dcmotor_instance[i].position_measure=(dcmotor_instance[i].Input_Dir==MOTOR_REVERSAL? -1:1)*(dcmotor_instance[i].encoder->total_count*ENCODER_TO_DISDAN_M);
		//计算速度
		dcmotor_instance[i].speed_measure=(dcmotor_instance[i].Input_Dir==MOTOR_REVERSAL? -1:1)*(dcmotor_instance[i].encoder->count*ENCODER_TO_SPEED_MS);
		//滤波
		DCMotor_Speed_Filter(&dcmotor_instance[i].filter,dcmotor_instance[i].speed_measure);
		PID_calc(&dcmotor_instance[i].position_pid,dcmotor_instance[i].position_ref,dcmotor_instance[i].position_measure);
		//pid计算
		PID_calc(&dcmotor_instance[i].speed_pid,dcmotor_instance[i].position_pid.out+dcmotor_instance[i].Trace_Compensation+dcmotor_instance[i].speed_ref,dcmotor_instance[i].filter.speed_filtered);
		//前馈
		dcmotor_instance[i].speed_pid.out += (float)dcmotor_instance[i].feedforward*(dcmotor_instance[i].encoder->dir==FORWARD?-1:1);
		if(dcmotor_instance[i].State == DISABLE)
		{
			PID_clear(&dcmotor_instance[i].speed_pid);
		}
		//赋值
		set_motor(&dcmotor_instance[i]);
	}
		
}

