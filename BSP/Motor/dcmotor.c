#include "dcmotor.h"
#include "misc.h"
#include "stdlib.h"
#include "string.h"
#include "dwt.h"
#include "motor_def.h"
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
	instance->encoder = Encoder_Init(&config->encoder_config);
	instance->PortPin = config->PortPin;
	instance->Input_Dir = config->Input_Dir;
	instance->Output_Dir = config->Output_Dir;
	instance->feedforward = config->feedforward;
	instance->loop_mode = config->loop_mode;
	/* 基础速度命令单独初始化，后续不与PID内部Ref共用存储。 */
	instance->Speed_Ref_Command = 0.0f;
	instance->Trace_Compensation = 0.0f;
	DCMotor_Cmd(instance,DISABLE);
	
	return instance;
}
static void set_motor(DCMotorInstance *motor)
{
	int16_t speed = (int16_t)(motor->speed_pid.out*motor->State);
	if(motor->State == DISABLE)
	{
		DL_GPIO_clearPins(motor->PortPin.EN_1_PORT,motor->PortPin.EN_1_pin);
		DL_GPIO_clearPins(motor->PortPin.EN_2_PORT,motor->PortPin.EN_2_pin);
	}
	else
	{
		if(speed*(motor->Output_Dir == MOTOR_REVERSAL? 1:-1)<0)
		{                                                                                                      
			DL_GPIO_setPins(motor->PortPin.EN_1_PORT,motor->PortPin.EN_1_pin);
			DL_GPIO_clearPins(motor->PortPin.EN_2_PORT,motor->PortPin.EN_2_pin);
		}                                                                                                      
		else                                                                                               
		{                                                                                                      
			DL_GPIO_setPins(motor->PortPin.EN_2_PORT,motor->PortPin.EN_2_pin);
			DL_GPIO_clearPins(motor->PortPin.EN_1_PORT,motor->PortPin.EN_1_pin);
		}
	}


	Abs(speed);
	DL_TimerG_setCaptureCompareValue(motor->PortPin.inst,speed,motor->PortPin.idx);
		
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

void DC_Motor_SetRef(DCMotorInstance * motor,float ref) {
	if (motor->loop_mode == SPEED_MODE) {
		/*
		 * 保存规划器基础轮速，不直接写PID内部Ref。PID_calc自身会写Ref，
		 * 两个任务共用该字段会造成基础值与补偿后数值周期性交替。
		 */
		if (fabsf(ref - motor->Speed_Ref_Command) > 1e-6f) {
			motor->Speed_Ref_Command = ref;
			motor->speed_pid.pid_update_flag = 1;
		}
	}
	if (motor->loop_mode == ANGLE_MODE) {
		motor->position_pid.Ref = ref;
		motor->position_pid.pid_update_flag = 1;
	}
}
void Hw_Motor_Task(void)
{
	static float last_control_s=0;
	static float now_control_s=0;
	static float control_period=0;
	//Encoder_Update();
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
		if (dcmotor_instance[i].loop_mode == ANGLE_MODE) {
			//计算角度环输出，角度环out和巡线补偿作为速度环ref
			PID_calc(&dcmotor_instance[i].position_pid,dcmotor_instance[i].position_pid.Ref,dcmotor_instance[i].position_measure);
			//计算速度环输出
			PID_calc(&dcmotor_instance[i].speed_pid,dcmotor_instance[i].position_pid.out+dcmotor_instance[i].Trace_Compensation,dcmotor_instance[i].filter.speed_filtered);
		}
		if (dcmotor_instance[i].loop_mode == SPEED_MODE) {
			//计算速度环输出
			/*
			 * 实际PID目标只在电机任务中合成一次。speed_pid.Ref供PID内部记录，
			 * Speed_Ref_Command始终保留上层给出的基础轮速。
			 */
			float speed_target = dcmotor_instance[i].Speed_Ref_Command +
			                     dcmotor_instance[i].Trace_Compensation;
			PID_calc(&dcmotor_instance[i].speed_pid, speed_target,
			         dcmotor_instance[i].filter.speed_filtered);
		}
		//前馈
		if (fabsf(dcmotor_instance[i].speed_pid.Ref) > 0.005f) //在有目标值的时候进行累加，目标值为0停下的时候不给前馈
		dcmotor_instance[i].speed_pid.out += (float)dcmotor_instance[i].feedforward*((dcmotor_instance[i].speed_pid.Ref > 0 )? 1.0f : -1.0f);

		if(dcmotor_instance[i].State == DISABLE)
		{
			PID_clear(&dcmotor_instance[i].speed_pid);
		}
		LIMIT_MIN_MAX(dcmotor_instance[i].speed_pid.out,-2499,2499);
		//赋值
		set_motor(&dcmotor_instance[i]);
	}
		
}
