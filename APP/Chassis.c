#include "Chassis.h"
#include "trace.h"
#include "dcmotor.h"
#include "Robot_Cmd.h"
#include "JY901S.h"
#include "FreeRTOS.h"
#include "task.h"
#include "PAW3395.h"
#include "JY901S.h"
#include "math.h"
#include "misc.h"
#include "dwt.h"
#include "trace.h"
#include "misc.h"
#include "IMU.h"
chassis_cmd_q chassis_cmd_receive={0};
DCMotorInstance *motor_l,*motor_r;
Chassis_Move_State_e Chassis_Move_State;
float trace_dt;
float trace_starttime;
uint8_t stop_count,spin_count;
uint8_t Spin_start_flag = 0, Spin_succeed_flag = 0 , Stop_Flag = 0;
uint8_t Line_flag = 0, Turn_flag = 0 ;
uint8_t state = 0;
float ICM42688_Data[3] = {0};
void Stop_Detect(void);
void Chassis_State_Turn(void);
void Chassis_Set_Turn(void);

/**
 * @brief 初始化底盘左右电机和IMU
 */
void Chassis_Init(void)
{
	DCMotorInitConfig_s motor_l_config ={
		.Input_Dir = MOTOR_NORMAL,
		.Output_Dir = MOTOR_REVERSAL,
		.PortPin ={
			.EN_1_PORT = Motor_dir_EN1_B_PORT,
			.EN_1_pin = Motor_dir_EN1_B_PIN,
			.EN_2_PORT = Motor_dir_EN2_B_PORT,
			.EN_2_pin = Motor_dir_EN2_B_PIN,
			.inst = Motor_INST,
			.idx = GPIO_Motor_C1_IDX,
		},
		.encoder_config = {
			.A_PORT = ENCODER_PORT,
			.A_pin = ENCODER_ENC_A2_PIN,
			.B_PORT = ENCODER_PORT,
			.B_pin = ENCODER_ENC_B2_PIN,
		},
		.speed_pid_config = {
			.mode = PID_POSITION,
			.Kp = 15000.0f,
			.Ki = 0.0f,
			.Kd = 50000.0f,
			.max_out = 2499.0f,
			.max_iout = 500.0f, 
		},
		.position_pid_config= {
			.mode = PID_POSITION,
			.Kp = 4.0f,
			.Kd = 0.0f,
			.Ki = 0.0f,
			.max_out = 2.0f,
			.max_iout = 0.0f, 
		},
		.feedforward = 85,
	};
	size_t free_heap = xPortGetFreeHeapSize();
	motor_l = DCMotor_Init(&motor_l_config);
	free_heap = xPortGetFreeHeapSize();
	DCMotorInitConfig_s motor_r_config ={
		.Input_Dir = MOTOR_REVERSAL,
		.Output_Dir = MOTOR_REVERSAL,
		.PortPin ={
			.EN_1_PORT = Motor_dir_EN1_A_PORT,
			.EN_1_pin = Motor_dir_EN1_A_PIN,
			.EN_2_PORT = Motor_dir_EN2_A_PORT,
			.EN_2_pin = Motor_dir_EN2_A_PIN,
			.inst = Motor_INST,
			.idx = GPIO_Motor_C0_IDX,
		},
		.encoder_config = {
			.A_PORT = ENCODER_PORT,
			.A_pin = ENCODER_ENC_A1_PIN,
			.B_PORT = ENCODER_PORT,
			.B_pin = ENCODER_ENC_B1_PIN,
		},
		.speed_pid_config = {
			.mode = PID_POSITION,
			.Kp = 17000.0f,
			.Ki = 0.0f,
			.Kd = 200000.0f,
			.max_out = 2499.0f,
			.max_iout = 500.0f, 
		},
		.position_pid_config= {
			.mode = PID_POSITION,
			.Kp = 4.0f,
			.Kd = 0.0f,
			.Ki = 0.0f,
			.max_out = 2.0f,
			.max_iout = 0.0f, 
		},
		.feedforward = 85,
	};
	motor_r = DCMotor_Init(&motor_r_config);
	//PAW3395_Init();
	IMU_init();
	DWT_Delay(1);
}

static volatile uint8_t left_disable_flag = 0;
static volatile uint8_t right_disable_flag = 0;
/**
 * @brief 底盘主要任务，根据菜单不同模式执行对应任务，目前以200Hz运行
 */
void Chassis(void)
{
	motor_l->State = left_disable_flag;
	motor_r->State = right_disable_flag;
	static uint8_t debug=0;
	xQueueReceive(chassis_cmd_queue, &chassis_cmd_receive, 1);
	//PAW3395_Read_Motion(Chassis_axis);
	float trace_compensation;
	switch(chassis_cmd_receive.Chassis_Mode)
	{
		case TRACE_MODE:
			if(Spin_start_flag && Spin_succeed_flag == 0)
			{
				trace_compensation = 0;
			}
			Chassis_State_Turn();
			trace_starttime = DWT_GetTimeline_ms();
			trace_compensation=Trace_task();
			trace_dt = DWT_GetTimeline_ms() - trace_starttime;//调试用，计算巡线任务耗时

			//DCMotor_SetTraceCompensation(motor_l,-trace_compensation);
			//DCMotor_SetTraceCompensation(motor_r,trace_compensation);
			Stop_Detect();
			break;
		case IMU_MODE:
			IMU_getYawPitchRoll((float *)ICM42688_Data);
			break;
		case NORMAL_MODE:
				if(debug == 0)
				{
					Chassis_Set_Turn();
					debug++;
				}
			break;
		case POSITION_MODE:
				break;
		default:
			break;
	}
	
	
}

/**
 * @brief 被cmd调用，根据菜单值回调选择是否使能电机
 */
void Motor_Cmd_CallBack(uint8_t i)
{
	if(i ==0)
	{
		DCMotor_Cmd(motor_l,ENABLE);
		DCMotor_Cmd(motor_r,ENABLE);
	}
	else if(i == 1)
	{
		DCMotor_Cmd(motor_l,DISABLE);
		DCMotor_Cmd(motor_r,DISABLE);
	}
	
}

void Stop_Detect(void)
{
	if(Line_flag)
		{
				if((abs_out(motor_l->position_ref-motor_l->position_measure)<0.008f)&&(abs_out(motor_r->position_ref-motor_r->position_measure)<0.008f))
				{
						stop_count++;
						if(stop_count >= 40)
						{
								Line_flag = 0;
								Stop_Flag = 1; //这个标志位可以用来判断是否执行下一阶段任务
								stop_count = 0;
								// motor_l->State = DISABLE;
								// motor_r->State = DISABLE;
//								ctrl_mode = MOTOR_CTRL_STOP;
						}
				}
				else
				{
						Stop_Flag = 0;
						stop_count = 0;
				}
		}
		if(Spin_start_flag)
		{
			spin_count++;
			if(spin_count >= 200 &&(abs_out(motor_l->position_ref-motor_l->position_measure)<0.008f)&&(abs_out(motor_r->position_ref-motor_r->position_measure)<0.008f))
			{
					Spin_start_flag = 0;
					spin_count = 0;
					Spin_succeed_flag = 1;
					// motor_l->State = DISABLE;
					// motor_r->State = DISABLE;
			}
		}
	
}

void Chassis_Set_Turn(void)
{
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;
	Line_flag = 0;  //不进行巡线的补偿了
	Stop_Flag = 0;   //执行转弯时，将直走完成的标志位清零. 即如果上一次是直行，
	Spin_start_flag = 1;
	Spin_succeed_flag = 0;
	motor_l->encoder->total_count = 0;
	motor_r->encoder->total_count = 0;
	motor_l->position_pid.max_out = 0.08;
	motor_l->position_pid.max_iout = 0.05;
	motor_r->position_pid.max_out = 0.08;
	motor_r->position_pid.max_iout = 0.05;

	motor_l->position_ref = -0.01;
	motor_r->position_ref = 0.01;
	
}

void Chassis_Set_Line(float position)
{
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;
	Line_flag = 1;
	Stop_Flag = 0;
	Spin_start_flag = 0;
	Spin_succeed_flag = 0;
	motor_l->encoder->total_count = 0;
	motor_r->encoder->total_count = 0;
	motor_l->position_pid.max_out = 0.03;
	motor_l->position_pid.max_iout = 0.0;
	motor_r->position_pid.max_out = 0.03;
	motor_r->position_pid.max_iout = 0.0;

	motor_l->position_ref = position;
	motor_r->position_ref = position;
}
static volatile uint16_t tar_count = 0;
static volatile float temp_tar = 0.2f;
void Chassis_State_Turn(void)
{
	tar_count++;
	motor_l->position_ref = temp_tar * sin(tar_count / 100.0f);
	motor_r->position_ref = temp_tar * sin(tar_count / 100.0f);


	// static uint8_t quan=0;
	// switch(state)
	// {
	// 	case 0:
	// 		if(quan < chassis_cmd_receive.circle_set)
	// 		{
	// 			state ++;
	// 			Chassis_Set_Line(0.08);
	// 		}
	// 	break;
	// 	case 1:
	// 		if(Stop_Flag)
	// 		{
	// 			Chassis_Set_Turn();
	// 			state ++;
	// 		}
	// 	break;
	// 	case 2:
	// 		if(Spin_succeed_flag)
	// 		{
	// 			Chassis_Set_Line(0.082);
	// 			state ++;
	// 		}
	// 	break;
	// 	case 3:
	// 		if(Stop_Flag)
	// 		{
	// 			Chassis_Set_Turn();
	// 			state ++;
	// 		}
	// 	break;
	// 	case 4:
	// 		if(Spin_succeed_flag)
	// 		{
	// 			Chassis_Set_Line(0.082);
	// 			state ++;
	// 		}
	// 	break;
	// 		case 5:
	// 		if(Stop_Flag)
	// 		{
	// 			Chassis_Set_Turn();
	// 			state ++;
	// 		}
	// 	break;
	// 		case 6:
	// 		if(Spin_succeed_flag)
	// 		{
	// 			Chassis_Set_Line(0.082);
	// 			state ++;
	// 		}
	// 	break;
	// 		case 7:
	// 		if(Stop_Flag)
	// 		{
	// 			Chassis_Set_Turn();
	// 			state ++;
	// 		}
	// 	break;
	// 	case 8:
	// 		if(Spin_succeed_flag)
	// 		{
	// 			Chassis_Set_Line(0.01);
	// 			state ++;
	// 		}
	// 	break;
	// 	case 9:
	// 		if(Stop_Flag)
	// 		{
	// 			quan ++;
	// 			state = 0;
	// 		}
	// 	break;
	// 	default:
	// 		break;
	// }
	//
	
	
	
}