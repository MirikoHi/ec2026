#include "Gimbal.h"
#include "ZDT_Motor.h"
#include "Robot_cmd.h"
#include "math.h"
#include "misc.h"
#include "PID.h"
#include "K230.h"
#include "daemon.h"
#include "dwt.h"

ZDT_Motor_t *yaw_motor,*pitch_motor;
float angle_debug;
gimbal_cmd_q gimbal_cmd_receive={0};

float relay_on_time;
uint8_t relay_first_on_flag=0;
void Gimbal_Attitude_Solving(void);

void Gimbal_task_2(void);
void Gimbal_Init(void)
{
	// ZDT_Motor_Config_s yaw_config = {
	// 	.PortPin={
	// 		.Stp_PORT = ZDT_Motor_PORT,
	//     .Stp_pin = ZDT_Motor_Yaw_Stp_PIN,
	//     .Dir_PORT = ZDT_Motor_PORT,
	//     .Dir_pin = ZDT_Motor_Yaw_Dir_PIN,
	// 	},
	// 	.delay_ms = 0.1f,
	// };
	// yaw_motor = ZDT_Motor_Init(&yaw_config);
	//
	//
	// ZDT_Motor_Config_s pitch_config = {
	// 	.PortPin={
	// 		.Stp_PORT = ZDT_Motor_PORT,
	//     .Stp_pin = ZDT_Motor_Pitch_Stp_PIN,
	//     .Dir_PORT = ZDT_Motor_PORT,
	//     .Dir_pin = ZDT_Motor_Pitch_Dir_PIN,
	// 	},
	// 	.delay_ms = 0.1f,
	// };
	// pitch_motor = ZDT_Motor_Init(&pitch_config);
	ZDT_TICK_Init();

	
	
}

void Gimbal(void)
{
	xQueueReceive(gimbal_cmd_queue, &gimbal_cmd_receive, 1);
//	
//	Gimbal_Attitude_Solving();
	ZDT_Set_Position(yaw_motor,gimbal_cmd_receive.yaw);
	ZDT_Set_Position(pitch_motor,gimbal_cmd_receive.pitch);
	switch(gimbal_cmd_receive.task_flag)
	{
		case 1:
			break;
		case 2:
			Gimbal_task_2();
			break;
		default:
			break;
	}
	
}


void Gimbal_Attitude_Solving(void)
{
	float aim_x = gimbal_cmd_receive.aim_x;
	float aim_y = gimbal_cmd_receive.aim_y;
	gimbal_cmd_receive.yaw = -atan2f(aim_x,GIMBAL_LENGTH_TO_CENTER)*180.0f/PI;
	gimbal_cmd_receive.pitch = atan2f(aim_y,sqrtf(GIMBAL_LENGTH_TO_CENTER*GIMBAL_LENGTH_TO_CENTER+aim_x*aim_x))*180.0f/PI;
}

void Gimbal_task_2(void)
{
	ZDT_Set_Position(yaw_motor,gimbal_cmd_receive.yaw);
	ZDT_Set_Position(pitch_motor,gimbal_cmd_receive.pitch);
	if((abs_out(K230_err[0])<=1)&&(abs_out(K230_err[1])<=1)&&DaemonIsOnline(K230_daemon)&&(!relay_first_on_flag))
	{
		gimbal_cmd_receive.relay_on_flag = 1;
		relay_first_on_flag = 1;
		relay_on_time = DWT_GetTimeline_ms();
	}
	if(DWT_GetTimeline_ms()-relay_on_time>200)
	{
		gimbal_cmd_receive.relay_on_flag = 0;
	}
	else if(relay_first_on_flag)
	{
		gimbal_cmd_receive.relay_on_flag = 1;
	}
	if(gimbal_cmd_receive.relay_on_flag)
	{
		DL_GPIO_setPins(RELAY_PORT,RELAY_Control_PIN);
	}
	else
	{
		DL_GPIO_clearPins(RELAY_PORT,RELAY_Control_PIN);
	}
}

