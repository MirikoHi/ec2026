#include "Gimbal.h"
#include "ZDT_Motor.h"
#include "Robot_cmd.h"
#include "math.h"
#include "misc.h"
#include "PID.h"
#include "K230.h"
#include "daemon.h"
#include "dwt.h"
#include "Servo.h"

ZDT_Motor_t *yaw_motor,*pitch_motor;
float angle_debug;
gimbal_cmd_q gimbal_cmd_receive={0};

float relay_on_time;
uint8_t relay_first_on_flag=0;

pid_type_def gimbal_yaw_PID={0};
pid_type_def gimbal_pitch_PID={0};
pid_type_def gimbal_yaw_forwardfeed_PID = {0};
gimbal_cmd_q gimbal_cmd_send ={0};

steel_ball_movement_typedef steel_ball_movement_data;
ServoInstance*  servo_yaw;

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
	// pid_init_config_s gimbal_yaw_pid_config={
	// 	.mode = PID_POSITION,
	// 	.Kp = 0.003f,
	// 	.Kd = 0.0001f,
	// 	.Ki = 0.0f,
	// 	.max_out = 4.0f,
	// 	.max_iout = 1.0f,
	// };
	// PID_init(&gimbal_yaw_PID,&gimbal_yaw_pid_config);

	pid_init_config_s gimbal_pitch_pid_config={
		.mode = PID_POSITION,
		.Kp = -0.003f,
		.Kd = -0.0001f,
		.Ki = 0.0f,
		.max_out = 4.0f,
		.max_iout = 1.0f,
	};
	PID_init(&gimbal_pitch_PID,&gimbal_pitch_pid_config);

	pid_init_config_s gimbal_yaw_forwardfeed_pid_config={
		.mode = PID_POSITION,
		.Kp = 0.000f,
		.Kd = -0.0001f,
		.Ki = 0.0f,
		.max_out = 4.0f,
		.max_iout = 1.0f,
	};
	PID_init(&gimbal_yaw_forwardfeed_PID,&gimbal_yaw_forwardfeed_pid_config);

	Servo_Init_Config_s servo_yaw_config = {
		.Servo_type = Servo180,
		.inst = Servo_INST,
		.idx = 0,    //对应DL_TIMER_CC_0_INDEX，PA17
	};
	servo_yaw = ServoInit(&servo_yaw_config);
	Servo_Motor_Type_Select(servo_yaw, Free_Angle_mode);
}

void Gimbal(void)
{
	xQueueReceive(gimbal_cmd_queue, &gimbal_cmd_receive, 1);

	if (K230_Read(&steel_ball_movement_data)) {
		Servo_Motor_FreeAngle_Set(servo_yaw, (uint8_t)steel_ball_movement_data.x_position*180/640);
		ServeoMotorControl();
	}
	Gimbal_Task();
	// switch(gimbal_cmd_receive.task_flag)
	// {
	// 	case 0:
	//
	// 		break;
	// 	default:
	// 		break;
	// }
}

void Gimbal_Pid_Cal(void)
{
	// PID_calc(&gimbal_yaw_PID,0,K230_err[0]);
	// PID_calc(&gimbal_pitch_PID,0,K230_err[1]);
	// gimbal_cmd_send.yaw += gimbal_yaw_PID.out;
	gimbal_cmd_send.pitch +=gimbal_pitch_PID.out;
}
void Gimbal_Attitude_Solving(void)
{
	float aim_x = gimbal_cmd_receive.aim_x;
	float aim_y = gimbal_cmd_receive.aim_y;
	gimbal_cmd_receive.yaw = -atan2f(aim_x,GIMBAL_LENGTH_TO_CENTER)*180.0f/PI;
	gimbal_cmd_receive.pitch = atan2f(aim_y,sqrtf(GIMBAL_LENGTH_TO_CENTER*GIMBAL_LENGTH_TO_CENTER+aim_x*aim_x))*180.0f/PI;
}
void Gimbal_Task() {

}

