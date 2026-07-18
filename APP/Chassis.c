#include "Chassis.h"
#include "trace.h"
#include "dcmotor.h"
#include "Robot_Cmd.h"
#include "JY901S.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "misc.h"
#include "dwt.h"
#include "trace.h"
#include "IMU.h"

static DCMotorInstance *motor_l,*motor_r;

static chassis_cmd_q chassis_cmd_receive={0};	//来自cmd的控制命令
Chassis_Move_State_e Chassis_Move_State;		//底盘当前的移动状态（停止、前进、转向）

float trace_dt;					//调试用，记录巡线任务的耗时
float trace_starttime;

uint8_t stop_count,spin_count;			// 直线/转弯到达目标后会累加，用于判断是否完成直线/转弯
uint8_t Spin_start_flag = 0, Spin_succeed_flag = 0 ;
uint8_t Stop_Flag = 0;
uint8_t Line_flag = 0, Turn_flag = 0 ;  // 1表示正在执行巡线/转弯任务，0表示未在巡线/转弯
uint8_t state = 0;						//底盘巡线状态，从0到9个状态
static float trace_compensation;		//巡线补偿量

static uint8_t remote_mode_active = 0;

float IMU_data[3] = {0};
volatile JY901s_IMU_Data_s* JY901s_IMU_Data;

void Stop_Detect(void);
void Chassis_State_Turn(void);
void Chassis_Set_Turn(void);
static void Chassis_RemoteControl(void);
static void Chassis_ClearRemoteSpeed(void);
static void Chassis_RemoteLostDisable(void);
static void Chassis_Trace_Cal(void);
static void Motor_FeedForward_Update(void);
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
		.loop_mode = ANGLE_MODE,
		.speed_pid_config = {
			.mode = PID_POSITION,
			.Kp = 2000.0f,
			.Ki = 0.0f,
			.Kd = 0.0f,
			.max_out = 2499.0f,
			.max_iout = 500.0f,
			.kf_p = 8000.0f,
			.ff_type = FF_Proportional | FF_Velocity ,
			.ff_lpf_gain = 0.15f,
			.d_lpf_gain = 0.15f,
		},
		.position_pid_config= {
			.mode = PID_POSITION,
			.Kp = 15.0f,
			.Kd = 0.0f,
			.Ki = 0.5f,
			.max_out = 0.3f,
			.max_iout = 0.05f,
		},
		.feedforward = 85,  //这里的feedforward只是相当于一个阻尼前馈
	};
	motor_l = DCMotor_Init(&motor_l_config);

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
		.loop_mode = ANGLE_MODE,
		.speed_pid_config = {
			.mode = PID_POSITION,
			.Kp = 2000.0f,
			.Ki = 0.0f,
			.Kd = 500.0f,
			.max_out = 2499.0f,
			.max_iout = 500.0f,
			.kf_p = 8000.0f,
			.ff_type = FF_Proportional | FF_Velocity ,
			.ff_lpf_gain = 0.15f,
			.d_lpf_gain = 0.15f,
		},
		.position_pid_config= {
			.mode = PID_POSITION,
			.Kp = 15.0f,
			.Kd = 0.0f,
			.Ki = 0.5f,
			.max_out = 0.3f,
			.max_iout = 0.05f,
		},
		.feedforward = 85,
	};
	motor_r = DCMotor_Init(&motor_r_config);

	//陀螺仪初始化，目前同时使用两个陀螺仪，可根据需要注释掉一个
	IMU_init();							//ICM42688陀螺仪初始化，ICM需要主动轮询读取并解算
	JY901s_IMU_Data = JY901s_IMU_Init(); //JY901s陀螺仪初始化,这里直接获取指针
	DWT_Delay(1);
}


/**
 * @brief 底盘主要任务，根据菜单不同模式执行对应任务，目前以200Hz运行
 */
void Chassis(void)
{
	//接收Cmd发来的控制指令
	xQueueReceive(chassis_cmd_queue, &chassis_cmd_receive, 1);

	//更新ICM陀螺仪数据
	IMU_getYawPitchRoll((float *)IMU_data);  //耗时约1ms

	if (chassis_cmd_receive.remote_lost_disable != 0U)
	{
		Chassis_RemoteLostDisable();
		return;
	}

	if (chassis_cmd_receive.Chassis_Mode != REMOTE_MODE)
	{
		Chassis_ClearRemoteSpeed();
	}
	switch(chassis_cmd_receive.Chassis_Mode)
	{
		case TRACE_MODE:
			//计算并设定巡线补偿量
			Chassis_Trace_Cal();
			//按照设定的路径前进
			Chassis_State_Turn();
			//检测直行/转弯是否完成，更新标志位
			Stop_Detect();
			break;
		case IMU_MODE:

			break;
		case NORMAL_MODE:

			Chassis_Set_Turn();

			break;
		case POSITION_MODE:
				break;
		case REMOTE_MODE:
			Chassis_RemoteControl();
			break;
		default:
			break;
	}
	
	
}

/**
 * @brief 计算并更新巡线补偿量
 */
static void Chassis_Trace_Cal(void) {
	trace_compensation=Trace_task();
	DCMotor_SetTraceCompensation(motor_l,-trace_compensation);
	DCMotor_SetTraceCompensation(motor_r,trace_compensation);
}
/**
 * @brief 被cmd调用，根据菜单值回调选择是否使能电机
 */
static void Chassis_RemoteControl(void)
{
	static float left_out;
	static float right_out;
	//遥控器下改成速度环
	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	//计算差速轮输出
	float left_speed = chassis_cmd_receive.remote_forward - chassis_cmd_receive.remote_turn;
	float right_speed = chassis_cmd_receive.remote_forward + chassis_cmd_receive.remote_turn;

	// left_out += left_speed;
	// right_out += right_speed;

	motor_l->State = ENABLE;
	motor_r->State = ENABLE;
	Line_flag = 0;
	Stop_Flag = 0;
	Spin_start_flag = 0;
	Spin_succeed_flag = 0;
	// DCMotor_SetTraceCompensation(motor_l, 0.0f);
	// DCMotor_SetTraceCompensation(motor_r, 0.0f);
	Chassis_Trace_Cal();

	if (remote_mode_active == 0U)
	{
		PID_clear(&motor_l->position_pid);
		PID_clear(&motor_r->position_pid);
		remote_mode_active = 1U;
	}

	DC_Motor_SetRef(motor_l, left_speed);
	DC_Motor_SetRef(motor_r, right_speed);
}

static void Chassis_ClearRemoteSpeed(void)
{
	if (remote_mode_active == 0U)
	{
		return;
	}

	DC_Motor_SetRef(motor_l, 0.0f);
	DC_Motor_SetRef(motor_r, 0.0f);
}

static void Chassis_RemoteLostDisable(void)
{
	// ELRS断线时直接关闭H桥输入，避免保持最后一次遥控输出。
	Chassis_ClearRemoteSpeed();
	DC_Motor_SetRef(motor_l, 0.0f);
	DC_Motor_SetRef(motor_r, 0.0f);
	DCMotor_Cmd(motor_l, DISABLE);
	DCMotor_Cmd(motor_r, DISABLE);
}

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
				if((abs_out(motor_l->position_pid.Ref-motor_l->position_measure)<0.008f)&&(abs_out(motor_r->position_pid.Ref-motor_r->position_measure)<0.008f))
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
			if(spin_count >= 200 &&(abs_out(motor_l->position_pid.Ref-motor_l->position_measure)<0.008f)&&(abs_out(motor_r->position_pid.Ref-motor_r->position_measure)<0.008f))
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
	motor_l->loop_mode = ANGLE_MODE;
	motor_r->loop_mode = ANGLE_MODE;
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

	DC_Motor_SetRef(motor_l, -0.01);
	DC_Motor_SetRef(motor_r, 0.01);
}

void Chassis_Set_Line(float position)
{
	motor_l->loop_mode = ANGLE_MODE;
	motor_r->loop_mode = ANGLE_MODE;
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

	DC_Motor_SetRef(motor_l, position);
	DC_Motor_SetRef(motor_r, position);
}

void Chassis_State_Turn(void)
{

	static uint8_t quan=0;
	switch(state)
	{
		case 0:
			if(quan < chassis_cmd_receive.circle_set)
			{
				state ++;
				Chassis_Set_Line(0.08);
			}
		break;
		case 1:
			if(Stop_Flag)
			{
				Chassis_Set_Turn();
				state ++;
			}
		break;
		case 2:
			if(Spin_succeed_flag)
			{
				Chassis_Set_Line(0.082);
				state ++;
			}
		break;
		case 3:
			if(Stop_Flag)
			{
				Chassis_Set_Turn();
				state ++;
			}
		break;
		case 4:
			if(Spin_succeed_flag)
			{
				Chassis_Set_Line(0.082);
				state ++;
			}
		break;
			case 5:
			if(Stop_Flag)
			{
				Chassis_Set_Turn();
				state ++;
			}
		break;
			case 6:
			if(Spin_succeed_flag)
			{
				Chassis_Set_Line(0.082);
				state ++;
			}
		break;
			case 7:
			if(Stop_Flag)
			{
				Chassis_Set_Turn();
				state ++;
			}
		break;
		case 8:
			if(Spin_succeed_flag)
			{
				Chassis_Set_Line(0.01);
				state ++;
			}
		break;
		case 9:
			if(Stop_Flag)
			{
				quan ++;
				state = 0;
			}
		break;
		default:
			break;
	}

	
	
	
}
