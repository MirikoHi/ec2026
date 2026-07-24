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
#include "IMU_Mahony.h"
#include "PID.h"
#include "flash_param_store.h"
#include "bsp_log.h"

static DCMotorInstance *motor_l,*motor_r;

static chassis_cmd_q chassis_cmd_receive = {0};    // 来自 cmd 的底盘控制命令
Chassis_Move_State_e Chassis_Move_State;           // 底盘当前移动状态

float trace_dt;					// 调试用，记录巡线任务耗时
float trace_starttime;

uint8_t stop_count,spin_count;			// 直线/转弯到达目标后的稳定计数
uint8_t Spin_start_flag = 0, Spin_succeed_flag = 0 ;
uint8_t Stop_Flag = 0;
uint8_t Line_flag = 0, Turn_flag = 0 ;  // 1 表示正在执行巡线/转弯任务，0 表示未执行
uint8_t state = 0;                                  // 巡线状态机状态
static float trace_compensation;                    // 巡线补偿量
static uint8_t remote_mode_active = 0;

#define CHASSIS_ACTION_DONE          1U
#define CHASSIS_ACTION_RUNNING       0U
static pid_type_def chassis_line_yaw_pid;
static pid_type_def chassis_turn_pid;
static FlashParam_Data_s chassis_param;
static Chassis_Mode_e chassis_last_mode = NORMAL_MODE;
static uint8_t chassis_action_active = 0U;
static uint8_t chassis_action_type = 0U;
static uint8_t chassis_action_done_count = 0U;
static float chassis_action_target_yaw = 0.0f;
static float chassis_action_target_distance = 0.0f;
static uint8_t chassis_imu_action_step = 0U;

float IMU_data[3] = {0};
volatile JY901s_IMU_Data_s* JY901s_IMU_Data;

void Stop_Detect(void);
void Chassis_State_Turn(void);
void Chassis_Set_Turn(void);
static void Chassis_RemoteControl(void);
static void Chassis_ClearRemoteSpeed(void);
static void Chassis_RemoteLostDisable(void);
static void Chassis_Trace_Cal(void);
static void Chassis_ImuModeAction(void);
static void Chassis_ResetEncoderOdom(void);
static float Chassis_GetForwardOdom(void);
static float Chassis_GetYawDeg(void);
static float Chassis_AngleNormalize(float angle);

/**
  * @brief 填充底盘相关默认参数
  * @param params 待填充参数结构体指针
  * @note 默认值保留在底盘使用处，Flash 参数无效时由存储库调用本函数恢复默认底盘参数
  */
void Chassis_FillDefaultParams(FlashParam_Data_s *params)
{
	if (params == NULL)
	{
		return;
	}

	params->line_yaw_pid = (pid_init_config_s) {
		.mode = PID_POSITION,
		.Kp = 0.0008f,
		.Ki = 0.0f,
		.Kd = 0.000003f,
		.max_out = 0.08f,
		.max_iout = 0.0f,
		.deadzone = 0.5f,
		.ff_type = FF_None,
	};

	params->turn_pid = (pid_init_config_s) {
		.mode = PID_POSITION,
		.Kp = 0.0008f,
		.Ki = 0.000012f,
		.Kd = 0.000003f,
		.max_out = 0.08f,
		.max_iout = 0.01f,
		.deadzone = 0.001f,
		.ff_type = FF_None,
	};

	params->line_distance_m[0] = 0.96f;
	params->line_distance_m[1] = 0.96f;
	params->line_distance_m[2] = 0.96f;
	params->line_distance_m[3] = 0.96f;

	params->turn_angle_deg[0] = 92.0f;
	params->turn_angle_deg[1] = 92.0f;
	params->turn_angle_deg[2] = 92.0f;
	params->turn_angle_deg[3] = 92.0f;

	params->line_accel_m = 0.20f;
	params->line_slowdown_m = 0.25f;
	params->line_min_speed_mps = 0.025f;
	params->line_done_err_m = 0.005f;
	params->line_done_ticks = 10U;
	params->turn_done_err_deg = 3.0f;
	params->turn_done_ticks = 10U;
	params->action_speed_mps = 0.2f;
	params->turn_speed_mps = 0.05f;
}

/**
  * @brief 应用底盘运行参数
  * @param params 待应用参数结构体指针
  * @note 用法：上电初始化或屏幕端 FlashParam_Save() 成功后调用，使底盘 PID 和动作参数立即生效
  */
void Chassis_ApplyParams(const FlashParam_Data_s *params)
{
	if (params == NULL)
	{
		return;
	}

	chassis_param = *params;
	PID_init(&chassis_line_yaw_pid, &chassis_param.line_yaw_pid);
	PID_init(&chassis_turn_pid, &chassis_param.turn_pid);
	Chassis_ResetAction();

	LOGINFO("[param] chassis apply source=%d, rev=%u, seq=%u",
	        FlashParam_GetSource(),
	        FlashParam_GetDefaultRevision(),
	        FlashParam_GetSequence());
	LOGINFO("[param] line=%.3f %.3f %.3f %.3f, turn=%.1f %.1f %.1f %.1f",
	        chassis_param.line_distance_m[0],
	        chassis_param.line_distance_m[1],
	        chassis_param.line_distance_m[2],
	        chassis_param.line_distance_m[3],
	        chassis_param.turn_angle_deg[0],
	        chassis_param.turn_angle_deg[1],
	        chassis_param.turn_angle_deg[2],
	        chassis_param.turn_angle_deg[3]);
}

/**
 * @brief 初始化底盘左右电机和 IMU
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
		.feedforward = 85,  // feedforward is used as a simple damping term.
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

	// ICM42688 陀螺仪初始化，ICM 需要主动轮询读取并解算。
	IMU_Mahony_Init();
	// JY901s_IMU_Data = JY901s_IMU_Init();

	// 上电参数已经由 FlashParam_Init() 读取，这里复制一份给底盘动作使用。
	Chassis_ApplyParams(FlashParam_GetActive());
	DWT_Delay(1);
}


/**
 * @brief 底盘主任务，根据菜单模式执行对应动作
 */
void Chassis(void)
{
	// 接收 Robot_Cmd 发来的底盘控制命令。
	xQueueReceive(chassis_cmd_queue, &chassis_cmd_receive, 1);

	// 更新 ICM42688 姿态数据，耗时约 1ms。
	IMU_Mahony_GetYawPitchRoll((float *)IMU_data);

	if (chassis_cmd_receive.remote_disable != 0U)
	{
		Chassis_RemoteLostDisable();
		return;
	}

	if (chassis_cmd_receive.Chassis_Mode != REMOTE_MODE)
	{
		Chassis_ClearRemoteSpeed();
	}

	if (chassis_cmd_receive.Chassis_Mode != chassis_last_mode)
	{
		if ((chassis_last_mode == IMU_MODE) || (chassis_cmd_receive.Chassis_Mode == IMU_MODE))
		{
			Chassis_ResetAction();
			chassis_imu_action_step = 0U;
		}
		chassis_last_mode = chassis_cmd_receive.Chassis_Mode;
	}

	switch(chassis_cmd_receive.Chassis_Mode)
	{
		case TRACE_MODE:
			// 计算并设置巡线补偿量。
			Chassis_Trace_Cal();
			// 按设定路径执行巡线动作。
			Chassis_State_Turn();
			// 检测直行/转弯是否完成，并更新完成标志。
			Stop_Detect();
			break;
		case IMU_MODE:
			Chassis_ImuModeAction();
			break;
		case NORMAL_MODE:

			Chassis_Set_Turn();

			break;
		case POSITION_MODE:
				break;
		case REMOTE_MODE:
			Chassis_RemoteControl();
			//Chassis_ImuModeAction();
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
 * @brief IMU_MODE 下的测试动作：按边长 0.2m 的正方形循环行走
 */
static void Chassis_ImuModeAction(void)
{
	switch (chassis_imu_action_step)
	{
		case 0:
			// 第一条边：使用 ICM42688 yaw 做方向保持，直行 1.0m。
			if (Chassis_MoveStraight(chassis_param.line_distance_m[0], chassis_param.action_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 1U;
			}
			else {
				if (Chassis_GetForwardOdom()>0.7f)//大于0.7米时不再循迹
				{
					Trace_ResetLineError();
					DCMotor_SetTraceCompensation(motor_l,0);
					DCMotor_SetTraceCompensation(motor_r,0);
				}
				else {
					Chassis_Trace_Cal();//小于0.7继续循迹
					if(Chassis_GetForwardOdom()>0.4f&&Chassis_GetForwardOdom()<0.5f) {
						chassis_action_target_yaw = Chassis_GetYawDeg();//在0.4米到0.5米之间记录yaw角，作为循迹的目标角度
					}
				}
			}
			break;
		case 1:
			// 第一次转角：原地转向 90 度。
			if (Chassis_TurnAngle(chassis_param.turn_angle_deg[0], chassis_param.turn_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 2U;
			}
			break;
		case 2:
			// 第二条边：直行 1.0m。
			if (Chassis_MoveStraight(chassis_param.line_distance_m[1], chassis_param.action_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 3U;
			}
			else {
				if (Chassis_GetForwardOdom()>0.7f) {
					Trace_ResetLineError();
					DCMotor_SetTraceCompensation(motor_l,0);
					DCMotor_SetTraceCompensation(motor_r,0);
				}
				else {
					Chassis_Trace_Cal();//小于0.7继续循迹
					if(Chassis_GetForwardOdom()>0.4f&&Chassis_GetForwardOdom()<0.5f) {
						chassis_action_target_yaw = Chassis_GetYawDeg();//在0.4米到0.5米之间记录yaw角，作为循迹的目标角度
					}
				}
			}
			break;
		case 3:
			// 第二次转角：原地转向 90 度。
			if (Chassis_TurnAngle(chassis_param.turn_angle_deg[1], chassis_param.turn_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 4U;
			}
			break;
		case 4:
			// 第三条边：直行 1.0m。
			if (Chassis_MoveStraight(chassis_param.line_distance_m[2], chassis_param.action_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 5U;
			}
			else {
				if (Chassis_GetForwardOdom()>0.7f) {
					Trace_ResetLineError();
					DCMotor_SetTraceCompensation(motor_l,0);
					DCMotor_SetTraceCompensation(motor_r,0);
				}
				else {
					Chassis_Trace_Cal();//小于0.7继续循迹
					if(Chassis_GetForwardOdom()>0.4f&&Chassis_GetForwardOdom()<0.5f) {
						chassis_action_target_yaw = Chassis_GetYawDeg();//在0.4米到0.5米之间记录yaw角，作为循迹的目标角度
					}
				}
			}
			break;
		case 5:
			// 第三次转角：原地转向 90 度。
			if (Chassis_TurnAngle(chassis_param.turn_angle_deg[2], chassis_param.turn_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 6U;
			}
			break;
		case 6:
			// 第四条边：直行 1.0m。
			if (Chassis_MoveStraight(chassis_param.line_distance_m[3], chassis_param.action_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 7U;
			}
			else {
				if (Chassis_GetForwardOdom()>0.7f) {
					Trace_ResetLineError();
					DCMotor_SetTraceCompensation(motor_l,0);
					DCMotor_SetTraceCompensation(motor_r,0);
				}
				else {
					Chassis_Trace_Cal();//小于0.7继续循迹
					if(Chassis_GetForwardOdom()>0.4f&&Chassis_GetForwardOdom()<0.5f) {
						chassis_action_target_yaw = Chassis_GetYawDeg();//在0.4米到0.5米之间记录yaw角，作为循迹的目标角度
					}
				}
			}
			break;
		case 7:
			// 第四次转角完成后回到第一条边，形成正方形循环。
			if (Chassis_TurnAngle(chassis_param.turn_angle_deg[3], chassis_param.turn_speed_mps) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 0U;
			}
			break;
		default:
			// 异常状态下复位到第一条边。
			DC_Motor_SetRef(motor_l, 0.0f);
			DC_Motor_SetRef(motor_r, 0.0f);
			chassis_imu_action_step = 0U;
			break;
	}
}

static void Chassis_RemoteControl(void)
{
	// 遥控模式使用速度环，直接给左右轮差速速度。
	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	// 计算差速轮输出。
	float left_speed = chassis_cmd_receive.remote_forward - chassis_cmd_receive.remote_turn;
	float right_speed = chassis_cmd_receive.remote_forward + chassis_cmd_receive.remote_turn;

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
	// ELRS 离线时直接关闭电机输出，避免保持最后一次遥控量。
	Chassis_ClearRemoteSpeed();
	DC_Motor_SetRef(motor_l, 0.0f);
	DC_Motor_SetRef(motor_r, 0.0f);
	DCMotor_Cmd(motor_l, DISABLE);
	DCMotor_Cmd(motor_r, DISABLE);
}

/**
 * @brief 清除底盘自动动作状态，并将左右轮速度给定清零
 */
void Chassis_ResetAction(void)
{
	// 清除当前自动动作状态；下次调用直线/转向函数时会重新锁定目标。
	chassis_action_active = 0U;
	chassis_action_type = 0U;
	chassis_action_done_count = 0U;
	chassis_action_target_yaw = 0.0f;
	chassis_action_target_distance = 0.0f;

	// 清除 PID 历史，避免上一次动作的误差和微分项影响下一次动作。
	PID_clear(&chassis_line_yaw_pid);
	PID_clear(&chassis_turn_pid);

	// 清零速度给定，但不直接失能电机，方便状态机继续接管底盘。
	DC_Motor_SetRef(motor_l, 0.0f);
	DC_Motor_SetRef(motor_r, 0.0f);
}

/**
 * @brief 按启动瞬间的 ICM42688 yaw 保持方向，直行指定距离
 * @param distance_m 目标距离，单位 m，正数前进，负数后退
 * @param speed_mps 速度给定，单位 m/s，只取绝对值，方向由 distance_m 决定
 * @return CHASSIS_ACTION_DONE 表示完成，否则返回 CHASSIS_ACTION_RUNNING
 */
uint8_t Chassis_MoveStraight(float distance_m, float speed_mps)
{
	const float abs_distance = fabsf(distance_m);
	float abs_speed = fabsf(speed_mps);
	float remain;
	float progress;
	float base_speed;
	float yaw_error;
	float yaw_compensation;

	if (abs_distance <= chassis_param.line_done_err_m)
	{
		Chassis_ResetAction();
		return CHASSIS_ACTION_DONE;
	}

	if (abs_speed < 0.001f)
	{
		abs_speed = 0.001f;
	}

	if ((chassis_action_active == 0U) || (chassis_action_type != 1U))
	{
		chassis_action_active = 1U;
		chassis_action_type = 1U;
		chassis_action_done_count = 0U;

		// 只在动作启动瞬间锁定目标 yaw，后续周期不能刷新，否则直线方向会漂。
		chassis_action_target_yaw = Chassis_GetYawDeg();
		chassis_action_target_distance = distance_m;

		Chassis_ResetEncoderOdom();
		PID_clear(&chassis_line_yaw_pid);
		PID_clear(&chassis_turn_pid);
	}

	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

	remain = chassis_action_target_distance - Chassis_GetForwardOdom();
	if (fabsf(remain) <= chassis_param.line_done_err_m)
	{
		chassis_action_done_count++;
		DC_Motor_SetRef(motor_l, 0.0f);
		DC_Motor_SetRef(motor_r, 0.0f);
		if (chassis_action_done_count >= chassis_param.line_done_ticks)
		{
			Chassis_ResetAction();
			return CHASSIS_ACTION_DONE;
		}
		return CHASSIS_ACTION_RUNNING;
	}

	chassis_action_done_count = 0U;
	progress = fabsf(chassis_action_target_distance) - fabsf(remain);
	if (progress < 0.0f)
	{
		progress = 0.0f;
	}

	base_speed = (remain > 0.0f) ? abs_speed : -abs_speed;
	if ((chassis_param.line_accel_m > 0.001f) && (progress < chassis_param.line_accel_m))
	{
		float accel_speed = chassis_param.line_min_speed_mps +
		                    (abs_speed - chassis_param.line_min_speed_mps) * progress / chassis_param.line_accel_m;

		/*
		 * 起步前 0.1m 线性加速，避免一进入直线动作就给满速度。
		 */
		base_speed = (remain > 0.0f) ? accel_speed : -accel_speed;
	}
	if ((chassis_param.line_slowdown_m > 0.001f) && (fabsf(remain) < chassis_param.line_slowdown_m))
	{
		float slowdown_speed = abs_speed * fabsf(remain) / chassis_param.line_slowdown_m;

		/*
		 * 直线末段提前按剩余距离线性降速。
		 * 原逻辑只剩约 2.4cm 才减速，实际表现接近急停。
		 */
		if (slowdown_speed < chassis_param.line_min_speed_mps)
		{
			slowdown_speed = chassis_param.line_min_speed_mps;
		}
		if (slowdown_speed > abs_speed)
		{
			slowdown_speed = abs_speed;
		}

		if (fabsf(slowdown_speed) < fabsf(base_speed))
		{
			base_speed = (remain > 0.0f) ? slowdown_speed : -slowdown_speed;
		}
	}

	yaw_error = Chassis_AngleNormalize(chassis_action_target_yaw - Chassis_GetYawDeg());
	yaw_compensation = PID_calc(&chassis_line_yaw_pid, 0.0f, yaw_error);

	DC_Motor_SetRef(motor_l, base_speed + yaw_compensation);
	DC_Motor_SetRef(motor_r, base_speed - yaw_compensation);

	return CHASSIS_ACTION_RUNNING;
}

/**
 * @brief 使用 ICM42688 yaw 归一化角度，原地转向指定相对角度
 * @param angle_deg 目标相对角度，单位 deg，正负决定转向方向
 * @param max_turn_speed 转向外环 PID 最大速度输出幅值
 * @return CHASSIS_ACTION_DONE 表示完成，否则返回 CHASSIS_ACTION_RUNNING
 */
uint8_t Chassis_TurnAngle(float angle_deg, float max_turn_speed)
{
	float abs_turn_speed = fabsf(max_turn_speed);
	float yaw_error;
	float turn_speed;

	if (fabsf(angle_deg) <= chassis_param.turn_done_err_deg)
	{
		Chassis_ResetAction();
		return CHASSIS_ACTION_DONE;
	}

	if (abs_turn_speed < 0.001f)
	{
		abs_turn_speed = 0.001f;
	}

	if ((chassis_action_active == 0U) || (chassis_action_type != 2U))
	{
		chassis_action_active = 1U;
		chassis_action_type = 2U;
		chassis_action_done_count = 0U;
		chassis_action_target_yaw = Chassis_AngleNormalize(Chassis_GetYawDeg() + angle_deg);

		Chassis_ResetEncoderOdom();
		PID_clear(&chassis_line_yaw_pid);
		PID_clear(&chassis_turn_pid);
	}

	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

	// 直接读取当前归一化 yaw，和启动时锁定的目标 yaw 做最短角误差。
	yaw_error = Chassis_AngleNormalize(chassis_action_target_yaw - Chassis_GetYawDeg());
	if (fabsf(yaw_error) <= chassis_param.turn_done_err_deg)
	{
		chassis_action_done_count++;
		DC_Motor_SetRef(motor_l, 0.0f);
		DC_Motor_SetRef(motor_r, 0.0f);
		if (chassis_action_done_count >= chassis_param.turn_done_ticks)
		{
			Chassis_ResetAction();
			return CHASSIS_ACTION_DONE;
		}
		return CHASSIS_ACTION_RUNNING;
	}

	chassis_action_done_count = 0U;
	chassis_turn_pid.max_out = abs_turn_speed;
	turn_speed = PID_calc(&chassis_turn_pid, 0.0f, yaw_error);

	DC_Motor_SetRef(motor_l, turn_speed);
	DC_Motor_SetRef(motor_r, -turn_speed);

	return CHASSIS_ACTION_RUNNING;
}

static void Chassis_ResetEncoderOdom(void)
{
	// 清除编码器累计值，后续电机任务会继续更新 position_measure。
	motor_l->encoder->total_count = 0;
	motor_r->encoder->total_count = 0;

	// 同步清零位置测量值，避免本周期读到清零前的旧里程。
	motor_l->position_measure = 0.0f;
	motor_r->position_measure = 0.0f;
}

/**
 * @brief 获取左右轮平均直线里程
 * @return 当前动作开始后的平均里程，单位 m
 */
static float Chassis_GetForwardOdom(void)
{
	// position_measure 已经在 dcmotor.c 中换算为米，取左右轮平均作为车体直线里程。
	return (motor_l->position_measure + motor_r->position_measure) * 0.5f;
}

/**
 * @brief 获取 ICM42688 解算出的 yaw 角
 * @return yaw 角度，单位 deg
 */
static float Chassis_GetYawDeg(void)
{
	// Chassis() 每周期调用 IMU_getYawPitchRoll()，IMU_data[0] 为 ICM42688 yaw，单位 deg。 1是pitch，2是roll
	return IMU_data[0];
}

/**
 * @brief 将角度归一化到 [-180, 180]
 * @param angle 输入角度，单位 deg
 * @return 归一化后的角度，单位 deg
 */
static float Chassis_AngleNormalize(float angle)
{
	// 将角度归一化到 [-180, 180]，避免 yaw 跨边界时误差突变。
	while (angle > 180.0f)
	{
		angle -= 360.0f;
	}
	while (angle < -180.0f)
	{
		angle += 360.0f;
	}
	return angle;
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
								Stop_Flag = 1; // 可用于判断是否进入下一阶段任务
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
	Line_flag = 0;
	Stop_Flag = 0;
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
