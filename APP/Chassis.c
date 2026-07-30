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
#include "bsp_log.h"
#include "K230.h"

static DCMotorInstance *motor_l,*motor_r;

static chassis_cmd_q chassis_cmd_receive = {0};    // 来自 cmd 的底盘控制命令
Chassis_Move_State_e Chassis_Move_State;           // 底盘当前移动状态

float trace_dt;					// 调试用，记录巡线任务耗时
float trace_starttime;

static FlashParam_Data_s chassis_param;
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
static Chassis_Mode_e chassis_last_mode = NORMAL_MODE;
static uint8_t chassis_action_active = 0U;
static uint8_t chassis_action_type = 0U;
static uint8_t chassis_action_done_count = 0U;
static float chassis_action_target_yaw = 0.0f;
static float chassis_action_target_distance = 0.0f;
static uint8_t chassis_imu_action_step = 0U;

/* ── 路径级梯形加减速 ────────────────────────────────────────── */
#define IMU_PATH_LINE_M        0.5f//1.70f
#define IMU_PATH_RADIUS_M      0.6f
#define IMU_PATH_ARC_M         1.5707963f          /* π × 0.5 */
#define IMU_PATH_TOTAL_M       ((IMU_PATH_LINE_M + IMU_PATH_ARC_M) * 0.01f)  /* ≈ 6.142m */
#define IMU_PATH_ACCEL_M       0.03f                /* 路径起步加速段 */
#define IMU_PATH_SLOWDOWN_M    0.03f                /* 路径末尾减速段 */

static float imu_path_cumulative = 0;              /* 已完成步骤的累计里程 */
static float imu_path_step_entry_odom = 0;         /* 当前步骤开始时的全局里程 */

float IMU_data[3] = {0};
volatile JY901s_IMU_Data_s* JY901s_IMU_Data;

extern State robotcmd_control_state;


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
		.loop_mode = SPEED_MODE,
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
		.loop_mode = SPEED_MODE,
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

	pid_init_config_s chassis_line_yaw_pid_config={
		.mode = PID_POSITION,
		.Kp = 0.01f,
		.Kd = 0.0001f,
		.Ki = 0.0f,
		.max_out = 40.0f,
		.max_iout = 1.0f,
	};
	PID_init(&chassis_line_yaw_pid,&chassis_line_yaw_pid_config);

	pid_init_config_s chassis_turn_pid_config={
		.mode = PID_POSITION,
		.Kp = 0.3f,
		.Kd = 0.0001f,
		.Ki = 0.0f,
		.max_out = 400.0f,
		.max_iout = 1.0f,
	};
	PID_init(&chassis_turn_pid,&chassis_turn_pid_config);

	chassis_param.line_distance_m[0] = 0.5f;
	chassis_param.line_distance_m[1] = 0.5f;
	chassis_param.line_done_err_m = 0.1f;
	chassis_param.line_done_ticks = 20;
	chassis_param.line_accel_m = 0.2f;        /* 加速段 0.2m */
	chassis_param.line_slowdown_m = 0.3f;     /* 减速段 0.3m */
	chassis_param.line_min_speed_mps = 0.08f; /* 最低速 0.08m/s, 必须 > 0 否则起步死锁 */
	chassis_param.turn_angle_deg[0] = 90.0f;
	chassis_param.turn_speed_mps = 0.005f;
	chassis_param.action_speed_mps = 0.05f;
	chassis_param.turn_done_err_deg = 5.0f;   /* 转弯完成阈值 5° */
	chassis_param.turn_done_ticks = 10;

	// BMI088 陀螺仪初始化
	IMU_Mahony_Init();

	DWT_Delay(1);
}

/**
 * @brief 底盘主任务，根据菜单模式执行对应动作
 */
void Chassis(void)
{
	// 接收 Robot_Cmd 发来的底盘控制命令。
	xQueueReceive(chassis_cmd_queue, &chassis_cmd_receive, 1);

	// 更新 BMI088 姿态数据，耗时约 1ms。
	IMU_Mahony_GetYawPitchRoll((float *)IMU_data);


	if (!chassis_cmd_receive.remote_lost)
	{
		DCMotor_Cmd(motor_l,ENABLE);
		DCMotor_Cmd(motor_r,ENABLE);
	}
	if (chassis_cmd_receive.Chassis_Mode != chassis_last_mode)
	{
		if ((chassis_last_mode == IMU_MODE) || (chassis_cmd_receive.Chassis_Mode == IMU_MODE))
		{
			Chassis_ResetAction();
			chassis_imu_action_step = 0U;
			imu_path_cumulative = 0;  /* 重置路径里程 */
		}
		chassis_last_mode = chassis_cmd_receive.Chassis_Mode;
	}

	switch(chassis_cmd_receive.Chassis_Mode)
	{
		case TRACE_MODE:
			// 计算并设置巡线补偿量。
			Chassis_Trace_Cal();
			// 按设定路径执行巡线动作。
			// Chassis_State_Turn();
			// 检测直行/转弯是否完成，并更新完成标志。
			// Stop_Detect();
			break;
		case IMU_MODE:
			Chassis_ImuModeAction();
			break;
		case NORMAL_MODE:
			// Chassis_Set_Turn();
			break;
		case REMOTE_MODE:
			// Chassis_RemoteControl();
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
	DCMotor_SetTraceCompensation(motor_l,-2*trace_compensation);
	DCMotor_SetTraceCompensation(motor_r,2*trace_compensation);
}
/**
 * @brief IMU_MODE 下的测试动作：按边长 0.2m 的正方形循环行走
 */
/**
 * @brief IMU_MODE 下的测试动作：药丸形(体育场形)封闭路径
 *
 *  路径: 直线1.5m → 半圆R=0.5m右转 → 直线1.5m → 半圆R=0.5m右转 → 循环
 *
 *               直线 1.5m
 *            ┌────────────┐
 *           ↗              ↘
 *   半圆 R=0.5m          半圆 R=0.5m
 *   右转180°             右转180°
 *           ↖              ↙
 *            └────────────┘
 *               直线 1.5m
 */
uint8_t result=0;
static void Chassis_ImuModeAction(void)
{


	switch (chassis_imu_action_step)
	{
		case 0:
			result = Chassis_MoveStraight(IMU_PATH_LINE_M, chassis_param.action_speed_mps);
			if (result == CHASSIS_ACTION_DONE) {
				imu_path_cumulative += IMU_PATH_LINE_M;
				chassis_imu_action_step = 1U;
			}
			break;
		case 1:
			result = Chassis_SemiCircle(IMU_PATH_RADIUS_M, chassis_param.turn_speed_mps, 1);
			if (result == CHASSIS_ACTION_DONE) {
				imu_path_cumulative += IMU_PATH_ARC_M;
				chassis_imu_action_step = 2U;
			}
			break;
		case 2:
			result = Chassis_MoveStraight(IMU_PATH_LINE_M, chassis_param.action_speed_mps);
			if (result == CHASSIS_ACTION_DONE) {
				imu_path_cumulative += IMU_PATH_LINE_M;
				chassis_imu_action_step = 3U;
			}
			break;
		case 3:
			result = Chassis_SemiCircle(IMU_PATH_RADIUS_M, chassis_param.turn_speed_mps, 1);
			if (result == CHASSIS_ACTION_DONE) {
				imu_path_cumulative = 0;  /* 一圈结束, 下一圈从 0 开始 */
				chassis_imu_action_step = 0U;
			}
			break;
		default:
			DC_Motor_SetRef(motor_l, 0.0f);
			DC_Motor_SetRef(motor_r, 0.0f);
			imu_path_cumulative = 0;
			chassis_imu_action_step = 0U;
			break;
	}
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
 * @brief 按启动瞬间的  IMU yaw 保持方向，直行指定距离
 * @param distance_m 目标距离，单位 m，正数前进，负数后退
 * @param speed_mps 速度给定，单位 m/s，只取绝对值，方向由 distance_m 决定
 * @return CHASSIS_ACTION_DONE 表示完成，否则返回 CHASSIS_ACTION_RUNNING
 */
float base_speed;
float yaw_compensation=0;
uint8_t Chassis_MoveStraight(float distance_m, float speed_mps)
{
	const float abs_distance = fabsf(distance_m);
	float abs_speed = fabsf(speed_mps);
	float remain;
	float progress;
	;
	float yaw_error;
	;

	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

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

	/* ── 路径级梯形加减速 ── */
	float path_progress = imu_path_cumulative + progress;       /* 整条路径已走距离 */
	float path_remain   = IMU_PATH_TOTAL_M - path_progress;     /* 整条路径剩余距离 */

	base_speed = (remain > 0.0f) ? abs_speed : -abs_speed;

	/* 起步段加速 */
	if (path_progress < IMU_PATH_ACCEL_M)
	{
		float accel_speed = chassis_param.line_min_speed_mps +
			(abs_speed - chassis_param.line_min_speed_mps) *
				path_progress / IMU_PATH_ACCEL_M;
		base_speed = (remain > 0.0f) ? accel_speed : -accel_speed;
	}
	/* 末尾段减速 */
	else if (path_remain < IMU_PATH_SLOWDOWN_M)
	{
		float slowdown_speed = abs_speed * path_remain / IMU_PATH_SLOWDOWN_M;
		if (slowdown_speed < chassis_param.line_min_speed_mps)
			slowdown_speed = chassis_param.line_min_speed_mps;
		if (slowdown_speed > abs_speed)
			slowdown_speed = abs_speed;
		if (fabsf(slowdown_speed) < fabsf(base_speed))
			base_speed = (remain > 0.0f) ? slowdown_speed : -slowdown_speed;
	}

	yaw_error = Chassis_AngleNormalize(chassis_action_target_yaw - Chassis_GetYawDeg());
	yaw_compensation = PID_calc(&chassis_line_yaw_pid, 0.0f, yaw_error);

	DC_Motor_SetRef(motor_l, base_speed - 3.0f*yaw_compensation);
	DC_Motor_SetRef(motor_r, base_speed + 3.0f*yaw_compensation);

	return CHASSIS_ACTION_RUNNING;
}

/**
 * @brief 使用 IMU yaw 归一化角度，原地转向指定相对角度
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

/**
 * @brief 走半圆曲线: 前进的同时连续转弯 180°, 轨迹为半径 R 的半圆
 * @param radius_m   半圆半径, 单位 m
 * @param speed_mps  前进线速度, 单位 m/s
 * @param direction  转向方向: +1 = 右转(顺时针), -1 = 左转(逆时针)
 * @return CHASSIS_ACTION_DONE 完成, 否则 CHASSIS_ACTION_RUNNING
 *
 * 工作原理:
 *   1. 弧长 = π × R, yaw 变化 = 180°
 *   2. 根据已走距离, 线性插值期望 yaw
 *   3. 转向 PID 跟踪期望 yaw, 产生差速
 *   4. 前进距离走完 + yaw 到位 → 完成
 *
 *                ┌──────────┐
 *                │ 直线1.5m │
 *                └──────────┘
 *               ↗           ↘
 *   半圆 R=0.5m              半圆 R=0.5m
 *   右转 180°                右转 180°
 *               ↖           ↙
 *                ┌──────────┐
 *                │ 直线1.5m │
 *                └──────────┘
 */
float yaw_error;
float turn_compensation;
static uint8_t Chassis_SemiCircle(float radius_m, float speed_mps, int direction)
{
	const float arc_length     = 3.1415926f * radius_m;
	const float total_yaw_deg  = 180.0f * (float)direction;
	float abs_speed = fabsf(speed_mps);

	float progress, remain, base_speed;

	if (fabsf(radius_m) < 0.001f) return CHASSIS_ACTION_DONE;

	/* 首次调用: 锁定目标距离和目标 yaw */
	if ((chassis_action_active == 0U) || (chassis_action_type != 3U))
	{
		chassis_action_active = 1U;
		chassis_action_type   = 3U;  /* type 3 = 半圆 */
		chassis_action_done_count = 0U;
		chassis_action_target_distance = Chassis_GetForwardOdom() + arc_length;
		chassis_action_target_yaw = Chassis_AngleNormalize(
			Chassis_GetYawDeg() + total_yaw_deg);

		Chassis_ResetEncoderOdom();
		PID_clear(&chassis_turn_pid);
	}

	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

	progress = Chassis_GetForwardOdom();
	remain   = chassis_action_target_distance - progress;
	yaw_error = Chassis_AngleNormalize(
		chassis_action_target_yaw - Chassis_GetYawDeg());

	/* 完成判定: 距离走完 + yaw 偏差小于阈值 */
	if (fabsf(remain) <= chassis_param.line_done_err_m &&
	    fabsf(yaw_error) <= chassis_param.turn_done_err_deg)
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

	/* ── 路径级梯形加减速 ── */
	float path_progress = imu_path_cumulative + progress;       /* 整条路径已走距离 */
	float path_remain   = IMU_PATH_TOTAL_M - path_progress;     /* 整条路径剩余距离 */

	base_speed = abs_speed;

	/* 起步段加速 */
	if (path_progress < IMU_PATH_ACCEL_M)
	{
		base_speed = chassis_param.line_min_speed_mps +
			(abs_speed - chassis_param.line_min_speed_mps) *
				path_progress / IMU_PATH_ACCEL_M;
	}
	/* 末尾段减速 */
	else if (path_remain < IMU_PATH_SLOWDOWN_M)
	{
		float sd = abs_speed * path_remain / IMU_PATH_SLOWDOWN_M;
		if (sd < chassis_param.line_min_speed_mps)
			sd = chassis_param.line_min_speed_mps;
		if (sd < base_speed) base_speed = sd;
	}

	/* 线性插值期望 yaw: progress/arc_length → 0%~100% → yaw 从 0 到 180° */
	float ratio = (arc_length > 0.001f) ? (progress / arc_length) : 0.0f;
	if (ratio > 1.0f) ratio = 1.0f;
	float expected_yaw = Chassis_AngleNormalize(
		Chassis_AngleNormalize(chassis_action_target_yaw - total_yaw_deg) +
		total_yaw_deg * ratio);
	yaw_error = Chassis_AngleNormalize(expected_yaw - Chassis_GetYawDeg());

	/* 转向 PID, 限幅防止翻车 */
	float turn_limit = abs_speed * 6.6f;
	if (turn_limit < 0.1f) turn_limit = 0.1f;
	chassis_turn_pid.max_out = turn_limit;
	turn_compensation = PID_calc(&chassis_turn_pid, 0.0f, yaw_error);

	DC_Motor_SetRef(motor_l, base_speed + turn_compensation);
	DC_Motor_SetRef(motor_r, base_speed - turn_compensation);

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
 * @brief 获取 IMU解算出的 yaw 角
 * @return yaw 角度，单位 deg
 */
static float Chassis_GetYawDeg(void)
{
	// Chassis() 每周期调用 IMU_getYawPitchRoll()，IMU_data[0] 为 ICM42688 yaw，单位 deg。
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
	motor_l->position_pid.max_out = 0.03f;
	motor_l->position_pid.max_iout = 0.0f;
	motor_r->position_pid.max_out = 0.03f;
	motor_r->position_pid.max_iout = 0.0f;

	DC_Motor_SetRef(motor_l, position);
	DC_Motor_SetRef(motor_r, position);
}

void Chassis_State_Turn(void)
{

	// static uint8_t quan=0;
	DC_Motor_SetRef(motor_l, 0.05f);
	DC_Motor_SetRef(motor_r, 0.05f);
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
}
static uint8_t testtt;
void Motor_Cmd_CallBack(uint8_t i)
{
	if(i ==0)
	{
		testtt++;
		DCMotor_Cmd(motor_l,ENABLE);
		DCMotor_Cmd(motor_r,ENABLE);
	}
	else if(i == 1)
	{
		DCMotor_Cmd(motor_l,DISABLE);
		DCMotor_Cmd(motor_r,DISABLE);
	}

}