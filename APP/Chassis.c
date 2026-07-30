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
#include "../BSP/Motor/Servo.h"

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
#define CHASSIS_LINE_DONE_ERR_M      0.005f
#define CHASSIS_LINE_DONE_TICKS      10U
#define CHASSIS_LINE_ACCEL_M         0.20f
#define CHASSIS_LINE_SLOWDOWN_M      0.25f
#define CHASSIS_LINE_MIN_SPEED       0.025f
#define CHASSIS_TURN_DONE_ERR_DEG    3.0f
#define CHASSIS_TURN_DONE_TICKS      10U
#define CHASSIS_ACTION_SPEED		 0.31f
#define CHASSIS_ACTION_TURN_SPEED	 0.16f

/* ---- 2026h 体育场赛道参数 ---- */
#define CHASSIS_ARC_SPEED             0.20f   /* 弧线跟随线速度 (m/s) */
#define CHASSIS_STRAIGHT_DIST_M       1.50f   /* 直线段距离 (m) */
#define CHASSIS_ARC_RADIUS_M          0.50f   /* 半圆半径 (m) */
#define CHASSIS_ARC_ANGLE_DEG         180.0f  /* 半圆总转角 (deg) */
#define CHASSIS_FINAL_APPROACH_SPEED  0.08f   /* 终点逼近慢速 (m/s) */
#define CHASSIS_FINAL_MAX_DIST_M      0.50f   /* 终点逼近最大搜索距离 (m) */
#define CHASSIS_START_LINE_SENSORS    3U      /* 起止线最少连续探头数 */
#define CHASSIS_LINE_DETECT_DEBOUNCE  5U      /* 起止线检测消抖周期数 */
#define CHASSIS_ACTION_ARC            3U      /* 弧线动作类型 ID */
#define CHASSIS_STOP_DONE_STEP        99U     /* 停车完成步骤号 */
#define CHASSIS_PI                    3.14159265f

static pid_type_def chassis_line_yaw_pid;
static pid_type_def chassis_turn_pid;
static Chassis_Mode_e chassis_last_mode = NORMAL_MODE;
static uint8_t chassis_action_active = 0U;
static uint8_t chassis_action_type = 0U;
static uint8_t chassis_action_done_count = 0U;
static float chassis_action_target_yaw = 0.0f;
static float chassis_action_target_distance = 0.0f;
static uint8_t chassis_imu_action_step = 0U;

/* 弧线跟随专用状态 */
static float chassis_arc_start_yaw = 0.0f;             /* 弧线起点的 IMU yaw (deg) */
static float chassis_arc_total_distance = 0.0f;        /* 弧线总路径长度 (m) */

/* 终点起止线检测状态 */
static uint8_t chassis_stop_line_detect_count = 0U;    /* 连续检测到起止线的周期计数 */
static uint8_t chassis_step4_initialized = 0U;         /* 终点逼近是否已初始化 */
static float   chassis_step4_target_yaw = 0.0f;        /* 终点逼近保持方向 (deg) */

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
 * @brief 初始化底盘左右电机和 IMU
 */
void Chassis_Init(void)
{
	DCMotorInitConfig_s motor_l_config ={
		.Input_Dir =  MOTOR_NORMAL,
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
			.Kp = 600.0f,
			.Ki = 100.0f,
			.Kd = 0.0f,
			.max_out = 2499.0f,
			.max_iout = 500.0f,
			.kf_p = 1800.0f,
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
			.Kp = 600.0f,
			.Ki = 100.0f,
			.Kd = 0.0f,
			.max_out = 2499.0f,
			.max_iout = 500.0f,
			.kf_p = 1800.0f,
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


	pid_init_config_s line_yaw_pid_config = {
		.mode = PID_POSITION,
		.Kp = 0.0012f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.kf_p = 1000,//2000
		.kf_v = 1000,
		.max_out = 0.2f,
		.max_iout = 0.0f,
		.deadzone = 0.5f,
	};
	PID_init(&chassis_line_yaw_pid, &line_yaw_pid_config);

	pid_init_config_s turn_pid_config = {
		.mode = PID_POSITION,
		.Kp = 0.0025f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.kf_p = 2000,
		.kf_v = 2000,
		.max_out = 0.2f,
		.max_iout = 0.0f,
		.deadzone = 0.5f,
	};
	PID_init(&chassis_turn_pid, &turn_pid_config);
	DWT_Delay(1);

	trace_mode=TRACE_NORMAL;
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
			Chassis_Trace_Cal();
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
 * @brief IMU_MODE 下的 2026h 体育场赛道动作
 *
 * 赛道形状（体育场/田径跑道形）：
 *   A —— 1.5m 直线 —— B
 *                        \
 *                        (  半径 0.5m 半圆 (B→C, 180°)
 *                        /
 *   D —— 1.5m 直线 —— C
 *    \
 *    (  半径 0.5m 半圆 (D→A, 180°)
 *    /
 *   A (起止线)
 *
 * 状态机步骤：
 *   Step 0: A→B 直线 1.5m（前半程循迹，后半程 IMU 保向）
 *   Step 1: B→C 弧线（半径 0.5m, +180°）
 *   Step 2: C→D 直线 1.5m（前半程循迹，后半程 IMU 保向）
 *   Step 3: D→A 弧线（半径 0.5m, +180°）
 *   Step 4: 终点逼近 → 检测起止线(≥3 连续探头) → 停车(误差≤2cm)
 *   Step 99: 停车完成，保持静止
 */
static void Chassis_ImuModeAction(void)
{
	float odom;
	float yaw_error;
	float yaw_compensation;

	switch (chassis_imu_action_step)
	{
		/* ================================================================
		 * Step 0: A → B  直线 1.5m
		 *  - 前 1.2m：循迹 + IMU 保向
		 *  - 1.2m 后：纯 IMU 保向
		 *  - 0.7~0.8m 窗口记录 yaw 基准角
		 * ================================================================ */
		case 0:
			if (Chassis_MoveStraight(CHASSIS_STRAIGHT_DIST_M, CHASSIS_ACTION_SPEED) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 1U;
			}
			else
			{
				if (Chassis_GetForwardOdom() > 1.2f)
				{
					/* 后段：停止循迹，纯 IMU 保向 */
					Trace_ResetLineError();
					DCMotor_SetTraceCompensation(motor_l, 0.0f);
					DCMotor_SetTraceCompensation(motor_r, 0.0f);
				}
				else
				{
					/* 前段：循迹辅助修正横向偏差 */
					Chassis_Trace_Cal();
					odom = Chassis_GetForwardOdom();
					if (odom > 0.7f && odom < 0.8f)
					{
						/* 车身稳定后记录当前 yaw 作为后续 IMU 保向的参考基准 */
						chassis_action_target_yaw = Chassis_GetYawDeg();
					}
				}
			}
			break;

		/* ================================================================
		 * Step 1: B → C  半径 0.5m 半圆 (弧长 ≈ 1.57m, +180°)
		 *  - 编码器里程计 + IMU yaw 联合控制弧线跟随
		 *  - 左转弯（CCW），半径为正
		 * ================================================================ */
		case 1:
			if (Chassis_MoveArc(CHASSIS_ARC_RADIUS_M, CHASSIS_ARC_ANGLE_DEG, CHASSIS_ARC_SPEED) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 2U;
			}
			break;

		/* ================================================================
		 * Step 2: C → D  直线 1.5m
		 *  - 前 0.7m：循迹辅助修正弧线结束后的横向偏差
		 *  - 0.7m 后：纯 IMU 保向
		 *  - 0.4~0.5m 窗口记录 yaw 基准角
		 * ================================================================ */
		case 2:
			if (Chassis_MoveStraight(CHASSIS_STRAIGHT_DIST_M, CHASSIS_ACTION_SPEED) == CHASSIS_ACTION_DONE)
			{
				chassis_imu_action_step = 3U;
			}
			else
			{
				odom = Chassis_GetForwardOdom();
				if (odom > 0.7f)
				{
					Trace_ResetLineError();
					DCMotor_SetTraceCompensation(motor_l, 0.0f);
					DCMotor_SetTraceCompensation(motor_r, 0.0f);
				}
				else
				{
					Chassis_Trace_Cal();
					if (odom > 0.4f && odom < 0.5f)
					{
						chassis_action_target_yaw = Chassis_GetYawDeg();
					}
				}
			}
			break;

		/* ================================================================
		 * Step 3: D → A  半径 0.5m 半圆 (弧长 ≈ 1.57m, +180°)
		 *  - 与 Step 1 同向的弧线，完成后应回到 A 点附近
		 * ================================================================ */
		case 3:
			if (Chassis_MoveArc(CHASSIS_ARC_RADIUS_M, CHASSIS_ARC_ANGLE_DEG, CHASSIS_ARC_SPEED) == CHASSIS_ACTION_DONE)
			{
				/* 弧线完成后进入终点逼近阶段，初始化状态 */
				chassis_step4_initialized = 0U;
				chassis_stop_line_detect_count = 0U;
				chassis_imu_action_step = 4U;
			}
			break;

		/* ================================================================
		 * Step 4: 终点逼近 → 检测起止线 → 精确停车
		 *
		 * 策略：
		 *   1. 慢速前进（0.08 m/s），IMU 保向
		 *   2. 每周期读取 8 路灰度探头原始数据
		 *   3. 连续 ≥3 个探头踩到黑线 → 判定为起止线
		 *   4. 消抖：连续 5 周期检测到后才确认停车
		 *   5. 安全兜底：前进超过 0.5m 仍未检测到则强制停车
		 *
		 * 起止线说明：
		 *   A 点有一条垂直于前进方向的黑线（起止线）。
		 *   车体经过时，至少 3 个连续的灰度探头会同时检测到黑线，
		 *   与普通循迹线（约 1~2 个探头）有显著区别。
		 * ================================================================ */
		case 4:
		{
			/* 首周期初始化 */
			if (chassis_step4_initialized == 0U)
			{
				chassis_step4_initialized = 1U;
				chassis_stop_line_detect_count = 0U;
				chassis_step4_target_yaw = Chassis_GetYawDeg();
				Chassis_ResetEncoderOdom();
				PID_clear(&chassis_line_yaw_pid);
				DCMotor_SetTraceCompensation(motor_l, 0.0f);
				DCMotor_SetTraceCompensation(motor_r, 0.0f);
			}

			motor_l->loop_mode = SPEED_MODE;
			motor_r->loop_mode = SPEED_MODE;
			motor_l->State = ENABLE;
			motor_r->State = ENABLE;

			odom = Chassis_GetForwardOdom();

			/* ---- 起止线检测（核心逻辑）---- */
			if (Trace_DetectCrossLine(CHASSIS_START_LINE_SENSORS))
			{
				chassis_stop_line_detect_count++;
				if (chassis_stop_line_detect_count >= CHASSIS_LINE_DETECT_DEBOUNCE)
				{
					/* 确认检测到起止线：立即停车 */
					DC_Motor_SetRef(motor_l, 0.0f);
					DC_Motor_SetRef(motor_r, 0.0f);
					Chassis_ResetAction();
					chassis_step4_initialized = 0U;
					chassis_stop_line_detect_count = 0U;
					chassis_imu_action_step = CHASSIS_STOP_DONE_STEP;
					break;
				}
			}
			else
			{
				chassis_stop_line_detect_count = 0U;
			}

			/* ---- 安全兜底：超过最大搜索距离 ---- */
			if (odom > CHASSIS_FINAL_MAX_DIST_M)
			{
				DC_Motor_SetRef(motor_l, 0.0f);
				DC_Motor_SetRef(motor_r, 0.0f);
				Chassis_ResetAction();
				chassis_step4_initialized = 0U;
				chassis_stop_line_detect_count = 0U;
				chassis_imu_action_step = CHASSIS_STOP_DONE_STEP;
				break;
			}

			/* ---- 慢速前进 + IMU yaw 保向 ---- */
			yaw_error = Chassis_AngleNormalize(chassis_step4_target_yaw - Chassis_GetYawDeg());
			yaw_compensation = PID_calc(&chassis_line_yaw_pid, 0.0f, yaw_error);

			DC_Motor_SetRef(motor_l, CHASSIS_FINAL_APPROACH_SPEED + yaw_compensation);
			DC_Motor_SetRef(motor_r, CHASSIS_FINAL_APPROACH_SPEED - yaw_compensation);
		}
		break;

		/* ================================================================
		 * Step 99: 停车完成，保持静止
		 *  - 等待上位机切出 IMU_MODE 后复位
		 * ================================================================ */
		case CHASSIS_STOP_DONE_STEP:
			/* 静默保持，不做任何动作 */
			break;

		default:
			/* 异常状态：关闭电机输出，复位到起点 */
			DC_Motor_SetRef(motor_l, 0.0f);
			DC_Motor_SetRef(motor_r, 0.0f);
			chassis_imu_action_step = 0U;
			chassis_step4_initialized = 0U;
			chassis_stop_line_detect_count = 0U;
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
 *
 * @note 清除层级递进：
 *       1. 底盘级 PID（chassis_line_yaw_pid / chassis_turn_pid）
 *       2. 电机自身 PID（speed_pid / position_pid）— 防止模式切换时积分卷绕
 *       3. 循迹补偿量（Trace_Compensation）— 防止上一模式残留叠加
 *       4. 弧线/终点检测状态 — 防止跨模式残留
 */
void Chassis_ResetAction(void)
{
	// 清除当前自动动作状态；下次调用直线/转向函数时会重新锁定目标。
	chassis_action_active = 0U;
	chassis_action_type = 0U;
	chassis_action_done_count = 0U;
	chassis_action_target_yaw = 0.0f;
	chassis_action_target_distance = 0.0f;

	// 清除底盘级 PID 历史，避免上一次动作的误差和微分项影响下一次动作。
	PID_clear(&chassis_line_yaw_pid);
	PID_clear(&chassis_turn_pid);


    /* ---- 清除电机自身 PID 状态 ---- */
	/* 关键：模式切换（如 TRACE_MODE 的 ANGLE_MODE → IMU_MODE 的 SPEED_MODE）
	 * 时，电机 speed_pid 内残留的积分/微分/误差历史会导致输出突变。
	 * 不清除的话，Hw_Motor_Task 里 speed_pid.out 会基于上一模式的旧状态
	 * 加上新模式的新 Ref，产生不可预期的输出，表现为轮子乱转/振荡。 */
	PID_clear(&motor_l->speed_pid);
	PID_clear(&motor_r->speed_pid);
	PID_clear(&motor_l->position_pid);
	PID_clear(&motor_r->position_pid);

	/* ---- 清除循迹补偿量 ---- */
	/* Hw_Motor_Task 中 SPEED_MODE 的实际参考值 = Trace_Compensation + speed_pid.Ref。
	 * 如果上一模式残留非零 Trace_Compensation，会叠加到新 Ref 上造成偏差。 */
	DCMotor_SetTraceCompensation(motor_l, 0.0f);
	DCMotor_SetTraceCompensation(motor_r, 0.0f);

	// 清零速度给定，但不直接失能电机，方便状态机继续接管底盘。
	DC_Motor_SetRef(motor_l, 0.0f);
	DC_Motor_SetRef(motor_r, 0.0f);

	/* ---- 清除弧线跟随和终点检测状态 ---- */
	chassis_arc_start_yaw = 0.0f;
	chassis_arc_total_distance = 0.0f;
	chassis_step4_initialized = 0U;
	chassis_stop_line_detect_count = 0U;
	chassis_step4_target_yaw = 0.0f;
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

	if (abs_distance <= CHASSIS_LINE_DONE_ERR_M)
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
	if (fabsf(remain) <= CHASSIS_LINE_DONE_ERR_M)
	{
		chassis_action_done_count++;
		DC_Motor_SetRef(motor_l, 0.0f);
		DC_Motor_SetRef(motor_r, 0.0f);
		if (chassis_action_done_count >= CHASSIS_LINE_DONE_TICKS)
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
	if (progress < CHASSIS_LINE_ACCEL_M)
	{
		float accel_speed = CHASSIS_LINE_MIN_SPEED +
		                    (abs_speed - CHASSIS_LINE_MIN_SPEED) * progress / CHASSIS_LINE_ACCEL_M;

		/*
		 * 起步前 0.1m 线性加速，避免一进入直线动作就给满速度。
		 */
		base_speed = (remain > 0.0f) ? accel_speed : -accel_speed;
	}
	if (fabsf(remain) < CHASSIS_LINE_SLOWDOWN_M)
	{
		float slowdown_speed = abs_speed * fabsf(remain) / CHASSIS_LINE_SLOWDOWN_M;

		/*
		 * 直线末段提前按剩余距离线性降速。
		 * 原逻辑只剩约 2.4cm 才减速，实际表现接近急停。
		 */
		if (slowdown_speed < CHASSIS_LINE_MIN_SPEED)
		{
			slowdown_speed = CHASSIS_LINE_MIN_SPEED;
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
	DCMotor_SetTraceCompensation(motor_l,0);//todo:这个要不要删除
	DCMotor_SetTraceCompensation(motor_r,0);
	float abs_turn_speed = fabsf(max_turn_speed);
	float yaw_error;
	float turn_speed;

	if (fabsf(angle_deg) <= CHASSIS_TURN_DONE_ERR_DEG)
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
	if (fabsf(yaw_error) <= CHASSIS_TURN_DONE_ERR_DEG)
	{
		chassis_action_done_count++;
		DC_Motor_SetRef(motor_l, 0.0f);
		DC_Motor_SetRef(motor_r, 0.0f);
		if (chassis_action_done_count >= CHASSIS_TURN_DONE_TICKS)
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

/**
 * @brief 使用编码器里程计 + IMU yaw 联合控制，沿定半径圆弧前进
 *
 * 核心原理（差速轮弧线跟随）：
 *   对于半径 R 的圆弧，机器人沿弧线前进距离 d 时：
 *     yaw 变化量 = d / R (弧度) = d / R * 180/π (度)
 *
 *   每个控制周期根据当前里程动态计算目标 yaw：
 *     target_yaw_deg = arc_start_yaw + direction * (current_odom / |R|) * 180/π
 *
 *   前进速度用 IMU yaw PID 分解到左右轮，实现"边前进边转弯"。
 *
 * 与 Chassis_TurnAngle（原地转向）的本质区别：
 *   - TurnAngle：线速度 = 0，纯旋转，用 chassis_turn_pid
 *   - MoveArc：始终有前进速度，yaw 动态追赶，用 chassis_line_yaw_pid
 *
 * @param radius_m       圆弧半径 (m)，正数 = 逆时针(CCW)，负数 = 顺时针(CW)
 * @param total_angle_deg 圆弧总转角 (deg)，如 +180 表示半圆，-90 表示右转四分之一圆
 * @param speed_mps      前进线速度 (m/s)，取绝对值
 * @return CHASSIS_ACTION_DONE 表示弧线完成，否则返回 CHASSIS_ACTION_RUNNING
 */
uint8_t Chassis_MoveArc(float radius_m, float total_angle_deg, float speed_mps)
{
	/* 切线速度只取大小，方向由 radius_m 符号和 total_angle_deg 符号共同决定 */
	float abs_speed = fabsf(speed_mps);
	if (abs_speed < 0.001f) { abs_speed = 0.001f; }

	/* 计算弧线总长：arc_length = |R| * |θ| (弧度) */
	float abs_radius   = fabsf(radius_m);
	float abs_angle    = fabsf(total_angle_deg);
	float total_arc_distance = abs_radius * abs_angle * (CHASSIS_PI / 180.0f);

	/* 转向方向：radius > 0 且 angle > 0 → 逆时针 (CCW = +1)
	 *           radius > 0 且 angle < 0 → 顺时针 (CW  = -1)
	 * 取两者符号的乘积作为最终转动方向 */
	int turn_dir = ((radius_m > 0.0f) ? 1 : -1) * ((total_angle_deg > 0.0f) ? 1 : -1);

	/* 转角太小，无需执行 */
	if (abs_angle <= CHASSIS_TURN_DONE_ERR_DEG || total_arc_distance <= CHASSIS_LINE_DONE_ERR_M)
	{
		Chassis_ResetAction();
		return CHASSIS_ACTION_DONE;
	}

	/* ---- 首次调用：锁定弧线起点状态 ---- */
	if ((chassis_action_active == 0U) || (chassis_action_type != CHASSIS_ACTION_ARC))
	{
		chassis_action_active      = 1U;
		chassis_action_type        = CHASSIS_ACTION_ARC;
		chassis_action_done_count  = 0U;
		chassis_arc_start_yaw      = Chassis_GetYawDeg();
		chassis_arc_total_distance = total_arc_distance;

		Chassis_ResetEncoderOdom();
		PID_clear(&chassis_line_yaw_pid);
		PID_clear(&chassis_turn_pid);

		/* 清空循迹补偿，弧线完全由 IMU 控制 */
		DCMotor_SetTraceCompensation(motor_l, 0.0f);
		DCMotor_SetTraceCompensation(motor_r, 0.0f);
	}

	/* ---- 每周期：速度环 + yaw 保向 ---- */
	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

	float current_distance = fabsf(Chassis_GetForwardOdom());
	float remain = chassis_arc_total_distance - current_distance;

	/* ---- 到达终点判定 ---- */
	if (remain <= CHASSIS_LINE_DONE_ERR_M)
	{
		chassis_action_done_count++;
		DC_Motor_SetRef(motor_l, 0.0f);
		DC_Motor_SetRef(motor_r, 0.0f);
		if (chassis_action_done_count >= CHASSIS_LINE_DONE_TICKS)
		{
			Chassis_ResetAction();
			return CHASSIS_ACTION_DONE;
		}
		return CHASSIS_ACTION_RUNNING;
	}
	chassis_action_done_count = 0U;

	/* ---- 动态目标 yaw：根据已走弧长线性内插 ---- */
	/* target_yaw = start_yaw + turn_dir * (distance / radius) * (180 / PI) */
	float target_yaw = chassis_arc_start_yaw
	                   + (float)turn_dir * current_distance / abs_radius * (180.0f / CHASSIS_PI);
	target_yaw = Chassis_AngleNormalize(target_yaw);

	/* ---- 速度曲线：起步加速 + 末段减速 ---- */
	float progress = chassis_arc_total_distance - remain;
	float base_speed = abs_speed;

	if (progress < CHASSIS_LINE_ACCEL_M)
	{
		/* 起步加速段 */
		base_speed = CHASSIS_LINE_MIN_SPEED
		             + (abs_speed - CHASSIS_LINE_MIN_SPEED) * progress / CHASSIS_LINE_ACCEL_M;
	}
	if (remain < CHASSIS_LINE_SLOWDOWN_M)
	{
		/* 末段减速段 */
		float slowdown = abs_speed * remain / CHASSIS_LINE_SLOWDOWN_M;
		if (slowdown < CHASSIS_LINE_MIN_SPEED) { slowdown = CHASSIS_LINE_MIN_SPEED; }
		if (slowdown < base_speed)             { base_speed = slowdown; }
	}

	/* ---- IMU yaw 误差 → 补偿 ---- */
	float yaw_error = Chassis_AngleNormalize(target_yaw - Chassis_GetYawDeg());
	float yaw_compensation = PID_calc(&chassis_line_yaw_pid, 0.0f, yaw_error);

	/* ---- 左右轮速度分配：base_speed ± yaw_compensation ---- */
	DC_Motor_SetRef(motor_l, base_speed + yaw_compensation);
	DC_Motor_SetRef(motor_r, base_speed - yaw_compensation);

	return CHASSIS_ACTION_RUNNING;
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

	DC_Motor_SetRef(motor_l, -0.1);
	DC_Motor_SetRef(motor_r, 0.1);
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
				Chassis_Set_Line(0.8);
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
				Chassis_Set_Line(0.82);
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
