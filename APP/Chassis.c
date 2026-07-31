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
#include "bsp_beep.h"
#include "../BSP/Motor/Servo.h"
#include "K230.h"

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
#define CHASSIS_ACTION_SPEED         0.31f
#define CHASSIS_ACTION_TURN_SPEED    0.16f

#define CHASSIS_ACTION_ARC            3U      /* 弧线动作类型 ID */
#define CHASSIS_STOP_DONE_STEP        99U     /* 停车完成步骤号 */
#define CHASSIS_ARC_DONE_YAW_ERR_DEG  3.0f

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

/* 体育场连续轨迹控制状态，同时作为滚球控制的底盘运动前馈。 */
static Chassis_Stadium_Step_e chassis_stadium_step = CHASSIS_STADIUM_STRAIGHT_1; /* 当前所在AB/BC/CD/DA步骤 */
static uint8_t chassis_stadium_initialized = 0U;       /* 0：本圈尚未初始化；1：状态机正在运行 */
static uint8_t chassis_stadium_finish_line_latched = 0U; /* 1：已确认A横线，之后不再重复检测 */
static float chassis_stadium_finish_line_odom = 0.0f; /* 探头确认A横线瞬间的编码器前向里程(m) */
static float chassis_stadium_segment_start_odom = 0.0f; /* 当前直线/圆弧开始时的前向里程(m) */
static float chassis_stadium_segment_start_yaw = 0.0f;  /* 当前段开始时的IMU航向(deg)，用于调试记录 */
static float chassis_stadium_initial_yaw = 0.0f;      /* 上电从A出发时的绝对航向；最终停车也对准它 */
static float chassis_stadium_speed_cmd = 0.0f;        /* jerk规划后的车体中心速度指令(m/s) */
static float chassis_stadium_accel_cmd = 0.0f;        /* jerk规划器当前加速度状态(m/s^2) */
static float chassis_stadium_curvature_cmd = 0.0f;    /* 当前下发曲率(1/m)，供滚球前馈与调试读取 */
static H_Task_e chassis_stadium_task = H_TASK_NONE;   /* 本圈执行的H题任务编号，决定速度/加速度参数 */
static uint8_t chassis_last_task_start_seq = 0U;      /* 最近一次启动序号，序号变化即可重新跑一圈 */
static volatile uint8_t chassis_emergency_stop_requested = 0U; /* KEY4等异步来源置1，请求立即停机 */
static float chassis_stadium_start_time_s = 0.0f;     /* 本圈实际开始的DWT时间戳(s) */
static float chassis_stadium_elapsed_s = 0.0f;        /* 已用时间(s)，停车后冻结用于显示 */
static uint8_t chassis_stadium_timer_running = 0U;    /* 1：计时中；0：尚未开始或已经停车 */
static uint8_t chassis_finish_beep_mode = 0U;         /* 0关闭，1成功两声，2漏检持续鸣叫 */
static uint8_t chassis_finish_beep_count = 0U;        /* 成功提示已经完成的鸣叫次数 */
static uint8_t chassis_finish_beep_level = 0U;        /* 当前蜂鸣器输出状态：0关闭，1打开 */
static float chassis_finish_beep_change_time = 0.0f;  /* 下一次切换蜂鸣器状态的时间戳(s) */

static uint8_t chassis_task3_init_done = 0U;          /* 任务3滚球的初始状态机是否已经完成初始化 */

float IMU_data[3] = {0};
volatile JY901s_IMU_Data_s* JY901s_IMU_Data;

void Stop_Detect(void);
void Chassis_State_Turn(void);
void Chassis_Set_Turn(void);
static void Chassis_RemoteControl(void);
static void Chassis_ClearRemoteSpeed(void);
static void Chassis_RemoteLostDisable(void);
static void Chassis_Trace_Cal(void);
static void Chassis_StadiumControl(void);
static void Chassis_ResetEncoderOdom(void);
static float Chassis_GetForwardOdom(void);
static float Chassis_GetYawDeg(void);
static float Chassis_AngleNormalize(float angle);
static float Chassis_StadiumUpdateSpeed(float target_speed);
static void Chassis_StadiumSetDrive(float center_speed, float curvature, float target_yaw);
static void Chassis_StadiumEnterStep(Chassis_Stadium_Step_e next_step);
static void Chassis_StadiumDetectFinishLine(void);
static uint8_t Chassis_StadiumStopCenterAtFinish(float curvature, float target_yaw,
                                                 float speed_limit);
static void Chassis_StadiumStartFinishBeep(uint8_t detected);
static void Chassis_StadiumUpdateFinishBeep(void);
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
		/* 航向反馈只修正模型误差，不能大到反客为主改变圆弧方向。 */
		.max_out = CHASSIS_YAW_FEEDBACK_MAX_MPS,
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

	/* 跑道循迹需要在短时丢线时保持最后方向，而不是把丢线当作居中。 */
	trace_mode=TRACE_LOST_DETECT;
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

	/* 启动序号变化表示再次按下任务确认键，同一任务也要从头运行。 */
	if (chassis_cmd_receive.task_start_seq != chassis_last_task_start_seq)
	{
		chassis_last_task_start_seq = chassis_cmd_receive.task_start_seq;
		chassis_emergency_stop_requested = 0U;
		chassis_task3_init_done = 0U;
		Chassis_ResetAction();
		chassis_imu_action_step = 0U;
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
			if (chassis_cmd_receive.competition_task == H_TASK_2_FAST_LAP
				|| chassis_cmd_receive.competition_task == H_TASK_4_AB_BALL
				|| chassis_cmd_receive.competition_task == H_TASK_5_CENTER_BALL_LAP
				|| chassis_cmd_receive.competition_task == H_TASK_6_TARGET_BALL_LAP){
				if (chassis_emergency_stop_requested != 0U)
				{
					chassis_stadium_elapsed_s = DWT_GetTimeline_s() - chassis_stadium_start_time_s;
					chassis_stadium_timer_running = 0U;
					chassis_stadium_step = CHASSIS_STADIUM_STOP;
					DCMotor_SetTraceCompensation(motor_l, 0.0f);
					DCMotor_SetTraceCompensation(motor_r, 0.0f);
					DC_Motor_SetRef(motor_l, 0.0f);
					DC_Motor_SetRef(motor_r, 0.0f);
					DCMotor_Cmd(motor_l, DISABLE);
					DCMotor_Cmd(motor_r, DISABLE);
					return;
				}
				if (IMU_Mahony_IsReady() != 0U)
				{
					Chassis_StadiumControl();
				}
				else
				{
					DCMotor_SetTraceCompensation(motor_l, 0.0f);
					DCMotor_SetTraceCompensation(motor_r, 0.0f);
					DC_Motor_SetRef(motor_l, 0.0f);
					DC_Motor_SetRef(motor_r, 0.0f);
					DCMotor_Cmd(motor_l, DISABLE);
					DCMotor_Cmd(motor_r, DISABLE);
					return;
				}
			}
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
 * @brief 平滑更新体育场轨迹的中心速度
 *
 * 先限制目标加速度的变化率（jerk），再积分得到速度。这样起步、直弯切换和
 * 终点制动都不会突然改变车体加速度，可显著减小钢球受到的惯性冲击。
 */
static float Chassis_StadiumUpdateSpeed(float target_speed)
{
	/* 根据当前任务选择加速度、减速度和加加速度上限。 */
	float max_accel;
	float max_decel;
	float max_jerk;
	if (chassis_stadium_task == H_TASK_2_FAST_LAP)
	{
		max_accel = CHASSIS_FAST_MAX_ACCEL;
		max_decel = CHASSIS_FAST_MAX_DECEL;
		max_jerk = CHASSIS_FAST_MAX_JERK;
	}
	else
	{
		max_accel = CHASSIS_STADIUM_MAX_ACCEL;
		max_decel = CHASSIS_STADIUM_MAX_DECEL;
		max_jerk = CHASSIS_STADIUM_MAX_JERK;
	}
	/* 根据目标速度和当前速度计算理想加速度。 */
	float desired_accel = (target_speed - chassis_stadium_speed_cmd) /
	                      CHASSIS_STADIUM_CONTROL_DT;
	/* 限制理想加速度，防止电机突然加速或制动。 */
	limit_max_min(desired_accel, max_accel, -max_decel);

	/* 计算本周期允许的最大加速度变化量。 */
	float max_accel_step = max_jerk * CHASSIS_STADIUM_CONTROL_DT;
	/* 只允许加速度按jerk限制变化。 */
	float accel_delta = desired_accel - chassis_stadium_accel_cmd;
	limit_max_min(accel_delta, max_accel_step, -max_accel_step);
	/* 保存新的加速度状态。 */
	chassis_stadium_accel_cmd += accel_delta;

	/* 保存积分前的速度误差，用于判断是否越过目标速度。 */
	float previous_error = target_speed - chassis_stadium_speed_cmd;
	/* 按控制周期积分得到新的速度。 */
	chassis_stadium_speed_cmd += chassis_stadium_accel_cmd * CHASSIS_STADIUM_CONTROL_DT;
	/* 计算积分后的速度误差。 */
	float current_error = target_speed - chassis_stadium_speed_cmd;
	if ((previous_error * current_error) <= 0.0f)
	{
		/* 发生越过时直接钳位，避免速度在目标值附近来回振荡。 */
		chassis_stadium_speed_cmd = target_speed;
		/* 目标速度已达到，清除加速度状态。 */
		chassis_stadium_accel_cmd = 0.0f;
	}
	/* 返回本周期最终速度指令。 */
	return chassis_stadium_speed_cmd;
}

/**
 * @brief 将中心速度、曲率和目标航向转换为左右轮速度
 * @param center_speed 车体中心线速度，单位 m/s
 * @param curvature 路径曲率；本车实测顺时针转弯使用正值
 * @param target_yaw 当前路径期望航向角，单位 deg
 */
static void Chassis_StadiumSetDrive(float center_speed, float curvature, float target_yaw)
{
	/* 保存曲率，供滚球控制器读取横向加速度前馈。 */
	chassis_stadium_curvature_cmd = curvature;
	/* 确保两个电机工作在速度环。 */
	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	/* 使能两个驱动电机。 */
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

	/*
	 * yaw_error 是规划航向与IMU实测航向的最短角差，范围为[-180, 180]。
	 * 曲率前馈先给出半径0.5m所需的左右轮速差，yaw反馈只修正机械误差、
	 * 轮胎打滑和轮距估计误差。当前底盘顺时针时左轮应更快，但IMU yaw
	 * 递减，所以航向PID输出还需乘以CHASSIS_CLOCKWISE_YAW_SIGN完成坐标转换。
	 */
	/* 计算期望航向与实际航向的最短角度误差。 */
	float yaw_error = Chassis_AngleNormalize(target_yaw - Chassis_GetYawDeg());
	/* 将航向误差转换成左右轮的差速修正量。 */
	float yaw_feedback = CHASSIS_CLOCKWISE_YAW_SIGN *
	                     PID_calc(&chassis_line_yaw_pid, yaw_error, 0.0f);
	/* 计算曲率造成的左右轮速度比例差。 */
	float wheel_ratio = 0.5f * CHASSIS_TRACK_WIDTH_M * curvature;

	/* 左轮速度增加，右轮速度减少，形成当前方向的差速转弯。 */
	DC_Motor_SetRef(motor_l, center_speed * (1.0f + wheel_ratio) + yaw_feedback);
	DC_Motor_SetRef(motor_r, center_speed * (1.0f - wheel_ratio) - yaw_feedback);
}

/** @brief 连续切换体育场轨迹步骤，不停车、不清速度环。 */
static void Chassis_StadiumEnterStep(Chassis_Stadium_Step_e next_step)
{
	/* 保存新的路线状态。 */
	chassis_stadium_step = next_step;
	/* 记录新状态的里程起点。 */
	chassis_stadium_segment_start_odom = Chassis_GetForwardOdom();
	/* 记录新状态的航向起点，便于调试和故障分析。 */
	chassis_stadium_segment_start_yaw = Chassis_GetYawDeg();
	/* 清除上一段的航向PID历史，避免旧误差影响新段。 */
	PID_clear(&chassis_line_yaw_pid);
}

/**
 * @brief 对A点横向黑线进行连续帧消抖，并记录探头过线瞬间的驱动轮里程
 *
 * 横线由前置探头检测。记录此刻里程后，车辆继续行驶“探头到车体中心”
 * 的实测距离，最终保证题目规定的车体中心而不是驱动轮轴线停在A线上。
 */
static void Chassis_StadiumDetectFinishLine(void)
{
	/* 已锁存时不再重复采样，避免重复刷新过线里程。 */
	if (chassis_stadium_finish_line_latched != 0U)
	{
		return;
	}

	/* 读取横线判定结果。 */
	if (Trace_DetectCrossLine(CHASSIS_START_LINE_SENSORS) != 0U)
	{
		/* 累加连续检测次数。 */
		if (++chassis_stop_line_detect_count >= CHASSIS_LINE_DETECT_DEBOUNCE)
		{
			/* 连续确认后锁存终点状态。 */
			chassis_stadium_finish_line_latched = 1U;
			/* 保存探头过线瞬间的编码器里程。 */
			chassis_stadium_finish_line_odom = Chassis_GetForwardOdom();
			/* 只在首次锁存时启动两声成功提示，避免每个周期重复重置提示音。 */
			Chassis_StadiumStartFinishBeep(1U);
		}
	}
	else
	{
		/* 当前帧不是横线，清除连续计数。 */
		chassis_stop_line_detect_count = 0U;
	}
}

/**
 * @brief 启动终点提示音。
 * @param detected 1表示检测到A线，0表示DA弧线走完但漏检A线。
 */
static void Chassis_StadiumStartFinishBeep(uint8_t detected)
{
	/* 保存提示音模式：成功两声，漏检持续鸣叫。 */
	if (detected != 0U)
	{
		chassis_finish_beep_mode = 1U;
	}
	else
	{
		chassis_finish_beep_mode = 2U;
	}
	/* 从关闭状态开始，避免切换状态时蜂鸣器残留。 */
	chassis_finish_beep_count = 0U;
	chassis_finish_beep_level = 0U;
	chassis_finish_beep_change_time = DWT_GetTimeline_s();
	beep_off();
}

/**
 * @brief 非阻塞更新终点蜂鸣器。
 * 成功模式输出两次“响-停”，漏检模式保持一直响，不占用控制周期。
 */
static void Chassis_StadiumUpdateFinishBeep(void)
{
	/* 读取当前时间，所有时间均由DWT提供。 */
	float now = DWT_GetTimeline_s();
	/* 没有提示任务时保持蜂鸣器关闭。 */
	if (chassis_finish_beep_mode == 0U)
	{
		beep_off();
		return;
	}
	/* 漏检模式：持续打开蜂鸣器，直到重新启动任务或复位状态。 */
	if (chassis_finish_beep_mode == 2U)
	{
		beep_on();
		return;
	}
	/* 成功模式尚未到切换时间时保持当前电平。 */
	if (now < chassis_finish_beep_change_time)
	{
		return;
	}
	/* 切换蜂鸣器电平；每个完整的响段计数一次。 */
	if (chassis_finish_beep_level == 0U)
	{
		beep_on();
		chassis_finish_beep_level = 1U;
		chassis_finish_beep_change_time = now + 0.12f;
	}
	else
	{
		beep_off();
		chassis_finish_beep_level = 0U;
		chassis_finish_beep_count++;
		chassis_finish_beep_change_time = now + 0.12f;
		/* 两次响声完成后关闭提示任务。 */
		if (chassis_finish_beep_count >= 2U)
		{
			chassis_finish_beep_mode = 0U;
		}
	}
}

/**
 * @brief 探头锁存A横线后，按“探头到车体中心”的机械偏置让中心停到A
 * @param curvature 当前路径曲率；仍在第二半圆时保持圆弧，进入直线后为0
 * @param target_yaw 当前期望航向角
 * @param speed_limit 偏置补偿阶段允许的最高中心速度
 * @return 1表示已经停车，0表示仍在接近目标
 */
static uint8_t Chassis_StadiumStopCenterAtFinish(float curvature, float target_yaw,
                                                 float speed_limit)
{
	/* 探头确认横线后，编码器已经继续前进的距离(m)。 */
	float traveled_after_line = fabsf(Chassis_GetForwardOdom() -
	                                  chassis_stadium_finish_line_odom);
	/* 剩余距离=探头到中心的安装距离-过线后已走距离；小于0表示中心已越过A线。 */
	float remaining = CHASSIS_TRACE_TO_CENTER_M - traveled_after_line;
	float target_speed;

	/* 最后4 cm直接撤销速度给定，避免速度规划器的惯性拖过A线20~30 cm。
	 * 任务5/6：A点冻结计时但不在此停车，继续直行0.4m后减速停止。 */
	if (remaining <= 0.04f)
	{
		if ((chassis_stadium_task == H_TASK_5_CENTER_BALL_LAP) ||
		    (chassis_stadium_task == H_TASK_6_TARGET_BALL_LAP))
		{
			/* A点冻结计时，清除速度状态以便下一段重新规划。 */
			chassis_stadium_elapsed_s = DWT_GetTimeline_s() - chassis_stadium_start_time_s;
			chassis_stadium_timer_running = 0U;
			chassis_stadium_speed_cmd = 0.0f;
			chassis_stadium_accel_cmd = 0.0f;
			Chassis_StadiumEnterStep(CHASSIS_STADIUM_FINISH_COAST);
			return 1U;
		}
		DCMotor_SetTraceCompensation(motor_l, 0.0f);
		DCMotor_SetTraceCompensation(motor_r, 0.0f);
		DC_Motor_SetRef(motor_l, 0.0f);
		DC_Motor_SetRef(motor_r, 0.0f);
		chassis_stadium_speed_cmd = 0.0f;
		chassis_stadium_accel_cmd = 0.0f;
		chassis_stadium_elapsed_s = DWT_GetTimeline_s() - chassis_stadium_start_time_s;
		chassis_stadium_timer_running = 0U;
		Chassis_StadiumEnterStep(CHASSIS_STADIUM_STOP);
		return 1U;
	}

	if (remaining <= 0.003f)
	{
		target_speed = 0.0f;
	}
	else
	{
		/*
		 * 按剩余距离生成平方根减速曲线。0.65系数给jerk限制和实车惯性留余量，
		 * 避免走满补偿距离后才开始制动。最低速度用于克服低速静摩擦。
		 */
		target_speed = 0.65f * sqrtf(2.0f * CHASSIS_FAST_MAX_DECEL * remaining);
		if (target_speed > speed_limit) target_speed = speed_limit;
		if (target_speed < CHASSIS_FAST_FINISH_SPEED)
		{
			target_speed = CHASSIS_FAST_FINISH_SPEED;
		}
	}

	/* 经过jerk限制后的实际中心速度指令，可能不会立刻等于target_speed。 */
	float center_speed = Chassis_StadiumUpdateSpeed(target_speed);
	/* 在探头到中心的补偿行程内把曲率线性减到0，使车身到A时恢复切线方向。 */
	float curvature_scale = remaining / CHASSIS_TRACE_TO_CENTER_M;
	if (curvature_scale < 0.0f) curvature_scale = 0.0f;
	if (curvature_scale > 1.0f) curvature_scale = 1.0f;
	Chassis_StadiumSetDrive(center_speed, curvature * curvature_scale, target_yaw);

	if ((remaining <= 0.0f) && (fabsf(center_speed) < 0.002f))
	{
		chassis_stadium_elapsed_s = DWT_GetTimeline_s() - chassis_stadium_start_time_s;
		chassis_stadium_timer_running = 0U;
		Chassis_StadiumEnterStep(CHASSIS_STADIUM_STOP);
		return 1U;
	}
	return 0U;
}

/**
 * @brief H题体育场路线连续循迹控制器
 *
 * 红外模块每周期只采样和计算一次，负责全程压线；编码器判断段落进度；
 * IMU修正直线航向和圆弧航向；速度规划器保证B/C/D处连续通过。
 */
static void Chassis_StadiumControl(void)
{
	if (chassis_stadium_initialized == 0U)
	{
		/* 第一次进入时初始化整圈状态，后续周期不重复清零。 */
		chassis_stadium_initialized = 1U;
		/* 清除上一圈的终点检测结果。 */
		chassis_stadium_finish_line_latched = 0U;
		chassis_stadium_finish_line_odom = 0.0f;
		chassis_stop_line_detect_count = 0U;
		chassis_stadium_speed_cmd = 0.0f;
		chassis_stadium_accel_cmd = 0.0f;
		/* 保存当前任务类型，统一决定本圈速度参数。 */
		chassis_stadium_task = chassis_cmd_receive.competition_task;
		K230_TransmitData((uint8_t)chassis_stadium_task);
		/* 保存A点的绝对航向，后续各段均由它推导，避免逐段累计角度误差。 */
		chassis_stadium_initial_yaw = Chassis_GetYawDeg();
		chassis_stadium_start_time_s = DWT_GetTimeline_s();
		chassis_stadium_elapsed_s = 0.0f;
		chassis_stadium_timer_running = 1U;
		/* 将驱动轮轴线在A点的里程设为0。 */
		Chassis_ResetEncoderOdom();
		/* 从A到B直线开始。 */
		Chassis_StadiumEnterStep(CHASSIS_STADIUM_STRAIGHT_1);
	}

	/* 每周期读取探头并更新巡线补偿。 */
	Chassis_Trace_Cal();
	/* 当前步骤已经行驶的弧长/直线距离(m)，切换步骤时会重新从0累计。 */
	float segment_distance = fabsf(Chassis_GetForwardOdom() -
	                               chassis_stadium_segment_start_odom);
	float center_speed; /* 本周期经过速度规划后，下发的车体中心速度(m/s)。 */
	float target_yaw;   /* 本周期按理论路线生成的IMU目标航向(deg)。 */
	float straight_speed;
	float arc_speed;
	if (chassis_stadium_task == H_TASK_2_FAST_LAP)
	{
		straight_speed = CHASSIS_FAST_STRAIGHT_SPEED;
		arc_speed = CHASSIS_FAST_ARC_SPEED;
	}
	else
	{
		straight_speed = CHASSIS_STADIUM_STRAIGHT_SPEED;
		arc_speed = CHASSIS_STADIUM_ARC_SPEED;
	}

	/* 当前状态只负责当前的一段路线。 */
	switch (chassis_stadium_step)
	{
		case CHASSIS_STADIUM_STRAIGHT_1:
		case CHASSIS_STADIUM_STRAIGHT_2:
		{
			/*
			 * B点提前6 cm切换，以抵消速度规划器建立左右轮差速时继续前行的距离。
			 * D点使用实测23 cm探头前置距离，让探头到弯道时同步进入圆弧前馈。
			 */
			float straight_exit_distance;
			if (chassis_stadium_step == CHASSIS_STADIUM_STRAIGHT_2)
			{
				straight_exit_distance = CHASSIS_STRAIGHT_DIST_M - CHASSIS_TRACE_TO_CENTER_M;
			}
			else
			{
				/* AB也按探头前置距离提前切换，避免探头已经进入弯道而底盘仍按直线跑。 */
				straight_exit_distance = CHASSIS_STRAIGHT_DIST_M - CHASSIS_TRACE_FORWARD_OFFSET_M;
			}
			center_speed = Chassis_StadiumUpdateSpeed(straight_speed);
			float straight_target_yaw;
			if (chassis_stadium_step == CHASSIS_STADIUM_STRAIGHT_1)
			{
				straight_target_yaw = chassis_stadium_initial_yaw;
			}
			else
			{
				straight_target_yaw = Chassis_AngleNormalize(chassis_stadium_initial_yaw +
				                                           CHASSIS_CLOCKWISE_YAW_SIGN * 180.0f);
			}
			Chassis_StadiumSetDrive(center_speed, 0.0f, straight_target_yaw);
			if (segment_distance >= straight_exit_distance)
			{
				if ((chassis_stadium_step == CHASSIS_STADIUM_STRAIGHT_1) &&
				    (chassis_stadium_task == H_TASK_4_AB_BALL))
				{
					Chassis_StadiumEnterStep(CHASSIS_STADIUM_FINISH_BRAKE);
				}
				else
				{
					if (chassis_stadium_step == CHASSIS_STADIUM_STRAIGHT_1)
					{
						Chassis_StadiumEnterStep(CHASSIS_STADIUM_ARC_1);
					}
					else
					{
						Chassis_StadiumEnterStep(CHASSIS_STADIUM_ARC_2);
					}
				}
			}
			break;
		}

		case CHASSIS_STADIUM_ARC_1:
		case CHASSIS_STADIUM_ARC_2:
		{
			float arc_exit_angle_deg;
			if (chassis_stadium_step == CHASSIS_STADIUM_ARC_1)
			{
				/* C点容易超调，第一弯提前15度结束圆弧前馈。 */
				arc_exit_angle_deg = CHASSIS_BC_EXIT_ANGLE_DEG;
			}
			else
			{
				arc_exit_angle_deg = CHASSIS_ARC_ANGLE_DEG;
			}
			/* 用题目规定的0.5 m几何半径计算状态切换所需弧长。 */
			float arc_length = CHASSIS_ARC_RADIUS_M * arc_exit_angle_deg *
			                   (CHASSIS_PI / 180.0f);
			float remaining_arc = arc_length - segment_distance;
			float max_decel;
			float finish_speed;
			if (chassis_stadium_task == H_TASK_2_FAST_LAP)
			{
				max_decel = CHASSIS_FAST_MAX_DECEL;
				finish_speed = CHASSIS_FAST_FINISH_SPEED;
			}
			else
			{
				max_decel = CHASSIS_STADIUM_MAX_DECEL;
				finish_speed = CHASSIS_FINAL_APPROACH_SPEED;
			}
			float brake_distance = (arc_speed * arc_speed - finish_speed * finish_speed) /
			                       (2.0f * max_decel);
			if (chassis_stadium_task == H_TASK_2_FAST_LAP)
			{
				brake_distance *= CHASSIS_FAST_BRAKE_MARGIN;
			}
			float arc_target_speed = arc_speed;
			if ((chassis_stadium_step == CHASSIS_STADIUM_ARC_2) &&
			    (remaining_arc <= brake_distance))
			{
				arc_target_speed = finish_speed;
			}
			/*
			 * 实车顺时针转动时IMU yaw递减，因此目标航向按负方向变化。
			 * segment_distance/R得到弧度，再换算为角度；走完pi*R正好增加180°。
			 */
			float yaw_distance = segment_distance;
			if (yaw_distance > arc_length)
			{
				yaw_distance = arc_length;
			}
			float arc_base_yaw;
			if (chassis_stadium_step == CHASSIS_STADIUM_ARC_1)
			{
				arc_base_yaw = chassis_stadium_initial_yaw;
			}
			else
			{
				arc_base_yaw = Chassis_AngleNormalize(chassis_stadium_initial_yaw +
				                                    CHASSIS_CLOCKWISE_YAW_SIGN * 180.0f);
			}
			target_yaw = arc_base_yaw +
			             CHASSIS_CLOCKWISE_YAW_SIGN * yaw_distance /
			             CHASSIS_ARC_RADIUS_M * (180.0f / CHASSIS_PI);
			target_yaw = Chassis_AngleNormalize(target_yaw);
			/*
			 * 第二半圆后65%开始寻找A横线，避免到几何圆弧结束后才检测而漏线。
			 * 一旦锁存横线，立即按探头前置距离规划停车，不再依赖固定圆弧里程。
			 */
			if ((chassis_stadium_step == CHASSIS_STADIUM_ARC_2) &&
			    (segment_distance >= CHASSIS_FINISH_DETECT_RATIO * arc_length))
			{
				Chassis_StadiumDetectFinishLine();
				if (chassis_stadium_finish_line_latched != 0U)
				{
					/* 使用第二半圆最终180°航向收正，避免A点斜停。 */
					(void)Chassis_StadiumStopCenterAtFinish(1.0f / CHASSIS_ARC_DRIVE_RADIUS_M,
					                                          chassis_stadium_initial_yaw, arc_speed);
					break;
				}
			}
			/* 未锁存终点线时，本周期只执行一次常规圆弧速度更新。 */
			center_speed = Chassis_StadiumUpdateSpeed(arc_target_speed);
			Chassis_StadiumSetDrive(center_speed, 1.0f / CHASSIS_ARC_DRIVE_RADIUS_M, target_yaw);
			if (segment_distance >= arc_length)
			{
				if (chassis_stadium_step == CHASSIS_STADIUM_ARC_1)
				{
					Chassis_StadiumEnterStep(CHASSIS_STADIUM_STRAIGHT_2);
				}
				else
				{
					/* DA理论弧线已完成，说明A线应已越过；漏检时不再继续搜索。 */
					Chassis_StadiumStartFinishBeep(0U);
					chassis_stadium_elapsed_s = DWT_GetTimeline_s() - chassis_stadium_start_time_s;
					chassis_stadium_timer_running = 0U;
					Chassis_StadiumEnterStep(CHASSIS_STADIUM_STOP);
				}
			}
			break;
		}

		case CHASSIS_STADIUM_FINISH_BRAKE:
		{
			/* 任务4专用：通过B点后沿半圆继续行驶，按平方根减速曲线逐渐减速停车。
			 * 停车弧长从B点起算：切弧发生在探头到达B（轴线距B还有CHASSIS_TRACE_FORWARD_OFFSET_M），
			 * 因此段内停车点 = 探头前置距离 + 任务要求的B点后弧长。 */
			float stop_segment = CHASSIS_TRACE_FORWARD_OFFSET_M + CHASSIS_TASK4_STOP_ARC_M;
			float remaining = stop_segment - segment_distance;
			float target_speed;

			/* 最后4 cm直接撤销速度给定，避免速度规划器的惯性拖过停车点。 */
			if (remaining <= 0.04f)
			{
				DCMotor_SetTraceCompensation(motor_l, 0.0f);
				DCMotor_SetTraceCompensation(motor_r, 0.0f);
				DC_Motor_SetRef(motor_l, 0.0f);
				DC_Motor_SetRef(motor_r, 0.0f);
				chassis_stadium_speed_cmd = 0.0f;
				chassis_stadium_accel_cmd = 0.0f;
				chassis_stadium_elapsed_s = DWT_GetTimeline_s() - chassis_stadium_start_time_s;
				chassis_stadium_timer_running = 0U;
				Chassis_StadiumStartFinishBeep(1U);
				Chassis_StadiumEnterStep(CHASSIS_STADIUM_STOP);
				break;
			}

			/* 平方根减速曲线；0.65系数给jerk限制和实车惯性留余量，最低速度克服低速静摩擦。 */
			target_speed = 0.65f * sqrtf(2.0f * CHASSIS_STADIUM_MAX_DECEL * remaining);
			if (target_speed > arc_speed) target_speed = arc_speed;
			if (target_speed < CHASSIS_FAST_FINISH_SPEED) target_speed = CHASSIS_FAST_FINISH_SPEED;

			/* 经jerk限制后的实际中心速度指令，航向沿B→C半圆继续变化。 */
			center_speed = Chassis_StadiumUpdateSpeed(target_speed);
			float arc_angle = segment_distance / CHASSIS_ARC_RADIUS_M * (180.0f / CHASSIS_PI);
			if (arc_angle > CHASSIS_ARC_ANGLE_DEG) arc_angle = CHASSIS_ARC_ANGLE_DEG;
			target_yaw = Chassis_AngleNormalize(chassis_stadium_initial_yaw +
			                                    CHASSIS_CLOCKWISE_YAW_SIGN * arc_angle);
			Chassis_StadiumSetDrive(center_speed, 1.0f / CHASSIS_ARC_DRIVE_RADIUS_M, target_yaw);
			break;
		}

		case CHASSIS_STADIUM_FINISH_COAST:
		{
			/* 任务5/6专用：通过A点后继续直行，按平方根减速曲线逐渐减速停车。 */
			float remaining = CHASSIS_TASK56_COAST_M - segment_distance;
			float target_speed;

			/* 最后4 cm直接撤销速度给定，避免速度规划器的惯性拖过停车点。 */
			if (remaining <= 0.04f)
			{
				DCMotor_SetTraceCompensation(motor_l, 0.0f);
				DCMotor_SetTraceCompensation(motor_r, 0.0f);
				DC_Motor_SetRef(motor_l, 0.0f);
				DC_Motor_SetRef(motor_r, 0.0f);
				chassis_stadium_speed_cmd = 0.0f;
				chassis_stadium_accel_cmd = 0.0f;
				Chassis_StadiumStartFinishBeep(1U);
				Chassis_StadiumEnterStep(CHASSIS_STADIUM_STOP);
				break;
			}

			/* 平方根减速曲线；0.65系数给jerk限制和实车惯性留余量。 */
			target_speed = 0.65f * sqrtf(2.0f * CHASSIS_STADIUM_MAX_DECEL * remaining);
			if (target_speed > straight_speed) target_speed = straight_speed;
			if (target_speed < CHASSIS_FAST_FINISH_SPEED) target_speed = CHASSIS_FAST_FINISH_SPEED;

			/* 经jerk限制后的实际中心速度指令；过A后沿AB方向直行，曲率为0。 */
			center_speed = Chassis_StadiumUpdateSpeed(target_speed);
			Chassis_StadiumSetDrive(center_speed, 0.0f, chassis_stadium_initial_yaw);
			break;
		}

		case CHASSIS_STADIUM_STOP:
		default:
			chassis_stadium_speed_cmd = 0.0f;
			chassis_stadium_accel_cmd = 0.0f;
			DCMotor_SetTraceCompensation(motor_l, 0.0f);
			DCMotor_SetTraceCompensation(motor_r, 0.0f);
			DC_Motor_SetRef(motor_l, 0.0f);
			DC_Motor_SetRef(motor_r, 0.0f);
			/* 停车状态仍更新蜂鸣器，成功两声或漏检持续鸣叫。 */
			Chassis_StadiumUpdateFinishBeep();
			break;
	}
}
/* 旧版分段停车状态机仅留作参数迁移参考，不参与构建。 */
#if 0
/**
 * @brief 旧版 IMU_MODE 体育场赛道动作
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
			/* 弧线只采样灰度，不把循迹补偿叠加到 IMU 弧线控制。 */
			Trace_UpdateSensor();
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
			Trace_UpdateSensor();
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
			/* 终点阶段只需要最新原始值进行横线判断。 */
			Trace_UpdateSensor();
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

#endif

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

	/* 退出体育场模式后，下次进入必须从A点重新初始化整圈轨迹。 */
	chassis_stadium_step = CHASSIS_STADIUM_STRAIGHT_1;
	chassis_stadium_initialized = 0U;
	chassis_stadium_finish_line_latched = 0U;
	chassis_stadium_finish_line_odom = 0.0f;
	chassis_stadium_segment_start_odom = 0.0f;
	chassis_stadium_segment_start_yaw = 0.0f;
	chassis_stadium_initial_yaw = 0.0f;
	chassis_stadium_speed_cmd = 0.0f;
	chassis_stadium_accel_cmd = 0.0f;
	chassis_stadium_curvature_cmd = 0.0f;
	chassis_stadium_task = H_TASK_NONE;
	chassis_stadium_timer_running = 0U;
	/* 清除上一圈的蜂鸣器提示状态，并确保蜂鸣器关闭。 */
	chassis_finish_beep_mode = 0U;
	chassis_finish_beep_count = 0U;
	chassis_finish_beep_level = 0U;
	chassis_finish_beep_change_time = 0.0f;
	beep_off();
}

/** @brief 获取体育场轨迹规划器当前中心速度指令，供滚球控制前馈使用。 */
float Chassis_GetCommandedSpeed(void)
{
	return chassis_stadium_speed_cmd;
}

/** @brief 获取体育场轨迹规划器当前纵向加速度指令，供滚球控制前馈使用。 */
float Chassis_GetCommandedAcceleration(void)
{
	return chassis_stadium_accel_cmd;
}

/** @brief 获取期望横向加速度 v^2*k，左转为正，单位 m/s^2。 */
float Chassis_GetCommandedLateralAcceleration(void)
{
	return chassis_stadium_speed_cmd * chassis_stadium_speed_cmd *
	       chassis_stadium_curvature_cmd;
}

/** @brief 获取期望偏航角速度 v*k，左转为正，单位 rad/s。 */
float Chassis_GetCommandedYawRate(void)
{
	return chassis_stadium_speed_cmd * chassis_stadium_curvature_cmd;
}

/** @brief 获取本次任务用时；运行中返回实时值，停车后保持最终值。 */
float Chassis_GetRunTimeSeconds(void)
{
	return chassis_stadium_timer_running ?
	       (DWT_GetTimeline_s() - chassis_stadium_start_time_s) :
	       chassis_stadium_elapsed_s;
}

uint8_t Chassis_IsRunTimerActive(void)
{
	return chassis_stadium_timer_running;
}

uint8_t Chassis_GetStadiumStep(void)
{
	return (uint8_t)chassis_stadium_step;
}

void Chassis_RequestEmergencyStop(void)
{
	chassis_emergency_stop_requested = 1U;
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
	float odom;
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

	odom = Chassis_GetForwardOdom();
	remain = chassis_action_target_distance - odom;
	uint8_t crossed_target = ((chassis_action_target_distance >= 0.0f) &&
	                          (odom >= chassis_action_target_distance)) ||
	                         ((chassis_action_target_distance < 0.0f) &&
	                          (odom <= chassis_action_target_distance));
	if ((fabsf(remain) <= CHASSIS_LINE_DONE_ERR_M) || crossed_target)
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

	if (remain > 0.0f) base_speed = abs_speed;
	else base_speed = -abs_speed;
	if (progress < CHASSIS_LINE_ACCEL_M)
	{
		float accel_speed = CHASSIS_LINE_MIN_SPEED +
		                    (abs_speed - CHASSIS_LINE_MIN_SPEED) * progress / CHASSIS_LINE_ACCEL_M;

		/*
		 * 起步前 0.1m 线性加速，避免一进入直线动作就给满速度。
		 */
		if (remain > 0.0f) base_speed = accel_speed;
		else base_speed = -accel_speed;
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
			if (remain > 0.0f) base_speed = slowdown_speed;
			else base_speed = -slowdown_speed;
		}
	}

	yaw_error = Chassis_AngleNormalize(chassis_action_target_yaw - Chassis_GetYawDeg());
	/* 与体育场控制器保持同一符号：正yaw误差应产生正修正量。 */
	yaw_compensation = PID_calc(&chassis_line_yaw_pid, yaw_error, 0.0f);

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
	int radius_dir;
	int angle_dir;
	if (radius_m > 0.0f) radius_dir = 1;
	else radius_dir = -1;
	if (total_angle_deg > 0.0f) angle_dir = 1;
	else angle_dir = -1;
	int turn_dir = radius_dir * angle_dir;

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

	}

	/* 弧线阶段始终禁止灰度补偿，避免上一周期或模式切换残留。 */
	DCMotor_SetTraceCompensation(motor_l, 0.0f);
	DCMotor_SetTraceCompensation(motor_r, 0.0f);

	/* ---- 每周期：速度环 + yaw 保向 ---- */
	motor_l->loop_mode = SPEED_MODE;
	motor_r->loop_mode = SPEED_MODE;
	motor_l->State = ENABLE;
	motor_r->State = ENABLE;

	float current_distance = fabsf(Chassis_GetForwardOdom());
	float remain = chassis_arc_total_distance - current_distance;

	float final_target_yaw = Chassis_AngleNormalize(chassis_arc_start_yaw
	                                               + (float)turn_dir * abs_angle);
	float final_yaw_error = Chassis_AngleNormalize(final_target_yaw - Chassis_GetYawDeg());

	/* 里程和最终航向都满足后才结束，避免打滑时提前进入下一条直线。 */
	if ((remain <= CHASSIS_LINE_DONE_ERR_M) &&
	    (fabsf(final_yaw_error) <= CHASSIS_ARC_DONE_YAW_ERR_DEG))
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
	float base_speed;
	if (remain > 0.0f) base_speed = abs_speed;
	else base_speed = 0.0f;

	if (progress < CHASSIS_LINE_ACCEL_M)
	{
		/* 起步加速段 */
		base_speed = CHASSIS_LINE_MIN_SPEED
		             + (abs_speed - CHASSIS_LINE_MIN_SPEED) * progress / CHASSIS_LINE_ACCEL_M;
	}
	if ((remain > 0.0f) && (remain < CHASSIS_LINE_SLOWDOWN_M))
	{
		/* 末段减速段 */
		float slowdown = abs_speed * remain / CHASSIS_LINE_SLOWDOWN_M;
		if (slowdown < CHASSIS_LINE_MIN_SPEED) { slowdown = CHASSIS_LINE_MIN_SPEED; }
		if (slowdown < base_speed)             { base_speed = slowdown; }
	}

	/* ---- IMU yaw 误差 → 补偿 ---- */
	if (remain <= 0.0f)
	{
		target_yaw = final_target_yaw;
	}
	float yaw_error = Chassis_AngleNormalize(target_yaw - Chassis_GetYawDeg());
	/* PID_calc内部计算ref-measure，传入(yaw_error, 0)避免二次反号。 */
	float yaw_compensation = PID_calc(&chassis_line_yaw_pid, yaw_error, 0.0f);

	/* 差速运动学前馈负责产生曲率，IMU PID 只修正模型和打滑误差。 */
	float radius_ratio = CHASSIS_TRACK_WIDTH_M / (2.0f * abs_radius);
	float left_feedforward = base_speed * (1.0f - (float)turn_dir * radius_ratio);
	float right_feedforward = base_speed * (1.0f + (float)turn_dir * radius_ratio);
	DC_Motor_SetRef(motor_l, left_feedforward + yaw_compensation);
	DC_Motor_SetRef(motor_r, right_feedforward - yaw_compensation);

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
