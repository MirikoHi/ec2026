#include "Gimbal.h"
#include "ZDT_Motor.h"
#include "ZDT_Emm.h"
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


float x;
float dt;
float target_vel = 300;

float target_angle_deg = 0;
float velocity_ff = 0.00f;

steel_ball_movement_typedef steel_ball_movement_data;
ServoInstance*  servo_yaw;
/* ═══════════════════════════════════════════════════════════════════════
 * Slide Ball Control — 滑槽小球【位置外环+速度内环 双环PID】
 *
 * 双环结构：
 * 外环(位置环): 位置误差 → PID输出【目标小球速度 px/s】
 * 内环(速度环): 目标速度 - 实际小球速度 → PID输出倾角角度
 *
 *   - 滑槽一端合页固定，另一端连杆连接舵机
 *   - 舵机角度控制滑槽倾角，从而控制小球位置
 *   - 视觉回传 steel_ball_movement_data.x_position (0~640)
 *   - 视觉回传 steel_ball_movement_data.dt (帧间隔, 秒)
 * ═══════════════════════════════════════════════════════════════════════ */

float SLIDE_TARGET_X   =  120.00f ;    /* 目标位置: 画面目标坐标 */
uint32_t motor_zero_point =  0;
#define SLIDE_SERVO_RANGE      50     /* 最大倾角角度范围 */
#define SLIDE_VEL_LPF_ALPHA    0.3f    /* 速度低通滤波系数 */
#define SLIDE_VEL_FF_GAIN      0.3f   /* 速度前馈增益（可选保留） */
#define SLIDE_X_LPF_ALPHA      0.25f   /* X坐标低通滤波系数 */

#define SLIDE_MAX_BALL_SPEED   150.0f  /* 位置环输出限幅：小球最大允许速度 px/s */

/************************* PID配置修改 *************************/
// 【外环：位置环PID】输入：位置误差(像素)，输出：期望小球速度(px/s)
static pid_init_config_s slide_pos_cfg = {
	.mode    = PID_POSITION,
	.Kp      = 0.12f,
	.Ki      = 0.01f,
	.Kd      = 0.0005f,
	.max_out = SLIDE_MAX_BALL_SPEED,    //输出上限：最大目标速度 px/s
	.max_iout = 80.0f,
};

// 【内环：速度环PID】输入：速度误差(px/s)，输出：倾角角度(deg)
static pid_init_config_s slide_vel_cfg = {
	.mode    = PID_POSITION,
	.Kp      = 0.115f,
	.Ki      = 0.001f,
	.Kd      = 0.065f,
	.max_out = SLIDE_SERVO_RANGE,
	.max_iout = 40.0f,
};

static pid_type_def slide_pos_pid;       /* 外环：位置环PID */
static pid_type_def slide_vel_pid;       /* 内环：速度环PID */

static float        slide_prev_x = 320;   /* 上一帧 X 位置 */
int16_t        slide_velocity = 0;        /* 滤波后的小球速度 (px/s) */
static float     stepper_current_clock = 0;

/* 一阶低通滤波器状态 */
static float slide_x_lpf_out = 320.0f;
static float slide_v_lpf_out = 0.0f;


static void Gimbal_ZDT_UART_Send(const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) {
		while (DL_UART_isBusy(STEPPER_MOTOR_INST)) {}
		DL_UART_Main_transmitData(STEPPER_MOTOR_INST, data[i]);
	}
}


static void Slide_Control_Init(void)
{
    /* 初始化双环PID */
    PID_init(&slide_pos_pid, &slide_pos_cfg);
    PID_init(&slide_vel_pid, &slide_vel_cfg);

    /* 滤波器初始化 */
    slide_x_lpf_out = SLIDE_TARGET_X;
    slide_v_lpf_out = 0.0f;
    slide_prev_x = SLIDE_TARGET_X;
}

/**
 * @brief 滑槽小球闭环控制【位置-速度双环PID】(在 Gimbal 200Hz 循环中调用)
 *
 * 算法流程：
 *   1. 获取视觉x坐标 + dt，坐标低通滤波
 *   2. 计算小球实际运动速度并滤波
 *   3. 外环位置PID：位置误差 → 输出【目标速度 vel_ref】
 *   4. 内环速度PID：vel_ref - 实际速度 → 输出基础倾角角度
 *   5. 叠加速度前馈（可选）
 *   6. 角度限幅，转换脉冲发送步进电机
 */
static void Slide_Control_Run(void)
{
    if (!K230_Read(&steel_ball_movement_data)) return;

    /* 原始视觉坐标 */
    float x_raw  = (float)steel_ball_movement_data.x_position;

    /* ========= X坐标一阶低通滤波 ========= */
    slide_x_lpf_out = SLIDE_X_LPF_ALPHA * x_raw + (1.0f - SLIDE_X_LPF_ALPHA) * slide_x_lpf_out;
    x = slide_x_lpf_out;
    /* ===================================== */

    dt = steel_ball_movement_data.dt;

    /* 保护: dt 异常时使用默认值 */
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f;  /* 默认 50ms (20fps) */
    }

    /* ── 1. 小球实际速度估计 ── */
    float raw_v = (x - slide_prev_x) / dt;
    slide_prev_x = x;

    /* 速度一阶低通滤波 */
    slide_v_lpf_out = SLIDE_VEL_LPF_ALPHA * raw_v + (1.0f - SLIDE_VEL_LPF_ALPHA) * slide_v_lpf_out;
    slide_velocity = (int16_t)(slide_v_lpf_out); //原始速度px/s，不再缩放

    /* ── 2. 外环【位置环PID】────────────────────────────── */
    float pos_error = SLIDE_TARGET_X - x;
    PID_calc(&slide_pos_pid, 0.0f, pos_error);
    float vel_ref = slide_pos_pid.out; //位置环输出 = 期望小球目标速度(px/s)


	//vel_ref = target_vel;
    /* ── 3. 内环【速度环PID】────────────────────────────── */
    float vel_error = vel_ref - slide_v_lpf_out;
    PID_calc(&slide_vel_pid, 0.0f, vel_error);
    float angle_base = slide_vel_pid.out;

    /* ── 4. 可选：叠加速度前馈（保留原有逻辑，可注释关闭） ──
     * 球向右运动 (v>0) → 需要左倾 (减小角度) 来"接住"球
     */
    velocity_ff = slide_v_lpf_out * SLIDE_VEL_FF_GAIN;
	if (velocity_ff > 60.0f) {velocity_ff = 60.0f;}
	else if (velocity_ff < -60.0f) {velocity_ff = -60.0f;}

    /* ── 5. 合成最终倾角角度 ── */
	target_angle_deg = angle_base;

	if (target_angle_deg >   SLIDE_SERVO_RANGE)  target_angle_deg =   SLIDE_SERVO_RANGE;
	if (target_angle_deg < -(SLIDE_SERVO_RANGE)) target_angle_deg = -(SLIDE_SERVO_RANGE);

	int32_t pulses = (int32_t)(target_angle_deg * 3200.0f / 360.0f);

	uint32_t pulse_count = (uint32_t)((pulses >= 0) ? pulses : -pulses);
	uint8_t dir = pulses >= 0 ? 1 : 0;

    ZDT_Emm_Pos_Control(1, dir, 1000, 250, (uint32_t)pulse_count, 1, false);
}


void Gimbal_Init(void)
{
	/* RX FIFO */
	fifo_initQueue(&zdt_emm_rx_fifo);

	/* UART 发送回调 */
	ZDT_Emm_RegisterSendCallback(Gimbal_ZDT_UART_Send);

	/* RX 中断 */
	NVIC_ClearPendingIRQ(STEPPER_MOTOR_INST_INT_IRQN);
	NVIC_EnableIRQ(STEPPER_MOTOR_INST_INT_IRQN);

	/* 使能电机 (rotate, addr=1) */
	ZDT_Emm_En_Control(1, true, false);


	Servo_Init_Config_s servo_yaw_config = {
		.Servo_type = Servo180,
		.inst = Servo_INST,
		.idx = 0,    //对应DL_TIMER_CC_0_INDEX，PA17
	};
	servo_yaw = ServoInit(&servo_yaw_config);
	Servo_Motor_Type_Select(servo_yaw, Free_Angle_mode);

	/* 初始化滑槽小球双环PID */
	Slide_Control_Init();

	DWT_Delay(1);
	ZDT_Emm_Pos_Control(1, 1, 10, 0, 0, 1, false);
}


void Gimbal(void)
{
	Slide_Control_Run();
}

void Gimbal_Pid_Cal(void)
{
	gimbal_cmd_send.pitch +=gimbal_pitch_PID.out;
}
void Gimbal_Attitude_Solving(void)
{
	float aim_x = gimbal_cmd_receive.aim_x;
	float aim_y = gimbal_cmd_receive.aim_y;
	gimbal_cmd_receive.yaw = -atan2f(aim_x,GIMBAL_LENGTH_TO_CENTER)*180.0f/PI;
	gimbal_cmd_receive.pitch = atan2f(aim_y,sqrtf(GIMBAL_LENGTH_TO_CENTER*GIMBAL_LENGTH_TO_CENTER+aim_x*aim_x))*180.0f/PI;
}

void UART3_IRQHandler(void)
{
	uint8_t byte = DL_UART_receiveData(STEPPER_MOTOR_INST);
	ZDT_Emm_RxPushByte(byte);
	DL_UART_clearInterruptStatus(STEPPER_MOTOR_INST, DL_UART_INTERRUPT_RX);
}