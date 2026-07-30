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

steel_ball_movement_typedef steel_ball_movement_data;
ServoInstance*  servo_yaw;
/* ═══════════════════════════════════════════════════════════════════════
 * Slide Ball Control — 滑槽小球位置闭环
 *
 *   - 滑槽一端合页固定，另一端连杆连接舵机
 *   - 舵机角度控制滑槽倾角，从而控制小球位置
 *   - 视觉回传 steel_ball_movement_data.x_position (0~640)
 *   - 视觉回传 steel_ball_movement_data.dt (帧间隔, 秒)
 *
 *   控制策略:
 *     PID(目标位置 - 当前位置) → 基础角度
 *     + 速度前馈(dt 用于精确速度估计) → 预测性角度补偿
 *     → 最终舵机角度
 * ═══════════════════════════════════════════════════════════════════════ */

#define SLIDE_TARGET_X         200     /* 目标位置: 画面中心 */
#define SLIDE_SERVO_RANGE      10      /* PID 最大输出 (角度单位) */
#define SLIDE_VEL_LPF_ALPHA    0.3f    /* 速度低通滤波系数 */
#define SLIDE_VEL_FF_GAIN      0.001f  /* 速度前馈增益 */

/* ── 步进电机绝对位置控制参数 ── */
#define STEPPER_ADDR           1           /* 电机地址 */
#define STEPPER_MAX_RPM        300         /* 最大转速 RPM */
#define STEPPER_ACC            50          /* 加速度 */
#define STEPPER_CENTER_CLK     0           /* 滑槽水平时的绝对脉冲位置 */
#define STEPPER_MAX_CLK        800         /* 最大脉冲偏移量 (±800 ≈ ±90° @3200ppr) */
#define STEPPER_UPDATE_DIV     4           /* 200Hz / 4 = 50Hz 下发频率 */

static pid_type_def slide_ball_pid;       /* 位置PID控制器 */
static float        slide_prev_x = 320;   /* 上一帧 X 位置 */
static float        slide_velocity = 0;   /* 滤波后的小球速度 (px/s) */
static float     stepper_current_clock = 0;

static void Gimbal_ZDT_UART_Send(const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) {
		while (DL_UART_isBusy(STEPPER_MOTOR_INST)) {}
		DL_UART_Main_transmitData(STEPPER_MOTOR_INST, data[i]);
	}
}


static void Slide_Control_Init(void)
{
    pid_init_config_s cfg = {
        .mode    = PID_POSITION,
        .Kp      = 0.50f,     /* 比例: 每像素误差产生多少度倾角 */
        .Kd      = 0.00f,     /* 微分: 抑制震荡 */
        .Ki      = 0.00f,    /* 积分: 消除静差 */
        .max_out = SLIDE_SERVO_RANGE,
        .max_iout = 10.0f,
    };
    PID_init(&slide_ball_pid, &cfg);
}

/**
 * @brief 滑槽小球闭环控制 (在 Gimbal 200Hz 循环中调用)
 *
 * 算法:
 *   1. 从 K230 数据中取 x_position (0~640) 和 dt (帧间隔)
 *   2. 速度估计: v = Δx / dt, 低通滤波
 *   3. 位置误差 → PID → 基础舵机角度
 *   4. 速度前馈: 球速越大 → 倾角补偿越大
 *   5. 合成最终角度, 限幅后输出到舵机
 */
static void Slide_Control_Run(void)
{
    if (!K230_Read(&steel_ball_movement_data)) return;

    x  = (float)steel_ball_movement_data.x_position;
    dt = steel_ball_movement_data.dt;

    /* 保护: dt 异常时使用默认值 */
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f;  /* 默认 50ms (20fps) */
    }

    /* ── 1. 速度估计 (利用 dt 精确计算) ── */
    float raw_v = (x - slide_prev_x) / dt;
    slide_prev_x = x;

    /* 一阶低通滤波, 滤除视觉抖动 */
    slide_velocity = slide_velocity * (1.0f - SLIDE_VEL_LPF_ALPHA)
                   + raw_v * SLIDE_VEL_LPF_ALPHA;

    /* ── 2. 位置 PID ── */
    float error = SLIDE_TARGET_X - x;
    PID_calc(&slide_ball_pid, 0.0f, error);

    /* ── 3. 速度前馈 ──
     * 球向右运动 (v>0) → 需要左倾来"接住"球 → 反方向补偿 */
    float velocity_ff = -slide_velocity * SLIDE_VEL_FF_GAIN;

    /* ── 4. 合成输出 → 限幅 ── */
    float output = slide_ball_pid.out + velocity_ff;
    if (output >  SLIDE_SERVO_RANGE)  output =  SLIDE_SERVO_RANGE;
    if (output < -SLIDE_SERVO_RANGE)  output = -SLIDE_SERVO_RANGE;

    /* ── 5. 转为电机脉冲 (绝对位置) ──
     * output ±10 → 脉冲 ±800 (@3200ppr = ±90° 电机转角)
     * raF=1: 绝对位置模式, 电机内部闭环自动到位 */
    float scale = (float)STEPPER_MAX_CLK / (float)SLIDE_SERVO_RANGE;
    int32_t target_clk = STEPPER_CENTER_CLK + (int32_t)(output * scale);

    /* ── 6. 降频发送 (200Hz / 4 = 50Hz), 避免 UART 拥塞 ── */
    static uint8_t tick = 0;
    if (++tick >= STEPPER_UPDATE_DIV) {
        tick = 0;
        ZDT_Emm_QPos_Control(STEPPER_ADDR, target_clk);
    }
}


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
	//ZDT_TICK_Init();
	// pid_init_config_s gimbal_yaw_pid_config={
	// 	.mode = PID_POSITION,
	// 	.Kp = 0.003f,
	// 	.Kd = 0.0001f,
	// 	.Ki = 0.0f,
	// 	.max_out = 4.0f,
	// 	.max_iout = 1.0f,
	// };
	// PID_init(&gimbal_yaw_PID,&gimbal_yaw_pid_config);

	// pid_init_config_s gimbal_pitch_pid_config={
	// 	.mode = PID_POSITION,
	// 	.Kp = -0.003f,
	// 	.Kd = -0.0001f,
	// 	.Ki = 0.0f,
	// 	.max_out = 4.0f,
	// 	.max_iout = 1.0f,
	// };
	// PID_init(&gimbal_pitch_PID,&gimbal_pitch_pid_config);
	//
	// pid_init_config_s gimbal_yaw_forwardfeed_pid_config={
	// 	.mode = PID_POSITION,
	// 	.Kp = 0.000f,
	// 	.Kd = -0.0001f,
	// 	.Ki = 0.0f,
	// 	.max_out = 4.0f,
	// 	.max_iout = 1.0f,
	// };
	// PID_init(&gimbal_yaw_forwardfeed_PID,&gimbal_yaw_forwardfeed_pid_config);
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

	/* 初始化滑槽小球位置闭环 */
	Slide_Control_Init();

	/* 步进电机: 绝对位置模式参数 (raF=1), 只设置一次 */
	ZDT_Emm_Set_QPos_Params(STEPPER_ADDR, STEPPER_MAX_RPM, STEPPER_ACC, 1, false);

	/* 回到原点 */
	ZDT_Emm_QPos_Control(STEPPER_ADDR, STEPPER_CENTER_CLK);
}


void Gimbal(void)
{
	Slide_Control_Run();

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

void UART3_IRQHandler(void)
{
	uint8_t byte = DL_UART_receiveData(STEPPER_MOTOR_INST);
	ZDT_Emm_RxPushByte(byte);
	DL_UART_clearInterruptStatus(STEPPER_MOTOR_INST, DL_UART_INTERRUPT_RX);
}
