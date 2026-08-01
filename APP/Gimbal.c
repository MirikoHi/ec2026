#include "Gimbal.h"

#include "Chassis.h"
#include "ZDT_Motor.h"
#include "ZDT_Emm.h"
#include "Robot_cmd.h"
#include "math.h"
#include "misc.h"
#include "PID.h"
#include "K230.h"
#include "daemon.h"
#include "dcmotor.h"
#include "dwt.h"
#include "Servo.h"
#include "chassis.h"
#include "lichee_rec.h"

static DCMotorInstance *motor_l,*motor_r;

ZDT_Motor_t *yaw_motor,*pitch_motor;
float angle_debug;
static gimbal_cmd_q gimbal_cmd_receive={0};
static chassis_cmd_q chassis_cmd_receive = {0};    // 来自 cmd 的底盘控制命令

float relay_on_time;
uint8_t relay_first_on_flag=0;

pid_type_def gimbal_yaw_PID={0};
pid_type_def gimbal_pitch_PID={0};
pid_type_def gimbal_yaw_forwardfeed_PID = {0};
gimbal_cmd_q gimbal_cmd_send ={0};
static volatile Licheervnano_Frame LicheeRec_Frame;

float x;
float dt;

float target_angle_deg = 0;
float velocity_ff = 0.00f;

steel_ball_movement_typedef steel_ball_movement_data;
ServoInstance*  servo_yaw;
extern Chassis_Move_State_e car_stop;
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
#define SLIDE_ORIGIN_POS        312
#define SLIDE_5CM_POS           190
#define SLIDE_D5CM_POS        440
static float slide_target_x   =  SLIDE_ORIGIN_POS ;    /* 目标位置: 画面中心 (640/2) */
 float slide_target_debug = 312;
uint32_t motor_zero_point =  0;

#define SLIDE_SERVO_RANGE      45    /* 最大角度范围，需保证一次循环能转完 */
#define SLIDE_VEL_LPF_ALPHA    0.3f    /* 速度低通滤波系数 */
#define SLIDE_VEL_FF_GAIN      0.35f   /* 速度前馈增益 */
#define SLIDE_X_LPF_ALPHA      0.3f   /* X坐标低通滤波系数，越小越平滑 */
#define SLIDE_ACC_GAIN         50.00f
#define ANGLE_COMPENSATION     3

static pid_init_config_s cfg = {   //动态pid这一块
	.mode    = PID_POSITION,
	.Kp      = 0.112f,     /* 比例: 每像素误差产生多少度倾角 */
	.Kd      = 0.012f,     /* 微分: 抑制震荡 */
	.Ki      = 0.001f,     /* 积分: 消除静差 */
	.max_out = SLIDE_SERVO_RANGE,
	.max_iout = 10.0f,
};

static pid_type_def slide_ball_pid;       /* 位置PID控制器 */
static float        slide_prev_x = SLIDE_ORIGIN_POS;   /* 上一帧 X 位置 */
int16_t        slide_velocity = 0;        /* 滤波后的小球速度 (px/s) */

/* 一阶低通滤波器状态 */
static float slide_x_lpf_out = SLIDE_ORIGIN_POS;
static float slide_v_lpf_out = 0.0f;

void Slider_Set_Pos_Pixel(const uint16_t pix_pos);

static void Gimbal_ZDT_UART_Send(const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) {
		while (DL_UART_isBusy(STEPPER_MOTOR_INST)) {}
		DL_UART_Main_transmitData(STEPPER_MOTOR_INST, data[i]);
	}
}


static void Slide_Control_Init(void)
{

    PID_init(&slide_ball_pid, &cfg);
	motor_l = get_motor_l_instance();
	motor_r = get_motor_r_instance();
    /* 滤波器初始化 */
    slide_x_lpf_out = slide_target_x;
    slide_v_lpf_out = 0.0f;
    slide_prev_x = slide_target_x;
}

/**
 * @brief 滑槽小球闭环控制 (在 Gimbal 200Hz 循环中调用)
 *
 * 算法:
 *   1. 从 K230 数据中取 x_position (0~640) 和 dt (帧间隔)
 *   2. 原始坐标一阶低通滤波
 *   3. 速度估计: v = Δx / dt, 再低通滤波
 *   4. 位置误差 → PID → 基础舵机角度
 *   5. 速度前馈: 球速越大 → 倾角补偿越大
 *   6. 合成最终角度, 限幅后输出到步进电机
 */
float acc_r = 0;
float acc_l = 0;
float x_raw = 0;
uint16_t speed = 0;
static void Slide_Control_Run(void)
{
    if (!K230_Read(&steel_ball_movement_data)) return;
	static float dt_pid = 0;
	float last_time = DWT_GetTimeline_ms();

	acc_r = motor_r -> acceleration;
	acc_l = motor_l -> acceleration;
	float acc_total = 0;
	if ((acc_l / acc_r) >= 0.8 && (acc_l / acc_r) <= 1.2) {
		acc_total = (acc_r >= acc_l) ? acc_r : acc_l;
	}
    /* 原始视觉坐标 */
    x_raw  = (float)steel_ball_movement_data.x_position;

    /* ========= X坐标一阶低通滤波 ========= */
    slide_x_lpf_out = SLIDE_X_LPF_ALPHA * x_raw + (1.0f - SLIDE_X_LPF_ALPHA) * slide_x_lpf_out;
    x = slide_x_lpf_out;
    /* ===================================== */

    dt = steel_ball_movement_data.dt;

    /* 保护: dt 异常时使用默认值 */
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f;  /* 默认 50ms (20fps) */
    }

    /* ── 1. 速度估计 (利用 dt 精确计算) ── */
    float raw_v = (x - slide_prev_x) / dt;
    slide_prev_x = x;

    /* 速度一阶低通滤波 */
    slide_v_lpf_out = SLIDE_VEL_LPF_ALPHA * raw_v + (1.0f - SLIDE_VEL_LPF_ALPHA) * slide_v_lpf_out;
    slide_velocity = (int16_t)(slide_v_lpf_out / 10.0f);

    /* ── 2. 位置 PID ── */
    float error = slide_target_x - x;
	//float error = slide_target_debug - x;
    PID_calc(&slide_ball_pid, 0.0f, error);

    /* ── 3. 速度前馈 ──
     * 球向右运动 (v>0) → 需要左倾 (减小角度) 来"接住"球
     * 球速越大 → 前馈幅度越大 → 电机提前动作 */
    velocity_ff = (float)slide_velocity * SLIDE_VEL_FF_GAIN;
	if (velocity_ff > 60.0f) {velocity_ff = 60.0f;}
	else if (velocity_ff < -60.0f) {velocity_ff = -60.0f;}

    /* ── 4. 合成角度 = PID输出 + 速度前馈 ── */
	target_angle_deg = slide_ball_pid.out + velocity_ff + (acc_total * SLIDE_ACC_GAIN);

	Chassis_State_Flag_To_Gimbal_e chassis_current_state = get_chassis_current_state();

	if (chassis_current_state == Chassis_Launching && chassis_cmd_receive.task_flag == 4) target_angle_deg += 2.6f;
	else if (chassis_current_state == Chassis_Stoping && chassis_cmd_receive.task_flag == 4) target_angle_deg -= 5;

	if (chassis_current_state == Chassis_Launching && chassis_cmd_receive.task_flag == 5) target_angle_deg += 2.6f;
	else if (chassis_current_state == Chassis_Stoping && chassis_cmd_receive.task_flag == 5) target_angle_deg -= 3;

	if (target_angle_deg >   SLIDE_SERVO_RANGE)  target_angle_deg =   SLIDE_SERVO_RANGE;
	if (target_angle_deg < -(SLIDE_SERVO_RANGE)) target_angle_deg = -(SLIDE_SERVO_RANGE);
	target_angle_deg += ANGLE_COMPENSATION;
	int32_t pulses = (int32_t)(target_angle_deg * 3200.0f / 360.0f);

	uint32_t pulse_count = (uint32_t)((pulses >= 0) ? pulses : -pulses);
	uint8_t dir = pulses >= 0 ? 1 : 0;

	float time_minute = dt_pid / 6000.0f;

	speed = dt_pid == 0 ? 40 : (uint16_t)(((float)pulse_count / 3200.0f) / time_minute);

    ZDT_Emm_Pos_Control(1, dir, speed, 250, (uint32_t)pulse_count, 1, false);

	dt_pid = DWT_GetTimeline_ms() - last_time;
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


	// Servo_Init_Config_s servo_yaw_config = {
	// 	.Servo_type = Servo180,
	// 	.inst = Servo_INST,
	// 	.idx = 0,    //对应DL_TIMER_CC_0_INDEX，PA17
	// };
	// servo_yaw = ServoInit(&servo_yaw_config);
	// Servo_Motor_Type_Select(servo_yaw, Free_Angle_mode);

	/* 初始化滑槽小球位置闭环 */
	Slide_Control_Init();

	DWT_Delay(1);
	//ZDT_Emm_Pos_Control(1, 1, 2000, 253, 0, 1, false);
	ZDT_Emm_Origin_Trigger_Return(1, 0, 0);
}

static uint8_t change_flag1 = 0;
static uint8_t change_flag2 = 0;
static uint8_t count1 = 0;
static uint8_t count2 = 0;
static uint8_t change_flag_mid = 0;   // 新增：0位到达标志
static uint8_t count_mid = 0;        // 新增：0位稳定计数
uint8_t disable_pid_flag = 0;
uint8_t enable_pid_flag = 0;
uint8_t set_zero_cmd_flag = 0;
uint8_t go_to_zero_cmd_flag = 0;
uint8_t forward_cmd_flag = 0;
uint8_t reverse_cmd_flag = 0;
void Gimbal(void)
{   //ZDT_Emm_Pos_Control(1, 1, 190, 0, 0, 1, false);
	xQueueReceive(gimbal_cmd_queue, &gimbal_cmd_receive, 1);
	xQueueReceive(chassis_cmd_queue, &chassis_cmd_receive, 1);
	//获取licheerv数据
	LicheeRec_Frame = LicheeRec_GetFrame();
	static uint8_t if_run_PID = 1;



	if (if_run_PID) Slide_Control_Run();

	// static uint16_t cntr = 0;
	// cntr ++;
	// if (cntr <= 1000) {
	// 	slide_target_debug = 312;
	// } else if (cntr >= 1000 && cntr < 2000) {
	// 	slide_target_debug = 180;
	// } else if (cntr >= 2000) {
	// 	slide_target_debug = 440;
	// 	if (cntr >= 3000) {cntr = 0;}
	// }

	static uint8_t cmd217_done_flag = 0;
	static uint8_t cmd216_done_flag = 0;

	switch (gimbal_cmd_receive.task_flag) {
		case 0:    //复位模式
			slide_target_x = SLIDE_ORIGIN_POS;
			change_flag1 = 0;
			change_flag2 = 0;
			change_flag_mid = 0;
			break;
		case 3:   //任务3，静止状态，使小球在+5——-5间折返
			if (!change_flag1) {
				slide_ball_pid.Iout = 0;
				slide_target_x = SLIDE_5CM_POS;   // 先到 +5
				change_flag1 = 1;
			}
			// 判断是否到达当前目标 (±30 像素内)
			if (fabsf(x_raw - slide_target_x) < 30) {
				// 阶段1：到达 +5
				if (change_flag1 && !change_flag_mid) {
					count1++;
				}
				// 阶段2：到达 0
				if (change_flag_mid && !change_flag2) {
					count_mid++;
				}
				// 阶段3：到达 -5
				if (change_flag2) {
					count2++;
				}
			}
			// 从 +5 切换到 0
			if (count1 > 40 && change_flag1 && !change_flag_mid) {
				count1 = 0;
				slide_target_x = SLIDE_ORIGIN_POS;  // 去原点
				change_flag_mid = 1;
			}
			// 从 0 切换到 -5
			if (count_mid > 20 && change_flag_mid && !change_flag2) {
				count_mid = 0;
				slide_target_x = SLIDE_D5CM_POS;    // 去 -5
				change_flag2 = 1;
			}
			// 到达 -5 后停车
			if (count2 > 120 && change_flag2) {
				car_stop = 1;
				count2 = 0;
			}
			break;
		case 216:
			//slide_target_x = 600 - LicheeRec_Frame.relative_position * 600;
			if (cmd216_done_flag == 0) {
				if_run_PID = 0;
				ZDT_Emm_Origin_Trigger_Return(1, 0, 0);
				cmd216_done_flag = 1;
			}
			cmd217_done_flag = 0;
			break;
		case 217:
			if (cmd217_done_flag == 0) {
				if_run_PID = 1;
				while (!K230_Read(&steel_ball_movement_data)){};
				Slider_Set_Pos_Pixel(steel_ball_movement_data.x_position);
				cmd217_done_flag = 1;
			}
			cmd216_done_flag = 0;
			break;
		case 218:
			cmd216_done_flag = 0;
			cmd217_done_flag = 0;
			break;
		case 77:
			if_run_PID = 0;
			if (disable_pid_flag ==0) {
				ZDT_Emm_En_Control(1, false, false);
				disable_pid_flag = 1;
			}
			break;
		case 78:
			if (set_zero_cmd_flag == 0) {
				if_run_PID = 0;
				ZDT_Emm_En_Control(1, true, false);
				DWT_Delay(1);
				ZDT_Emm_Origin_Set_O(1, 1);
				set_zero_cmd_flag = 1;
			}
			break;
		case 79:
			if (go_to_zero_cmd_flag == 0) {
				if_run_PID = 0;
				ZDT_Emm_En_Control(1, true, false);
				DWT_Delay(1);
				ZDT_Emm_Origin_Trigger_Return(1, 0, 0);
				go_to_zero_cmd_flag = 1;
			}
			break;
		case 80:
			if (enable_pid_flag == 0) {
				ZDT_Emm_En_Control(1, false, false);
				DWT_Delay(1);
				ZDT_Emm_En_Control(1, true, false);
				enable_pid_flag = 1;
				if_run_PID = 1;
			}
			break;
		case 81:
			if (forward_cmd_flag == 0) {
				if_run_PID = 0;
				ZDT_Emm_Pos_Control(1, 1, 1000, 0, 2, 0, false);\
				forward_cmd_flag = 1;
			}
			break;
		case 82:
			if (reverse_cmd_flag == 0) {
				if_run_PID = 0;
				ZDT_Emm_Pos_Control(1, 0, 1000, 0, 2, 0, false);
				reverse_cmd_flag = 1;
			}


			break;
		default:
			break;
	}
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

void Slider_Set_Pos_Pixel(const uint16_t pix_pos) {
	slide_target_x = (float)pix_pos;
}