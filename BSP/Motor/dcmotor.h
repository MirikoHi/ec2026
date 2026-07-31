#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

#include "ti_msp_dl_config.h"
#include "PID.h"
#include "encoder.h"
#include "misc.h"
#define MOTOR_PWM_MAX (2500-1)
#define SPEED_SMOOTH_COEF 0.85f //滤波系数
#define Control_Period 10   //单位ms
#define ENCODER_COUNTS_PER_WHEEL_REV 728.0f
#define PulseofCirlce                728U
#define MOTOR_MAX_NUM 2
#define ENCODER_TO_SPEED_MS (100.0f * 0.065f * PI / ENCODER_COUNTS_PER_WHEEL_REV)
#define ENCODER_TO_DISDAN_M (0.065f * PI / ENCODER_COUNTS_PER_WHEEL_REV)




typedef enum {
	MOTOR_NORMAL=0,
	MOTOR_REVERSAL
}Motor_Dir;

#define FILTER_NUM 20  // 定义滤波器窗口大小
typedef struct {
    float MotorSpeedBuf[FILTER_NUM];
		uint8_t filter_index;  // 当前存储位置
		uint8_t filter_count;  // 有效数据计数
		float speed_filtered;
}Motor_Speed_Filter_e;

// EN_1:EN_2 = 1:0  电机正转
// EN_1:EN_2 = 0:1  电机反转
// EN_1:EN_2 = 0:0  电机停止
// EN_1:EN_2 = 1:1  电机刹车
typedef struct {
	GPIO_Regs* EN_1_PORT;
	uint32_t EN_1_pin;
	GPIO_Regs* EN_2_PORT;
	uint32_t EN_2_pin;
	GPTIMER_Regs* inst;
	DL_TIMER_CC_INDEX idx;
} DCMotor_PortPin_s;

typedef enum {
	ANGLE_MODE = 0,
	SPEED_MODE,
} Motor_Loop_Mode;
//todo:考虑加入更多状态，比如加入巡线的PID计算和不加入巡线的闭环

typedef struct {
		pid_type_def speed_pid;
		pid_type_def position_pid;
		ENCODER_RES *encoder;
		DCMotor_PortPin_s PortPin;
		Motor_Dir Input_Dir;
		Motor_Dir Output_Dir;
		uint8_t feedforward;

		Motor_Loop_Mode loop_mode;
		Motor_Speed_Filter_e filter;
		float speed_measure;//rpm
		float position_measure;
		/* 上层规划器基础轮速；与PID内部实际Ref分离，避免两个任务交替覆盖。 */
		float Speed_Ref_Command;
		float acceleration;        /* 加速度 (m/s²) */
		float prev_speed;          /* 上一周期速度, 用于计算加速度 */
		float dt;                  /* 两次回传数据间的时间间隔 (s) */
		float Trace_Compensation;
		State State;
}__attribute__((aligned(4)))DCMotorInstance;

typedef struct {
		pid_init_config_s speed_pid_config;
		pid_init_config_s position_pid_config;

		encoder_PortPin_s encoder_config;
		DCMotor_PortPin_s PortPin;
		uint8_t feedforward;
		Motor_Dir Input_Dir;
		Motor_Dir Output_Dir;
		Motor_Loop_Mode loop_mode;

}DCMotorInitConfig_s;





DCMotorInstance* DCMotor_Init(DCMotorInitConfig_s *config);
void DC_Motor_SetRef(DCMotorInstance * motor,float ref) ;
void DCMotor_SetTraceCompensation(DCMotorInstance *motor,float compensation);
void Hw_Motor_Task(void);
void DCMotor_Cmd(DCMotorInstance* motor,State state);
#endif
