#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

#include "ti_msp_dl_config.h"
#include "PID.h"
#include "encoder.h"
#include "misc.h"
#define MOTOR_PWM_MAX (2500-1)
#define SPEED_SMOOTH_COEF 0.85f //滤波系数
#define Control_Period 10   //单位ms
#define PulseofCirlce 	(13*10*40/3) 
#define MOTOR_MAX_NUM 2
#define ENCODER_TO_SPEED_MS (100 * 0.065 * PI * 3 / 10 / 13 / 2 / 40)  // 编码器到速度的转换系数//最大0.5m/s
#define ENCODER_TO_DISDAN_M (0.065 * PI * 3 / 13 / 10 / 2 / 40) //0.01==10cm




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

typedef struct {
	GPIO_Regs* EN_A_PORT;
	uint32_t EN_A_pin;
	GPIO_Regs* EN_B_PORT;
	uint32_t EN_B_pin;
	GPTIMER_Regs* inst;
	DL_TIMER_CC_INDEX idx;
} DCMotor_PortPin_s;

typedef struct {
		pid_type_def speed_pid;
		pid_type_def position_pid;
		ENCODER_RES *encoder;
		DCMotor_PortPin_s PortPin;
		Motor_Dir Input_Dir;
		Motor_Dir Output_Dir;
		uint8_t feedforward;
	
		Motor_Speed_Filter_e filter;
		float speed_measure;//rpm
		float speed_ref;
		float position_ref;
		float position_measure;
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
		
}DCMotorInitConfig_s;





DCMotorInstance* DCMotor_Init(DCMotorInitConfig_s *config);
void DCMotor_SetTraceCompensation(DCMotorInstance *motor,float compensation);
void Hw_Motor_Task(void);
void DCMotor_Cmd(DCMotorInstance* motor,State state);
void DCMotor_SetPosition(DCMotorInstance *motor,float Position);
#endif