#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "PID.h"
#include "ti_msp_dl_config.h"
#define CHASSIS_LENGTH_TO_CENTER 0.166 //PAW3395到车体中心的距离,单位m //PAW3395���������ľ���,��λm
void Chassis(void);
void Chassis_Init(void);

void Motor_Cmd_CallBack(uint8_t i);

void Chassis_get_init_angle(void);
void Chassis_ResetAction(void);
uint8_t Chassis_MoveStraight(float distance_m, float speed_mps);
uint8_t Chassis_TurnAngle(float angle_deg, float max_turn_speed);

typedef enum {
	Chassis_Line = 0,
	Chassis_Turn,
	Chassis_Stop,
}Chassis_Move_State_e;

/* 当前比赛动作预留 4 段直线和 4 次转弯，后续扩展需要同步提升版本号。 */
#define FLASH_PARAM_LINE_SEGMENT_COUNT  4U
#define FLASH_PARAM_TURN_SEGMENT_COUNT  4U
typedef struct
{
	/* 循迹、直线航向、转向三个 PID 参数。 */
	pid_init_config_s trace_pid;
	pid_init_config_s line_yaw_pid;
	pid_init_config_s turn_pid;

	/* 每段动作的距离和角度，单位分别是 m 和 deg。 */
	float line_distance_m[FLASH_PARAM_LINE_SEGMENT_COUNT];
	float turn_angle_deg[FLASH_PARAM_TURN_SEGMENT_COUNT];

	/* 直线速度曲线和完成判定参数。 */
	float line_accel_m;
	float line_slowdown_m;
	float line_min_speed_mps;
	float line_done_err_m;
	uint16_t line_done_ticks;
	uint16_t reserved0;

	// /* 曲线速度曲线和完成判定参数。 */
	// float semi_accel_m;
	// float line_slowdown_m;
	// float line_min_speed_mps;
	// float line_done_err_m;
	// uint16_t line_done_ticks;
	// uint16_t reserved0;

	/* 转向完成判定和动作默认速度。 */
	float turn_done_err_deg;
	uint16_t turn_done_ticks;
	uint16_t reserved1;
	float action_speed_mps;
	float turn_speed_mps;
} FlashParam_Data_s;
/**
 * @brief 获取左右轮滤波后的实际速度，单位m/s
 */
void Chassis_GetMotorSpeed(float *left_speed,
						   float *right_speed);
void Stop_Detect(void);
void Chassis_State_Turn(void);
void Chassis_Set_Turn(void);
static void Chassis_RemoteControl(void);
static void Chassis_ClearRemoteSpeed(void);
static void Chassis_RemoteLostDisable(void);
static void Chassis_Trace_Cal(void);
static void Chassis_ImuModeAction(void);
static void Motor_FeedForward_Update(void);
static void Chassis_ResetEncoderOdom(void);
static float Chassis_GetForwardOdom(void);
static float Chassis_GetYawDeg(void);
static float Chassis_AngleNormalize(float angle);
static float Chassis_LimitAbs(float value, float limit);
void Chassis_Set_Line(float position);
static void Chassis_Test_Line(void);
static uint8_t Chassis_SemiCircle(float radius_m, float speed_mps, int direction);
#endif
