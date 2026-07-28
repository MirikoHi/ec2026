#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "ti_msp_dl_config.h"
#include "flash_param_store.h"
#define CHASSIS_LENGTH_TO_CENTER 0.166 //PAW3395到车体中心的距离,单位m //PAW3395���������ľ���,��λm
void Chassis_task(void);
void Chassis_Init(void);
void Chassis_FillDefaultParams(FlashParam_Data_s *params);
void Chassis_ApplyParams(const FlashParam_Data_s *params);

// void Motor_Cmd_CallBack(uint8_t i);

void Chassis_get_init_angle(void);
void Chassis_ResetAction(void);
uint8_t Chassis_MoveStraight(float distance_m, float speed_mps);
uint8_t Chassis_TurnAngle(float angle_deg, float max_turn_speed);

typedef enum {
	Chassis_Line = 0,
	Chassis_Turn,
	Chassis_Stop,
}Chassis_Move_State_e;
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
#endif
