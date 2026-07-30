#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "ti_msp_dl_config.h"
#define CHASSIS_LENGTH_TO_CENTER 0.166 //PAW3395到车体中心的距离,单位m //PAW3395���������ľ���,��λm
void Chassis(void);
void Chassis_Init(void);

void Motor_Cmd_CallBack(uint8_t i);

void Chassis_get_init_angle(void);
void Chassis_ResetAction(void);
uint8_t Chassis_MoveStraight(float distance_m, float speed_mps);
uint8_t Chassis_TurnAngle(float angle_deg, float max_turn_speed);
uint8_t Chassis_MoveArc(float radius_m, float total_angle_deg, float speed_mps);

typedef enum {
	Chassis_Line = 0,
	Chassis_Turn,
	Chassis_Stop,
}Chassis_Move_State_e;

#endif
