#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "ti_msp_dl_config.h"
#define LENGTH_TO_CENTER 0.166 //PAW3395到两轮中心距离,单位m
void Chassis(void);
void Chassis_Init(void);

void Motor_Cmd_CallBack(uint8_t i);

void Chassis_get_init_angle(void);

typedef enum {
	Chassis_Line = 0,
	Chassis_Turn,
	Chassis_Stop,
}Chassis_Move_State_e;

#endif
