#ifndef _ROBOT_CMD_H_
#define _ROBOT_CMD_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"



void RobotCmd_Init(void);
void Robot_Cmd(void);

extern QueueHandle_t chassis_cmd_queue;
extern QueueHandle_t chassis_fetch_data_queue;
extern QueueHandle_t trace_fetch_data_queue;
extern QueueHandle_t gimbal_cmd_queue;
typedef enum {
	NORMAL_MODE=0,
	TRACE_MODE,
	IMU_MODE,
	POSITION_MODE,
}Chassis_Mode_e;

typedef struct {
		Chassis_Mode_e Chassis_Mode;
		uint8_t circle_set;
}chassis_cmd_q;

typedef struct {
		
}chassis_fetch_data_q;
typedef struct {
		float yaw;
		float pitch;
		float aim_x;
		float aim_y;
		uint8_t relay_on_flag;
		uint8_t task_flag;
}gimbal_cmd_q;

void Control_Switch_Callback(uint8_t i);
void Task_Callback(uint8_t i);
void Chassis_Mode_Switch_Callback(uint8_t i);
#endif

