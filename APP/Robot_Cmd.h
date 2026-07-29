#ifndef _ROBOT_CMD_H_
#define _ROBOT_CMD_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
/* ELRS 已弃用 */
#include "BlueTooth_uart.h"



void RobotCmd_Init(void);
void Robot_Cmd(void);

extern QueueHandle_t chassis_cmd_queue;
extern QueueHandle_t chassis_fetch_data_queue;
extern QueueHandle_t trace_fetch_data_queue;
extern QueueHandle_t gimbal_cmd_queue;
extern BlueTooth_Tx_t g_bt_tx;
extern volatile BlueTooth_Rx_t g_bt_rx;

typedef enum {
	NORMAL_MODE=0,
	TRACE_MODE,
	IMU_MODE,
	POSITION_MODE,
	REMOTE_MODE,
}Chassis_Mode_e;

typedef struct {
		Chassis_Mode_e Chassis_Mode;
		uint8_t circle_set;
		uint8_t remote_disable;
		float remote_forward;
		float remote_turn;
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

//这里一定要pack取消对齐字节，会出现双板字节位数对不上，回调函数中直接return
//todo：这里是和英雄云台C板的通信帧，待修改
#pragma pack(1)
typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
	// attitude_t chassis_imu_data;
#endif
	// 后续增加底盘的真实速度
	float real_vx;
	float real_vy;
	float real_wz;
	float cap_vol;
	uint8_t INS_IsOnline;//底盘陀螺仪状态
} Chassis_Upload_Data_s;

typedef enum
{
	CHASSIS_ZERO_FORCE = 0,    // 电流零输入
	CHASSIS_ROTATE,            // 小陀螺模式
	CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
	CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
	CHASSIS_FOLLOW_GIMBAL_YAW_REVERSE,
	CHASSIS_CLIMB_GIMBAL_YAW_REVERSE,//回头爬台阶模式
	CHASSIS_VERTICAL_YAW,
	CHASSIS_FIXED,
} chassis_mode_e;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
	// 控制部分
	float vx;           // 前进方向速度,   范围-660x100 -- 660x100
	float vy;           // 横移方向速度
	float wz;           // 旋转速度
	float offset_angle; // 底盘和归中位置的夹角
	chassis_mode_e chassis_mode;
	uint16_t chassis_power_buffer;  //底盘缓冲能量
	uint16_t chassis_power_limit;   //底盘功率上限
	uint8_t Chassis_Power_On; //底盘供电显示
	float gimbal_wz;//单位度每秒
	uint8_t cap_open;
	uint8_t push_manual;//手动推推杆
	uint8_t system_reset;//双板软件复位
	// UI部分
	//  ...

} Chassis_Ctrl_Cmd_s;
#pragma pack()

void Control_Switch_Callback(uint8_t i);
void Task_Callback(uint8_t i);
void Chassis_Mode_Switch_Callback(uint8_t i);
#endif
