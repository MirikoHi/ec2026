#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "ti_msp_dl_config.h"

/* BC实际退出角度，单位deg；调小可减轻C点超调。 */
#define CHASSIS_BC_EXIT_ANGLE_DEG 165.0f

/* H题体育场路线参数：长度单位m，速度单位m/s，角度单位deg。 */
#define CHASSIS_STRAIGHT_DIST_M 1.50f									/* AB/CD直线长度，单位m。 */
#define CHASSIS_ARC_RADIUS_M 0.50f										/* 赛道半圆几何半径，单位m。 */
#define CHASSIS_TRACK_WIDTH_M 0.21f										/* 左右驱动轮接地点间距，单位m。 */
#define CHASSIS_TRACE_FORWARD_OFFSET_M 0.22f							/* 驱动轴中点到探头的前向距离，单位m。 */
#define CHASSIS_AXLE_TO_CENTER_FORWARD_M 0.0f							/* 车体中心在驱动轴前方为正，单位m。 */
#define CHASSIS_TRACE_TO_CENTER_M (CHASSIS_TRACE_FORWARD_OFFSET_M - CHASSIS_AXLE_TO_CENTER_FORWARD_M) /* 探头过线后轴线到A线的补偿距离，单位m。 */
#define CHASSIS_B_TURN_PREVIEW_M 0.00f									/* B点提前切换距离，单位m；调大可抵消差速建立延迟。 */
#define CHASSIS_ARC_DRIVE_RADIUS_M 0.60f								/* 实车差速控制半径，单位m；调小会转得更紧。 */
#define CHASSIS_FINISH_DETECT_RATIO 0.85f								/* DA弧线完成该比例后才允许识别A线。 */
#define CHASSIS_ARC1_EXIT_ANGLE_DEG 170.0f								/* BC弧线状态的退出角度，单位deg。 */
#define CHASSIS_ARC_ANGLE_DEG 180.0f									/* 半圆理论转角，单位deg。 */
#define CHASSIS_FINAL_APPROACH_SPEED 0.08f								/* 漏检时搜索A线的低速，单位m/s。 */
#define CHASSIS_FINAL_MAX_DIST_M 0.50f									/* 漏检保护最多额外前进距离，单位m。 */
#define CHASSIS_START_LINE_SENSORS 4U									/* 判定横向A线所需的连续黑色探头数。 */
#define CHASSIS_LINE_DETECT_DEBOUNCE 3U									/* 横线连续确认周期数。 */
#define CHASSIS_PI 3.14159265f											/* 圆周率，用于弧长和航向换算。 */
#define CHASSIS_STADIUM_STRAIGHT_SPEED 0.26f							/* 普通任务直线速度，单位m/s。 */
#define CHASSIS_STADIUM_ARC_SPEED 0.22f									/* 普通任务弧线速度，单位m/s。 */
#define CHASSIS_STADIUM_MAX_ACCEL 0.18f									/* 普通任务最大加速度，单位m/s^2。 */
#define CHASSIS_STADIUM_MAX_DECEL 0.22f									/* 普通任务最大减速度，单位m/s^2。 */
#define CHASSIS_STADIUM_MAX_JERK 0.80f									/* 普通任务最大加加速度，单位m/s^3。 */
#define CHASSIS_STADIUM_CONTROL_DT 0.005f								/* 控制器调用周期，单位s。 */
#define CHASSIS_FAST_STRAIGHT_SPEED 0.42f								/* 快速任务直线速度，单位m/s。 */
#define CHASSIS_FAST_ARC_SPEED 0.34f									/* 快速任务弧线速度，单位m/s。 */
#define CHASSIS_FAST_MAX_ACCEL 0.65f									/* 快速任务最大加速度，单位m/s^2。 */
#define CHASSIS_FAST_MAX_DECEL 0.75f									/* 快速任务最大减速度，单位m/s^2。 */
#define CHASSIS_FAST_MAX_JERK 4.00f										/* 快速任务最大加加速度，单位m/s^3。 */
#define CHASSIS_FAST_FINISH_SPEED 0.03f									/* 快速任务终点搜索最低速度，单位m/s。 */
#define CHASSIS_FAST_BRAKE_MARGIN 1.50f									/* 快速任务制动距离放大系数。 */
#define CHASSIS_CLOCKWISE_YAW_SIGN (-1.0f)								/* 顺时针时IMU yaw的变化符号。 */
#define CHASSIS_YAW_FEEDBACK_MAX_MPS 0.06f								/* 航向反馈转换成轮速补偿的最大值，单位m/s。 */

/* 体育场路线状态：直线、半圆、终点搜索和停车。 */
typedef enum {
    CHASSIS_STADIUM_STRAIGHT_1 = 0, /* A到B直线。 */
    CHASSIS_STADIUM_ARC_1,          /* B到C半圆。 */
    CHASSIS_STADIUM_STRAIGHT_2,     /* C到D直线。 */
    CHASSIS_STADIUM_ARC_2,          /* D到A半圆。 */
    CHASSIS_STADIUM_FINISH_BRAKE,   /* 圆弧结束后的终点搜索。 */
    CHASSIS_STADIUM_STOP            /* 停车并保持零速度。 */
} Chassis_Stadium_Step_e;
#define CHASSIS_LENGTH_TO_CENTER 0.166 //PAW3395到车体中心的距离,单位m //PAW3395���������ľ���,��λm
void Chassis(void);
void Chassis_Init(void);

void Motor_Cmd_CallBack(uint8_t i);

void Chassis_get_init_angle(void);
void Chassis_ResetAction(void);
uint8_t Chassis_MoveStraight(float distance_m, float speed_mps);
uint8_t Chassis_TurnAngle(float angle_deg, float max_turn_speed);
uint8_t Chassis_MoveArc(float radius_m, float total_angle_deg, float speed_mps);

/* 供滚球控制器读取的底盘轨迹前馈，单位分别为 m/s 和 m/s^2。 */
float Chassis_GetCommandedSpeed(void);
float Chassis_GetCommandedAcceleration(void);
float Chassis_GetCommandedLateralAcceleration(void);
float Chassis_GetCommandedYawRate(void);
float Chassis_GetRunTimeSeconds(void);
uint8_t Chassis_IsRunTimerActive(void);
uint8_t Chassis_GetStadiumStep(void);

/** 请求比赛任务立即停车；由底盘控制任务在下一个 5 ms 周期执行。 */
void Chassis_RequestEmergencyStop(void);

typedef enum {
	Chassis_Line = 0,
	Chassis_Turn,
	Chassis_Stop,
}Chassis_Move_State_e;

#endif
