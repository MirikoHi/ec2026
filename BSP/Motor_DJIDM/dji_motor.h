/**
 * @file dji_motor.h
 * @brief DJI 智能电机驱动（M2006/M3508/GM6020）
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        适配: 单MCAN0, 当前项目 PID (BSP/Control/PID.h)
 *        支持 ≤4 台DJI电机的分组发送
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "PID.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define DJI_MOTOR_CNT 4  /**< 最大DJI电机数量 */

/* 滤波系数 */
#define SPEED_SMOOTH_COEF   0.85f
#define CURRENT_SMOOTH_COEF 0.9f
/* 编码器角度系数 (360/8192) */
#define ECD_ANGLE_COEF_DJI  0.043945f

/* ========================== 数据结构 ========================== */

/** DJI电机测量值 */
typedef struct
{
    uint16_t last_ecd;         /**< 上次编码器值 */
    uint16_t ecd;              /**< 当前编码器值 (0-8191) */
    float    angle_single_round; /**< 单圈角度 */
    float    speed_aps;        /**< 角速度 (度/秒) */
    int16_t  real_current;     /**< 实际电流 */
    uint8_t  temperature;      /**< 温度 (°C) */

    float    total_angle;      /**< 总角度 */
    int32_t  total_round;      /**< 总圈数 */
} DJI_Motor_Measure_s;

/** DJI电机实例 */
typedef struct
{
    DJI_Motor_Measure_s    measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s     motor_controller;

    CANInstance           *motor_can_instance;  /**< 接收用CAN实例 */
    uint8_t                sender_group;        /**< 发送分组 */
    uint8_t                message_num;         /**< 组内序号 */

    Motor_Type_e           motor_type;
    Motor_Control_Type_e   motor_control_type;
    Motor_Working_Type_e   stop_flag;
    Motor_Close_Type       motor_close_type;

    DaemonInstance        *daemon;
    uint32_t               feed_cnt;
    float                  dt;
} DJIMotorInstance;

/* ========================== 公共API ========================== */

/**
 * @brief 初始化DJI电机
 *
 * @param config 电机初始化配置
 * @return DJIMotorInstance* 实例指针
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置电机参考值
 *
 * @param motor 电机实例
 * @param ref   参考值（取决于最外层闭环类型：角度/速度/电流）
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/**
 * @brief 切换外层闭环类型
 *
 * @param motor      电机实例
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

/**
 * @brief 电机控制任务（由 motor_task 周期性调用）
 */
void DJIMotorControl(void);

/**
 * @brief 停止电机（直接置零电流）
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief 使能电机
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief 切换反馈来源（如改用IMU反馈做小陀螺）
 *
 * @param motor 电机实例
 * @param loop  要切换的闭环
 * @param type  反馈来源类型
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

#endif // DJI_MOTOR_H
