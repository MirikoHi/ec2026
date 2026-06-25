/**
 * @file motor_def.h
 * @brief 电机通用数据结构定义
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        适配当前项目：PID 使用 BSP/Control/PID.h 的 pid_type_def
 *        CAN 使用 BSP/CAN/bsp_can.h 的 CAN_Init_Config_s（单MCAN0）
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "PID.h"
#include "bsp_can.h"
#include "stdint.h"
#include "misc.h"         /* PI, limit_max_min 等 */

/* ========================== 宏工具 ========================== */

#define LIMIT_MIN_MAX(x, min, max) \
    do {                           \
        if ((x) < (min))           \
            (x) = (min);           \
        else if ((x) > (max))      \
            (x) = (max);           \
    } while (0)

/** RPM 转 角度/秒 (度/s) */
#define RPM_2_ANGLE_PER_SEC   6.0f
/** 弧度 → 度 */
#define RAD_2_DEGREE          57.2957795f
/** 度 → 弧度 */
#define DEGREE_2_RAD          0.01745329252f

/* ========================== 枚举定义 ========================== */

/** 闭环类型 */
typedef enum
{
    OPEN_LOOP              = 0b0000,
    CURRENT_LOOP           = 0b0001,
    SPEED_LOOP             = 0b0010,
    ANGLE_LOOP             = 0b0100,

    /* 组合标记（仅用于检查） */
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP   = 0b0110,
    ALL_THREE_LOOP         = 0b0111,
} Closeloop_Type_e;

/** 前馈类型 */
typedef enum
{
    FEEDFORWARD_NONE               = 0b00,
    CURRENT_FEEDFORWARD            = 0b01,
    SPEED_FEEDFORWARD              = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD  = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/** 反馈来源 */
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED = 1,
} Feedback_Source_e;

/** 电机转向 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL  = 0,
    MOTOR_DIRECTION_REVERSE = 1,
} Motor_Reverse_Flag_e;

/** 反馈方向 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL  = 0,
    FEEDBACK_DIRECTION_REVERSE = 1,
} Feedback_Reverse_Flag_e;

/** 启停标志 */
typedef enum
{
    MOTOR_STOP    = 0,
    MOTOR_ENABLED = 1,
} Motor_Working_Type_e;

/** 功率限制 */
typedef enum
{
    NO_POWER_LIMIT = 0,
    POWER_LIMIT_ON = 1,
} Power_Limit_Type_e;

/** 控制方式 */
typedef enum
{
    CONTROL_TYPE_NONE = 0,
    CURRENT_CONTROL,
    VOLTAGE_CONTROL,
} Motor_Control_Type_e;

/** 电机类型 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    DM4310,
    DM6006,
} Motor_Type_e;

/** 闭环角类型 */
typedef enum
{
    SINGLE_ANGLE = 0,
    TOTAL_ANGLE  = 1,
} Motor_Close_Type;

/* ========================== 结构体定义 ========================== */

/** 电机控制设置 */
typedef struct
{
    Closeloop_Type_e        outer_loop_type;         /**< 最外层闭环 */
    Closeloop_Type_e        close_loop_type;         /**< 启用的闭环级数 */
    Motor_Reverse_Flag_e    motor_reverse_flag;      /**< 是否反转 */
    Feedback_Reverse_Flag_e feedback_reverse_flag;   /**< 反馈是否反向 */
    Feedback_Source_e       angle_feedback_source;   /**< 角度反馈源 */
    Feedback_Source_e       speed_feedback_source;   /**< 速度反馈源 */
    Feedfoward_Type_e       feedforward_flag;        /**< 前馈标志 */
    Power_Limit_Type_e      power_limit_flag;        /**< 功率限制标志 */
    float                   angle_direction;         /**< 角度正方向偏移 */
} Motor_Control_Setting_s;

/** 电机控制器（PID + 反馈指针） */
typedef struct
{
    float *other_angle_feedback_ptr;    /**< 外部角度反馈数据指针 */
    float *other_speed_feedback_ptr;    /**< 外部速度反馈数据指针 */
    float *speed_feedforward_ptr;       /**< 速度前馈数据指针 */
    float *current_feedforward_ptr;     /**< 电流前馈数据指针 */

    pid_type_def current_PID;           /**< 电流环PID */
    pid_type_def speed_PID;             /**< 速度环PID */
    pid_type_def angle_PID;             /**< 角度环PID */

    float pid_ref;                      /**< PID参考输入 */
    float motor_power_max;              /**< 功率上限 */
    float motor_power_predict;          /**< 预测功率 */
    float motor_power_scale;            /**< 功率缩放系数 */
} Motor_Controller_s;

/** 电机控制器初始化配置 */
typedef struct
{
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    pid_init_config_s current_PID;
    pid_init_config_s speed_PID;
    pid_init_config_s angle_PID;
} Motor_Controller_Init_s;

/** 电机通用初始化配置（所有CAN电机类型共用） */
typedef struct
{
    Motor_Controller_Init_s  controller_param_init_config;
    Motor_Control_Setting_s  controller_setting_init_config;
    Motor_Type_e             motor_type;
    CAN_Init_Config_s        can_init_config;
    Motor_Close_Type         motor_close_type;
    Motor_Control_Type_e     motor_control_type;
    float _DM_T_MAX;            /**< DM电机最大扭矩 */
    float _DM_V_MAX;            /**< DM电机最大速度 */
    float _DM_P_MAX;            /**< DM电机最大位置 */
} Motor_Init_Config_s;

#endif // MOTOR_DEF_H
