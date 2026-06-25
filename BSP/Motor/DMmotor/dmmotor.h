/**
 * @file dmmotor.h
 * @brief DM 达妙电机驱动（DM4310/DM6006 等MIT协议电机）
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        适配: 单MCAN0, 当前项目 PID (BSP/Control/PID.h)
 *        支持MIT模式（位置/速度/力矩+KP/KD）
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "PID.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4  /**< 最大DM电机数量 */

/* DM电机物理限幅 */
#define DM_P_MIN  (-12.56637f)
#define DM_P_MAX   12.56637f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX   45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/* ========================== 数据结构 ========================== */

/** DM电机测量值 */
typedef struct
{
    uint8_t  id;
    uint8_t  state;
    float    velocity;
    float    last_position;
    float    position;
    float    torque;
    float    T_Mos;
    float    T_Rotor;
    float    angle_single_round;
    int32_t  total_round;
    float    total_angle;
} DM_Motor_Measure_s;

/** DM电机发送帧 */
typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

/** DM电机实例 */
typedef struct
{
    DM_Motor_Measure_s     measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s     motor_controller;

    CANInstance           *motor_can_instance;

    Motor_Type_e           motor_type;
    Motor_Working_Type_e   stop_flag;

    DaemonInstance        *motor_daemon;
    uint32_t               lost_cnt;

    float _DM_T_MAX;   /**< 最大扭矩 */
    float _DM_V_MAX;   /**< 最大速度 */
    float _DM_P_MAX;   /**< 最大位置 */
} DMMotorInstance;

/** DM电机模式命令 */
typedef enum
{
    DM_CMD_MOTOR_MODE    = 0xFC,  /**< 使能 */
    DM_CMD_RESET_MODE    = 0xFD,  /**< 停止 */
    DM_CMD_ZERO_POSITION = 0xFE,  /**< 设置零位 */
    DM_CMD_CLEAR_ERROR   = 0xFB,  /**< 清除错误 */
} DMMotor_Mode_e;

/* ========================== 公共API ========================== */

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);

/** @brief DM电机控制（由 motor_task 以 1kHz 调用） */
void DMMotorControl(void);

/**
 * @brief 浮点数与无符号整数的映射
 * @param x     浮点值
 * @param x_min 最小值
 * @param x_max 最大值
 * @param bits  位宽
 * @return 映射后的无符号整数值
 */
static inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

#endif // DMMOTOR_H
