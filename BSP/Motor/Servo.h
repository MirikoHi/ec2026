/**
* @file servo_motor.h
 * @author panrui
 * @brief 舵机控制头文件
 * @version 0.1
 * @date 2022-12-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define SERVO_MOTOR_CNT 7

#define SERVO_TICKS_PER_MS (Servo_INST_CLK_FREQ / 1000) // 每毫秒的tick数，400
#define SERVO_PERIOD_TICKS 8000                         // 20ms周期对应的tick数
#define SERVO_SET_PULSE(servo, pulse_ticks) \
DL_Timer_setCaptureCompareValue((servo)->inst, SERVO_PERIOD_TICKS - (pulse_ticks), (servo)->idx)

/*各种舵机类型*/
typedef enum
{
    Servo180 = 0,
    Servo270 = 1,
    Servo360 = 2,
} Servo_Type_e;

/*舵机模式选择*/
typedef enum
{
    Free_Angle_mode, // 任意角度模式
    Start_mode,      // 起始角度模式
    Final_mode,      // 终止角度模式
} Servo_Angle_Type_e;
/*角度设置*/
typedef struct
{
    /*起止角度模式设置值*/
    uint16_t Init_angle;
    uint16_t Final_angle;
    /*任意角度模式设置值*/
    uint16_t free_angle;
    /*下述值仅仅适用于360°舵机
     *设定值为0-100 为速度值百分比
     *0-50为正转 速度由快到慢
     *51-100为反转 速度由慢到快
     */
    int16_t servo360speed;
} Servo_Angle_s;

/* 用于初始化不同舵机的结构体,各类舵机通用 */
typedef struct
{
    Servo_Type_e Servo_type;
    Servo_Angle_Type_e Servo_Angle_Type;
    // 使用的定时器实例及比较通道（syscfg生成）
    GPTIMER_Regs *inst;
    /*idx值设定
     *DL_TIMER_CC_0_INDEX ~ DL_TIMER_CC_3_INDEX
     *如PA28对应 Servo_INST + GPIO_Servo_C0_IDX
     */
    DL_TIMER_CC_INDEX idx;

} Servo_Init_Config_s;
typedef struct
{
    Servo_Angle_Type_e Servo_Angle_Type;
    Servo_Angle_s Servo_Angle;
    Servo_Type_e Servo_type;
    // 使用的定时器实例及比较通道（syscfg生成）
    GPTIMER_Regs *inst;
    /*idx值设定
     *DL_TIMER_CC_0_INDEX ~ DL_TIMER_CC_3_INDEX
     *如PA28对应 Servo_INST + GPIO_Servo_C0_IDX
     */
    DL_TIMER_CC_INDEX idx;
} ServoInstance;

ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config);
void Servo_Motor_FreeAngle_Set(ServoInstance *Servo_Motor, uint16_t S_angle);
void Servo_Motor_StartSTOP_Angle_Set(ServoInstance *Servo_Motor, int16_t Start_angle, int16_t Final_angle);
void Servo_Motor_Type_Select(ServoInstance *Servo_Motor,int16_t mode);
void ServeoMotorControl();
#endif // SERVO_MOTOR_H