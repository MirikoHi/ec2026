/**
 * @file motor_task.c
 * @brief 电机控制任务调度实现
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        集中调度所有CAN电机类型的控制函数
 */

#include "motor_task.h"
#include "dji_motor.h"
#include "dmmotor.h"


/**
 * @brief 电机控制总调度
 *
 * @note  1kHz 调用
 *        按需取消注释其他电机类型
 */
void MotorControlTask(void)
{
    DJIMotorControl();
    DMMotorControl();
}
