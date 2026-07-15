/**
 * @file motor_task.h
 * @brief 电机控制任务调度
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        MotorControlTask() 在 FreeRTOS 中以 1kHz 频率运行
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

/**
 * @brief 电机控制闭环任务
 *
 * @note  推荐在 RTOS 中以 1kHz 频率调用
 *        任务内调用: DJIMotorControl(), DMMotorControl()
 *        可根据需要添加其他电机类型
 */
void MotorControlTask(void);

#endif // MOTOR_TASK_H
