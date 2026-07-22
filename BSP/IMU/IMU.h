#ifndef __IMU_H
#define __IMU_H

#include "board.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════
   ★ IMU 传感器选择宏开关
   1: 使用 BMI088 传感器
   0: 使用 ICM42688 传感器
   ═══════════════════════════════════════════════════════════════════════ */
#define USE_BMI088   1

#define M_PI  (float)3.14159265358979323846

typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;

extern xyz_f_t north, west;
extern volatile float yaw[5];
extern float TTangles_gyro[7];

// API 函数声明
void IMU_init(void);                    // IMU 初始化
void IMU_getYawPitchRoll(float * ypr);  // 获取姿态角 [0]:Yaw, [1]:Pitch, [2]:Roll
void IMU_TT_getgyro(float * zsjganda);  // 获取原始传感器数据
void MPU6050_InitAng_Offset(void);

#endif /* __IMU_H */