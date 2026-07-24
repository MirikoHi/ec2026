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

#ifndef M_PI
#define M_PI  (float)3.14159265358979323846f
#endif

#define DEG_TO_RAD  (M_PI / 180.0f)
#define RAD_TO_DEG  (180.0f / M_PI)

typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

/* 外部全局变量 */
extern xyz_f_t north, west;
extern volatile float yaw[5];
extern float motion6[7];

/* API 函数声明 */
void IMU_init(void);                    // 初始化硬件与 Mahony 算法
void IMU_getYawPitchRoll(float *ypr);  // 获取姿态角 ypr[0]=Yaw, ypr[1]=Pitch, ypr[2]=Roll (度)
void IMU_TT_getgyro(float *zsjganda);  // 获取原始传感器数据
void MPU6050_InitAng_Offset(void);

#endif /* __IMU_H */