#ifndef __IMU_MAHONY_H__
#define __IMU_MAHONY_H__

#include "board.h"

typedef struct {
    float x;
    float y;
    float z;
} imu_mahony_xyz_t;

extern imu_mahony_xyz_t imu_mahony_north;
extern imu_mahony_xyz_t imu_mahony_west;
extern volatile float imu_mahony_yaw[5];
extern float imu_mahony_motion6[7];

void IMU_Mahony_Init(void);
uint8_t IMU_Mahony_IsReady(void);
void IMU_Mahony_GetYawPitchRoll(float *ypr);
void IMU_Mahony_GetRawMotion(float *motion);
void IMU_Mahony_ResetOffset(void);

#endif
