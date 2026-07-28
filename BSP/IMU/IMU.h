/**
 ******************************************************************************
 * @file    IMU.h
 * @author  Geng LX
 * @brief   IMU attitude estimation using Mahony complementary filter
 *
 * @note    Uses the Mahony AHRS algorithm (ported from Legacy/Mahony.c)
 *          to fuse ICM42688 gyroscope + accelerometer data.
 *          Outputs Euler angles (yaw/pitch/roll) in degrees.
 ******************************************************************************
 */

#ifndef __IMU_H
#define __IMU_H

#include "board.h"
#include <math.h>

#define M_PI  (float)3.1415926535f

/* ═══════════════════════════════════════════════════════════════════════
 * Type Definitions
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

/* ═══════════════════════════════════════════════════════════════════════
 * Global Variables (extern)
 * ═══════════════════════════════════════════════════════════════════════ */

/** North / West direction vectors in body frame (from rotation matrix) */
extern xyz_f_t north, west;

/** Raw yaw history buffer */
extern volatile float yaw[5];

/** Raw sensor data: [ax, ay, az, gx, gy, gz, reserved] */
extern float motion6[7];

/* ═══════════════════════════════════════════════════════════════════════
 * Mahony AHRS API (from Legacy/Mahony.c)
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize Mahony filter (Kp=1.5, Ki=0.05, quaternion=[1,0,0,0])
 */
void MahonyAHRSinit(void);

/**
 * @brief  Mahony complementary filter update
 * @param  gx, gy, gz  Gyroscope data [rad/s]
 * @param  ax, ay, az  Accelerometer data [m/s²]
 * @param  dt_acc      Time delta for accel correction [s]
 * @param  dt_gyro     Time delta for gyro integration [s]
 */
void MahonyAHRSupdate(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float dt_acc, float dt_gyro);

/**
 * @brief  Get current quaternion [w, x, y, z]
 */
void MahonyGetQuaternion(float *q0, float *q1, float *q2, float *q3);

/**
 * @brief  Get Euler angles in degrees (ZYX convention)
 * @param  roll   [out] Roll angle [deg]
 * @param  pitch  [out] Pitch angle [deg]
 * @param  yaw    [out] Yaw angle [deg]
 */
void MahonyGetEuler(float *roll, float *pitch, float *yaw);

/**
 * @brief  Get 3x3 rotation matrix (row-major)
 */
void MahonyGetRotationMatrix(float R[3][3]);

/* ═══════════════════════════════════════════════════════════════════════
 * Application-level IMU API (backward compatible)
 * ═══════════════════════════════════════════════════════════════════════ */

/** Initialize IMU hardware + Mahony filter */
void IMU_init(void);

/** Get current yaw/pitch/roll in degrees: angles[0]=yaw, [1]=pitch, [2]=roll */
void IMU_getYawPitchRoll(float *ypr);

/** Get raw sensor data from last read */
void IMU_TT_getgyro(float *zsjganda);

/** Stub (kept for compatibility) */
void MPU6050_InitAng_Offset(void);

#endif /* __IMU_H */
