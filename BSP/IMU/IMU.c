/**
 ******************************************************************************
 * @file    IMU.c
 * @author  Geng LX
 * @brief   IMU attitude estimation — Mahony complementary filter
 *
 * @note    Fuses ICM42688 gyroscope (rad/s) + accelerometer (m/s²) using
 *          the Mahony AHRS algorithm ported from Legacy/Mahony.c.
 *
 *          Data flow:
 *            ICM42688 (SPI burst read)
 *              → acc [m/s²], gyro [rad/s]
 *              → MahonyAHRSupdate (gyro + acc → quaternion)
 *              → MahonyGetEuler (quaternion → yaw/pitch/roll in degrees)
 *
 *          Called at ~200Hz from the Chassis task.
 ******************************************************************************
 */

#include "IMU.h"
#include "icm42688.h"
#include "dwt.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Global Variables (extern in IMU.h)
 * ═══════════════════════════════════════════════════════════════════════ */

xyz_f_t north = {0.0f, 0.0f, 0.0f};
xyz_f_t west  = {0.0f, 0.0f, 0.0f};

volatile float yaw[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float motion6[7] = {0.0f};

/* ═══════════════════════════════════════════════════════════════════════
 * Mahony AHRS — Internal State
 *
 *   q0, q1, q2, q3  — attitude quaternion (w, x, y, z)
 *   exInt, eyInt, ezInt — integral error terms (gyro bias estimate)
 *   twoKp, twoKi     — 2× proportional / integral gain
 * ═══════════════════════════════════════════════════════════════════════ */

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float twoKp = 3.0f;    // 2 × 1.5
static float twoKi = 0.1f;    // 2 × 0.05

/* ═══════════════════════════════════════════════════════════════════════
 * MahonyAHRSinit
 *
 * Initialize filter state:
 *   - Kp = 1.5 (proportional gain for accelerometer correction)
 *   - Ki = 0.05 (integral gain for gyro bias estimation)
 *   - quaternion = identity (no rotation)
 *   - integral terms = zero
 * ═══════════════════════════════════════════════════════════════════════ */

void MahonyAHRSinit(void)
{
    twoKp = 2.0f * 1.5f;       // Kp = 1.5
    twoKi = 2.0f * 0.05f;      // Ki = 0.05
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════
 * normalize — keep quaternion unit-length
 * ═══════════════════════════════════════════════════════════════════════ */

static void normalize(void)
{
    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (n < 1e-12f) {
        /* Degenerate — reset to identity */
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv;
}

/* ═══════════════════════════════════════════════════════════════════════
 * MahonyAHRSupdate — main complementary filter step
 *
 * Called once per sensor sample. Does:
 *   1. Normalize accelerometer vector
 *   2. Predict gravity direction from current quaternion
 *   3. Cross product error = measured_acc × predicted_gravity
 *   4. PI controller: feed error back into gyro to correct drift
 *   5. Integrate quaternion: q += 0.5 * q⊗ω * dt
 *   6. Normalize quaternion
 *
 * Parameters:
 *   gx/gy/gz — gyroscope [rad/s]
 *   ax/ay/az — accelerometer [m/s²]
 *   dt_acc   — time delta for accel PI correction [s]
 *   dt_gyro  — time delta for quaternion integration [s]
 * ═══════════════════════════════════════════════════════════════════════ */

void MahonyAHRSupdate(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float dt_acc, float dt_gyro)
{
    /* ── 1. Normalize accelerometer (direction only) ── */
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-12f) return;   // free-fall or sensor fault
    float inv = 1.0f / n;
    ax *= inv; ay *= inv; az *= inv;

    /* ── 2. Predict gravity direction in body frame ── */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ── 3. Cross product error: e = a_meas × a_pred ── */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* ── 4. PI controller ── */
    /* Integral term — estimates gyro bias */
    exInt += ex * twoKi * dt_acc;
    eyInt += ey * twoKi * dt_acc;
    ezInt += ez * twoKi * dt_acc;

    /* Anti-windup: clamp integral to ±0.1 rad/s (≈ ±6 dps) */
    float lim = 0.1f;
    if (exInt >  lim) exInt =  lim;
    if (exInt < -lim) exInt = -lim;
    if (eyInt >  lim) eyInt =  lim;
    if (eyInt < -lim) eyInt = -lim;
    if (ezInt >  lim) ezInt =  lim;
    if (ezInt < -lim) ezInt = -lim;

    /* Proportional + integral correction */
    gx += twoKp * ex + exInt;
    gy += twoKp * ey + eyInt;
    gz += twoKp * ez + ezInt;

    /* ── 5. Quaternion integration: q += 0.5 * q⊗ω * dt_gyro ── */
    float qa = q0, qb = q1, qc = q2;
    float half_dt = 0.5f * dt_gyro;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
    q1 += ( qa * gx + qc * gz - q3 * gy) * half_dt;
    q2 += ( qa * gy - qb * gz + q3 * gx) * half_dt;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_dt;

    /* ── 6. Normalize ── */
    normalize();
}

/* ═══════════════════════════════════════════════════════════════════════
 * MahonyGetQuaternion — export current quaternion [w, x, y, z]
 * ═══════════════════════════════════════════════════════════════════════ */

void MahonyGetQuaternion(float *o0, float *o1, float *o2, float *o3)
{
    *o0 = q0;
    *o1 = q1;
    *o2 = q2;
    *o3 = q3;
}

/* ═══════════════════════════════════════════════════════════════════════
 * MahonyGetEuler — quaternion → Euler angles (ZYX intrinsic)
 *
 *   Roll  (φ): rotation around X axis
 *   Pitch (θ): rotation around Y axis
 *   Yaw   (ψ): rotation around Z axis
 *
 * Output unit: degrees
 * ═══════════════════════════════════════════════════════════════════════ */

void MahonyGetEuler(float *roll, float *pitch, float *yaw)
{
    /* Roll */
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2));

    /* Pitch — clamp asin argument to [-1, 1] */
    float arg = 2.0f * (q0 * q2 - q3 * q1);
    if (arg >  1.0f) arg =  1.0f;
    if (arg < -1.0f) arg = -1.0f;
    *pitch = asinf(arg);

    /* Yaw */
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));

    /* Radians → degrees */
    const float r2d = 57.29578f;
    *roll  *= r2d;
    *pitch *= r2d;
    *yaw   *= r2d;
}

/* ═══════════════════════════════════════════════════════════════════════
 * MahonyGetRotationMatrix — quaternion → 3×3 rotation matrix
 *
 *       [ 1-2(y²+z²)   2(xy-wz)     2(xz+wy)  ]
 *   R = [ 2(xy+wz)     1-2(x²+z²)   2(yz-wx)  ]
 *       [ 2(xz-wy)     2(yz+wx)     1-2(x²+y²)]
 *
 * Row-major: R[row][col]
 * ═══════════════════════════════════════════════════════════════════════ */

void MahonyGetRotationMatrix(float R[3][3])
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    R[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    R[0][1] = 2.0f * (q1q2 - q0q3);
    R[0][2] = 2.0f * (q1q3 + q0q2);

    R[1][0] = 2.0f * (q1q2 + q0q3);
    R[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    R[1][2] = 2.0f * (q2q3 - q0q1);

    R[2][0] = 2.0f * (q1q3 - q0q2);
    R[2][1] = 2.0f * (q2q3 + q0q1);
    R[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Application-level IMU API
 * ═══════════════════════════════════════════════════════════════════════ */

/** Last update timestamp [ms] for delta-T calculation */
static float last_update_ms = 0.0f;
static uint8_t imu_initialized = 0;

/**
 * @brief  Initialize IMU hardware + Mahony filter
 */
void IMU_init(void)
{
    if (bsp_Icm42688Init() == 0) {
        MahonyAHRSinit();
        last_update_ms = DWT_GetTimeline_ms();
        imu_initialized = 1;
    }
}

/**
 * @brief  Read sensors + run Mahony update + return Euler angles
 * @param  ypr  [out] ypr[0]=Yaw, ypr[1]=Pitch, ypr[2]=Roll [degrees]
 *
 * @note   Called at ~200Hz from Chassis task
 */
void IMU_getYawPitchRoll(float *ypr)
{
    if (!imu_initialized) {
        ypr[0] = 0.0f; ypr[1] = 0.0f; ypr[2] = 0.0f;
        return;
    }

    icm42688RealData_t acc, gyro;
    bsp_IcmGetRawData(&acc, &gyro);

    /* Store raw data for external consumers */
    motion6[0] = acc.x;
    motion6[1] = acc.y;
    motion6[2] = acc.z;
    motion6[3] = gyro.x;
    motion6[4] = gyro.y;
    motion6[5] = gyro.z;

    /* Calculate dt in seconds */
    float now_ms = DWT_GetTimeline_ms();
    float dt = (now_ms - last_update_ms) / 1000.0f;
    last_update_ms = now_ms;

    /* Clamp dt to prevent jumps after long pauses */
    if (dt > 0.05f) dt = 0.005f;   // cap at 50ms, default to 5ms
    if (dt <= 0.0f) dt = 0.005f;   // minimum 5ms

    /* Run Mahony filter: acc [m/s²], gyro [rad/s], dt [s] */
    MahonyAHRSupdate(gyro.x, gyro.y, gyro.z,
                     acc.x, acc.y, acc.z,
                     dt, dt);

    /* Get Euler angles */
    float roll, pitch, yaw_val;
    MahonyGetEuler(&roll, &pitch, &yaw_val);

    ypr[0] = yaw_val;
    ypr[1] = pitch;
    ypr[2] = roll;

    /* Update north/west direction vectors from rotation matrix */
    float R[3][3];
    MahonyGetRotationMatrix(R);

    north.x = R[0][0];
    north.y = R[1][0];
    north.z = R[2][0];

    west.x = R[0][1];
    west.y = R[1][1];
    west.z = R[2][1];

    /* Shift yaw history */
    yaw[4] = yaw[3];
    yaw[3] = yaw[2];
    yaw[2] = yaw[1];
    yaw[1] = yaw[0];
    yaw[0] = yaw_val;
}

/**
 * @brief  Get raw sensor data from last read
 * @param  zsjganda  [out] [ax, ay, az, gx, gy, gz, reserved]
 */
void IMU_TT_getgyro(float *zsjganda)
{
    zsjganda[0] = motion6[0];
    zsjganda[1] = motion6[1];
    zsjganda[2] = motion6[2];
    zsjganda[3] = motion6[3];
    zsjganda[4] = motion6[4];
    zsjganda[5] = motion6[5];
    zsjganda[6] = motion6[6];
}

/**
 * @brief  Stub — kept for API compatibility
 */
void MPU6050_InitAng_Offset(void)
{
    /* No-op: gyro bias is handled by the ICM42688 driver's
       icm42688_correct_gyro_bias() and the Mahony filter's
       integral term (exInt/eyInt/ezInt). */
}
