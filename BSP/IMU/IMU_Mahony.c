#include "IMU_Mahony.h"
#include "icm42688_improved.h"
#include "dwt.h"
#include <math.h>
#include <stdint.h>

imu_mahony_xyz_t imu_mahony_north = {0.0f, 0.0f, 0.0f};
imu_mahony_xyz_t imu_mahony_west = {0.0f, 0.0f, 0.0f};
volatile float imu_mahony_yaw[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float imu_mahony_motion6[7] = {0.0f};

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float ex_int = 0.0f;
static float ey_int = 0.0f;
static float ez_int = 0.0f;
static float two_kp = 3.0f;
static float two_ki = 0.1f;
static float last_update_ms = 0.0f;
static uint8_t imu_initialized = 0U;

static void mahony_init(void)
{
    two_kp = 2.0f * 1.5f;
    two_ki = 2.0f * 0.05f;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    ex_int = 0.0f;
    ey_int = 0.0f;
    ez_int = 0.0f;
}

static void mahony_normalize_quaternion(void)
{
    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (n < 1e-12f) {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        return;
    }

    n = 1.0f / n;
    q0 *= n;
    q1 *= n;
    q2 *= n;
    q3 *= n;
}

static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt_acc, float dt_gyro)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-12f) {
        return;
    }

    n = 1.0f / n;
    ax *= n;
    ay *= n;
    az *= n;

    const float vx = 2.0f * (q1 * q3 - q0 * q2);
    const float vy = 2.0f * (q0 * q1 + q2 * q3);
    const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;

    ex_int += ex * two_ki * dt_acc;
    ey_int += ey * two_ki * dt_acc;
    ez_int += ez * two_ki * dt_acc;

    const float int_limit = 0.1f;
    if (ex_int > int_limit) {
        ex_int = int_limit;
    } else if (ex_int < -int_limit) {
        ex_int = -int_limit;
    }
    if (ey_int > int_limit) {
        ey_int = int_limit;
    } else if (ey_int < -int_limit) {
        ey_int = -int_limit;
    }
    if (ez_int > int_limit) {
        ez_int = int_limit;
    } else if (ez_int < -int_limit) {
        ez_int = -int_limit;
    }

    gx += two_kp * ex + ex_int;
    gy += two_kp * ey + ey_int;
    gz += two_kp * ez + ez_int;

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    const float half_dt = 0.5f * dt_gyro;

    q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
    q1 += (qa * gx + qc * gz - q3 * gy) * half_dt;
    q2 += (qa * gy - qb * gz + q3 * gx) * half_dt;
    q3 += (qa * gz + qb * gy - qc * gx) * half_dt;

    mahony_normalize_quaternion();
}

static void mahony_get_euler(float *roll, float *pitch, float *yaw)
{
    float asin_arg;
    const float rad_to_deg = 57.29578f;

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2));

    asin_arg = 2.0f * (q0 * q2 - q3 * q1);
    if (asin_arg > 1.0f) {
        asin_arg = 1.0f;
    } else if (asin_arg < -1.0f) {
        asin_arg = -1.0f;
    }
    *pitch = asinf(asin_arg);

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));

    *roll *= rad_to_deg;
    *pitch *= rad_to_deg;
    *yaw *= rad_to_deg;
}

static void mahony_get_rotation_matrix(float r[3][3])
{
    const float q1q1 = q1 * q1;
    const float q2q2 = q2 * q2;
    const float q3q3 = q3 * q3;
    const float q0q1 = q0 * q1;
    const float q0q2 = q0 * q2;
    const float q0q3 = q0 * q3;
    const float q1q2 = q1 * q2;
    const float q1q3 = q1 * q3;
    const float q2q3 = q2 * q3;

    r[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    r[0][1] = 2.0f * (q1q2 - q0q3);
    r[0][2] = 2.0f * (q1q3 + q0q2);
    r[1][0] = 2.0f * (q1q2 + q0q3);
    r[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    r[1][2] = 2.0f * (q2q3 - q0q1);
    r[2][0] = 2.0f * (q1q3 - q0q2);
    r[2][1] = 2.0f * (q2q3 + q0q1);
    r[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

void IMU_Mahony_Init(void)
{
    /*
     * ICM 初始化内部包含约 1.7 s 的静止陀螺校准。
     * 传感器通信成功是允许底盘运行的必要条件；校准失败只表示偏置
     * 未能可靠估计，不能把底盘永久锁死，否则会出现上电偶发不动。
     * 校准失败时偏置保持为 0，系统仍可运行，但正式测试必须保证上电
     * 期间车体静止，以尽量获得有效偏置。
     */
    imu_initialized = 0U;

    /*
     * 上电瞬间电源或SPI时序尚未稳定时，第一次WHO_AM_I读取可能失败。
     * 这里最多尝试3次；使用有限重试而不是死循环，避免传感器断线时
     * 整个系统永远卡在初始化函数中。
     */
    for (uint8_t retry = 0U; retry < 3U; ++retry) {
        if (ICM42688_Improved_Init() == 0) {
            mahony_init();
            last_update_ms = DWT_GetTimeline_ms();
            imu_initialized = 1U;
            break;
        }
        DWT_Delay(0.02f);
    }
}

uint8_t IMU_Mahony_IsReady(void)
{
    return imu_initialized;
}

void IMU_Mahony_GetYawPitchRoll(float *ypr)
{
    icm42688_imp_data_t acc;
    icm42688_imp_data_t gyro;
    float now_ms;
    float dt;
    float roll;
    float pitch;
    float yaw_value;
    float r[3][3];

    if (imu_initialized == 0U) {
        ypr[0] = 0.0f;
        ypr[1] = 0.0f;
        ypr[2] = 0.0f;
        return;
    }

    (void)ICM42688_Improved_GetRawData(&acc, &gyro);

    imu_mahony_motion6[0] = acc.x;
    imu_mahony_motion6[1] = acc.y;
    imu_mahony_motion6[2] = acc.z;
    imu_mahony_motion6[3] = gyro.x;
    imu_mahony_motion6[4] = gyro.y;
    imu_mahony_motion6[5] = gyro.z;

    now_ms = DWT_GetTimeline_ms();
    dt = (now_ms - last_update_ms) / 1000.0f;
    last_update_ms = now_ms;

    if ((dt <= 0.0f) || (dt > 0.05f)) {
        dt = 0.005f;
    }

    mahony_update(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, dt, dt);
    mahony_get_euler(&roll, &pitch, &yaw_value);

    ypr[0] = yaw_value;
    ypr[1] = pitch;
    ypr[2] = roll;

    mahony_get_rotation_matrix(r);
    imu_mahony_north.x = r[0][0];
    imu_mahony_north.y = r[1][0];
    imu_mahony_north.z = r[2][0];
    imu_mahony_west.x = r[0][1];
    imu_mahony_west.y = r[1][1];
    imu_mahony_west.z = r[2][1];

    imu_mahony_yaw[4] = imu_mahony_yaw[3];
    imu_mahony_yaw[3] = imu_mahony_yaw[2];
    imu_mahony_yaw[2] = imu_mahony_yaw[1];
    imu_mahony_yaw[1] = imu_mahony_yaw[0];
    imu_mahony_yaw[0] = yaw_value;
}

void IMU_Mahony_GetRawMotion(float *motion)
{
    for (uint8_t i = 0U; i < 7U; i++) {
        motion[i] = imu_mahony_motion6[i];
    }
}

void IMU_Mahony_ResetOffset(void)
{
    ex_int = 0.0f;
    ey_int = 0.0f;
    ez_int = 0.0f;
}
