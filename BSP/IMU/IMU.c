/**
 ******************************************************************************
 * @file    IMU.c
 * @brief   IMU 姿态解算 — 融合 BMI088/ICM42688 驱动 + Mahony 互补滤波
 ******************************************************************************
 */

#include "IMU.h"
#include "dwt.h"
#include "bsp_log.h"

#if USE_BMI088
    #include "BMI088.h"
#else
    #include "icm42688.h"
#endif

/* ═══════════════════════════════════════════════════════════════════════
   全局变量定义
   ═══════════════════════════════════════════════════════════════════════ */
xyz_f_t north = {0.0f, 0.0f, 0.0f};
xyz_f_t west  = {0.0f, 0.0f, 0.0f};

volatile float yaw[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float motion6[7] = {0.0f};

/* ═══════════════════════════════════════════════════════════════════════
   Mahony 算法内部状态量
   ═══════════════════════════════════════════════════════════════════════ */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 姿态四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 陀螺仪零偏积分项
static float twoKp = 3.0f;                               // 2 * Kp (Kp = 1.5)
static float twoKi = 0.1f;                               // 2 * Ki (Ki = 0.05)

static float last_update_ms = 0.0f;
static uint8_t imu_initialized = 0;

/* ═══════════════════════════════════════════════════════════════════════
   Mahony 算法核心函数
   ═══════════════════════════════════════════════════════════════════════ */

static void MahonyAHRSinit(void)
{
    twoKp = 2.0f * 1.5f;       // 比例增益 (控制加速度计修正权重)
    twoKi = 2.0f * 0.05f;      // 积分增益 (控制零偏估计收敛速度)
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
}

static void normalize_quaternion(void)
{
    float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (n < 1e-12f) {
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv;
}

/**
 * @brief Mahony 姿态更新步
 * @param gx, gy, gz 陀螺仪角速度 [rad/s]
 * @param ax, ay, az 加速度计数据 [g 或 m/s²]
 * @param dt 时间间隔 [s]
 */
static void MahonyAHRSupdate(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt)
{
    /* 1. 归一化加速度计数据 */
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-12f) return; // 自由落体或传感器故障时跳过
    float inv = 1.0f / n;
    ax *= inv; ay *= inv; az *= inv;

    /* 2. 依据当前四元数推算重力方向向量 (机体坐标系) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 3. 叉乘计算测量重力与预测重力的误差 e = a_meas × a_pred */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* 4. PI 控制器纠正陀螺仪零偏 */
    exInt += ex * twoKi * dt;
    eyInt += ey * twoKi * dt;
    ezInt += ez * twoKi * dt;

    /* 积分抗饱和限幅 (±0.1 rad/s ≈ ±5.7 deg/s) */
    float lim = 0.1f;
    if (exInt >  lim) exInt =  lim;
    if (exInt < -lim) exInt = -lim;
    if (eyInt >  lim) eyInt =  lim;
    if (eyInt < -lim) eyInt = -lim;
    if (ezInt >  lim) ezInt =  lim;
    if (ezInt < -lim) ezInt = -lim;

    /* 加上 PI 补偿 */
    gx += twoKp * ex + exInt;
    gy += twoKp * ey + eyInt;
    gz += twoKp * ez + ezInt;

    /* 5. 四元数微分方程积分: q += 0.5 * q ⊗ ω * dt */
    float qa = q0, qb = q1, qc = q2;
    float half_dt = 0.5f * dt;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
    q1 += ( qa * gx + qc * gz - q3 * gy) * half_dt;
    q2 += ( qa * gy - qb * gz + q3 * gx) * half_dt;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_dt;

    /* 6. 四元数单位化 */
    normalize_quaternion();
}

/**
 * @brief 四元数转欧拉角 (ZYX 顺序)
 */
static void MahonyGetEuler(float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    float arg = 2.0f * (q0 * q2 - q3 * q1);
    if (arg >  1.0f) arg =  1.0f;
    if (arg < -1.0f) arg = -1.0f;
    *pitch = asinf(arg) * RAD_TO_DEG;

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

/**
 * @brief 获取 3x3 旋转矩阵
 */
static void MahonyGetRotationMatrix(float R[3][3])
{
    float q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;

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
   应用层 API 接口
   ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化 IMU 硬件与 Mahony 算法
 */
void IMU_init(void)
{
    int8_t init_status = -1;

#if USE_BMI088
    init_status = bsp_Bmi088Init();
#else
    init_status = bsp_Icm42688Init();
#endif

    if (init_status == 0) {
        MahonyAHRSinit();
        last_update_ms = DWT_GetTimeline_ms();
        imu_initialized = 1;
        LOGINFO("IMU Init Success!");
    } else {
        LOGERROR("IMU Init Failed, status: %d", init_status);
    }
}

/**
 * @brief  读取传感器数据并执行 Mahony 解算
 * @param  ypr [out] ypr[0]=Yaw, ypr[1]=Pitch, ypr[2]=Roll (单位：度)
 * @note   底盘任务中以 ~200Hz 周期调用
 */
void IMU_getYawPitchRoll(float *ypr)
{
    if (!imu_initialized) {
        ypr[0] = 0.0f; ypr[1] = 0.0f; ypr[2] = 0.0f;
        return;
    }

    float ax = 0, ay = 0, az = 0;
    float gx_dps = 0, gy_dps = 0, gz_dps = 0;

/* 1. 读取底层硬件原始物理量数据 */
#if USE_BMI088
    bmi088RealData_t accval, gyroval;
    bsp_Bmi088GetRawData(&accval, &gyroval);
    ax = accval.x; ay = accval.y; az = accval.z;
    gx_dps = gyroval.x; gy_dps = gyroval.y; gz_dps = gyroval.z;
#else
    icm42688RealData_t accval, gyroval;
    bsp_IcmGetRawData(&accval, &gyroval);
    ax = accval.x; ay = accval.y; az = accval.z;
    gx_dps = gyroval.x; gy_dps = gyroval.y; gz_dps = gyroval.z;
#endif

    /* 备份原始数据 */
    motion6[0] = ax; motion6[1] = ay; motion6[2] = az;
    motion6[3] = gx_dps; motion6[4] = gy_dps; motion6[5] = gz_dps;
    motion6[6] = 0.0f;

    /* 2. 计算动态时间间隔 dt (秒) */
    float now_ms = DWT_GetTimeline_ms();
    float dt = (now_ms - last_update_ms) / 1000.0f;
    last_update_ms = now_ms;

    /* dt 异常限幅处理 (卡顿保护) */
    if (dt > 0.05f || dt <= 0.0f) {
        dt = 0.005f; // 默认 200Hz (5ms)
    }

    /* 3. 关键一步：将陀螺仪数据从 deg/s 转换为 rad/s 传入 Mahony 解算 */
    float gx_rad = gx_dps * DEG_TO_RAD;
    float gy_rad = gy_dps * DEG_TO_RAD;
    float gz_rad = gz_dps * DEG_TO_RAD;

    MahonyAHRSupdate(gx_rad, gy_rad, gz_rad,
                     ax, ay, az,
                     dt);

    /* 4. 提取姿态角欧拉角 */
    float roll, pitch, yaw_val;
    MahonyGetEuler(&roll, &pitch, &yaw_val);

    ypr[0] = yaw_val; // Yaw
    ypr[1] = pitch;   // Pitch
    ypr[2] = roll;    // Roll

    /* 5. 更新正北/正西方向向量 (供底盘全向导航参考) */
    float R[3][3];
    MahonyGetRotationMatrix(R);
    north.x = R[0][0]; north.y = R[1][0]; north.z = R[2][0];
    west.x  = R[0][1]; west.y  = R[1][1]; west.z  = R[2][1];

    /* 6. 更新 Yaw 历史队列 */
    yaw[4] = yaw[3];
    yaw[3] = yaw[2];
    yaw[2] = yaw[1];
    yaw[1] = yaw[0];
    yaw[0] = yaw_val;
}

/**
 * @brief 获取传感器最后一次读取的原始数据
 */
void IMU_TT_getgyro(float *zsjganda)
{
    for (int i = 0; i < 7; i++) {
        zsjganda[i] = motion6[i];
    }
}

void MPU6050_InitAng_Offset(void) {}