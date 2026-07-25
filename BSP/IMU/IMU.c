#include "IMU.h"
#include <stdio.h>
#include "dwt.h"
#include "bsp_log.h"

// 根据宏选择引入不同的硬件驱动
#if USE_BMI088
    #include "BMI088.h"  // ✅ 已修改为引入 BMI088.h
#else
    #include "icm42688.h"
#endif

xyz_f_t north, west;
volatile float exInt, eyInt, ezInt;
volatile float q0, q1, q2, q3;
volatile float integralFBhand, handdiff;
volatile uint32_t lastUpdate, now;
volatile float yaw[5] = {0};
int16_t Ax_offset = 0, Ay_offset = 0;
float TTangles_gyro[7];
float Angle_Final[3];

uint32_t nowtime = 0;

float invSqrt1(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 初始化 IMU 硬件与姿态算法
 */
static volatile int test_init_status;
void IMU_init(void)
{
    int8_t init_status = -1;

#if USE_BMI088
    init_status = bsp_Bmi088Init();
    test_init_status = init_status;
#else
    init_status = bsp_Icm42688Init();
#endif

    if (init_status == 0)
    {
        LOGINFO("BMI088 Init Success!");
    }
    else
    {
        LOGERROR("BMI088 Init Failed, status code: %d", init_status);
    }

    if (init_status == 0)
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        exInt = 0.0f;
        eyInt = 0.0f;
        ezInt = 0.0f;
        lastUpdate = nowtime;
        now = nowtime;
    }
}

static float Gyro_fill[3][300];
static double Gyro_total[3];
static double sqrGyro_total[3];
static int GyroinitFlag = 0;
static int GyroCount = 0;

void calGyroVariance(float data[], int length, float sqrResult[], float avgResult[])
{
    int i;
    double tmplen;
    if (GyroinitFlag == 0)
    {
        for (i = 0; i < 3; i++)
        {
            Gyro_fill[i][GyroCount] = data[i];
            Gyro_total[i] += data[i];
            sqrGyro_total[i] += data[i] * data[i];
            sqrResult[i] = 100;
            avgResult[i] = 0;
        }
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            Gyro_total[i] -= Gyro_fill[i][GyroCount];
            sqrGyro_total[i] -= Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
            Gyro_fill[i][GyroCount] = data[i];
            Gyro_total[i] += Gyro_fill[i][GyroCount];
            sqrGyro_total[i] += Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
        }
    }
    GyroCount++;
    if (GyroCount >= length)
    {
        GyroCount = 0;
        GyroinitFlag = 1;
    }
    if (GyroinitFlag == 0) return;

    tmplen = length;
    for (i = 0; i < 3; i++)
    {
        avgResult[i] = (float)(Gyro_total[i] / tmplen);
        sqrResult[i] = (float)((sqrGyro_total[i] - Gyro_total[i] * Gyro_total[i] / tmplen) / tmplen);
    }
}

float gyro_offset[3] = {0};
int CalCount = 0;

/**
 * @brief 读取传感器底层数据并自动去零偏
 */
static bmi088RealData_t test_accval, test_gyroval;
void IMU_getValues(float * values) {
    float sqrResult_gyro[3];
    float avgResult_gyro[3];

#if USE_BMI088
    bmi088RealData_t accval, gyroval;
    bsp_Bmi088GetRawData(&accval, &gyroval);
    /* 同步更新调试用全局变量，避免重复 SPI 读取 */
    test_accval = accval;
    test_gyroval = gyroval;
#else
    icm42688RealData_t accval, gyroval;
    bsp_IcmGetRawData(&accval, &gyroval);
#endif

    TTangles_gyro[0] = accval.x;
    TTangles_gyro[1] = accval.y;
    TTangles_gyro[2] = accval.z;
    TTangles_gyro[3] = gyroval.x;
    TTangles_gyro[4] = gyroval.y;
    TTangles_gyro[5] = gyroval.z;
    TTangles_gyro[6] = 0;

    calGyroVariance(&TTangles_gyro[3], 100, sqrResult_gyro, avgResult_gyro);
    if (sqrResult_gyro[0] < 0.02f && sqrResult_gyro[1] < 0.02f && sqrResult_gyro[2] < 0.02f && CalCount >= 99)
    {
        gyro_offset[0] = avgResult_gyro[0];
        gyro_offset[1] = avgResult_gyro[1];
        gyro_offset[2] = avgResult_gyro[2];
        exInt = 0; eyInt = 0; ezInt = 0;
        CalCount = 0;
    }
    else if (CalCount < 100)
    {
        CalCount++;
    }

    values[0] = accval.x;
    values[1] = accval.y;
    values[2] = accval.z;
    values[3] = gyroval.x - gyro_offset[0];
    values[4] = gyroval.y - gyro_offset[1];
    values[5] = gyroval.z - gyro_offset[2];
}

#define Kp 0.5f
#define Ki 0.001f

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez, halfT;
    float tempq0, tempq1, tempq2, tempq3;

    float q0q0 = q0*q0; float q0q1 = q0*q1; float q0q2 = q0*q2; float q0q3 = q0*q3;
    float q1q1 = q1*q1; float q1q2 = q1*q2; float q1q3 = q1*q3;
    float q2q2 = q2*q2; float q2q3 = q2*q3; float q3q3 = q3*q3;

    now = nowtime;
    if(now < lastUpdate) {
        halfT = ((float)(now + (0xFFFFFFFF - lastUpdate)) / 2000000.0f);
    } else {
        halfT = ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;

    norm = invSqrt1(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f) {
        exInt += ex * Ki * halfT;
        eyInt += ey * Ki * halfT;
        ezInt += ez * Ki * halfT;

        gx += Kp*ex + exInt;
        gy += Kp*ey + eyInt;
        gz += Kp*ez + ezInt;
    }

    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    norm = invSqrt1(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

static float mygetqval[9];
void IMU_getQ(float * q) {
    IMU_getValues(mygetqval);
    nowtime = (uint32_t)DWT_GetTimeline_us();

    IMU_AHRSupdate(mygetqval[3] * M_PI/180.0f, mygetqval[4] * M_PI/180.0f, mygetqval[5] * M_PI/180.0f,
                   mygetqval[0], mygetqval[1], mygetqval[2], 0, 0, 0);

    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
}

void IMU_getYawPitchRoll(float * angles) {
    float q[4];
    IMU_getQ(q);

    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1) * 180.0f / M_PI; // yaw
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180.0f / M_PI; // pitch
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180.0f / M_PI; // roll
}

void IMU_TT_getgyro(float * zsjganda)
{
    zsjganda[0] = TTangles_gyro[0];
    zsjganda[1] = TTangles_gyro[1];
    zsjganda[2] = TTangles_gyro[2];
    zsjganda[3] = TTangles_gyro[3];
    zsjganda[4] = TTangles_gyro[4];
    zsjganda[5] = TTangles_gyro[5];
    zsjganda[6] = TTangles_gyro[6];
}

void MPU6050_InitAng_Offset(void) {}