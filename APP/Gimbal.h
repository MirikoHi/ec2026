#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ============ ballcontrol 已注释保留，恢复时取消 #if 0 即可（需同时恢复 Gimbal.c） ============ */
#if 0

/* ---------------- 必须根据实物标定的参数 ---------------- */
/* K230 的 x=0 表示横梁中心，当前约定 1 个数据单位=1 mm。若发送像素坐标，必须修改该值。 */
#define BALL_CAMERA_UNIT_TO_M       0.001f
/* 电机转角 / 横梁转角。直连为 1；减速 3:1（电机转 3°、横梁转 1°）则填 3。 */
#define BALL_MOTOR_TO_BEAM_RATIO    1.0f
/* 电机正转是否使小球坐标 x 增大。方向相反时改成 -1。 */
#define BALL_MOTOR_DIRECTION        1.0f

/* ---------------- 安全限制和初始控制参数 ---------------- */
#define BALL_BEAM_MAX_ANGLE_DEG     8.0f
#define BALL_BEAM_MAX_SLEW_DPS      35.0f
#define BALL_MOTOR_PULSES_PER_REV   3200.0f
#define BALL_MOTOR_MAX_RPM          120U
#define BALL_MOTOR_ACCEL            20U

typedef enum {
    BALL_CONTROL_IDLE = 0,
    BALL_CONTROL_CENTER,
    BALL_CONTROL_STATIC_SEQUENCE,
    BALL_CONTROL_TARGET,
    BALL_CONTROL_CAMERA_LOST,
} BallControlState_e;

typedef struct {
    float position_m;          /* 滤波后的小球位置，中心为 0 */
    float velocity_mps;        /* 估算的小球速度 */
    float target_m;            /* 当前目标位置 */
    float beam_angle_deg;      /* 当前下发的横梁目标角 */
    float max_abs_error_m;     /* 本次任务出现过的最大绝对误差 */
    uint8_t camera_online;
    BallControlState_e state;
} BallControlTelemetry_t;

/** 获取只读遥测快照，供 OLED 和调试器显示。 */
BallControlTelemetry_t BallControl_GetTelemetry(void);

#endif
/* ============ ballcontrol 注释保留结束 ============ */

/** 初始化 Gimbal 任务状态。 */
void Gimbal_Init(void);

/** 200 Hz 周期函数：接收任务命令，任务三时向 K230 发送 task_flag 并计时。 */
void Gimbal(void);

/** 返回任务三运行时长（秒）：计时中返回实时值，收到 K230 结束字节 0xAE 后返回冻结值。 */
float Gimbal_GetRunTimeSeconds(void);

/** 任务三计时是否进行中。 */
uint8_t Gimbal_IsRunTimerActive(void);

/** KEY4 等异步来源调用，请求立即停止任务三计时。 */
void Gimbal_RequestEmergencyStop(void);

#endif /* _GIMBAL_H_ */
