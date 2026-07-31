#include "Gimbal.h"

#include "Chassis.h"
#include "K230.h"
#include "Robot_Cmd.h"
#include "ZDT_Emm.h"
#include "dwt.h"
#include "fifo.h"
#include <math.h>
#include <string.h>

/* ============ ballcontrol 已注释保留，恢复时取消 #if 0 即可 ============ */
#if 0

#define BALL_CONTROL_PERIOD_S       0.005f
#define BALL_COMMAND_PERIOD_S       0.020f
#define BALL_CAMERA_TIMEOUT_S       0.20f
#define BALL_FILTER_ALPHA           0.45f
#define BALL_FILTER_BETA            0.08f
#define BALL_POSITION_KP_DPS_PER_M  80.0f
#define BALL_VELOCITY_KD_DPS_PER_MPS 24.0f
#define GRAVITY_MPS2                9.80665f
#define RAD_TO_DEG                  57.2957795f

static BallControlTelemetry_t ball_telemetry;
static uint32_t last_camera_frame;
static float last_camera_time_s;
static float last_control_time_s;
static float last_command_time_s;
static float task_start_time_s;
static int32_t last_command_pulses;

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void Gimbal_ZDT_UART_Send(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i) {
        while (DL_UART_isBusy(ELRS_INST)) {}
        DL_UART_Main_transmitData(ELRS_INST, data[i]);
    }
}

/* 只在相机产生新帧时更新 alpha-beta 状态，避免重复使用同一帧制造虚假速度。 */
static void BallControl_UpdateCamera(float now_s)
{
    K230_Data_t camera_data;
    uint32_t frame_id;
    if (!K230_GetSnapshot(&camera_data, &frame_id) || frame_id == last_camera_frame) return;

    const float dt = ClampFloat(now_s - last_camera_time_s, 0.005f, 0.10f);
    const float measured_m = (float)camera_data.x * BALL_CAMERA_UNIT_TO_M;
    const float predicted_m = ball_telemetry.position_m + ball_telemetry.velocity_mps * dt;
    const float residual_m = measured_m - predicted_m;

    if (last_camera_frame == 0U) {
        ball_telemetry.position_m = measured_m;
        ball_telemetry.velocity_mps = 0.0f;
    } else {
        ball_telemetry.position_m = predicted_m + BALL_FILTER_ALPHA * residual_m;
        ball_telemetry.velocity_mps += (BALL_FILTER_BETA / dt) * residual_m;
    }
    last_camera_frame = frame_id;
    last_camera_time_s = now_s;
}

/* 根据竞赛任务生成滚球目标。任务3按 O -> +5 cm -> -5 cm 执行。 */
static float BallControl_GetTaskTarget(uint8_t task, float elapsed_s)
{
    if (task == H_TASK_3_STATIC_BALL) {
        ball_telemetry.state = BALL_CONTROL_STATIC_SEQUENCE;
        if (elapsed_s < 0.40f) return 0.0f;
        if (elapsed_s < 2.40f) return 0.05f;
        return -0.05f;
    }
    if (task == H_TASK_4_AB_BALL || task == H_TASK_5_CENTER_BALL_LAP) {
        ball_telemetry.state = BALL_CONTROL_CENTER;
        return 0.0f;
    }
    if (task == H_TASK_6_TARGET_BALL_LAP) {
        ball_telemetry.state = BALL_CONTROL_TARGET;
        return gimbal_cmd_receive.aim_x;
    }
    ball_telemetry.state = BALL_CONTROL_IDLE;
    return 0.0f;
}

/* 将横梁角度转换为 ZDT 快速绝对位置命令，并限制命令发送频率。 */
static void BallControl_SendBeamAngle(float beam_angle_deg, float now_s)
{
    const float motor_angle_deg = beam_angle_deg * BALL_MOTOR_TO_BEAM_RATIO * BALL_MOTOR_DIRECTION;
    const int32_t pulses = (int32_t)lroundf(motor_angle_deg * BALL_MOTOR_PULSES_PER_REV / 360.0f);
    if ((now_s - last_command_time_s) < BALL_COMMAND_PERIOD_S || pulses == last_command_pulses) return;

    ZDT_Emm_QPos_Control(1U, pulses);
    last_command_pulses = pulses;
    last_command_time_s = now_s;
}

void Gimbal_Init(void)
{
    memset(&gimbal_cmd_receive, 0, sizeof(gimbal_cmd_receive));
    memset(&ball_telemetry, 0, sizeof(ball_telemetry));
    fifo_initQueue(&zdt_emm_rx_fifo);
    ZDT_Emm_RegisterSendCallback(Gimbal_ZDT_UART_Send);
    NVIC_ClearPendingIRQ(ELRS_INST_INT_IRQN);
    NVIC_EnableIRQ(ELRS_INST_INT_IRQN);

    ZDT_Emm_En_Control(1U, true, false);
    ZDT_Emm_Set_QPos_Params(1U, BALL_MOTOR_MAX_RPM, BALL_MOTOR_ACCEL, 1U, false);
    last_control_time_s = DWT_GetTimeline_s();
    last_camera_time_s = last_control_time_s;
    last_command_time_s = last_control_time_s - BALL_COMMAND_PERIOD_S;
    last_command_pulses = INT32_MIN;
}

void Gimbal(void)
{
    const float now_s = DWT_GetTimeline_s();
    float dt = now_s - last_control_time_s;
    last_control_time_s = now_s;
    dt = ClampFloat(dt, 0.001f, 0.02f);

    (void)xQueueReceive(gimbal_cmd_queue, &gimbal_cmd_receive, 0U);
    if ((gimbal_cmd_receive.task_flag != last_task) ||
        (gimbal_cmd_receive.task_start_seq != last_task_start_seq)) {
        last_task = gimbal_cmd_receive.task_flag;
        last_task_start_seq = gimbal_cmd_receive.task_start_seq;
        task_start_time_s = now_s;
        ball_telemetry.max_abs_error_m = 0.0f;
    }

    BallControl_UpdateCamera(now_s);
    ball_telemetry.camera_online = (uint8_t)(K230_IsOnline() &&
        ((now_s - last_camera_time_s) <= BALL_CAMERA_TIMEOUT_S));
    ball_telemetry.target_m = BallControl_GetTaskTarget(last_task, now_s - task_start_time_s);

    float requested_angle_deg = 0.0f;
    if (ball_telemetry.state != BALL_CONTROL_IDLE && ball_telemetry.camera_online) {
        const float error_m = ball_telemetry.target_m - ball_telemetry.position_m;
        const float abs_error_m = fabsf(error_m);
        if (abs_error_m > ball_telemetry.max_abs_error_m) ball_telemetry.max_abs_error_m = abs_error_m;

        /* 底盘纵向加速度前馈抵消惯性，PD 项负责小球位置和速度。 */
        const float chassis_ff_deg = atanf(Chassis_GetCommandedAcceleration() / GRAVITY_MPS2) *
                                     RAD_TO_DEG;
        requested_angle_deg = BALL_POSITION_KP_DPS_PER_M * error_m -
                              BALL_VELOCITY_KD_DPS_PER_MPS * ball_telemetry.velocity_mps +
                              chassis_ff_deg;
    } else if (ball_telemetry.state != BALL_CONTROL_IDLE) {
        ball_telemetry.state = BALL_CONTROL_CAMERA_LOST;
    }

    requested_angle_deg = ClampFloat(requested_angle_deg,
                                     -BALL_BEAM_MAX_ANGLE_DEG,
                                     BALL_BEAM_MAX_ANGLE_DEG);
    const float max_step_deg = BALL_BEAM_MAX_SLEW_DPS * dt;
    const float angle_delta_deg = ClampFloat(requested_angle_deg - ball_telemetry.beam_angle_deg,
                                             -max_step_deg, max_step_deg);
    ball_telemetry.beam_angle_deg += angle_delta_deg;
    BallControl_SendBeamAngle(ball_telemetry.beam_angle_deg, now_s);
}

BallControlTelemetry_t BallControl_GetTelemetry(void)
{
    return ball_telemetry;
}

#endif
/* ============ ballcontrol 注释保留结束 ============ */

/* ===== RTOS 消息队列内容 ===== */
static gimbal_cmd_q gimbal_cmd_receive;

void Gimbal_Init(void)
{
    memset(&gimbal_cmd_receive, 0, sizeof(gimbal_cmd_receive));
}

void Gimbal(void)
{
    (void)xQueueReceive(gimbal_cmd_queue, &gimbal_cmd_receive, 0U);
}
