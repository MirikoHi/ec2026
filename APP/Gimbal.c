#include "Gimbal.h"

#include "ZDT_Emm.h"
#include "Servo.h"
#include "Robot_cmd.h"
static ZDT_Emm_Motor_t *zdt_motor;
gimbal_cmd_q gimbal_cmd_receive = {0};
ServoInstance *servo_yaw;

/* ---- 调试变量：在调试器中修改 target_angle_deg，电机自动转到对应角度 ---- */
static  float target_angle_deg = 0.0f;
static float last_target_deg = -1.0f;  /* -1 确保首次匹配时触发 */

static void Gimbal_ZDT_UART_Send(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        while (DL_UART_isBusy(ELRS_INST)) {}
        DL_UART_Main_transmitData(ELRS_INST, data[i]);
    }
}

void Gimbal_Init(void)
{
    /* 舵机 */
    Servo_Init_Config_s servo_yaw_config = {
        .Servo_type = Servo180,
        .inst = Servo_INST,
        .idx = 0,
    };
    servo_yaw = ServoInit(&servo_yaw_config);
    Servo_Motor_Type_Select(servo_yaw, Free_Angle_mode);

    /* RX FIFO */
    fifo_initQueue(&zdt_emm_rx_fifo);

    /* UART 发送回调 */
    ZDT_Emm_RegisterSendCallback(Gimbal_ZDT_UART_Send);

    /* RX 中断 */
    NVIC_ClearPendingIRQ(ELRS_INST_INT_IRQN);
    NVIC_EnableIRQ(ELRS_INST_INT_IRQN);
    zdt_motor = ZDT_Emm_Motor_Create(1);

    /* 使能电机 (rotate, addr=1) */
    ZDT_Emm_En_Control(1, true, false);
}

void Gimbal(void)
{
    if(zdt_motor == NULL)
        return;
    xQueueReceive(gimbal_cmd_queue, &gimbal_cmd_receive, 1);


    /* ===== 更新电机实时反馈 ===== */

        ZDT_Emm_Motor_UpdateAll(zdt_motor);


    //接收上位机发送的yaw角度
    target_angle_deg = gimbal_cmd_receive.yaw;
    if (target_angle_deg == last_target_deg)
        return;
    last_target_deg = target_angle_deg;
    float     gimbal_real_angle = zdt_motor->position_deg;
    float         gimbal_real_speed = zdt_motor->speed_rpm;

    /* 角度 → 脉冲 (3200 脉冲/圈 = 360°, 16细分) */
    int32_t pulses = (int32_t)(target_angle_deg * 3200.0f / 360.0f);
    uint8_t dir = 0;


    if(pulses < 0)
    {
        dir = 1;
        pulses = -pulses;
    }
    /* 绝对位置模式 (raF=1) */
    ZDT_Emm_Pos_Control(1, dir, MOTOR_DEFAULT_VEL, MOTOR_DEFAULT_ACC,
                        (uint32_t)(pulses >= 0 ? pulses : -pulses),
                        1, false);
}

