/**
 * @file dmmotor.c
 * @brief DM 达妙电机驱动实现（MIT协议）
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        主要适配:
 *         - 单MCAN0，每个电机独立CANInstance
 *         - 当前项目PID API: PID_calc(ref, measure)，注意参数顺序
 *         - 默认使用力矩控制模式（USE_MIT_MODE 可选 MIT 位置/速度模式）
 *         - 去掉 cmsis_os.h 依赖（任务调度统一在 motor_task）
 */

#include "dmmotor.h"
#include "string.h"
#include "stdlib.h"
#include "daemon.h"
#include "dwt.h"
#include "bsp_log.h"
#include "misc.h"          /* PI */

//#define USE_MIT_MODE      /* 取消注释以启用MIT位置/速度模式 */

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];


void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xFF, 7);
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instance, 1);
}


static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp;
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);

    /* 状态: 0=失能, 1=使能, 8=超压, 9=欠压, A=过流, B=MOS过压, C=过温, D=通信丢失, E=过载 */
    tmp = (uint8_t)(rxbuff[0] >> 4);
    measure->state = tmp;
    if (measure->state)
    {
        DaemonReload(motor->motor_daemon);
    }

    /* 位置 */
    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
    measure->angle_single_round = measure->position;

    /* 速度 */
    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    /* 力矩 */
    tmp = (uint16_t)(((rxbuff[4] & 0x0F) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    /* 温度 */
    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    /* 多圈角度计算 */
    if (measure->position - measure->last_position > PI)
        measure->total_round--;
    else if (measure->position - measure->last_position < -PI)
        measure->total_round++;

    measure->total_angle = measure->total_round * (DM_P_MAX * 2.0f) + measure->angle_single_round;
}


static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    LOGWARNING("[dm_motor] Motor lost! tx_id=0x%03X", motor->motor_can_instance->tx_id);

    /* 尝试重启电机 */
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    LOGWARNING("[dm_motor] Trying to restart motor...");
}

/**
 * @brief DM电机编码器校准
 * @param motor
 */
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);
    DWT_Delay(0.1f);

    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1f);

    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1f);
}


DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    if (idx >= DM_MOTOR_CNT)
    {
        LOGERROR("[dm_motor] Motor count exceeded MAX (%d)", DM_MOTOR_CNT);
        return NULL;
    }

    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    if (motor == NULL)
    {
        LOGERROR("[dm_motor] malloc failed!");
        return NULL;
    }
    memset(motor, 0, sizeof(DMMotorInstance));

    /* 基本设置 */
    motor->motor_settings = config->controller_setting_init_config;
    motor->motor_type     = config->motor_type;
    motor->_DM_P_MAX      = config->_DM_P_MAX;
    motor->_DM_V_MAX      = config->_DM_V_MAX;
    motor->_DM_T_MAX      = config->_DM_T_MAX;

    /* PID初始化（使用当前项目PID API） */
    PID_init(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PID_init(&motor->motor_controller.speed_PID,    &config->controller_param_init_config.speed_PID);
    PID_init(&motor->motor_controller.angle_PID,    &config->controller_param_init_config.angle_PID);

    /* 外部反馈指针 */
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    /* 注册CAN实例（每个DM电机独立CAN实例，使用Buffer 1） */
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    config->can_init_config.tx_buf_idx = 1; // DM电机共享TX Buffer 1
    motor->motor_can_instance = CANRegister(&config->can_init_config);

    if (motor->motor_can_instance == NULL)
    {
        LOGERROR("[dm_motor] CANRegister failed!");
        free(motor);
        return NULL;
    }

    /* 注册守护进程 */
    Daemon_Init_Config_s conf = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    /* 使能电机 */
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1f);

    LOGINFO("[dm_motor] Init: type=%d, tx_id=0x%03X, rx_id=0x%03X",
            motor->motor_type, config->can_init_config.tx_id, config->can_init_config.rx_id);

    dm_motor_instance[idx++] = motor;
    return motor;
}


void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}


/**
 * @brief DM电机控制任务
 *
 * @note  由 motor_task 以 1kHz 频率调用
 *        打包MIT协议帧 → 发送
 */
void DMMotorControl(void)
{
    DMMotorInstance *motor;
    Motor_Controller_s *motor_controller;
    DM_Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting;
    DMMotor_Send_s motor_send_mailbox;
    float pid_ref, set, pid_measure;

    for (uint8_t i = 0; i < idx; ++i)
    {
        motor = dm_motor_instance[i];
        setting          = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure          = &motor->measure;
        pid_ref          = motor->motor_controller.pid_ref;

        /* 错误状态检查 */
        if (measure->state != 1 && measure->state != 0)
        {
            LOGERROR("[dm_motor] Motor[%d] state error: %d!", measure->id, measure->state);
        }

        /* 反转处理 */
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        /* 位置环 */
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->position;

            pid_ref = PID_calc(&motor_controller->angle_PID, pid_ref, pid_measure);
        }

        /* 速度环 */
        if ((setting->close_loop_type & SPEED_LOOP) &&
            (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->velocity;

            pid_ref = PID_calc(&motor_controller->speed_PID, pid_ref, pid_measure);
        }

        /* 电流前馈 */
        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;

        /* 反馈反向 */
        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

#ifdef USE_MIT_MODE
        /* MIT模式：使用内置位置/速度环 */
        switch (setting->outer_loop_type)
        {
        case ANGLE_LOOP: {
            float pos = motor_controller->pid_ref / 57.295779f;
            LIMIT_MIN_MAX(pos, -motor->_DM_P_MAX, motor->_DM_P_MAX);
            motor_send_mailbox.position_des = float_to_uint(pos, -motor->_DM_P_MAX, motor->_DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, -motor->_DM_V_MAX, motor->_DM_V_MAX, 12);
            motor_send_mailbox.torque_des   = float_to_uint(0, -motor->_DM_T_MAX, motor->_DM_T_MAX, 12);
            motor_send_mailbox.Kp = float_to_uint(motor_controller->angle_PID.Kp * 500.0f, KP_MIN, KP_MAX, 12);
            motor_send_mailbox.Kd = float_to_uint(motor_controller->angle_PID.Kd * 5.0f, KD_MIN, KD_MAX, 12);
            break;
        }
        case SPEED_LOOP: {
            float speed = motor_controller->pid_ref;
            motor_send_mailbox.position_des = float_to_uint(0, -motor->_DM_P_MAX, motor->_DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(speed, -motor->_DM_V_MAX, motor->_DM_V_MAX, 12);
            motor_send_mailbox.torque_des   = float_to_uint(0, -motor->_DM_T_MAX, motor->_DM_T_MAX, 12);
            motor_send_mailbox.Kp = float_to_uint(motor_controller->speed_PID.Kp * 500.0f, KP_MIN, KP_MAX, 12);
            motor_send_mailbox.Kd = float_to_uint(motor_controller->speed_PID.Kd * 5.0f, KD_MIN, KD_MAX, 12);
            break;
        }
        default:
            break;
        }
#else
        /* 力矩控制模式（默认） */
        set = pid_ref;
        if (motor->stop_flag == MOTOR_STOP)
            set = 0.0f;

        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des   = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;
#endif

        /* 打包MIT协议8字节帧 */
        motor->motor_can_instance->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instance->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instance->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instance->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) |
                                                          (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instance->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instance->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instance->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) |
                                                          (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instance->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        /* 独立发送 */
        CANTransmit(motor->motor_can_instance, 1);
    }
}
