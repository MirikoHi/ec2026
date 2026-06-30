/**
 * @file dji_motor.c
 * @brief DJI 智能电机驱动实现
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        主要适配:
 *         - 单MCAN0总线（源项目3路CAN → 简化为1路）
 *         - 当前项目PID API: PID_calc(ref, measure) 注意参数顺序与源项目相反
 *         - 去掉功率控制代码（小规模应用不需要）
 *         - 发送分组简化至最多2组
 */

#include "dji_motor.h"
#include "dwt.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "string.h"


static uint8_t idx = 0;
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL};

/* 发送分组（单MCAN0，最多2组，每组4电机） */
#define DJI_SENDER_GROUP_MAX 2
static CANInstance sender_assignment[DJI_SENDER_GROUP_MAX];
static uint8_t sender_enable_flag[DJI_SENDER_GROUP_MAX] = {0};
static uint8_t sender_initialized = 0;


static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config);
static void DecodeDJIMotor(CANInstance *_instance);
static void DJIMotorLostCallback(void *motor_ptr);
static void InitSenderAssignment(void);


/**
 * @brief 初始化发送分组
 * @note  sender_assignment 不经过 CANRegister，仅用于TX
 */
static void InitSenderAssignment(void)
{
    if (sender_initialized)
        return;

    /* Group 0: tx_id=0x1FF (电压控制ID1-4) / 0x1FE (电流控制ID1-4) */
    memset(&sender_assignment[0], 0, sizeof(CANInstance));
    sender_assignment[0].tx_id      = 0x1FF;
    sender_assignment[0].tx_buf_idx = 0;
    sender_assignment[0].tx_elem    = (DL_MCAN_TxBufElement){
        .id   = CAN_STD_ID_TO_REG(0x1FF),
        .rtr  = 0,
        .xtd  = 0,
        .esi  = 0,
        .dlc  = 8,
        .brs  = 0,
        .fdf  = 0,
        .efc  = 0,
        .mm   = 0,
        .data = {0},
    };

    /* Group 1: tx_id=0x200 (电压控制ID5-8) / 0x2FE (电流控制ID5-8) */
    memset(&sender_assignment[1], 0, sizeof(CANInstance));
    sender_assignment[1].tx_id      = 0x200;
    sender_assignment[1].tx_buf_idx = 1;
    sender_assignment[1].tx_elem    = (DL_MCAN_TxBufElement){
        .id   = CAN_STD_ID_TO_REG(0x200),
        .rtr  = 0,
        .xtd  = 0,
        .esi  = 0,
        .dlc  = 8,
        .brs  = 0,
        .fdf  = 0,
        .efc  = 0,
        .mm   = 0,
        .data = {0},
    };

    sender_initialized = 1;
}


/**
 * @brief 根据电机类型和ID进行发送分组
 *
 * @note  单MCAN0，简化分组:
 *        M2006/M3508: ID1-4 → group 0 (0x1FF), ID5-8 → group 1 (0x200)
 *        GM6020电压:  ID1-4 → group 0 (0x1FF), ID5-8 → group 1 (0x2FF)
 *        GM6020电流:  ID1-4 → group 0 (0x1FE), ID5-8 → group 1 (0x2FE)
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从0开始
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    InitSenderAssignment();

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        /* 电压控制模式：0x1FF / 0x200 */
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = 0;                           // 0x1FF
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = 1;                           // 0x200
        }
        config->rx_id = 0x200 + motor_id + 1;
        /* 确保分组 tx_id 正确 */
        if (motor_grouping == 0)
            sender_assignment[0].tx_elem.id = CAN_STD_ID_TO_REG(0x1FF);
        else
            sender_assignment[1].tx_elem.id = CAN_STD_ID_TO_REG(0x200);
        break;

    case GM6020:
        if (motor->motor_control_type == CURRENT_CONTROL)
        {
            /* 电流控制：0x1FE / 0x2FE */
            if (motor_id < 4)
            {
                motor_send_num = motor_id;
                motor_grouping = 0;                       // 0x1FE
            }
            else
            {
                motor_send_num = motor_id - 4;
                motor_grouping = 1;                       // 0x2FE
            }
            if (motor_grouping == 0)
                sender_assignment[0].tx_elem.id = CAN_STD_ID_TO_REG(0x1FE);
            else
                sender_assignment[1].tx_elem.id = CAN_STD_ID_TO_REG(0x2FE);
        }
        else
        {
            /* 电压控制：0x1FF / 0x2FF */
            if (motor_id < 4)
            {
                motor_send_num = motor_id;
                motor_grouping = 0;                       // 0x1FF
            }
            else
            {
                motor_send_num = motor_id - 4;
                motor_grouping = 1;                       // 0x2FF
            }
            if (motor_grouping == 0)
                sender_assignment[0].tx_elem.id = CAN_STD_ID_TO_REG(0x1FF);
            else
                sender_assignment[1].tx_elem.id = CAN_STD_ID_TO_REG(0x2FF);
        }
        config->rx_id = 0x204 + motor_id + 1;
        break;

    default:
        LOGERROR("[dji_motor] Unsupported motor type!");
        return;
    }

    sender_enable_flag[motor_grouping] = 1;
    motor->message_num  = motor_send_num;
    motor->sender_group = motor_grouping;

    /* ID冲突检查 */
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
        {
            LOGERROR("[dji_motor] ID crash! rx_id=0x%03X", config->rx_id);
        }
    }
}


/**
 * @brief DJI电机CAN反馈解析回调
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure;

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    /* 解析反馈报文 */
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF *
                         (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    /* 多圈角度计算 */
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = (float)measure->total_round * 360.0f +
                           measure->angle_single_round + motor->motor_settings.angle_direction;
}

/**
 * @brief 电机离线回调
 */
static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    LOGWARNING("[dji_motor] Motor lost! tx_id=0x%03X", motor->motor_can_instance->tx_id);
}


DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    if (idx >= DJI_MOTOR_CNT)
    {
        LOGERROR("[dji_motor] Motor count exceeded MAX (%d)", DJI_MOTOR_CNT);
        return NULL;
    }

    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    if (instance == NULL)
    {
        LOGERROR("[dji_motor] malloc failed!");
        return NULL;
    }
    memset(instance, 0, sizeof(DJIMotorInstance));

    /* 基本设置 */
    instance->motor_type       = config->motor_type;
    instance->motor_settings   = config->controller_setting_init_config;
    instance->motor_close_type = config->motor_close_type;
    instance->motor_control_type = config->motor_control_type;

    /* PID初始化（使用当前项目PID API） */
    PID_init(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PID_init(&instance->motor_controller.speed_PID,    &config->controller_param_init_config.speed_PID);
    PID_init(&instance->motor_controller.angle_PID,    &config->controller_param_init_config.angle_PID);

    /* 外部反馈指针 */
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    /* 分组 */
    MotorSenderGrouping(instance, &config->can_init_config);

    /* 注册CAN实例（用于接收） */
    config->can_init_config.can_module_callback = DecodeDJIMotor;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    if (instance->motor_can_instance == NULL)
    {
        LOGERROR("[dji_motor] CANRegister failed!");
        free(instance);
        return NULL;
    }

    LOGINFO("[dji_motor] Init: type=%d, tx_id=0x%03X, rx_id=0x%03X, group=%d",
            instance->motor_type, config->can_init_config.tx_id,
            config->can_init_config.rx_id, instance->sender_group);

    /* 注册守护进程 */
    Daemon_Init_Config_s daemon_config = {
        .callback     = DJIMotorLostCallback,
        .owner_id     = instance,
        .reload_count = 200, // 20ms未收到数据判定离线
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}


void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}


/**
 * @brief DJI电机控制任务（PID级联 + 分组发送）
 *
 * @note  由 motor_task 以 1kHz 频率调用
 *        遍历所有注册电机 → 级联PID计算 → 填充发送缓冲 → 发送
 */
void DJIMotorControl(void)
{
    uint8_t group, num;
    int16_t set;
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting;
    Motor_Controller_s *motor_controller;
    DJI_Motor_Measure_s *measure;
    float pid_measure, pid_ref;

    /* 遍历所有电机, 进行串级PID计算 */
    for (uint8_t i = 0; i < idx; ++i)
    {
        motor = dji_motor_instance[i];
        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure          = &motor->measure;
        pid_ref          = motor_controller->pid_ref;

        /* 反转处理 */
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        /*
         * 位置环
         * @attention PID_calc(ref, measure) —— 注意参数顺序！
         */
        if ((motor_setting->close_loop_type & ANGLE_LOOP) &&
            (motor_setting->outer_loop_type == ANGLE_LOOP))
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;

            pid_ref = PID_calc(&motor_controller->angle_PID, pid_ref, pid_measure);
        }

        /* 速度环 */
        if ((motor_setting->close_loop_type & SPEED_LOOP) &&
            (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            /* 速度前馈 */
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_measure = -measure->speed_aps;
            else
                pid_measure = measure->speed_aps;

            pid_ref = PID_calc(&motor_controller->speed_PID, pid_ref, pid_measure);
        }

        /* 电流前馈（不进行电流环PID计算，电调自带电流环） */
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;

        /* 限幅并转换为16位整数 */
        LIMIT_MIN_MAX(pid_ref, -16384.0f, 16384.0f);
        set = (int16_t)pid_ref;

        /* 填入分组发送缓冲 */
        group = motor->sender_group;
        num   = motor->message_num;
        sender_assignment[group].tx_buff[2 * num]     = (uint8_t)(set >> 8);
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00FF);

        /* 停止状态清零 */
        if (motor->stop_flag == MOTOR_STOP)
        {
            memset(sender_assignment[group].tx_buff + 2 * num, 0, 2);
            motor->motor_controller.angle_PID.Iout = 0.0f;
            motor->motor_controller.speed_PID.Iout = 0.0f;
        }
    }

    /* 发送所有启用分组 */
    for (uint8_t i = 0; i < DJI_SENDER_GROUP_MAX; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}

