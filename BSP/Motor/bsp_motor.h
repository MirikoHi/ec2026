// #ifndef	__BSP_MOTOR_H__
// #define __BSP_MOTOR_H__
//
// #include "ti_msp_dl_config.h"
// #include "bsp_algorithm.h"
// #include "bsp_pid.h"
// //#define ENCODER_TO_SPEED_MS (100 * 0.065 * PI / 30 / 13 / 2)  // 编码器到速度的转换系数//最大0.5m/s
// //#define ENCODER_TO_DISDAN_M (0.065 * PI * 3 / 13 / 10 / 2 / 40) //0.01==10cm
// #define ENCODER_TO_SPEED_MS (100 * 0.065 * PI * 3 / 10 / 13 / 2 / 40)  // 编码器到速度的转换系数//最大0.5m/s
// #define ENCODER_TO_DISDAN_M (0.065 * PI * 3 / 13 / 10 / 2 / 40) //0.01==10cm
// #define FILTER_NUMS 15  // 定义滤波器窗口大小
//
// typedef enum
// {
//   left_90,
// 	right_90,
// 	back_180
// }spin_dir_t;
// // 新增转弯参数结构体
// typedef struct {
//     float start_distance;  // 转向起始点距离顶点距离
//     float steer_angle;     // 舵机角度
//     float left_distance;   // 左轮行驶距离
//     float right_distance;  // 右轮行驶距离
// } TurnParams;
// typedef enum
// {
//     MOTOR_STOP = 0,
//     MOTOR_ENALBED = 1,
// } Motor_Working_Type_e;
// typedef enum {
// 		MOTOR_CTRL_STOP=0,
//     MOTOR_CTRL_VELOCITY,       // 速度控制模式（单环）
// 		MOTOR_CTRL_POS_VEL_TRACE,  //位置-速度串级控制加循迹
//     MOTOR_CTRL_POS_VEL,        // 位置-速度串级控制
//     MOTOR_CTRL_YAW,            // 角度控制模式
// 	MOTOR_CTRL_TRACK,            //循迹测速控制模式
// } MotorCtrlMode;
// // 电机速度滤波器结构
// typedef struct {
//     float last_ref;           // 上次目标速度
//     float last_filtered;      // 上次滤波值
//     uint8_t is_initialized;   // 初始化标志 (替换filter_count)
//     uint8_t bypass_filter;    // 是否绕过滤波标志
//     float buffer[FILTER_NUMS]; // 滤波缓冲区
//     uint8_t index;            // 缓冲区索引
// } MotorSpeedFilter;
// typedef struct {
//     float raw_speed;           // 原始速度
//     float real_speed;          // 滤波后的速度
//     float ref_speed;           // 速度设定值
//     int32_t Encode;            // 编码器计数（单次）
//     int32_t total_Encode;      // 总编码器计数
//     float real_distance;       // 实际距离（米）
//     float basie_speed;         // 基础速度
//     Motor_Working_Type_e motor_working_type; // 电机工作状态（启停）
//     MotorSpeedFilter motor_filter; // 滤波器
//     pid_type_def motor_pid;    // 速度环PID
//     float ref_position;     // 目标位置（单位：米）
//     float start_distance;   // 新增：设置目标时的起始位移（单位：米）
// 		float ref_yaw;             // 角度设定值
// } Motor;
//
// extern MotorCtrlMode ctrl_mode;   // 控制模式
// extern uint8_t Spin_start_flag , Spin_succeed_flag , Stop_Flag;
// extern uint8_t Line_flag, Turn_flag;
// extern Motor motor_left;
// extern Motor motor_right;
// void Set_SpeedLeft(int pwmb);
// void motor_init(void);
// void Motor_Task(void);
// void Set_Motor_Ref_Speed(float left_speed, float right_speed);
// void Motor_SetCtrlMode(MotorCtrlMode mode);
// void Motor_SetPositionTarget(float position);
// void Motor_SetYawTarget(float yaw);
// void Motor_SetVelocityTarget(float left_speed, float right_speed);
// void Motor_set_position(float left,float right);
// void Motor_Set_Turn(spin_dir_t mode);
// #endif
