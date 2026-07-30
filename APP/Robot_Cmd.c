#include "Robot_Cmd.h"
#include "bsp_log.h"
#include "../BSP/IMU/JY901S.h"
#include "trace.h"
#include "OLED.h"
#include "../BSP/Voltage/ADC_Voltage.h"
#include "can_comm.h"
#include "misc.h"
#include "dwt.h"
//#include "tjc.h"
#include "Chassis.h"
#include "math.h"
#include "K230.h"
/* ELRS 已弃用，UART3 改为 ZDT 电机通信 */
#include "../BSP/IMU/icm42688.h"
icm42688RawData_t Chassis_Gyro;

static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

QueueHandle_t chassis_cmd_queue = NULL,gimbal_cmd_queue =NULL;
QueueHandle_t chassis_fetch_data_queue = NULL;
QueueHandle_t trace_fetch_data_queue = NULL;

BlueTooth_Tx_t g_bt_tx = {0};
volatile BlueTooth_Rx_t g_bt_rx = {0};

chassis_cmd_q chassis_cmd_send={0};
gimbal_cmd_q gimbal_cmd_send ={0};
trace_fetch_data_q trace_fetch_data={0};
/* ELRS 已弃用 */

float last_trace_imu_switch_s=0;
float now_time=0;
State robotcmd_control_state = DISABLE;

/* ELRS 遥控已弃用，通道宏和拨杆规则已移除 */

void tjc_control(void);
void draw_sin(void);
static void RobotCmd_UpdateRemoteMode(void);
static void RobotCmd_ExitRemoteMode(Chassis_Mode_e restore_mode);
void RobotCmd_Init(void)
{
	chassis_cmd_queue = xQueueCreate(4, sizeof(chassis_cmd_q));
	trace_fetch_data_queue = xQueueCreate(4, sizeof(trace_fetch_data_q));
	
	gimbal_cmd_queue = xQueueCreate(4,sizeof(gimbal_cmd_q));
//	while(bsp_Icm42688Init()!=0x00);
	BSPLogInit();
	/* ELRS 已弃用，不再初始化 */

	// 蓝牙初始化，请在CmakeLists里面指定是1/2哪个蓝牙模块
	// BlueToothUart_Init();
	

	/* 上电直接执行任务2，不依赖菜单回调；task_start_seq 用于重新启动状态机。 */
	chassis_cmd_send.Chassis_Mode = IMU_MODE;
	chassis_cmd_send.competition_task = H_TASK_2_FAST_LAP;
	chassis_cmd_send.task_start_seq = 1U;
	gimbal_cmd_send.task_flag = (uint8_t)H_TASK_2_FAST_LAP;
	gimbal_cmd_send.task_start_seq = 1U;
	chassis_feedback_data.real_vy = 100;
	//双板通信can初始化
	CANComm_Init_Config_s comm_conf = {
		.can_config = {.tx_id = 0x311, .rx_id = 0x312 },
		.recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
		.send_data_len = sizeof(Chassis_Upload_Data_s),
};
	chasiss_can_comm = CANCommInit(&comm_conf);
	
}

/**
 * @brief 核心cmd任务，向云台和底盘发送命令，在RTOS中以200Hz运行
 */
void Robot_Cmd(void)
{
//	chassis_cmd_send.vx=70.0f;
	
	xQueueReceive(trace_fetch_data_queue, &trace_fetch_data, 1);
	// RobotCmd_UpdateRemoteMode();
//	bsp_IcmGetGyroscope(&Chassis_Gyro);
	/* 滚球目标由任务回调设置，禁止在主控制循环中自动扫动步进电机。 */

	//通过队列向云台和底盘发送命令
	xQueueSend(chassis_cmd_queue, &chassis_cmd_send, 0U);
	xQueueSend(gimbal_cmd_queue,&gimbal_cmd_send,0U);

	//CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);

	// // 蓝牙收发，需要时取消注释
	// BlueToothUart_Send(&g_bt_tx);
	// volatile BlueTooth_Rx_t *new_rx = BlueToothUart_Get();
	// if (new_rx != NULL)
	// {
	// 	g_bt_rx = *new_rx;
	// }
}
void Chassis_Mode_Switch_Callback(uint8_t i)
{
	if(i == 0)
	{
		chassis_cmd_send.Chassis_Mode = NORMAL_MODE;
	}
	else if(i == 1)
	{
		chassis_cmd_send.circle_set = 1;
		chassis_cmd_send.Chassis_Mode = TRACE_MODE;
	}
	else if(i == 2)
	{
		chassis_cmd_send.Chassis_Mode = IMU_MODE;
	}
	else if(i == 3)
	{
		chassis_cmd_send.Chassis_Mode = POSITION_MODE;
	}
}
void Control_Switch_Callback(uint8_t i)
{
	if(i == 0)
	{
		robotcmd_control_state = ENABLE;
	}
	else if(i ==1)
	{
		robotcmd_control_state = DISABLE;
	}
}


void draw_sin(void)
{
	static float start_time = 0;
	static float T_x = 0.1;
	float aim_x = (DWT_GetTimeline_s()-start_time)*0.01-0.15;
	float aim_y;
	if(aim_x>0.15)
	{
		start_time = DWT_GetTimeline_s();
		aim_x = (DWT_GetTimeline_s()-start_time)*0.01-0.15;
	}
	gimbal_cmd_send.aim_x=aim_x;
	gimbal_cmd_send.aim_y=0.1*sinf(aim_x/T_x*PI);
	
}

static void RobotCmd_UpdateRemoteMode(void)
{
	/* ELRS 已弃用，不再处理遥控器输入 */
	return;
}

static void RobotCmd_ExitRemoteMode(Chassis_Mode_e restore_mode)
{
	(void)restore_mode;
	if (chassis_cmd_send.Chassis_Mode == REMOTE_MODE)
	{
		chassis_cmd_send.Chassis_Mode = TRACE_MODE;
	}
	chassis_cmd_send.remote_turn = 0.0f;
	chassis_cmd_send.remote_forward = 0.0f;
}

void Task_Callback(uint8_t i)
{
	static const H_Task_e task_map[] = {
		H_TASK_2_FAST_LAP,
		H_TASK_3_STATIC_BALL,
		H_TASK_4_AB_BALL,
		H_TASK_5_CENTER_BALL_LAP,
		H_TASK_6_TARGET_BALL_LAP,
	};
	if (i >= (sizeof(task_map) / sizeof(task_map[0]))) return;

	chassis_cmd_send.competition_task = task_map[i];
	gimbal_cmd_send.task_flag = (uint8_t)task_map[i];
	++chassis_cmd_send.task_start_seq;
	gimbal_cmd_send.task_start_seq = chassis_cmd_send.task_start_seq;
	/* 任务3仅控制静止摆杆，其余任务启动体育场底盘控制器。 */
	chassis_cmd_send.Chassis_Mode = (task_map[i] == H_TASK_3_STATIC_BALL) ?
	                                 POSITION_MODE : IMU_MODE;
}
