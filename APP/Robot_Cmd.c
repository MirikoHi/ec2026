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
#include "dcmotor.h"
#include "math.h"
#include "K230.h"
#include "lichee_rec.h"

static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static gimbal_cmd_q gimbal_cmd_send;  //向云台发送的控制命令

QueueHandle_t chassis_cmd_queue = NULL,gimbal_cmd_queue =NULL;
QueueHandle_t chassis_fetch_data_queue = NULL;
QueueHandle_t trace_fetch_data_queue = NULL;
chassis_cmd_q chassis_cmd_send={0};
chassis_cmd_q last_chassis_cmd_send;         // 上一帧发送底盘的控制命令
trace_fetch_data_q trace_fetch_data={0};
extern Chassis_Move_State_e car_stop;

float last_trace_imu_switch_s=0;
float now_time=0;
float elapsed=0;  //任务运行时间
Control_mode robotcmd_control_state = MENU_CTL;

/*  Lichee通信  */
LicheervnanoStatus_t Licheervnano_status = OFFLINE;;   //判断无线通讯状态
static volatile Licheervnano_Frame LicheeRec_Frame;

/*  OLED显示  */
/* ── 任务显示 ────────────────────────────────────────────── */
uint8_t task_display_id    = 0;     /* 0=无任务, 1=任务一, 2=任务二 */
float   task_start_time_s  = 0;     /* 任务开始时刻 (秒) */

// void tjc_control(void);
void draw_sin(void);
void Gimbal_Pid_Cal(void);

void RobotCmd_Init(void)
{
	chassis_cmd_queue = xQueueCreate(4, sizeof(chassis_cmd_q));
	trace_fetch_data_queue = xQueueCreate(4, sizeof(trace_fetch_data_q));
	gimbal_cmd_queue = xQueueCreate(4,sizeof(gimbal_cmd_q));
	BSPLogInit();

	chassis_cmd_send.Chassis_Mode = NORMAL_MODE;  // 上电默认 IMU 测试模式
	last_chassis_cmd_send.Chassis_Mode = NORMAL_MODE;

	chassis_feedback_data.real_vy = 100;
	//双板通信can初始化
	CANComm_Init_Config_s comm_conf = {
		.can_config = {.tx_id = 0x311, .rx_id = 0x312 },
		.recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
		.send_data_len = sizeof(Chassis_Upload_Data_s),
};
	chasiss_can_comm = CANCommInit(&comm_conf);
	// 无线通信初始化
	LicheeRec_Init();
}

/**
 * @brief 核心cmd任务，向云台和底盘发送命令，在RTOS中以200Hz运行
 */
void Robot_Cmd(void)
{
	xQueueReceive(trace_fetch_data_queue, &trace_fetch_data, 1);
	LicheeRec_Frame = LicheeRec_GetFrame();
	Licheervnano_status = Licheervnano_CheckOnline(LicheeRec_Frame.cmdid,LicheeRec_Frame.data);

	if (Licheervnano_status == ONLINE) {
		switch (LicheeRec_Frame.cmdid) {
			case 1:  // 上位机上线
				chassis_cmd_send.remote_lost = 0;
				robotcmd_control_state = REMOTE_CTL;
				chassis_cmd_send.Chassis_Mode = REMOTE_MODE;
				last_chassis_cmd_send.Chassis_Mode = chassis_cmd_send.Chassis_Mode;
				break;
			case 2:
				chassis_cmd_send.remote_forward = LicheeRec_Frame.data;
				break;
			case 3:
				chassis_cmd_send.remote_forward = -LicheeRec_Frame.data;
				break;
			case 5:  // 心跳时，延续底盘模式
				chassis_cmd_send.Chassis_Mode = last_chassis_cmd_send.Chassis_Mode;
				break;
			case 6:  // 遥控控制模式
				chassis_cmd_send.Chassis_Mode = REMOTE_MODE;
				last_chassis_cmd_send.Chassis_Mode = chassis_cmd_send.Chassis_Mode;
				break;
			case 7:  // IMU控制模式
				chassis_cmd_send.Chassis_Mode = IMU_MODE;
				break;
			default:
				break;
		}
	}
	else {
		chassis_cmd_send.remote_lost = 1;
		last_chassis_cmd_send.Chassis_Mode = NORMAL_MODE;
		chassis_cmd_send.remote_forward = 0.0f;
		robotcmd_control_state = MENU_CTL;
	}

	// draw_sin();

	//通过队列向云台和底盘发送命令
	xQueueSend(chassis_cmd_queue, &chassis_cmd_send, 0U);
	xQueueSend(gimbal_cmd_queue,&gimbal_cmd_send,0U);

	//CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);

	/* 计时更新 (Robot_Cmd 200Hz 中仅更新变量, 不碰 OLED I2C) */
	if (chassis_mode_selected && task_display_id > 0 && !car_stop) {
		elapsed = DWT_GetTimeline_s() - task_start_time_s;
	}
}

void draw_sin(void)
{
	static float start_time = 0;
	static float T_x = 0.1f;
	float aim_x = (DWT_GetTimeline_s()-start_time)*0.01f-0.15f;
	float aim_y;
	if(aim_x>0.15f)
	{
		start_time = DWT_GetTimeline_s();
		aim_x = (DWT_GetTimeline_s()-start_time)*0.01f-0.15f;
	}
	// gimbal_cmd_send.aim_x=aim_x;
	// gimbal_cmd_send.aim_y=0.1f*sinf(aim_x/T_x*PI);
}

/**
	各种底盘模式的回调函数
**/
volatile bool chassis_mode_selected = false;

void Chassis_Mode_Switch_Callback(uint8_t i)  //选择底盘控制模式
{
	chassis_mode_selected = true;  /* 选中模式后隐藏菜单, 只显示任务信息 */
	task_display_id   = i + 1;    /* 显示当前选中的模式编号 */
	task_start_time_s = DWT_GetTimeline_s();

	if(i == 0)
	{
		chassis_cmd_send.Chassis_Mode = NORMAL_MODE;
	}
	else if(i == 1)   //任务0
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

void Task_Callback(uint8_t i)    //选择执行任务
{
	chassis_mode_selected = true;  /* 选中任务后同样隐藏菜单, 全屏显示任务信息 */

	if(i==0)  //任务2，巡线走一圈
	{
		chassis_cmd_send.task_flag = 2;
		// chassis_cmd_send.Chassis_Mode = TRACE_MODE;
		chassis_cmd_send.Chassis_Mode = IMU_MODE;
		task_display_id   = 2;
		task_start_time_s = DWT_GetTimeline_s();
	}
	else if(i==1)    //任务3，静止状态，使小球在+5——-5间折返
	{
		chassis_cmd_send.task_flag = 3;
		gimbal_cmd_send.task_flag = 3;
		task_display_id   = 3;
		task_start_time_s = DWT_GetTimeline_s();
	}
	else if(i==2)    //任务4，钢球置于中心点走AB线段
	{
		chassis_cmd_send.task_flag = 4;
		task_display_id   = 4;
		task_start_time_s = DWT_GetTimeline_s();
	}
	else if(i==3)    //任务5，钢球置于中心点走一圈
	{
		chassis_cmd_send.task_flag = 5;
		chassis_cmd_send.Chassis_Mode = TRACE_MODE;
		task_display_id   = 5;
		task_start_time_s = DWT_GetTimeline_s();
	}
	else if(i==4)    //任务6，钢球置于指定位置走一圈
	{
		chassis_cmd_send.task_flag = 6;
		task_display_id   = 6;
		task_start_time_s = DWT_GetTimeline_s();
	}
}
void Reset_task_callback(uint8_t i) {
	if ( i==3 ) {
		chassis_cmd_send.task_flag = 0;
		gimbal_cmd_send.task_flag = 0;
		chassis_cmd_send.Chassis_Mode = NORMAL_MODE;
		car_stop = 0;
		task_display_id = 0;
		task_start_time_s = 0;
		elapsed=0;
	}
}
