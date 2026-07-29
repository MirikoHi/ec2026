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

QueueHandle_t chassis_cmd_queue = NULL,gimbal_cmd_queue =NULL;
QueueHandle_t chassis_fetch_data_queue = NULL;
QueueHandle_t trace_fetch_data_queue = NULL;
chassis_cmd_q chassis_cmd_send={0};
chassis_cmd_q last_chassis_cmd_send;         // 上一帧发送底盘的控制命令
trace_fetch_data_q trace_fetch_data={0};


float last_trace_imu_switch_s=0;
float now_time=0;
Control_mode robotcmd_control_state = MENU_CTL;

/*  Lichee通信  */
LicheervnanoStatus_t Licheervnano_status = OFFLINE;;   //判断无线通讯状态
static volatile Licheervnano_Frame LicheeRec_Frame;

// void tjc_control(void);
void draw_sin(void);
void Gimbal_Pid_Cal(void);

void RobotCmd_Init(void)
{
	chassis_cmd_queue = xQueueCreate(4, sizeof(chassis_cmd_q));
	trace_fetch_data_queue = xQueueCreate(4, sizeof(trace_fetch_data_q));
	gimbal_cmd_queue = xQueueCreate(4,sizeof(gimbal_cmd_q));
	BSPLogInit();

	chassis_cmd_send.Chassis_Mode = NORMAL_MODE;  //todo:这里记得改回默认值，调试用

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
			case 0:  // 上位机下线
				chassis_cmd_send.remote_lost = 1;
				last_chassis_cmd_send.Chassis_Mode = NORMAL_MODE;
				chassis_cmd_send.remote_forward = 0.0f;
				robotcmd_control_state = MENU_CTL;
				break;
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

	}

	// draw_sin();

	//通过队列向云台和底盘发送命令
	xQueueSend(chassis_cmd_queue, &chassis_cmd_send, 0U);
	// xQueueSend(gimbal_cmd_queue,&gimbal_cmd_send,0U);

	//CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
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
void Chassis_Mode_Switch_Callback(uint8_t i)  //选择底盘控制模式
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

void Task_Callback(uint8_t i)    //选择执行任务
{
	if(i==0)
	{
		chassis_cmd_send.task_flag = 1;
	}
}
