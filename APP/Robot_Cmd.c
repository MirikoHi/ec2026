#include "Robot_Cmd.h"
#include "bsp_log.h"
#include "JY901S.h"
#include "trace.h"
#include "OLED.h"
#include "ADC_Voltage.h"
#include "can_comm.h"
#include "misc.h"
#include "dwt.h"
#include "tjc.h"
#include "Chassis.h"
#include "math.h"
#include "K230.h"
#include "icm42688.h"
icm42688RawData_t Chassis_Gyro;

static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

QueueHandle_t chassis_cmd_queue = NULL,gimbal_cmd_queue =NULL;
QueueHandle_t chassis_fetch_data_queue = NULL;
QueueHandle_t trace_fetch_data_queue = NULL;

chassis_cmd_q chassis_cmd_send={0};
gimbal_cmd_q gimbal_cmd_send ={0};
trace_fetch_data_q trace_fetch_data={0};

pid_type_def gimbal_yaw_PID={0};
pid_type_def gimbal_pitch_PID={0};
pid_type_def gimbal_yaw_forwardfeed_PID = {0};

float last_trace_imu_switch_s=0;
float now_time=0;
State robotcmd_control_state = DISABLE;

void tjc_control(void);
void draw_sin(void);
void Gimbal_Pid_Cal(void);
void RobotCmd_Init(void)
{
	chassis_cmd_queue = xQueueCreate(4, sizeof(chassis_cmd_q));
	trace_fetch_data_queue = xQueueCreate(4, sizeof(trace_fetch_data_q));
	
	gimbal_cmd_queue = xQueueCreate(4,sizeof(gimbal_cmd_q));
//	while(bsp_Icm42688Init()!=0x00);
	BSPLogInit();
	

	chassis_cmd_send.Chassis_Mode = TRACE_MODE;  //todo:这里记得改回默认值，调试用
		pid_init_config_s gimbal_yaw_pid_config={
		.mode = PID_POSITION,
		.Kp = 0.003f,
		.Kd = 0.0001f,
		.Ki = 0.0f,
		.max_out = 4.0f,
		.max_iout = 1.0f,
		
	};
	PID_init(&gimbal_yaw_PID,&gimbal_yaw_pid_config);
	
	
	pid_init_config_s gimbal_pitch_pid_config={
		.mode = PID_POSITION,
		.Kp = -0.003f,
		.Kd = -0.0001f,
		.Ki = 0.0f,
		.max_out = 4.0f,
		.max_iout = 1.0f,
		
	};
	PID_init(&gimbal_pitch_PID,&gimbal_pitch_pid_config);
	
	pid_init_config_s gimbal_yaw_forwardfeed_pid_config={
		.mode = PID_POSITION,
		.Kp = 0.000f,
		.Kd = -0.0001f,
		.Ki = 0.0f,
		.max_out = 4.0f,
		.max_iout = 1.0f,
		
	};
	PID_init(&gimbal_yaw_forwardfeed_PID,&gimbal_yaw_forwardfeed_pid_config);

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
//	bsp_IcmGetGyroscope(&Chassis_Gyro);
	
	if(gimbal_cmd_send.task_flag ==2)
	{
			Gimbal_Pid_Cal();
		if(!DaemonIsOnline(K230_Lost_Target_daemon))
		{
			gimbal_cmd_send.yaw+=0.6;
		}
	}
//	draw_sin();

	//通过队列向云台和底盘发送命令
	xQueueSend(chassis_cmd_queue, &chassis_cmd_send, 0U);
	xQueueSend(gimbal_cmd_queue,&gimbal_cmd_send,0U);

	CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
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
void Gimbal_Pid_Cal(void)
{
	PID_calc(&gimbal_yaw_PID,0,K230_err[0]);
	PID_calc(&gimbal_pitch_PID,0,K230_err[1]);
	gimbal_cmd_send.yaw += gimbal_yaw_PID.out;
	gimbal_cmd_send.pitch +=gimbal_pitch_PID.out;
}
void Task_Callback(uint8_t i)
{
	if(i==0)
	{
		gimbal_cmd_send.task_flag = 1;
	}
	else if(i==1)
	{
		gimbal_cmd_send.task_flag = 2;
	}
}