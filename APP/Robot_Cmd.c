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
#include "../BSP/Comm/TJC.h"
#include "Chassis.h"
#include "math.h"
#include "K230.h"
#include "elrs.h"
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
const volatile ELRS_Data_s *robotcmd_elrs = NULL;

pid_type_def gimbal_yaw_PID={0};
pid_type_def gimbal_pitch_PID={0};
pid_type_def gimbal_yaw_forwardfeed_PID = {0};

float last_trace_imu_switch_s=0;
float now_time=0;
State robotcmd_control_state = DISABLE;

/* ELRS 通道映射：这里用 1 开始的通道号，和遥控器界面保持一致。 */
#define ROBOTCMD_ELRS_CH_TURN             1U    /* CH1：手动遥控转向摇杆。 */
#define ROBOTCMD_ELRS_CH_FORWARD          2U    /* CH2：手动遥控前后摇杆。 */
#define ROBOTCMD_ELRS_CH_REMOTE_ENABLE    5U    /* CH5：进入手动遥控模式。 */
#define ROBOTCMD_ELRS_CH_SAFETY_DISABLE   6U    /* CH6：底盘安全失能。 */
#define ROBOTCMD_ELRS_CH_IMU_ACTION       7U    /* CH7：进入 IMU_MODE，执行 Chassis_ImuModeAction。 */

/* 拨杆阈值使用 CRSF 原始通道值，不是 PWM 微秒值。 */
#define ROBOTCMD_ELRS_REMOTE_ENABLE_TH    900U
#define ROBOTCMD_ELRS_SAFETY_DISABLE_TH   ELRS_CHANNEL_VALUE_MID
#define ROBOTCMD_ELRS_IMU_ACTION_TH       1000U

/* 手动遥控速度缩放：摇杆通道先由 CRSF 原始值归一化到 [-1, 1]。 */
#define ROBOTCMD_ELRS_REMOTE_DEADBAND     0.04f
#define ROBOTCMD_ELRS_MAX_FORWARD         0.25f
#define ROBOTCMD_ELRS_MAX_TURN            0.18f

typedef enum
{
	ROBOTCMD_SWITCH_ACTION_DISABLE = 0,  /* 立即失能底盘输出。 */
	ROBOTCMD_SWITCH_ACTION_MODE,         /* 拨杆保持时强制进入指定底盘模式。 */
} RobotCmd_SwitchAction_e;

typedef struct
{
	uint8_t channel;                 /* 1 开始的 ELRS 通道号。 */
	uint16_t threshold;              /* 原始通道值大于该阈值时触发。 */
	RobotCmd_SwitchAction_e action;  /* 该拨杆触发后的动作类型。 */
	Chassis_Mode_e mode;             /* MODE 类型动作对应的目标底盘模式。 */
} RobotCmd_SwitchRule_s;

/*
 * 模式/安全类拨杆统一写在这里，表的顺序就是优先级。
 * 后续增加新功能只需要按同样格式加一行，例如：
 * { 8U, 1000U, ROBOTCMD_SWITCH_ACTION_MODE, POSITION_MODE },
 */
static const RobotCmd_SwitchRule_s robotcmd_switch_rules[] = {
	{ ROBOTCMD_ELRS_CH_SAFETY_DISABLE, ROBOTCMD_ELRS_SAFETY_DISABLE_TH,
	  ROBOTCMD_SWITCH_ACTION_DISABLE,  NORMAL_MODE },
	{ ROBOTCMD_ELRS_CH_IMU_ACTION,     ROBOTCMD_ELRS_IMU_ACTION_TH,
	  ROBOTCMD_SWITCH_ACTION_MODE,     IMU_MODE },
};

void tjc_control(void);
void draw_sin(void);
void Gimbal_Pid_Cal(void);
static void RobotCmd_UpdateRemoteMode(void);
static uint8_t RobotCmd_ApplySwitchRules(Chassis_Mode_e *last_non_remote_mode,
                                         Chassis_Mode_e *switch_restore_mode,
                                         Chassis_Mode_e *active_switch_mode);
static void RobotCmd_ExitRemoteMode(Chassis_Mode_e restore_mode);
static float RobotCmd_ELRSChannelToNorm(uint16_t channel);
void RobotCmd_Init(void)
{
	chassis_cmd_queue = xQueueCreate(4, sizeof(chassis_cmd_q));
	trace_fetch_data_queue = xQueueCreate(4, sizeof(trace_fetch_data_q));
	
	gimbal_cmd_queue = xQueueCreate(4,sizeof(gimbal_cmd_q));
//	while(bsp_Icm42688Init()!=0x00);
	BSPLogInit();
	ELRS_Init();
	robotcmd_elrs = ELRS_GetData();

	// 蓝牙初始化，请在CmakeLists里面指定是1/2哪个蓝牙模块
	// BlueToothUart_Init();


	chassis_cmd_send.Chassis_Mode = IMU_MODE;  //todo:这里记得改回默认值，调试用
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
	RobotCmd_UpdateRemoteMode();
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

	TJC_Process();

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
void Gimbal_Pid_Cal(void)
{
	PID_calc(&gimbal_yaw_PID,0,K230_data.x);
	PID_calc(&gimbal_pitch_PID,0,K230_data.y);
	gimbal_cmd_send.yaw += gimbal_yaw_PID.out;
	gimbal_cmd_send.pitch +=gimbal_pitch_PID.out;
}

static void RobotCmd_UpdateRemoteMode(void)
{
	static Chassis_Mode_e last_non_remote_mode = TRACE_MODE;
	static Chassis_Mode_e switch_restore_mode = TRACE_MODE;
	static Chassis_Mode_e active_switch_mode = NORMAL_MODE;

	/* ELRS尚未初始化时，不接管原有菜单/循迹模式。 */
	if (robotcmd_elrs == NULL)
	{
		return;
	}

	/* 从未收到过遥控器有效帧：保持原模式，避免开机无遥控器时误触发失能。 */
	if (robotcmd_elrs->rc_frame_count == 0U)
	{
		return;
	}

	/*
	 * 曾经在线后又离线：退出遥控模式，并通知底盘失能电机。
	 * 这样不会保持最后一次遥控输出，也不会影响开机前从未接入遥控器的场景。
	 */
	if (ELRS_IsOnline() == 0U)
	{
		chassis_cmd_send.remote_disable = 1U;
		active_switch_mode = NORMAL_MODE;
		RobotCmd_ExitRemoteMode(last_non_remote_mode);
		return;
	}

	/*
	 * 先处理表驱动拨杆，再处理 CH5 手动遥控。
	 * 表内顺序就是优先级：安全失能优先，其次是动作/模式拨杆。
	 */
	if (RobotCmd_ApplySwitchRules(&last_non_remote_mode,
	                              &switch_restore_mode,
	                              &active_switch_mode) != 0U)
	{
		return;
	}

	chassis_cmd_send.remote_disable = 0U;

	/* 进入遥控前记录当前模式，退出遥控时恢复到这个模式。 */
	if (chassis_cmd_send.Chassis_Mode != REMOTE_MODE)
	{
		last_non_remote_mode = chassis_cmd_send.Chassis_Mode;
	}

	/*
	 * CH5 控制手动遥控，优先级低于 CH6 安全失能和 CH7 IMU 动作。
	 */
	if (robotcmd_elrs->channel[ROBOTCMD_ELRS_CH_REMOTE_ENABLE - 1U] > ROBOTCMD_ELRS_REMOTE_ENABLE_TH)
	{
		/* CH1 取反是为了匹配当前底盘转向方向约定。 */
		chassis_cmd_send.Chassis_Mode = REMOTE_MODE;
		chassis_cmd_send.remote_turn =
			-1.0f *
			RobotCmd_ELRSChannelToNorm(robotcmd_elrs->channel[ROBOTCMD_ELRS_CH_TURN - 1U]) *
			ROBOTCMD_ELRS_MAX_TURN;
		chassis_cmd_send.remote_forward =
			RobotCmd_ELRSChannelToNorm(robotcmd_elrs->channel[ROBOTCMD_ELRS_CH_FORWARD - 1U]) *
			ROBOTCMD_ELRS_MAX_FORWARD;
	}
	else if (chassis_cmd_send.Chassis_Mode == REMOTE_MODE)
	{
		RobotCmd_ExitRemoteMode(last_non_remote_mode);
	}
}

static uint8_t RobotCmd_ApplySwitchRules(Chassis_Mode_e *last_non_remote_mode,
                                         Chassis_Mode_e *switch_restore_mode,
                                         Chassis_Mode_e *active_switch_mode)
{
	for (uint8_t i = 0U; i < (sizeof(robotcmd_switch_rules) / sizeof(robotcmd_switch_rules[0])); i++)
	{
		const RobotCmd_SwitchRule_s *rule = &robotcmd_switch_rules[i];
		const uint16_t channel_value = robotcmd_elrs->channel[rule->channel - 1U];

		/* 表中的每一行都按同一格式判断：通道原始值超过阈值就执行动作。 */
		if (channel_value <= rule->threshold)
		{
			continue;
		}

		if (rule->action == ROBOTCMD_SWITCH_ACTION_DISABLE)
		{
			/* 安全失能最高优先级，同时清除正在保持的模式拨杆状态。 */
			chassis_cmd_send.remote_disable = 1U;
			*active_switch_mode = NORMAL_MODE;
			RobotCmd_ExitRemoteMode(*last_non_remote_mode);
			return 1U;
		}

		if (*active_switch_mode != rule->mode)
		{
			/* 记录拨杆释放后要恢复的模式。 */
			*switch_restore_mode = (chassis_cmd_send.Chassis_Mode == REMOTE_MODE) ?
			                       *last_non_remote_mode : chassis_cmd_send.Chassis_Mode;
			*active_switch_mode = rule->mode;
		}

		chassis_cmd_send.remote_disable = 0U;
		chassis_cmd_send.Chassis_Mode = rule->mode;
		chassis_cmd_send.remote_turn = 0.0f;
		chassis_cmd_send.remote_forward = 0.0f;
		return 1U;
	}

	/* 当前没有模式拨杆触发；若刚释放拨杆，只恢复一次原模式。 */
	if (*active_switch_mode != NORMAL_MODE)
	{
		if (chassis_cmd_send.Chassis_Mode == *active_switch_mode)
		{
			chassis_cmd_send.Chassis_Mode = *switch_restore_mode;
		}
		*active_switch_mode = NORMAL_MODE;
	}

	return 0U;
}

static void RobotCmd_ExitRemoteMode(Chassis_Mode_e restore_mode)
{
	if (chassis_cmd_send.Chassis_Mode == REMOTE_MODE)
	{
		chassis_cmd_send.Chassis_Mode = restore_mode;
	}
	/* 退出或绕过 REMOTE_MODE 时清零，避免底盘收到残留摇杆速度。 */
	chassis_cmd_send.remote_turn = 0.0f;
	chassis_cmd_send.remote_forward = 0.0f;
}

static float RobotCmd_ELRSChannelToNorm(uint16_t channel)
{
	float value;

	if (channel >= ELRS_CHANNEL_VALUE_MID)
	{
		value = (float)(channel - ELRS_CHANNEL_VALUE_MID) /
		        (float)(ELRS_CHANNEL_VALUE_MAX - ELRS_CHANNEL_VALUE_MID);
	}
	else
	{
		value = -((float)(ELRS_CHANNEL_VALUE_MID - channel) /
		          (float)(ELRS_CHANNEL_VALUE_MID - ELRS_CHANNEL_VALUE_MIN));
	}

	if ((value > -ROBOTCMD_ELRS_REMOTE_DEADBAND) && (value < ROBOTCMD_ELRS_REMOTE_DEADBAND))
	{
		value = 0.0f;
	}
	if (value > 1.0f)
	{
		value = 1.0f;
	}
	else if (value < -1.0f)
	{
		value = -1.0f;
	}

	return value;
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
