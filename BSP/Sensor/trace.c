#include "trace.h"
#include "Robot_Cmd.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "dwt.h"
#include "gray_serial.h"
#define SENSOR_WEIGHTS { -4.0f, -3.0f, -2.0f, -1.0f, 1.0f, 2.0f, 3.0f, 4.0f }
pid_type_def Trace_PID={0};
trace_fetch_data_q trace_feedback_data={0};
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "dcmotor.h"
unsigned short Anolog[8]={0};
unsigned short white[8]={3238,3247,3211,3185,3252,3278,3253,3175};
unsigned short black[8]={1305,1657,933,1278,2703,2686,2726,1375};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
unsigned char Digtal;

No_MCU_Sensor sensor;
float track_err;
void Trace_Init(void)
{
#ifdef USE_GRAY_SERIAL
    Gray_Serial_Init();
#else
	NVIC_EnableIRQ(ADC1_INST_INT_IRQN);
	// 初始化传感器并获取黑白值
    No_MCU_Ganv_Sensor_Init_Frist(&sensor);
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    Get_Anolog_Value(&sensor,Anolog);
    // 串口打印出ADC的值，可以通过ADC作为黑白值的校准
    // 也可以自己写电控逻辑做一次校准程序

    DWT_Delay(0.1);
    // 得到黑白校准值之后，初始化传感器
    No_MCU_Ganv_Sensor_Init(&sensor,white,black);
    DWT_Delay(0.1);
#endif
	pid_init_config_s trace_config={
		.mode = PID_POSITION,
		.Kp = 0.0075f,
		.Kd = 0.0f,
		.Ki = 0.0f,
		.max_out = 500.0f,
		.max_iout = 200.0f,
		//.feedforward = 0.0f,
	};
	PID_init(&Trace_PID,&trace_config);
}

float Cal_Trace_Err(uint8_t current_trace) {
    // 定义每个传感器的权重，8个传感器对称分布
    const int weights[8] = SENSOR_WEIGHTS;

    int sum = 0;            // 权重累加和
    int active_count = 0;   // 检测到黑线的传感器计数

    // 遍历每个传感器（bit0 - bit7）
    for (int i = 0; i < 8; i++) {
        // 检查i位是否为0（检测到黑线）
        if (!(current_trace & (1 << i))) {
            sum += weights[i];
            active_count++;
        }
    }

    // 处理未检测到黑线的情况
    if (active_count == 0) {
        // 额外处理：可根据需求返回默认值或最小值
        // 此处返回0.0f表示居中
        return 0.0f;
    }

    // 计算加权平均误差
    return (float)sum / (float)active_count;
}

float Trace_task(void)
{
	// 定时调用传感器任务，包含模拟数据采集和数字化一整个流程
#ifdef USE_GRAY_SERIAL
    Digtal = Gray_Serial_Read();
#else
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    // 定时调用传感器任务，包含模拟数据采集和数字化一整个流程
//            No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
    // 获取数字量传感器数据（只有当黑白值填进去之后才会有数字量输出）
    Digtal=Get_Digtal_For_User(&sensor);
#endif
    track_err = Cal_Trace_Err(Digtal);
    // 循迹任务频率1kHz只需要delay 1ms，如果是100Hz需要delay 10ms，根据需求选择使用
		PID_calc(&Trace_PID,0,track_err);
	return Trace_PID.out;
}
