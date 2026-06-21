#include "trace.h"
#include "Robot_Cmd.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "dwt.h"
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
	NVIC_EnableIRQ(ADC1_INST_INT_IRQN);
	//初始化传感器，不带黑白值
    No_MCU_Ganv_Sensor_Init_Frist(&sensor);
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    Get_Anolog_Value(&sensor,Anolog);
    //此时打印的ADC的值，可用通过这个ADC作为黑白值的校准
    //也可以自己写按键逻辑完成一键校准功能

    DWT_Delay(0.1);
    //得到黑白校准值之后，初始化传感器
    No_MCU_Ganv_Sensor_Init(&sensor,white,black);
    DWT_Delay(0.1);
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
    // 定义每个传感器的权重（8个传感器）
    const int weights[8] = SENSOR_WEIGHTS;
    
    int sum = 0;       // 权重累加和
    int active_count = 0; // 检测到黑线的传感器数量

    // 遍历每个传感器（bit0-bit7）
    for (int i = 0; i < 8; i++) {
        // 检查第i位是否为0（检测到黑线）
        if (!(current_trace & (1 << i))) {
            sum += weights[i];
            active_count++;
        }
    }

    // 处理未检测到黑线的情况
    if (active_count == 0) {
        // 特殊处理：可根据需求返回最大/最小值
        // 此处返回0.0f表示无误差
        return 0.0f;
    }

    // 计算加权平均误差
    return (float)sum / (float)active_count;
}

float Trace_task(void)
{
	//无时基传感器常规任务，包含模拟量，数字量，归一化量
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    //有时基传感器常规任务，包含模拟量，数字量，归一化量
//            No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
    //获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
    Digtal=Get_Digtal_For_User(&sensor);
    track_err = Cal_Trace_Err(Digtal);
    //经典版理论性能1khz，只需要delay1ms，青春版100hz，需要delay10ms，否则不能正常使用
		PID_calc(&Trace_PID,0,track_err);
	return Trace_PID.out;
}