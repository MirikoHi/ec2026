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

/**
 * @brief 从 8 路灰度传感器原始数据计算黑线位置误差
 *
 * 映射法则：找最大连续 0 块 → 中心位置 c → error = 7 - 2*c
 * - 2+ 相邻传感器见线 → 直接有效
 * - 单传感器见线 → 上下文感知 debounce（在线 2 帧 / 离线 8 帧）
 * - 全白 0xFF → 保持上一帧位置（间隙）
 * - 一阶 LPF 平滑输出
 *
 * @param raw 8 路传感器数字量（bit=0 表示黑线）
 * @return 线位置误差，范围约 -7.0 ~ +7.0，0 表示居中
 */
static float CalcLineError(uint8_t raw)
{
    static float   last_output = 0.0f;    // 上一帧输出（LPF 状态 + keep）
    static int     single_pos  = -1;      // 跟踪的孤立 0 位置，-1=无
    static uint8_t single_cnt  = 0;       // 同一孤立 0 连续帧数
    static uint8_t ff_gap_cnt  = 0;       // 当前孤立 0 出现前，连续 0xFF 帧数

    /* ---- 找最大连续 0 块 ---- */
    int best_start = -1, best_len = 0;
    int cur_start  = -1, cur_len  = 0;

    for (int i = 0; i < 8; i++) {
        if (!(raw & (1 << i))) {            // bit=0 → 黑线
            if (cur_start < 0) cur_start = i;
            cur_len++;
        } else {
            if (cur_len > best_len) {
                best_start = cur_start;
                best_len   = cur_len;
            }
            cur_start = -1;
            cur_len   = 0;
        }
    }
    if (cur_len > best_len) { best_start = cur_start; best_len = cur_len; }

    float raw_err;
    int   len = best_len;
    float c   = (len > 0) ? ((float)best_start + (float)(len - 1) * 0.5f) : 0.0f;

    /* ---- 分类处理 ---- */
    if (len >= 2) {
        /* 2+ 相邻传感器 → 有效线 */
        raw_err = 7.0f - 2.0f * c;
        single_pos  = -1;
        single_cnt  = 0;
        ff_gap_cnt  = 0;
    } else if (len == 1) {
        /* 单传感器 → 上下文感知 debounce */
        if (single_pos == best_start) {
            single_cnt++;
        } else {
            single_pos = best_start;
            single_cnt = 1;
        }

        uint8_t threshold = (ff_gap_cnt <= 1) ? 2 : 8;

        if (single_cnt >= threshold) {
            /* 确认有效 */
            raw_err = 7.0f - 2.0f * c;
        } else {
            /* debounce 中：保持上一帧 */
            raw_err = last_output;
        }
    } else {
        ff_gap_cnt++;
        if (ff_gap_cnt > 50) {  // 丢线超过 50 帧（约 50ms @ 1kHz）
            // 让输出向 0 衰减，而不是保持 last_output
            raw_err = 0.0f;     // 或 last_output * 0.95f 逐步衰减
        } else {
            raw_err = last_output;  // 短暂丢线，保持上一帧（过间隙）
        }
        single_pos = -1;
        single_cnt = 0;
    }

    /* ---- LPF 平滑 ---- */
    float output = last_output + 0.8 * (raw_err - last_output);  //这里滤波系数可以调，越靠近0越平滑
    last_output = output;

    return output;
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
    Digtal = Get_Digtal_For_User(&sensor);
#endif

    track_err = CalcLineError(Digtal); //todo:需新增一个从中间线出来，清除掉残留输出

    // LOGINFO("Digtal: %d %d %d %d %d %d %d %d  err:%.2f",
    //         (Digtal >> 7) & 1, (Digtal >> 6) & 1, (Digtal >> 5) & 1, (Digtal >> 4) & 1,
    //         (Digtal >> 3) & 1, (Digtal >> 2) & 1, (Digtal >> 1) & 1, (Digtal >> 0) & 1,
    //         track_err);

    // 循迹任务频率1kHz只需要delay 1ms，如果是100Hz需要delay 10ms，根据需求选择使用
    PID_calc(&Trace_PID, 0, -(track_err * (1.0f + 0.05f * fabsf(track_err)))); //todo:这里从原来的权重换成了-7到7的数据，边缘速度减小了，得细调
    return Trace_PID.out;
}
