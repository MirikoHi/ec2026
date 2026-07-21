#include "trace.h"
#include "Robot_Cmd.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "dwt.h"
#include "gray_serial.h"
#define SENSOR_WEIGHTS { -4.0f, -3.0f, -2.0f, -1.0f, 1.0f, 2.0f, 3.0f, 4.0f }

/* ---- filter_raw 可调参数 ---- */
#define FILTER_JUMP_THRESHOLD   3   /* 跳变确认所需连续帧数，可调 2~8 */
#define FILTER_LONG_GAP_FRAMES  50  /* 多少帧 0xFF 算长丢线 */
#define FILTER_GAP_BONUS        5   /* 长丢线后额外需要的确认帧数 */
pid_type_def Trace_PID={0};
trace_fetch_data_q trace_feedback_data={0};
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "dcmotor.h"
unsigned short Anolog[8]={0};
unsigned short white[8]={3238,3247,3211,3185,3252,3278,3253,3175};
unsigned short black[8]={1305,1657,933,1278,2703,2686,2726,1375};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
//巡线状态机
trace_state_e trace_state;
//原始数据
unsigned char Digtal;
// //滤除多余数据后的8位数据
// static volatile uint8_t raw_track_err;
// //8位映射成-7到7的数据，直接用来输出
// static volatile float process_digital;

No_MCU_Sensor sensor;

//处理原始八路数据后获得的-7.0-7.0的映射值
volatile float track_err;

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
		.Kp = 0.006f,
		.Kd = 0.0f,
		.Ki = 0.0f,
		.max_out = 500.0f,
		.max_iout = 200.0f,
		//.feedforward = 0.0f,
	};
	PID_init(&Trace_PID,&trace_config);
}

//旧版逻辑，暂时不用
// /**
//  * @brief 根据原始值计算巡线误差
//  * @param current_trace 传入Digital当前的八路灰度值
//  * @return 误差值
//  */
// float Cal_Trace_Err(uint8_t current_trace) {
//     // 定义每个传感器的权重，8个传感器对称分布
//     const int weights[8] = SENSOR_WEIGHTS;
//
//     int sum = 0;            // 权重累加和
//     int active_count = 0;   // 检测到黑线的传感器计数
//
//     // 遍历每个传感器（bit0 - bit7）
//     for (int i = 0; i < 8; i++) {
//         // 检查i位是否为0（检测到黑线）
//         if (!(current_trace & (1 << i))) {
//             sum += weights[i];
//             active_count++;
//         }
//     }
//
//     // 处理未检测到黑线的情况
//     if (active_count == 0) {
//         // 额外处理：可根据需求返回默认值或最小值
//         // 此处返回0.0f表示居中
//         return 0.0f;
//     }
//
//     // 计算加权平均误差
//     return (float)sum / (float)active_count;
// }

/* -------------------------------------------------- 含噪声滤除，现暂时不用，目前八路灰度高度合适无噪声 -------------------------------------------*/

/* ---- 辅助：提取 zero 区域 ---- */
static void find_zero_region(uint8_t raw, int *start, int *end)
{
    *start = -1;
    *end   = -1;
    if (raw == 0xFF) return;

    for (int i = 0; i < 8; i++) {
        if (!(raw & (1 << i))) {
            if (*start < 0) *start = i;
            *end = i;
        }
    }
}

/* ---- 辅助：两个 zero 区间是否相邻（含重叠、紧挨） ---- */
static int zero_region_adjacent(int a_start, int a_end, int b_start, int b_end)
{
    if (a_start < 0 || b_start < 0) return 0;
    return (a_start <= b_end + 1) && (a_end >= b_start - 1);
}

/* ---- 辅助：不连续 0 比特噪声纠正 ---- */
typedef struct {
    int start;
    int end;
    int len;
} zero_segment_t;

/**
 * @brief 纠正 8 路灰度数据中不连续的孤立 0 比特噪声
 *
 * 黑线在 8 个传感器上应为连续的 0 比特块。若出现多个不连续的 0 比特区域
 *（如 01100111 中 bit7 的孤立 0），取最长的连续 0 块为主区域，其余纠正为 1。
 * 等长时优先保留靠近已确认位置(trusted)的段。
 *
 * @param raw            原始 8 路灰度数字量
 * @param trusted_start  上一帧确认的 zero 区域起始 (-1=无信任信息)
 * @param trusted_end    上一帧确认的 zero 区域结束（仅用于参数一致性）
 * @return 纠正后的 8 路数据
 */
static uint8_t
correct_noise(uint8_t raw, int trusted_start, int trusted_end)
{
    (void)trusted_end;

    /* 全白或全黑：无需纠正 */
    if (raw == 0xFF || raw == 0x00) {
        return raw;
    }

    /* 扫描所有连续 0 比特段 */
    zero_segment_t segs[4];
    int seg_count = 0;
    int i = 0;

    while (i < 8) {
        while (i < 8 && (raw & (1 << i))) i++;
        if (i >= 8) break;

        int start = i;
        while (i < 8 && !(raw & (1 << i))) i++;
        int end = i - 1;

        segs[seg_count].start = start;
        segs[seg_count].end   = end;
        segs[seg_count].len   = end - start + 1;
        seg_count++;
    }

    /* 仅一段：检查是否为孤立单比特噪声 */
    if (seg_count == 1) {
        if (segs[0].len > 1) {
            return raw;                              /* 多比特连续零，正常线 */
        }
        /* 长度=1：检查与 trusted 区域的相邻性 */
        if (trusted_start >= 0 &&
            zero_region_adjacent(segs[0].start, segs[0].end,
                                 trusted_start, trusted_end)) {
            return raw;                              /* 与已知线相邻，正常移动 */
        }
        /* 孤立单比特零噪声：纠正该比特为 1 */
        return raw | (uint8_t)(1 << segs[0].start);
    }

    if (seg_count == 0) {
        return raw;
    }

    /* 选择主段：最长优先，等长时靠近 trusted 的优先 */
    int primary = 0;
    for (int s = 1; s < seg_count; s++) {
        if (segs[s].len > segs[primary].len) {
            primary = s;
        } else if (segs[s].len == segs[primary].len && trusted_start >= 0) {
            int dist_p = segs[primary].start - trusted_start;
            if (dist_p < 0) dist_p = -dist_p;
            int dist_s = segs[s].start - trusted_start;
            if (dist_s < 0) dist_s = -dist_s;
            if (dist_s < dist_p) {
                primary = s;
            }
        }
    }

    /* 将非主段的 0 比特翻转为 1 */
    uint8_t corrected = raw;
    for (int s = 0; s < seg_count; s++) {
        if (s == primary) continue;
        for (int bit = segs[s].start; bit <= segs[s].end; bit++) {
            corrected |= (uint8_t)(1 << bit);
        }
    }

    return corrected;
}

/**
 * @brief 对原始 8 路灰度位模式做时空滤波，过滤传感器噪声
 *
 * 规则：
 *  - 0xFF（全白/丢线）：保持上一帧非 0xFF 值，不更新内部 zero 位置
 *  - 非 0xFF：zero 区域与已确认区域相邻 → 立即接受
 *  - 非 0xFF：zero 区域跳变 → 需连续多帧确认（默认 3 帧）
 *  - 长丢线（≥50 帧 0xFF）后出现的跳变，需要额外 +5 帧确认
 *
 *  调用链：filter_raw → raw_transform_easy → second_process
 *
 * @param raw 8 路传感器数字量（bit=0 表示黑线，0xFF = 全白）
 * @return 滤波后的 8 路位模式
 */
uint8_t filter_raw(uint8_t raw)
{
    static uint8_t output         = 0xFF;  /* 滤波输出 */
    static int     trusted_start  = -1;    /* 已确认 zero 区间 */
    static int     trusted_end    = -1;
    static int     cand_start     = -1;    /* 候选跳变 zero 区间 */
    static int     cand_end       = -1;
    static int     cand_cnt       = 0;     /* 候选区间连续帧数 */
    static int     ff_cnt         = 0;     /* 连续 0xFF 帧数 */
    static int     was_long_gap   = 0;     /* 候选建立时是否处于长丢线 */

    //纠正孤立 0 比特噪声（不连续 0 比特 → 1），再进入时域滤波
    raw = correct_noise(raw, trusted_start, trusted_end);

    int z_start, z_end;
    find_zero_region(raw, &z_start, &z_end);

    /* ---- 0xFF：保持输出，累计丢线帧数 ---- */
    if (raw == 0xFF) {
        ff_cnt++;
        return output;
    }

    /* ---- 非 0xFF：记录本次丢线帧数，然后清零 ---- */
    int gap_before = ff_cnt;
    ff_cnt = 0;

    /* ---- 尚无已确认位置 ---- */
    if (output == 0xFF) {
        if (zero_region_adjacent(z_start, z_end, cand_start, cand_end)) {
            cand_cnt++;
            cand_start = z_start;
            cand_end   = z_end;
        } else {
            cand_start = z_start;
            cand_end   = z_end;
            cand_cnt   = 1;
            was_long_gap = (gap_before >= FILTER_LONG_GAP_FRAMES);
        }

        int threshold = was_long_gap
            ? FILTER_JUMP_THRESHOLD + FILTER_GAP_BONUS
            : FILTER_JUMP_THRESHOLD;

        if (cand_cnt >= threshold) {
            output        = raw;
            trusted_start = z_start;
            trusted_end   = z_end;
            cand_cnt      = 0;
        }
        return output;
    }

    /* ---- 有已确认位置：检查相邻性 ---- */
    if (zero_region_adjacent(z_start, z_end, trusted_start, trusted_end)) {
        /* 相邻 → 直接信任 */
        output        = raw;
        trusted_start = z_start;
        trusted_end   = z_end;
        cand_cnt      = 0;
        return output;
    }

    /* ---- 跳变：候选确认 ---- */
    if (zero_region_adjacent(z_start, z_end, cand_start, cand_end)) {
        cand_cnt++;
        cand_start = z_start;
        cand_end   = z_end;
    } else {
        cand_start = z_start;
        cand_end   = z_end;
        cand_cnt   = 1;
        was_long_gap = (gap_before >= FILTER_LONG_GAP_FRAMES);
    }

    int threshold = was_long_gap
        ? FILTER_JUMP_THRESHOLD + FILTER_GAP_BONUS
        : FILTER_JUMP_THRESHOLD;

    if (cand_cnt >= threshold) {
        output        = raw;
        trusted_start = z_start;
        trusted_end   = z_end;
        cand_cnt      = 0;
    }

    return output;
}

/**
 * @brief 把八路
 *
 * @param current_trace 8 路传感器数字量（bit=0 表示黑线）
 * @return 线位置，范围 -7.0 ~ 7.0，0 表示居中；0xFF 时返回 0.0f
 */
float raw_transform_easy(uint8_t current_trace)
{
    /* 全白：无有效信息，返回 0（居中） */
    if (current_trace == 0xFF) {
        return 0.0f;
    }

    int right_ones = 0;
    for (int i = 0; i < 8; i++) {
        if (current_trace & (1 << i))
            right_ones++;
        else
            break;
    }

    int left_ones = 0;
    for (int i = 7; i >= 0; i--) {
        if (current_trace & (1 << i))
            left_ones++;
        else
            break;
    }

    return (float)(right_ones - left_ones);
}

/* -------------------------------------------------- 含噪声滤除，现暂时不用，目前八路灰度高度合适无噪声 -------------------------------------------*/

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
        /* raw == 0xFF：全白 → 保持上一帧位置 */
        raw_err = last_output;
        ff_gap_cnt++;
        single_pos = -1;
        single_cnt = 0;
    }

    /* ---- LPF 平滑 ---- */
    float output = last_output + 0.8 * (raw_err - last_output);  //这里滤波系数可以调，越靠近0越平滑
    last_output = output;

    //这里反转一下方向，为了和PID输出对应
    return -output;
}

/**
 * @brief 根据处理后的数据判断当前巡线的状态
 */
static void Trace_State_Judge(uint8_t raw_data,float process_digital){
    static uint16_t count = 0;
    static uint8_t outline_flag = 0;
    if (raw_data == 0xFF) {//没有识别到线累加
        count++;
    }else {          //一旦识别到线就清零
        count = 0;
        outline_flag = 0;
    }
    if (count > 40) {  //目前是5ms运行一次，这里代表超过200ms都没有找到线
        outline_flag = 1 ;
    }
    //识别
    if (fabsf(process_digital) < 5.0f ) {
        if (outline_flag) {  //线从中间消失且识别不到线，认为到达终点
            trace_state = TRACE_END;
        }else {              //在线上一般输出小于6，且Digital!= 0xFF，则表明识别到线
            trace_state = TRACE_INLINE;
        }
    }else {  //0.2s八路灰度都识别不到，且输出残留上次7和-7这种较大值
        trace_state = TRACE_LOST;
    }
}

/**
 * @brief 在task里面以1kHz运行，保证数据采样的连续
 */
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
    //第一版数据处理
    track_err = CalcLineError(Digtal);
    // //第二版数据处理
    // raw_track_err = filter_raw(Digtal);   //先对原始数据处理优化，保留残留值这个逻辑
    // process_digital = raw_transform_easy(raw_track_err);//把八位数据映射成-7到7的数字
    //进行状态判断，这里传入原始数据方便检查0xFF，因为fliter_raw会忽略掉0xFF
    Trace_State_Judge(Digtal,track_err);

    // LOGINFO("Digtal: %d %d %d %d %d %d %d %d  err:%.2f",
    //         (Digtal >> 7) & 1, (Digtal >> 6) & 1, (Digtal >> 5) & 1, (Digtal >> 4) & 1,
    //         (Digtal >> 3) & 1, (Digtal >> 2) & 1, (Digtal >> 1) & 1, (Digtal >> 0) & 1,
    //         track_err);
    float temp = track_err;
    if (trace_state == TRACE_END) {
        temp =  0.0f;
    }
    PID_calc(&Trace_PID, 0, temp ); //todo:这里从原来的权重换成了-7到7的数据，边缘速度减小了，得细调
    return Trace_PID.out;
}
