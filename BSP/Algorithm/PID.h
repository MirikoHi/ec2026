#ifndef PID_H
#define PID_H


#include "ti_msp_dl_config.h"


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef enum
{
    FF_None = 0b00000000,               // 无前馈
    FF_Proportional = 0b00000001,       // 比例前馈
    FF_Velocity = 0b00000010,           // 速度前馈
} FeedForwardType;

typedef struct
{
    uint8_t mode;
    //PID 参数
    float Kp;
    float Ki;
    float Kd;
    float kf_v;     //速度前馈系数
    float kf_p;     //比例前馈系数
    float ff_lpf_gain;   //速度前馈低通滤波系数 0~1,
    float d_lpf_gain;    //Dout低通滤波系数 0~1,

    float max_out;  //输出限幅
    float max_iout; //积分限幅
//		float feedforward;
		float deadzone;
    FeedForwardType ff_type;

    float Ref;
    float Measure;

    float out;
    float Pout;
    float Iout;
    float ITerm;
    float Dout;
    float Dbuf[3];  //微分项 0本次 1上一次 2上上次
    float error[3]; //误差项 0本次 1上一次 2上上次
    float kf_v_out;
    float kf_p_out;
    float Last_Ref;
    float dRef_filt;     //速度前馈滤波后值

    uint32_t DWT_CNT;  //用于计算两次PID调用的间隔时间
    float dt;

    uint8_t pid_update_flag;

} pid_type_def;
typedef struct
{
    uint8_t mode;
    //PID 参数
    float Kp;
    float Ki;
    float Kd;
    float kf_v;     //速度前馈系数
    float kf_p;     //比例前馈系数
    float ff_lpf_gain;   //速度前馈低通滤波系数 0~1,
    float d_lpf_gain;    //Dout低通滤波系数 0~1,

    float max_out;  //输出限幅
    float max_iout; //积分限幅
//		float feedforward;
		float deadzone;
    FeedForwardType ff_type;
} pid_init_config_s;


void PID_init(pid_type_def *pid,pid_init_config_s *config);
float PID_calc(pid_type_def *pid, float ref, float Measure);
void PID_clear(pid_type_def *pid);

/**
 * @brief 一阶低通滤波器
 *
 * @param state 输出
 * @param input 输入
 * @param gain 滤波系数
 */
static inline float LPF_Update(float *state, float input, float gain)
{
    *state += gain * (input - *state);
    return *state;
}

#endif