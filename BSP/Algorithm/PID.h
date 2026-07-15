#ifndef PID_H
#define PID_H


#include "ti_msp_dl_config.h"


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
//		float feedforward;
		float deadzone;
	
    float Ref;
    float Measure;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
//		float feedforward;
		float deadzone;
} pid_init_config_s;


void PID_init(pid_type_def *pid,pid_init_config_s *config);
float PID_calc(pid_type_def *pid, float ref, float Measure);
void PID_clear(pid_type_def *pid);

#endif
