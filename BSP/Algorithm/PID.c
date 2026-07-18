#include "PID.h"

#include "dwt.h"
#include "string.h"
#include "misc.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


void PID_init(pid_type_def *pid,pid_init_config_s *config)
{
    if (pid == NULL || config == NULL)
    {
        return;
    }
    memset(pid, 0, sizeof(pid_type_def));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(pid_init_config_s));
    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
}


float PID_calc(pid_type_def *pid, float ref, float Measure)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Ref = ref;
    pid->Measure = Measure;
    pid->error[0] = ref - Measure;

    // 死区处理：误差在死区内，输出归零，不累加积分
    if (abs_out(pid->error[0]) < pid->deadzone) {
        pid->out = 0;
        return pid->out;
    }

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->ITerm = pid->Ki * pid->error[0] * pid->dt;
        pid->Iout += pid->ITerm;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        {
            float Dout_raw = pid->Kd * pid->Dbuf[0] / pid->dt;
            if (pid->d_lpf_gain > 0.0f) {
                LPF_Update(&pid->Dout, Dout_raw, pid->d_lpf_gain);
            } else {
                pid->Dout = Dout_raw;
            }
        }
        LimitMax(pid->Iout, pid->max_iout);
        //计算前馈
        if (pid->ff_type & FF_Proportional) {pid->kf_p_out = ref * pid->kf_p ;}
        if (pid->ff_type & FF_Velocity) {
            if (pid->pid_update_flag){ //只有更新了前馈值才会进行计算，否则速度前馈会变成0
                if (pid->dt > 1e-6f) {
                        float dRef_raw = (ref - pid->Last_Ref) / pid->dt;
                        LPF_Update(&pid->dRef_filt, dRef_raw, pid->ff_lpf_gain);
                        pid->kf_v_out = pid->kf_v * pid->dRef_filt;
                    }
                    pid->Last_Ref = ref;
                pid->pid_update_flag = 0;
            }
        }
        pid->out = pid->Pout + pid->Iout + pid->Dout + pid->kf_p_out + pid->kf_v_out;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->Measure = pid->Ref = 0.0f;
}