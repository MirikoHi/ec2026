#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#define GIMBAL_LENGTH_TO_CENTER 0.66 //��λ��

#include "ti_msp_dl_config.h"

void Gimbal_Attitude_Solving(void);
void Gimbal(void);
void Gimbal_Task();
void Gimbal_Init(void);

void Slider_Set_Pos_pixel(uint16_t pix_pos);
#endif
