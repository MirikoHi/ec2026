#ifndef _JY901S_H_
#define _JY901S_H_

#include "ti_msp_dl_config.h"

void JY901s_ReceiveData(uint8_t RxData);

void JY901s_Init(void);

typedef struct {
    float Roll;
		float Pitch;
		float Yaw;
		int16_t Yaw_Round_Count;
		float Yaw_Total_Angle;
}JY901s_IMU_Data_s;

volatile JY901s_IMU_Data_s *JY901s_IMU_GetData(void);

extern volatile JY901s_IMU_Data_s IMU_Data;
#endif

