#ifndef _JY901S_H_
#define _JY901S_H_

#include "ti_msp_dl_config.h"

/**
 * @brief IMU 姿态角度数据结构体
 */
typedef struct {
	float Roll;             // 翻滚角 (deg)
	float Pitch;            // 俯仰角 (deg)
	float Yaw;              // 偏航角 (-180 ~ +180 deg)
	int16_t Yaw_Round_Count;// 偏航角旋转圈数
	float Yaw_Total_Angle;  // 连续累计偏航角 (deg)
} JY901s_IMU_Data_s;

// 导出全局姿态数据变量
extern volatile JY901s_IMU_Data_s JY901S_IMU_Data;

// 函数声明
void JY901s_Init(void);
void JY901s_ReceiveData(uint8_t RxData);

#endif /* _JY901S_H_ */