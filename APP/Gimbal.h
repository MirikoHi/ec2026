#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "ti_msp_dl_config.h"

#define GIMBAL_LENGTH_TO_CENTER 0.66    /**< 摄像头到云台中心的距离 (m) */

#define MOTOR_DEFAULT_VEL   200    /**< 默认速度 (RPM) */
#define MOTOR_DEFAULT_ACC   0       /**< 默认加速度 (0=直接启动) */

void Gimbal(void);
void Gimbal_Init(void);

#endif /* _GIMBAL_H_ */
