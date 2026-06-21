#ifndef _ZDT_MOTOR_H_
#define _ZDT_MOTOR_H_
#define ZDT_MOTOR_MAX_NUM 2
#include "ti_msp_dl_config.h"
#include "misc.h"
typedef struct {
	GPIO_Regs* Stp_PORT;
	uint32_t Stp_pin;
	GPIO_Regs* Dir_PORT;
	uint32_t Dir_pin;
} ZDT_Motor_PortPin_s;

typedef enum {
	ZDT_MOTOR_ENABLE = 0,
	ZDT_MOTOR_DISABLE,
}ZDT_Motor_State_s;
typedef struct {        						
		ZDT_Motor_PortPin_s PortPin;
		float delay_ms;
		float last_pulse_ms;
		volatile uint32_t clk; 
		float now_angle;
		volatile uint8_t dir;
} ZDT_Motor_t;

typedef struct {
	ZDT_Motor_PortPin_s PortPin;
	float delay_ms;
}ZDT_Motor_Config_s;


ZDT_Motor_t *ZDT_Motor_Init(ZDT_Motor_Config_s* config);
void ZDT_Set_Position(ZDT_Motor_t *motor,float angle);
void ZDT_Motor(void);
void ZDT_TICK_Init(void);
#endif
