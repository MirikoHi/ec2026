#include "Robot.h"
#include "Robot_Cmd.h"
#include "Chassis.h"
#include "ADC_Voltage.h"
#include "trace.h"
#include "dwt.h"
#include "menu.h"
#include "JY901S.h"
#include "tjc.h"
#include "K230.h"
extern void main_blinky(void);
void Robot_Init(void)
{
	__disable_irq();
	DWT_Init(80);
	MenuInit();
	Chassis_Init();
//	Trace_Init();
	RobotCmd_Init();
	main_blinky();
	K230_Init();
	vTaskStartScheduler();
}