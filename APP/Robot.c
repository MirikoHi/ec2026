#include "Robot.h"
#include "Robot_Cmd.h"
#include "Chassis.h"
#include "../BSP/Voltage/ADC_Voltage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_log.h"
#include "trace.h"
#include "dwt.h"
#include "../BSP/Display/menu.h"
#include "../BSP/IMU/JY901S.h"
// #include "../BSP/Flash/flash_param_store.h"
//#include "tjc.h"
#include "K230.h"
void Robot_Init(void)
{
	size_t free_heap = 0;
	__disable_irq();
	DWT_Init(80);
	Chassis_Init();
	free_heap = xPortGetFreeHeapSize();
	LOGWARNING("heap after Chassis_Init: %u", (uint32_t)free_heap);
	Trace_Init();
	RobotCmd_Init();
	free_heap = xPortGetFreeHeapSize();
	LOGWARNING("heap after RobotCmd_Init: %u", (uint32_t)free_heap);
	app_tasks_init();
	free_heap = xPortGetFreeHeapSize();
	LOGWARNING("heap after app_tasks_init: %u", (uint32_t)free_heap);
	K230_Init();
	free_heap = xPortGetFreeHeapSize();
	LOGWARNING("heap after K230_Init: %u", (uint32_t)free_heap);
	MenuInit();
	free_heap = xPortGetFreeHeapSize();
	LOGWARNING("heap after MenuInit: %u", (uint32_t)free_heap);
	__enable_irq();
	vTaskStartScheduler();
}
