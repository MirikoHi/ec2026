/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/******************************************************************************
 * NOTE 1:  This project provides a simple blinky style project.
 * This file implements the simply blinky style demo.
 *
 * The blinky demo uses FreeRTOS's tickless idle mode to reduce power
 * consumption.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * basic demo. Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware, are defined in main.c.
 ******************************************************************************
 *
 * app_tasks_init() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 1 second, before sending the value 100 to the queue that
 * was created within app_tasks_init().  Once the value is sent, the task loops
 * back around to block for another 1 second.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * app_tasks_init().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, toggles the LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the
 * task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 1 second, the queue receive
 * task leaves the Blocked state every 1 second, and therefore toggles
 * the LED every 1 second.
 */

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "OLED.h"
#include "KEY.h"
#include "trace.h"
#include "DCmotor.h"
#include "Robot_Cmd.h"
#include "Chassis.h"
#include "encoder_timer.h"
#include "dwt.h"
#include "daemon.h"
#include "menu.h"
#include "NRF24L01.h"
#include "Gimbal.h"
#include "ZDT_Motor.h"
/* TI includes. */
#include "ti_msp_dl_config.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainQUEUE_SEND_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

/*
 * The rate at which data is sent to the queue.  The 1s (1000ms) value is
 * converted to ticks using the pdMS_TO_TICKS constant.
 */
#define mainQUEUE_SEND_FREQUENCY_MS (pdMS_TO_TICKS(1000UL))

/*
 * The number of items the queue can hold.  This is 1 as the receive task
 * will remove items as they are added, meaning the send task should always
 * find the queue empty.
 */
#define mainQUEUE_LENGTH (1)

/*
 * Values passed to the two tasks just to check the task parameter
 * functionality.
 */
#define mainQUEUE_SEND_PARAMETER (0x1111UL)
#define mainQUEUE_RECEIVE_PARAMETER (0x22UL)
#define Key_PARAMETER (0x114514UL)
#define Trace_PARAMETER (0x8UL)
#define HwMotor_PARAMETER (0x9UL)
#define RobotCmd_PARAMETER (0x10UL)
#define Chassis_PARAMETER (0x11UL)
#define	Daemon_PARAMETER (0x11UL)
#define NRF24L01_PARAMETER (0x12UL)
#define Gimbal_PARAMETER (0x13UL)
#define StepMotor_PARAMETER (0x14UL)
/*-----------------------------------------------------------*/

/* The tasks as described in the comments at the top of this file. */
static void prvQueueReceiveTask(void *pvParameters);
static void prvQueueSendTask(void *pvParameters);
static void KeyTask(void *pvParameters);
static void TraceTask(void *pvParameters);
static void HwMotorTask(void *pvParameters);
static void RobotCmdTask(void *pvParameters);
static void ChassisTask(void *pvParameters);
static void DaemonTask(void *pvParameters);
static void NRF24L01Task(void *pvParameters);
static void GimbalTask(void *pvParameters);
static void StepMotorTask(void *pvParameters);
/* Called by Robot_Init() to create all application tasks.
 * Defined in APP/app_tasks.h */
/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

void app_tasks_init(void)
{
    /* Create the queue. */
		BaseType_t xResult;
    xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
		size_t free_heap = xPortGetFreeHeapSize();
    if (xQueue != NULL) {
        /*
         * Start the two tasks as described in the comments at the top of this
         * file.
         */
//        xTaskCreate(
//            prvQueueReceiveTask, /* The function that implements the task. */
//            "Rx", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
//            configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
//            (void *)
//                mainQUEUE_RECEIVE_PARAMETER, /* The parameter passed to the task - just to check the functionality. */
//            mainQUEUE_RECEIVE_TASK_PRIORITY, /* The priority assigned to the task. */
//            NULL); /* The task handle is not required, so NULL is passed. */

//        xTaskCreate(prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE,
//            (void *) mainQUEUE_SEND_PARAMETER, mainQUEUE_SEND_TASK_PRIORITY,
//            NULL);
				xResult=xTaskCreate(KeyTask, "Key", configMINIMAL_STACK_SIZE,
            (void *) Key_PARAMETER, tskIDLE_PRIORITY+1,
            NULL);
				free_heap = xPortGetFreeHeapSize();
				xResult=xTaskCreate(HwMotorTask, "HwMotor", 256,
            (void *) HwMotor_PARAMETER, tskIDLE_PRIORITY+2,
            NULL);
				free_heap = xPortGetFreeHeapSize();
//						xResult=xTaskCreate(StepMotorTask, "StepMotor", 128,
//            (void *) StepMotor_PARAMETER, tskIDLE_PRIORITY+2,
//            NULL);
//				free_heap = xPortGetFreeHeapSize();
//				xResult=xTaskCreate(TraceTask, "Trace", configMINIMAL_STACK_SIZE,
//            (void *) Trace_PARAMETER, tskIDLE_PRIORITY+2,
//            NULL);
//				free_heap = xPortGetFreeHeapSize();
				xResult=xTaskCreate(RobotCmdTask, "RobotCmd", configMINIMAL_STACK_SIZE,
            (void *) RobotCmd_PARAMETER, tskIDLE_PRIORITY+2,
            NULL);
				free_heap = xPortGetFreeHeapSize();
				xResult=xTaskCreate(ChassisTask, "Chassis", 256,
            (void *) Chassis_PARAMETER, tskIDLE_PRIORITY+2,
            NULL);
				free_heap = xPortGetFreeHeapSize();
				xResult=xTaskCreate(GimbalTask, "Gimbal", 128,
            (void *) Gimbal_PARAMETER, tskIDLE_PRIORITY+2,
            NULL);
				free_heap = xPortGetFreeHeapSize();
				xResult=xTaskCreate(DaemonTask, "Daemon", configMINIMAL_STACK_SIZE,
            (void *) Daemon_PARAMETER, tskIDLE_PRIORITY,
            NULL);
				free_heap = xPortGetFreeHeapSize();
//				xResult=xTaskCreate(NRF24L01Task, "NRF24L01", configMINIMAL_STACK_SIZE+3,
//            (void *) NRF24L01_PARAMETER, tskIDLE_PRIORITY,
//            NULL);
//				free_heap = xPortGetFreeHeapSize();
        /* Start the tasks. */
        
    }

//    /*
//     * If all is well, the scheduler will now be running, and the following
//     * line will never be reached.  If the following line does execute, then
//     * there was insufficient FreeRTOS heap memory available for the idle
//     * and/or timer tasks to be created.  See the memory management section on
//     * the FreeRTOS web site for more details.
//     */
//    for (;;)
//        ;
}
/*-----------------------------------------------------------*/

static void prvQueueSendTask(void *pvParameters)
{
    TickType_t xNextWakeTime;
    const unsigned long ulValueToSend = 100UL;

    /* Check the task parameter is as expected. */
    configASSERT(((unsigned long) pvParameters) == mainQUEUE_SEND_PARAMETER);

    /* Initialize xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for (;;) {
        /*
         * Place this task in the blocked state until it is time to run again.
         * The block time is specified in ticks, the constant used converts
         * ticks to ms.  While in the Blocked state this task will not consume
         * any CPU time.
         */
        vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

        /*
         * Send to the queue - causing the queue receive task to unblock and
         * toggle the LED.  0 is used as the block time so the sending operation
         * will not block - it shouldn't need to block as the queue should always
         * be empty at this point in the code.
         */
        xQueueSend(xQueue, &ulValueToSend, 0U);
    }
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask(void *pvParameters)
{
    unsigned long ulReceivedValue;
    static const TickType_t xShortBlock = pdMS_TO_TICKS(50);

    /* Check the task parameter is as expected. */
    configASSERT(
        ((unsigned long) pvParameters) == mainQUEUE_RECEIVE_PARAMETER);

    for (;;) {
        /*
         * Wait until something arrives in the queue - this task will block
         * indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
         * FreeRTOSConfig.h.
         */
        xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);

        /*
         * To get here something must have been received from the queue, but
         * is it the expected value?  If it is, toggle the LED.
         */
        if (ulReceivedValue == 100UL) {
            /*
             * Blip the LED for a short while so as not to use too much
             * power.
             */
//            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
//            vTaskDelay(xShortBlock);
//            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            ulReceivedValue = 0U;
						
        }
    }
}

static void KeyTask(void *pvParameters)
{
  /* Check the task parameter is as expected. */
    configASSERT(
        ((unsigned long) pvParameters) == Key_PARAMETER);
	vTaskDelay(1000);
		static float Key_dt;
    static float Key_start;
		static uint8_t task_count;
		for (;;){
			
			Key_start = DWT_GetTimeline_ms();
			
			Key_Tick();
			Key_dt = DWT_GetTimeline_ms() - Key_start;
			if (Key_dt > 1)
            LOGERROR("[freeRTOS] Key Task is being DELAY! dt = [%f]", &Key_dt);
			task_count++;
			if(task_count>=100)
			{
				task_count=0;
				menu_task();
				//uint8_t cost=sizeof(MenuInstance);
			}
			vTaskDelay(pdMS_TO_TICKS(1));
			
		}
}
static void TraceTask(void *pvParameters)
{
  /* Check the task parameter is as expected. */
    configASSERT(
        ((unsigned long) pvParameters) == Trace_PARAMETER);
	vTaskDelay(1000);
		static float Trace_dt;
    static float Trace_start;
		for (;;){
			Trace_start = DWT_GetTimeline_ms();
			Trace_task();
			Trace_dt = DWT_GetTimeline_ms() - Trace_start;
			if (Trace_dt > 5)
            LOGERROR("[freeRTOS] Trace Task is being DELAY! dt = [%f]", &Trace_dt);
			vTaskDelay(pdMS_TO_TICKS(5));
			
		}
}
static void HwMotorTask(void *pvParameters)
{
  /* Check the task parameter is as expected. */
    configASSERT(
        ((unsigned long) pvParameters) == HwMotor_PARAMETER);
	vTaskDelay(1000);
		Encoder_InterruptBegin();
		EncoderTimer_Init();
		static float DCMotor_dt;
    static float DCMotor_start;
		for (;;){
			DCMotor_start = DWT_GetTimeline_ms();
			Hw_Motor_Task();
			DCMotor_dt = DWT_GetTimeline_ms() - DCMotor_start;
			if (DCMotor_dt > Control_Period)
            LOGERROR("[freeRTOS] DCMotor Task is being DELAY! dt = [%f]", &DCMotor_dt);
			vTaskDelay(pdMS_TO_TICKS(Control_Period));//Control_Period
			
		}
}
static void StepMotorTask(void *pvParameters)
{
  /* Check the task parameter is as expected. */
    configASSERT(
        ((unsigned long) pvParameters) == StepMotor_PARAMETER);
	vTaskDelay(1000);
		static float StepMotor_dt;
    static float StepMotor_start;
		for (;;){
			StepMotor_start = DWT_GetTimeline_ms();
			ZDT_Motor();
			StepMotor_dt = DWT_GetTimeline_ms() - StepMotor_start;
			if (StepMotor_dt > Control_Period)
            LOGERROR("[freeRTOS] StepMotor Task is being DELAY! dt = [%f]", &StepMotor_dt);

			vTaskDelay(pdMS_TO_TICKS(1));
			
		}
}
static void RobotCmdTask(void *pvParameters)
{
	configASSERT(
        ((unsigned long) pvParameters) == RobotCmd_PARAMETER);
	vTaskDelay(1000);
	static float RobotCmd_dt;
  static float RobotCmd_start;
	for (;;){
			RobotCmd_start = DWT_GetTimeline_ms();
			Robot_Cmd();
			RobotCmd_dt = DWT_GetTimeline_ms() - RobotCmd_start;
			if (RobotCmd_dt > 5)
          LOGERROR("[freeRTOS] RobotCmd Task is being DELAY! dt = [%f]", &RobotCmd_dt);
			vTaskDelay(pdMS_TO_TICKS(5));
			
		}
}
static void ChassisTask(void *pvParameters)
{
	configASSERT(
        ((unsigned long) pvParameters) == Chassis_PARAMETER);
	Trace_Init();
	vTaskDelay(1000);
	static float Chassis_dt;
  static float Chassis_start;
	for (;;){
			Chassis_start = DWT_GetTimeline_ms();
			Chassis();
			Chassis_dt = DWT_GetTimeline_ms() - Chassis_start;
			if (Chassis_dt > 5)
          LOGERROR("[freeRTOS] Chassis Task is being DELAY! dt = [%f]", &Chassis_dt);
			vTaskDelay(pdMS_TO_TICKS(5));
			
		}
}

static void GimbalTask(void *pvParameters)
{
	configASSERT(
        ((unsigned long) pvParameters) == Gimbal_PARAMETER);
	Gimbal_Init();
	vTaskDelay(2000);
	static float Gimbal_dt;
  static float Gimbal_start;
	for (;;){
			Gimbal_start = DWT_GetTimeline_ms();
			Gimbal();
			Gimbal_dt = DWT_GetTimeline_ms() - Gimbal_start;
			if (Gimbal_dt > 5)
          LOGERROR("[freeRTOS] Gimbal Task is being DELAY! dt = [%f]", &Gimbal_dt);
			vTaskDelay(pdMS_TO_TICKS(5));
			
		}
}
static void DaemonTask(void *pvParameters)
{
	configASSERT(
        ((unsigned long) pvParameters) == Daemon_PARAMETER);
	vTaskDelay(1000);
	static float Daemon_dt;
  static float Daemon_start;
	for (;;){
			Daemon_start = DWT_GetTimeline_ms();
			Daemon_Task();
			Daemon_dt = DWT_GetTimeline_ms() - Daemon_start;
			if (Daemon_dt > 10)
          LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &Daemon_dt);
			vTaskDelay(pdMS_TO_TICKS(10));
			
		}
}

static void NRF24L01Task(void *pvParameters)
{
	configASSERT(
        ((unsigned long) pvParameters) == NRF24L01_PARAMETER);
	NRF24L01_Init();
	vTaskDelay(1000);
	static float NRF24L01_dt;
  static float NRF24L01_start;
	for (;;){
			NRF24L01_start = DWT_GetTimeline_ms();
			
			NRF24L01_Task();
			
			NRF24L01_dt = DWT_GetTimeline_ms() - NRF24L01_start;
			if (NRF24L01_dt > 1)
          LOGERROR("[freeRTOS] NRF24L01 Task is being DELAY! dt = [%f]", &NRF24L01_dt);
			
			vTaskDelay(pdMS_TO_TICKS(1));
			
		}
}
/*-----------------------------------------------------------*/
