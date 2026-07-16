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
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * This project provides a simple blinky style project.
 * The application tasks are created in app_tasks.c (app_tasks_init).
 *
 * The blinky demo uses FreeRTOS's tickless idle mode to reduce power
 * consumption.
 *
 * This file implements the code that is not demo specific.
 */

/* Standard includes. */
#include <stdio.h>
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "Robot.h"
#include "board.h"
#include "dcmotor.h"
#include "encoder.h"
/* TI includes */
#include "bsp_log.h"
#include "ti_msp_dl_config.h"
//ababababaabababab
/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware(void);

static volatile U32 ulSysViewTickBaseCycles;
static U32 ulSysViewLastTimestamp;

void vApplicationTickHook(void) {
    ulSysViewTickBaseCycles += (SysTick->LOAD + 1U);
}

/*********************************************************************
*       SEGGER_SYSVIEW_X_GetTimestamp
*
*  Function description
*    Returns a monotonic timestamp in CPU cycles.
*/
U32 SEGGER_SYSVIEW_X_GetTimestamp(void) {
    U32 base_cycles;
    U32 base_cycles_check;
    U32 systick_val;
    U32 cycles_per_tick;
    U32 timestamp_cycles;

    do {
        base_cycles = ulSysViewTickBaseCycles;
        systick_val = SysTick->VAL;
        base_cycles_check = ulSysViewTickBaseCycles;
    } while (base_cycles != base_cycles_check);

    cycles_per_tick = SysTick->LOAD + 1U;
    timestamp_cycles = base_cycles + (cycles_per_tick - systick_val);

    if (timestamp_cycles < ulSysViewLastTimestamp) {
        timestamp_cycles = ulSysViewLastTimestamp;
    } else {
        ulSysViewLastTimestamp = timestamp_cycles;
    }

    return timestamp_cycles;
}

/*********************************************************************
*       SEGGER_SYSVIEW_X_GetInterruptId
*
*  Function description
*    Returns the currently active exception number on Cortex-M0+.
*/
U32 SEGGER_SYSVIEW_X_GetInterruptId(void) {
    return __get_IPSR();
}

/*-----------------------------------------------------------*/

int main(void)
{
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();
    SEGGER_RTT_Init();
    LOGINFO("Hardware init");
    SEGGER_SYSVIEW_Conf();     // 使用daplink时请注释掉该行，否则会导致程序无法运行
    LOGINFO("systemview start");
		Robot_Init();
    LOGERROR("unknow error");
    return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
}
/*-----------------------------------------------------------*/

#if (configCHECK_FOR_STACK_OVERFLOW)
/*
     *  ======== vApplicationStackOverflowHook ========
     *  When stack overflow checking is enabled the application must provide a
     *  stack overflow hook function. This default hook function is declared as
     *  weak, and will be used by default, unless the application specifically
     *  provides its own hook function.
     */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask;
    SEGGER_RTT_WriteString(0, "\r\n" RTT_CTRL_TEXT_BRIGHT_WHITE RTT_CTRL_BG_RED
                              "[FreeRTOS] Stack overflow detected in task: ");
    SEGGER_RTT_WriteString(0, pcTaskName != NULL ? pcTaskName : "(unnamed)");
    SEGGER_RTT_WriteString(0, "\r\n" RTT_CTRL_RESET);

    __disable_irq();
    while (1) {
    }
}
#endif

/*-----------------------------------------------------------*/
