#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include "ti_msp_dl_config.h"
#include "daemon.h"


void K230_Init(void);
void K230_ReceiveData(uint8_t RxData);

extern int16_t K230_err[2];

extern DaemonInstance* K230_daemon;
extern DaemonInstance* K230_Lost_Target_daemon;
#endif