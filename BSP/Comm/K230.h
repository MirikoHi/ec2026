#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include "ti_msp_dl_config.h"
#include "daemon.h"

typedef struct __attribute__((packed)) {
    int16_t x;
    int16_t y;
} K230_Data_t;

void K230_Init(void);
void K230_ReceiveData(uint8_t RxData);

extern K230_Data_t K230_data;

extern DaemonInstance* K230_daemon;
extern DaemonInstance* K230_Lost_Target_daemon;
#endif