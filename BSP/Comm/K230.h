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

/** 原子式读取最近一帧及帧序号；尚未收到有效帧时返回 0。 */
uint8_t K230_GetSnapshot(K230_Data_t *data, uint32_t *frame_id);

/** 摄像头串口在看门狗时间内收到过有效数据时返回 1。 */
uint8_t K230_IsOnline(void);

extern DaemonInstance* K230_daemon;
extern DaemonInstance* K230_Lost_Target_daemon;
#endif
