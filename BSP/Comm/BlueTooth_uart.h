//
// Created by xiaofangxing on 2026/7/27.
//
#ifndef ELECTRIC_COMPETITION_ROBOT_BLUETOOTH_UART_H
#define ELECTRIC_COMPETITION_ROBOT_BLUETOOTH_UART_H

#include <stdint.h>

//判断保证不能同时define两个板号宏,也不能都不定义
#if !defined(BLUETOOTH_BOARD_1) && !defined(BLUETOOTH_BOARD_2)
#error "Must define BLUETOOTH_BOARD_1 or BLUETOOTH_BOARD_2 (via add_compile_definitions in CMakeLists.txt)"
#elif defined(BLUETOOTH_BOARD_1) && defined(BLUETOOTH_BOARD_2)
#error "Cannot define both BLUETOOTH_BOARD_1 and BLUETOOTH_BOARD_2"
#endif

//组帧常量
#define BLUETOOTH_UART_HEADER         's'
#define BLUETOOTH_UART_TAIL           'e'
#define BLUETOOTH_UART_MAX_BUFFSIZE   64U
#define BLUETOOTH_UART_OFFSET_BYTES   4U   // 's' + datalen + crc8 + 'e'

#pragma pack(1)
typedef struct
{
    // TODO: 板1自行填充实际字段
    float example_field_1;
    float example_field_2;
    uint8_t example_status;
} BlueTooth_Board1_Upload_s;

typedef struct
{
    // TODO: 板2自行填充实际字段
    float example_field_1;
    float example_field_2;
    uint8_t example_status;
} BlueTooth_Board2_Upload_s;
#pragma pack()

#if defined(BLUETOOTH_BOARD_1)
typedef BlueTooth_Board1_Upload_s BlueTooth_Tx_t;
typedef BlueTooth_Board2_Upload_s BlueTooth_Rx_t;
#else
typedef BlueTooth_Board2_Upload_s BlueTooth_Tx_t;
typedef BlueTooth_Board1_Upload_s BlueTooth_Rx_t;
#endif

void BlueToothUart_Init(void);
void BlueToothUart_Send(const BlueTooth_Tx_t *data);
volatile BlueTooth_Rx_t *BlueToothUart_Get(void);
uint8_t BlueToothUart_IsOnline(void);
uint8_t BlueToothUart_IsFrameUpdated(void);
void BlueToothUart_ClearFrameUpdated(void);
uint32_t BlueToothUart_GetErrorCount(void);

#endif //ELECTRIC_COMPETITION_ROBOT_BLUETOOTH_UART_H
