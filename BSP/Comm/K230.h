#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include "ti_msp_dl_config.h"
#include "daemon.h"

typedef struct {
    uint16_t x_position;
    float    dt;
} steel_ball_movement_typedef;

extern volatile uint8_t k230_data_valid;

void K230_Init(void);
uint8_t K230_Read(steel_ball_movement_typedef *steel_ball_movement);

#endif