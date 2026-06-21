#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "ti_msp_dl_config.h"

#define MAX_ENCODER_NUM  2

typedef enum {
    FORWARD=0,  
    REVERSAL  
}ENCODER_DIR;

typedef struct {
	GPIO_Regs* A_PORT;
	uint32_t A_pin;
	GPIO_Regs* B_PORT;
	uint32_t B_pin;
} encoder_PortPin_s;
typedef struct {
    volatile int32_t temp_count; 
//    int count;     
    volatile int32_t count;         						
	  volatile int32_t total_count;         						

    ENCODER_DIR dir;
		encoder_PortPin_s PortPin;
} ENCODER_RES;


ENCODER_RES* encoder_init(encoder_PortPin_s* init);
int get_encoder_count(void);
ENCODER_DIR get_encoder_dir(void);
void encoder_update(void);
void encoder_interrupt_begin(void);
#endif