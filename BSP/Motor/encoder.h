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


ENCODER_RES* Encoder_Init(encoder_PortPin_s* init);
int Encoder_GetCount(void);
ENCODER_DIR Encoder_GetDir(void);
void Encoder_Update(void);
void Encoder_InterruptBegin(void);
#endif