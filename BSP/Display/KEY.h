#ifndef _KEY_H_
#define _KEY_H_

#include "ti_msp_dl_config.h"


#define KEY_HOLD    0x01
#define KEY_DOWN    0x02
#define KEY_UP    0x04
#define KEY_SINGLE    0x08
#define KEY_DOUBLE    0x10
#define KEY_LONG    0x20
#define KEY_REPEAT    0x40


uint8_t Key_GetState(uint8_t n);
uint8_t Key_Check(uint8_t n,uint8_t Flag);
void Key_Tick(void);
void Key_ClearAllFlags(void);

#endif