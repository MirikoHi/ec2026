#ifndef _PAW3395_H_
#define _PAW3395_H_


#include "ti_msp_dl_config.h"


#define SPI_CS(x)  ( (x) ? DL_GPIO_setPins(SPI_PORT,SPI_CS_PIN) : DL_GPIO_clearPins(SPI_PORT,SPI_CS_PIN) )
#define INCH_TO_METER 0.0254
void PAW3395_Init(void);
void PAW3395_Read_Motion(float* axis);

#endif
