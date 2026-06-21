#ifndef __FIFO_H
#define __FIFO_H

#include "ti_msp_dl_config.h"
#include "stdbool.h"
/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

#define 	FIFO_SIZE   128
typedef struct {
	uint8_t buffer[FIFO_SIZE];
	__IO uint8_t ptrWrite;
	__IO uint8_t ptrRead;
}FIFO_t;

//extern __IO FIFO_t rxFIFO;

void fifo_initQueue(__IO FIFO_t *fifo);
void fifo_enQueue(__IO FIFO_t *fifo,uint8_t data);
uint8_t fifo_deQueue(__IO FIFO_t *fifo);
bool fifo_isEmpty(__IO FIFO_t *fifo);
uint8_t fifo_queueLength(__IO FIFO_t *fifo);

#endif
