#include "fifo.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

//__IO FIFO_t rxFIFO = {0};

/**
	* @brief   初始化队列
	* @param   无
	* @retval  无
	*/
void fifo_initQueue(__IO FIFO_t *fifo)
{
	fifo->ptrRead  = 0;
	fifo->ptrWrite = 0;

}

/**
	* @brief   入队
	* @param   无
	* @retval  无
	*/
void fifo_enQueue(__IO FIFO_t *fifo,uint8_t data)
{
	fifo->buffer[fifo->ptrWrite] = data;
	
	++fifo->ptrWrite;
	
	if(fifo->ptrWrite >= FIFO_SIZE)
	{
		fifo->ptrWrite = 0;
	}
}

/**
	* @brief   出队
	* @param   无
	* @retval  无
	*/
uint8_t fifo_deQueue(__IO FIFO_t *fifo)
{
	uint8_t element = 0;

	element = fifo->buffer[fifo->ptrRead];

	++fifo->ptrRead;

	if(fifo->ptrRead >= FIFO_SIZE)
	{
		fifo->ptrRead = 0;
	}

	return element;
}

/**
	* @brief   判断空队列
	* @param   无
	* @retval  无
	*/
bool fifo_isEmpty(__IO FIFO_t *fifo)
{
	if(fifo->ptrRead == fifo->ptrWrite)
	{
		return true;
	}

	return false;
}

/**
	* @brief   计算队列长度
	* @param   无
	* @retval  无
	*/
uint8_t fifo_queueLength(__IO FIFO_t *fifo)
{
	if(fifo->ptrRead <= fifo->ptrWrite)
	{
		return (fifo->ptrWrite - fifo->ptrRead);
	}
	else
	{
		return (FIFO_SIZE - fifo->ptrRead + fifo->ptrWrite);
	}
}
