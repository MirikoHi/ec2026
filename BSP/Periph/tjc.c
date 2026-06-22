#include "tjc.h"

FIFO_t TJC_FIFO;
void tjc_init(void)
{
	fifo_initQueue(&TJC_FIFO);
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	DL_UART_clearInterruptStatus(UART_0_INST,DL_UART_INTERRUPT_RX);//清除中断标志位
	
}
void UART_0_INST_IRQHandler(void)
{
	uint8_t RxData = DL_UART_receiveData(UART_0_INST);
	fifo_enQueue(&TJC_FIFO,RxData);
}