#include "K230.h"
#include "daemon.h"
//#include <ti/devices/msp/msp.h>   // ????MSP?豸????
//#include <ti/driverlib/dl_uart.h> // ????UART??????????
#define DATA_PACKET_LENGTH 7

static uint8_t RxBuffer[DATA_PACKET_LENGTH]; // ????????????
static volatile uint8_t RxState = 0; // ?????????λ
static uint8_t RxIndex = 0;         // ????????????
int16_t K230_err[2]={0};
static void K230LostCallback(void *ptr);
DaemonInstance* K230_daemon,*K230_Lost_Target_daemon;
void Clear_UART_FIFO(void) {
    while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
        (void)DL_UART_receiveData(K230_INST); // ????????
    }
}
void K230_Init(void)
{
    Clear_UART_FIFO(); // ???????????????FIFO
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
    DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX); // ????ж???λ
}

/**
 * @brief       ?????????????
 * @param       ????????????RxData
 * @retval      ??
 */
void K230_ReceiveData(uint8_t RxData)
{
    static uint8_t sum = 0;
    // ????????????
    if(RxData == 0x15 && RxState != 0)
    {
        RxState = 0;  // ??????????????????
        RxIndex = 0;
    }
    if (RxState == 0) // ??????
    {
        if (RxData == 0x15)
        {
            RxBuffer[0] = RxData;
            RxState = 1;
            RxIndex = 1;
            sum = RxData; // ?????У???
        }
    }
    else if (RxState == 1) // ?ж?????????
    {
        if (RxData == 0x11) 
        {
            RxBuffer[RxIndex++] = RxData;
            RxState = 2;
            sum += RxData; // ???????????
        }
        else
        {
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2) // ????????
    {
        if(RxIndex < DATA_PACKET_LENGTH)
        {
            RxBuffer[RxIndex] = RxData;
            
            // ????????????У??????????
            if (RxIndex < (DATA_PACKET_LENGTH - 1)) {
                sum += RxData;
            }
            
            RxIndex++;
            
            if (RxIndex == DATA_PACKET_LENGTH) // ???????
            {
                // ??????????У??????????У?????????
                if (sum == RxBuffer[DATA_PACKET_LENGTH - 1])
                {
                    // ???????? (С????)
                    K230_err[0] = (RxBuffer[3] << 8) | RxBuffer[2];
                    K230_err[1] = (RxBuffer[5] << 8) | RxBuffer[4];
//                    int16_t data2 = (RxBuffer[7] << 8) | RxBuffer[6];
//                    int16_t data3 = (RxBuffer[9] << 8) | RxBuffer[8];
                    DaemonReload(K230_daemon);
										DaemonReload(K230_Lost_Target_daemon);
                    // ????????
                    // ????: ????????
                }
                // ????????
                RxState = 0;
                RxIndex = 0;
            }
        }
        else
        {
            RxState = 0;
            RxIndex = 0;
        }
    }
}
static void K230LostCallback(void *ptr)
{
	K230_err[0] = 0;
	K230_err[1] = 0;
}
void K230_INST_IRQHandler(void)
{
    uint8_t RxData = DL_UART_receiveData(K230_INST);
    K230_ReceiveData(RxData);
}