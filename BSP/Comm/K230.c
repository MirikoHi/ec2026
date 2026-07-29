#include "K230.h"
#include "daemon.h"
//#include <ti/devices/msp/msp.h>   // ����MSP�豸ͷ�ļ�
//#include <ti/driverlib/dl_uart.h> // ����UART������ͷ�ļ�
#define DATA_PACKET_LENGTH 7

static uint8_t RxBuffer[DATA_PACKET_LENGTH]; // ������������
static volatile uint8_t RxState = 0; // ����״̬��־λ
static uint8_t RxIndex = 0;         // ������������
int16_t K230_err[2]={0};
static void K230LostCallback(void *ptr);
DaemonInstance* K230_daemon,*K230_Lost_Target_daemon;
void Clear_UART_FIFO(void) {
    /* 限定最多清 16 次，防止 DL_UART_isRXFIFOEmpty 不可靠导致死循环 */
    for (int _i = 0; _i < 16; _i++) {
        if (DL_UART_isRXFIFOEmpty(K230_INST))
            break;
        (void)DL_UART_receiveData(K230_INST);
    }
}
void K230_Init(void)
{
    Clear_UART_FIFO(); // ��������ʼ��ǰ���FIFO
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
		DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX); // ����жϱ�־λ
		Daemon_Init_Config_s Daemon_Uart_Init_s = 
		{
			.owner_id = K230_INST,
			.reload_count = 11,
			.callback = K230LostCallback,
		};
		K230_daemon = DaemonRegister(&Daemon_Uart_Init_s);
		Daemon_Init_Config_s Daemon_Lost_Target_Init_s = 
		{
			.owner_id = K230_INST,
			.reload_count = 50,
			.callback = NULL,
		};
		K230_Lost_Target_daemon = DaemonRegister(&Daemon_Lost_Target_Init_s);
}

/**
 * @brief       ���ݰ���������
 * @param       ���ڽ��յ�����RxData
 * @retval      ��
 */
void K230_ReceiveData(uint8_t RxData)
{
    static uint8_t sum = 0;
    // ���Ӱ�ͷǿ��ͬ��
    if(RxData == 0x15 && RxState != 0)
    {
        RxState = 0;  // ���ְ�ͷ��������״̬��
        RxIndex = 0;
    }
    if (RxState == 0) // �ȴ���ͷ
    {
        if (RxData == 0x15)
        {
            RxBuffer[0] = RxData;
            RxState = 1;
            RxIndex = 1;
            sum = RxData; // ��ʼ��У���
        }
    }
    else if (RxState == 1) // �ж���������
    {
        if (RxData == 0x11) 
        {
            RxBuffer[RxIndex++] = RxData;
            RxState = 2;
            sum += RxData; // �ۼ���������
        }
        else
        {
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2) // ��������
    {
        if(RxIndex < DATA_PACKET_LENGTH)
        {
            RxBuffer[RxIndex] = RxData;
            
            // �ؼ��޸ģ�ֻ�ۼӵ�У���ǰһ���ֽ�
            if (RxIndex < (DATA_PACKET_LENGTH - 1)) {
                sum += RxData;
            }
            
            RxIndex++;
            
            if (RxIndex == DATA_PACKET_LENGTH) // �������
            {
                // �ؼ��޸ģ��Ƚ�У���ʱ������У����ֽڱ���
                if (sum == RxBuffer[DATA_PACKET_LENGTH - 1])
                {
                    // �������� (С��ģʽ)
                    K230_err[0] = (RxBuffer[3] << 8) | RxBuffer[2];
                    K230_err[1] = (RxBuffer[5] << 8) | RxBuffer[4];
//                    int16_t data2 = (RxBuffer[7] << 8) | RxBuffer[6];
//                    int16_t data3 = (RxBuffer[9] << 8) | RxBuffer[8];
                    DaemonReload(K230_daemon);
										DaemonReload(K230_Lost_Target_daemon);
                    // ��������
                    // ����: ���Ƶ����
                }
                // ����״̬��
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
    DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX);
}