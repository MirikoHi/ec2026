#include "K230.h"
#include "daemon.h"
//#include <ti/devices/msp/msp.h>   // ���MSP�豸ͷ�ļ�
//#include <ti/driverlib/dl_uart.h> // ���UART������ͷ�ļ�
#define DATA_PACKET_LENGTH (2 + sizeof(K230_Data_t) + 1)

static uint8_t RxBuffer[DATA_PACKET_LENGTH]; // ������������
static volatile uint8_t RxState = 0; // ����״̬��־λ
static uint8_t RxIndex = 0;         // ������������
K230_Data_t K230_data = {0};
static volatile uint32_t K230_frame_id = 0;
static void K230LostCallback(void *ptr);
DaemonInstance* K230_daemon,*K230_Lost_Target_daemon;
void Clear_UART_FIFO(void) {
    while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
        (void)DL_UART_receiveData(K230_INST); // ��������
    }
}
void K230_Init(void)
{
    Clear_UART_FIFO(); // ��������ʼ��ǰ���FIFO
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
		DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX); // ����жϱ�־λ
    //丢失则清空数据为0
		Daemon_Init_Config_s Daemon_Uart_Init_s =
		{
			.owner_id = K230_INST,
			.reload_count = 11,
			.callback = K230LostCallback,
		};
		K230_daemon = DaemonRegister(&Daemon_Uart_Init_s);
    //丢失Robot_Cmd.c检测到 !DaemonIsOnline(K230_Lost_Target_daemon)，则gimbal_cmd_send.yaw += 0.6 yaw扫描旋转寻找目标
		Daemon_Init_Config_s Daemon_Lost_Target_Init_s =
		{
			.owner_id = K230_INST,
			.reload_count = 50,
			.callback = NULL,
		};
		K230_Lost_Target_daemon = DaemonRegister(&Daemon_Lost_Target_Init_s);
}

/**
 * @brief       ���ݰ�������
 * @param       ���ڽ��յ�����RxData
 * @retval      ��
 */
void K230_ReceiveData(uint8_t RxData)
{
    static uint8_t sum = 0;
    // ��Ӱ�ͷǿ��ͬ��
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
                    // 直接拷贝到结构体（小端序，无需字节交换）
                    memcpy(&K230_data, &RxBuffer[2], sizeof(K230_Data_t));
                    ++K230_frame_id;
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

/**
 * @brief      发送1字节数据到K230
 * @param      Data  待发送的字节
 * @retval     无
 */
void K230_TransmitData(uint8_t Data)
{
    while (DL_UART_isTXFIFOFull(K230_INST)); // 等待发送FIFO有空位，避免覆盖未发数据
    DL_UART_transmitData(K230_INST, Data);
}

uint8_t K230_GetSnapshot(K230_Data_t *data, uint32_t *frame_id)
{
    uint32_t before;
    uint32_t after;
    if (data == NULL || frame_id == NULL) return 0U;
    do {
        before = K230_frame_id;
        *data = K230_data;
        after = K230_frame_id;
    } while (before != after);
    *frame_id = after;
    return (uint8_t)(after != 0U);
}

uint8_t K230_IsOnline(void)
{
    return (uint8_t)(K230_daemon != NULL && DaemonIsOnline(K230_daemon));
}
static void K230LostCallback(void *ptr)
{
	memset(&K230_data, 0, sizeof(K230_Data_t));
}
void K230_INST_IRQHandler(void)
{
    uint8_t RxData = DL_UART_receiveData(K230_INST);
    K230_ReceiveData(RxData);
}
