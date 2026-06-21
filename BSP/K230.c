#include "K230.h"
#include "daemon.h"
//#include <ti/devices/msp/msp.h>   // 添加MSP设备头文件
//#include <ti/driverlib/dl_uart.h> // 添加UART驱动库头文件
#define DATA_PACKET_LENGTH 7

static uint8_t RxBuffer[DATA_PACKET_LENGTH]; // 接收数据数组
static volatile uint8_t RxState = 0; // 接收状态标志位
static uint8_t RxIndex = 0;         // 接收数组索引
int16_t K230_err[2]={0};
static void K230LostCallback(void *ptr);
DaemonInstance* K230_daemon,*K230_Lost_Target_daemon;
void Clear_UART_FIFO(void) {
    while (!DL_UART_isRXFIFOEmpty(K230_INST)) {
        (void)DL_UART_receiveData(K230_INST); // 丢弃数据
    }
}
void K230_Init(void)
{
    Clear_UART_FIFO(); // 新增：初始化前清空FIFO
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
		DL_UART_clearInterruptStatus(K230_INST, DL_UART_INTERRUPT_RX); // 清除中断标志位
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
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void K230_ReceiveData(uint8_t RxData)
{
    static uint8_t sum = 0;
    // 添加包头强制同步
    if(RxData == 0x15 && RxState != 0)
    {
        RxState = 0;  // 发现包头立即重置状态机
        RxIndex = 0;
    }
    if (RxState == 0) // 等待包头
    {
        if (RxData == 0x15)
        {
            RxBuffer[0] = RxData;
            RxState = 1;
            RxIndex = 1;
            sum = RxData; // 初始化校验和
        }
    }
    else if (RxState == 1) // 判断数据类型
    {
        if (RxData == 0x11) 
        {
            RxBuffer[RxIndex++] = RxData;
            RxState = 2;
            sum += RxData; // 累加数据类型
        }
        else
        {
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2) // 接收数据
    {
        if(RxIndex < DATA_PACKET_LENGTH)
        {
            RxBuffer[RxIndex] = RxData;
            
            // 关键修改：只累加到校验和前一个字节
            if (RxIndex < (DATA_PACKET_LENGTH - 1)) {
                sum += RxData;
            }
            
            RxIndex++;
            
            if (RxIndex == DATA_PACKET_LENGTH) // 接收完成
            {
                // 关键修改：比较校验和时不包含校验和字节本身
                if (sum == RxBuffer[DATA_PACKET_LENGTH - 1])
                {
                    // 解析数据 (小端模式)
                    K230_err[0] = (RxBuffer[3] << 8) | RxBuffer[2];
                    K230_err[1] = (RxBuffer[5] << 8) | RxBuffer[4];
//                    int16_t data2 = (RxBuffer[7] << 8) | RxBuffer[6];
//                    int16_t data3 = (RxBuffer[9] << 8) | RxBuffer[8];
                    DaemonReload(K230_daemon);
										DaemonReload(K230_Lost_Target_daemon);
                    // 处理数据
                    // 例如: 控制电机等
                }
                // 重置状态机
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