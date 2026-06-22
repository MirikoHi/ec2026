#include "JY901S.h"
#include "dwt.h"
static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t RxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/
volatile IMU_Data_s IMU_Data={0};
float YawAngleLast=0;



void jy61p_Init(void)
{
	NVIC_ClearPendingIRQ(UART1_INT_IRQn);
	NVIC_EnableIRQ(UART1_INT_IRQn);
	DL_UART_clearInterruptStatus(UART1,DL_UART_INTERRUPT_RX);//清除中断标志位

}
/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	if (RxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1;	//进入下一状态
		}
	}
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2;	//进入下一状态
		}
	}
	else if (RxState == 2)	//接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i];	//计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
				/*计算数据，根据数据内容选择对应的计算公式*/
				IMU_Data.Roll = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				IMU_Data.Pitch = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 180.0f;
				IMU_Data.Yaw = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 180.0f;
				if (IMU_Data.Yaw - YawAngleLast > 180.0f)
				{
					IMU_Data.Yaw_Round_Count--;
				}
				else if (IMU_Data.Yaw - YawAngleLast < -180.0f)
				{
					IMU_Data.Yaw_Round_Count++;
				}
				IMU_Data.Yaw_Total_Angle = 360.0f * IMU_Data.Yaw_Round_Count + IMU_Data.Yaw;
				YawAngleLast = IMU_Data.Yaw;

				
				
			}
			RxState = 0;
			RxIndex = 0;	//读取完成，回到最初状态，等待包头
		}
	}
}

void UART1_IRQHandler(void)
{
	uint8_t RxData = DL_UART_receiveData(UART1);
	jy61p_ReceiveData(RxData);
}