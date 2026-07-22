#include "JY901S.h"
#include "dwt.h"

static uint8_t RxBuffer[11];            /* 接收数据缓存数组 */
static volatile uint8_t RxState = 0;    /* 接收状态机状态 */
static uint8_t RxIndex = 0;            /* 接收数组索引 */
static float YawAngleLast = 0.0f;       /* 上一次的 Yaw 角度 */

// 定义姿态数据全局变量
volatile JY901s_IMU_Data_s JY901S_IMU_Data = {0};

/**
 * @brief  JY901S 初始化函数
 * @note   使能串口中断
 */
void JY901s_Init(void)
{
    // 使能 UART_1 串口中断
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/**
 * @brief  串口数据包解析状态机
 * @param  RxData: 串口接收到的单个字节
 */
void JY901s_ReceiveData(uint8_t RxData)
{
    uint8_t i, sum = 0;

    if (RxState == 0) // === 状态0：等待包头 0x55 ===
    {
        if (RxData == 0x55)
        {
            RxBuffer[0] = RxData;
            RxIndex = 1;
            RxState = 1; // 进入下一步：判断包类型
        }
    }
    else if (RxState == 1) // === 状态1：判断标志位 0x53 (角度包) ===
    {
        if (RxData == 0x53)
        {
            RxBuffer[1] = RxData;
            RxIndex = 2;
            RxState = 2; // 进入下一步：接收数据内容
        }
        else if (RxData == 0x55)
        {
            // 如果收到的不是 0x53 而是另一个 0x55，说明上一个是误判，重新保持在状态1
            RxIndex = 1;
            RxState = 1;
        }
        else
        {
            // 收到既不是 0x53 也不是 0x55 的非法数据，立马复位状态机（防止死锁！）
            RxState = 0;
            RxIndex = 0;
        }
    }
    else if (RxState == 2) // === 状态2：接收后续数据并计算 ===
    {
        RxBuffer[RxIndex++] = RxData;

        if (RxIndex == 11) // 11个字节接收完成
        {
            // 校验和计算：前10个字节相加
            for (i = 0; i < 10; i++)
            {
                sum += RxBuffer[i];
            }

            if (sum == RxBuffer[10]) // 校验通过
            {
                /* 提取 Roll, Pitch, Yaw (16位有符号数换算) */
                JY901S_IMU_Data.Roll  = ((int16_t)((int16_t)RxBuffer[3] << 8 | RxBuffer[2])) / 32768.0f * 180.0f;
                JY901S_IMU_Data.Pitch = ((int16_t)((int16_t)RxBuffer[5] << 8 | RxBuffer[4])) / 32768.0f * 180.0f;
                JY901S_IMU_Data.Yaw   = ((int16_t)((int16_t)RxBuffer[7] << 8 | RxBuffer[6])) / 32768.0f * 180.0f;

                /* 偏航角跨过 +/-180 度边界时的圈数统计 */
                if (JY901S_IMU_Data.Yaw - YawAngleLast > 180.0f)
                {
                    JY901S_IMU_Data.Yaw_Round_Count--;
                }
                else if (JY901S_IMU_Data.Yaw - YawAngleLast < -180.0f)
                {
                    JY901S_IMU_Data.Yaw_Round_Count++;
                }

                // 计算连续累计角度
                JY901S_IMU_Data.Yaw_Total_Angle = 360.0f * JY901S_IMU_Data.Yaw_Round_Count + JY901S_IMU_Data.Yaw;
                YawAngleLast = JY901S_IMU_Data.Yaw;
            }

            // 读取完毕，复位状态机回到起点
            RxState = 0;
            RxIndex = 0;
        }
    }
}

/**
 * @brief  UART_1 串口中断服务函数 (TI MSPM0 平台)
 */
void UART_1_INST_IRQHandler(void)
{

    uint8_t RxData = DL_UART_receiveData(UART_1_INST);
    JY901s_ReceiveData(RxData);
}