#ifndef BLUETOOTH_H
#define BLUETOOTH_H



#include <stdint.h>

/* ================= 控制命令 ================= */

/* 模式控制 */
#define BLUETOOTH_CMD_MANUAL_MODE       0xA0U
#define BLUETOOTH_CMD_TRACE_MODE        0xA1U

/* 运动控制 */
#define BLUETOOTH_CMD_FORWARD           0xAAU
#define BLUETOOTH_CMD_BACKWARD          0xBBU
#define BLUETOOTH_CMD_FORWARD_LEFT      0xCCU
#define BLUETOOTH_CMD_FORWARD_RIGHT     0xDDU
#define BLUETOOTH_CMD_STOP              0xEEU

/* 电机使能 */
#define BLUETOOTH_CMD_DISABLE           0xF0U
#define BLUETOOTH_CMD_ENABLE            0xF1U

/**
 * @brief 初始化HC-06串口接收
 */
void Bluetooth_Init(void);

/**
 * @brief 从蓝牙接收队列读取一个有效命令
 *
 * @param command 命令输出地址
 * @return 1：成功读取
 *         0：当前没有新命令
 */
uint8_t Bluetooth_ReadCommand(uint8_t *command);

/**
 * @brief 将UART收到的字节送入命令队列
 */
void Bluetooth_ReceiveByte(uint8_t data);

/**
 * @brief 回传左右轮实际速度
 *
 * 输出格式：
 * L:0.125,R:0.118\r\n
 */
uint8_t Bluetooth_SendMotorSpeed(float left_speed,
                                 float right_speed);

/**
 * @brief 非阻塞地向UART TX FIFO填充待发送数据
 *
 * 在200 Hz任务中每次调用一次。
 */
void Bluetooth_PollTx(void);

/**
 * @brief 获取发送队列溢出次数
 */
uint32_t Bluetooth_GetTxDropCount(void);

/**
 * @brief 获取UART错误次数
 */
uint32_t Bluetooth_GetUartErrorCount(void);


#endif
