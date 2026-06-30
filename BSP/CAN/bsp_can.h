/**
 * @file bsp_can.h
 * @brief CAN总线抽象层 —— 基于 TI MSPM0 DriverLib MCAN API
 *
 * @note  移植自 Hero__2026_chasisis 项目，从 STM32 HAL 适配至 TI MSPM0 DriverLib
 *        使用 ClassicCAN 模式（fdMode=false），单 MCAN0 外设
 *        参考 TI SDK 例程: mcan_multi_message_tx
 */

#ifndef ELECTRIC_COMPETITION_ROBOT_BSP_CAN_H
#define ELECTRIC_COMPETITION_ROBOT_BSP_CAN_H

#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_mcan.h"
#include "stdint.h"


/** 最大CAN虚拟实例数 */
#define CAN_MX_REGISTER_CNT 8
/** CAN外设数量（单MCAN0） */
#define DEVICE_CAN_CNT       1
/** TX Buffer总数*/
#define CAN_TX_BUF_CNT       MCAN0_INST_MCAN_TX_BUFF_SIZE

/**
 * @brief MCAN标准ID与硬件寄存器格式互转宏
 * @note  TI MCAN 驱动中 DL_MCAN_TxBufElement.id 使用硬件寄存器位布局
 *        标准11位CAN ID位于消息RAM第一个字的 bits[28:18]
 *        因此需要: hw_id = std_id << 18
 */
#define CAN_STD_ID_TO_REG(id)  ((id) << 18)
#define CAN_REG_ID_TO_STD(reg) ((reg) >> 18)

/* TX Buffer分配方案（2个Buffer共享使用）：
 * Buffer 0: DJI电机分组 + can_comm 通信
 * Buffer 1: DM电机
 */

typedef struct CANInstance
{
    uint32_t tx_id;                                          /**< 发送ID */
    uint32_t rx_id;                                          /**< 接收ID */
    uint8_t  tx_buff[8];                                     /**< 发送缓存 */
    uint8_t  rx_buff[8];                                     /**< 接收缓存 */
    uint8_t  rx_len;                                         /**< 接收数据长度 */
    uint8_t  tx_buf_idx;                                     /**< 绑定的TX Buffer索引（0~1） */
    DL_MCAN_TxBufElement tx_elem;                            /**< TX Buffer元素（预配置复用） */
    void (*can_module_callback)(struct CANInstance *);       /**< 接收回调函数 */
    void *id;                                                /**< 拥有者ID（模块实例指针） */
} CANInstance;

/** CAN实例初始化配置 */
typedef struct
{
    uint32_t tx_id;                                          /**< 发送ID */
    uint32_t rx_id;                                          /**< 接收ID */
    uint8_t  tx_buf_idx;                                     /**< 分配的TX Buffer索引 */
    void   (*can_module_callback)(CANInstance *);            /**< 接收回调函数 */
    void    *id;                                             /**< 拥有者ID */
} CAN_Init_Config_s;


/**
 * @brief 注册一个CAN实例到CAN服务
 *
 * @attention 首次注册时会自动调用 CANServiceInit() 初始化硬件
 *
 * @param config 初始化配置结构体指针
 * @return CANInstance* 返回实例指针（NULL表示失败）
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的DLC（数据长度）
 *
 * @param instance CAN实例
 * @param length   数据长度（1~8）
 */
void CANSetDLC(CANInstance *instance, uint8_t length);

/**
 * @brief 通过CAN实例发送消息
 *
 * @attention 发送前需要向 instance->tx_buff 写入待发送数据
 *
 * @param instance CAN实例
 * @param timeout  超时时间（ms）
 * @return uint8_t  1=成功，0=超时/失败
 */
uint8_t CANTransmit(CANInstance *instance, float timeout);

/**
 * @brief 初始化CAN服务（硬件/中断/过滤）
 *
 * @note 首次调用 CANRegister() 时会自动调用，也可手动提前调用
 */
void CANServiceInit(void);

#endif // ELECTRIC_COMPETITION_ROBOT_BSP_CAN_H
