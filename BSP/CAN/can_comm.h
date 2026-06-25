/**
 * @file can_comm.h
 * @brief 多机CAN通信协议层
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        帧协议: 's' + data_len(1B) + data(NB) + crc8(1B) + 'e'
 *        支持大于8字节的分包发送/接收
 */

#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "bsp_can.h"
#include "daemon.h"

#define MX_CAN_COMM_COUNT       4    /**< 最大CAN通信实例数 */
#define CAN_COMM_MAX_BUFFSIZE   60   /**< 单包最大数据字节数 */
#define CAN_COMM_HEADER         's'  /**< 帧头 */
#define CAN_COMM_TAIL           'e'  /**< 帧尾 */
#define CAN_COMM_OFFSET_BYTES   4    /**< 帧开销: 's' + datalen + crc8 + 'e' */

#pragma pack(1)
/** CAN通信实例 */
typedef struct
{
    CANInstance *can_ins;

    /* 发送部分 */
    uint8_t send_data_len;                                                 /**< 发送数据长度 */
    uint8_t send_buf_len;                                                  /**< 发送缓冲总长度 */
    uint8_t raw_sendbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES];   /**< 发送原始缓冲区 */

    /* 接收部分 */
    uint8_t recv_data_len;                                                 /**< 接收数据长度 */
    uint8_t recv_buf_len;                                                  /**< 接收缓冲总长度 */
    uint8_t raw_recvbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES];   /**< 接收原始缓冲区 */
    uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE];                     /**< 解包后的有效数据 */

    /* 接收状态机 */
    uint8_t recv_state;        /**< 0=空闲, 1=接收中 */
    uint8_t cur_recv_len;      /**< 当前已接收字节数 */
    uint8_t update_flag;       /**< 新数据标志 */

    DaemonInstance *comm_daemon;
} CANCommInstance;
#pragma pack()

/** CAN通信初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;  /**< CAN底层配置 */
    uint8_t send_data_len;          /**< 发送数据长度 */
    uint8_t recv_data_len;          /**< 接收数据长度 */
    uint16_t daemon_count;          /**< 守护进程重载值 */
} CANComm_Init_Config_s;

/**
 * @brief 初始化CAN通信实例
 *
 * @param comm_config 初始化配置
 * @return CANCommInstance* 实例指针
 */
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);

/**
 * @brief 通过CAN通信发送数据（自动分包）
 *
 * @param instance CAN通信实例
 * @param data     待发送数据（长度须与初始化时一致）
 */
void CANCommSend(CANCommInstance *instance, uint8_t *data);

/**
 * @brief 获取接收到的数据
 *
 * @param instance CAN通信实例
 * @return void* 解包后的数据指针（需强制类型转换）
 */
void *CANCommGet(CANCommInstance *instance);

/**
 * @brief 检查CAN通信是否在线
 *
 * @param instance CAN通信实例
 * @return uint8_t 1=在线, 0=离线
 */
uint8_t CANCommIsOnline(CANCommInstance *instance);

#endif // CAN_COMM_H
