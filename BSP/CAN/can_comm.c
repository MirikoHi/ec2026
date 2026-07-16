/**
 * @file can_comm.c
 * @brief 多机CAN通信协议层实现
 *
 * @note  移植自 Hero__2026_chasisis 项目
 *        适配: 去掉 can_handle，使用单MCAN0 + tx_buf_idx
 */

#include "can_comm.h"
#include "stdlib.h"
#include "string.h"
#include "../Algorithm/crc8.h"
#include "dwt.h"
#include "bsp_log.h"


/**
 * @brief 重置接收状态机
 */
static void CANCommResetRx(CANCommInstance *ins)
{
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state   = 0;
    ins->cur_recv_len = 0;
}

/**
 * @brief CAN接收回调（底层中断 → 解析帧协议）
 *
 * @param _instance 触发中断的CAN实例
 */
static void CANCommRxCallback(CANInstance *_instance)
{
    CANCommInstance *comm = (CANCommInstance *)_instance->id;

    /* 状态0: 寻找帧头 */
    if (_instance->rx_buff[0] == CAN_COMM_HEADER && comm->recv_state == 0)
    {
        if (_instance->rx_buff[1] == comm->recv_data_len)
        {
            comm->recv_state = 1; // 进入接收状态
        }
        else
        {
            return; // 数据长度不匹配，丢弃
        }
    }

    /* 状态1: 接收数据 */
    if (comm->recv_state)
    {
        /* 溢出保护 */
        if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
        {
            CANCommResetRx(comm);
            return;
        }

        /* 追加数据到缓冲区 */
        memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        comm->cur_recv_len += _instance->rx_len;

        /* 收完完整数据包 */
        if (comm->cur_recv_len == comm->recv_buf_len)
        {
            /* 检查帧尾 */
            if (comm->raw_recvbuf[comm->recv_buf_len - 1] == CAN_COMM_TAIL)
            {
                /* 验证CRC8 */
                if (comm->raw_recvbuf[comm->recv_buf_len - 2] ==
                    crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
                {
                    /* 提取有效数据 */
                    memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
                    comm->update_flag = 1;
                    DaemonReload(comm->comm_daemon);
                }
            }
            CANCommResetRx(comm);
        }
    }
}

/**
 * @brief CAN通信离线回调
 */
static void CANCommLostCallback(void *cancomm)
{
    CANCommInstance *comm = (CANCommInstance *)cancomm;
    CANCommResetRx(comm);
    //LOGWARNING("[can_comm] rx[0x%03X] lost, reset rx state.", comm->can_ins->rx_id);
}


CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config)
{
    CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(CANCommInstance));
    if (ins == NULL)
        return NULL;
    memset(ins, 0, sizeof(CANCommInstance));

    ins->recv_data_len = comm_config->recv_data_len;
    ins->recv_buf_len  = comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES;
    ins->send_data_len = comm_config->send_data_len;
    ins->send_buf_len  = comm_config->send_data_len + CAN_COMM_OFFSET_BYTES;

    /* 预填充帧头和帧尾 */
    ins->raw_sendbuf[0] = CAN_COMM_HEADER;
    ins->raw_sendbuf[1] = comm_config->send_data_len; // 数据长度
    ins->raw_sendbuf[comm_config->send_data_len + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL;

    /* 注册底层CAN实例 */
    comm_config->can_config.id                     = ins;
    comm_config->can_config.can_module_callback    = CANCommRxCallback;
    ins->can_ins = CANRegister(&comm_config->can_config);

    if (ins->can_ins == NULL)
    {
        free(ins);
        return NULL;
    }

    /* 注册守护进程 */
    Daemon_Init_Config_s daemon_config = {
        .callback     = CANCommLostCallback,
        .owner_id     = (void *)ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);

    LOGINFO("[can_comm] Init: tx_id=0x%03X, rx_id=0x%03X, data=%dB",
            comm_config->can_config.tx_id, comm_config->can_config.rx_id, ins->send_data_len);

    return ins;
}

void CANCommSend(CANCommInstance *instance, uint8_t *data)
{
    /* 拷贝数据到发送缓冲区 */
    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);

    /* 计算CRC8并填入 */
    uint8_t crc = crc_8(data, instance->send_data_len);
    instance->raw_sendbuf[2 + instance->send_data_len] = crc;

    /* 分包发送（每帧最多8字节） */
    for (uint16_t i = 0; i < instance->send_buf_len; i += 8)
    {
        uint8_t send_len = (instance->send_buf_len - i >= 8) ? 8 : (instance->send_buf_len - i);
        CANSetDLC(instance->can_ins, send_len);
        memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
        CANTransmit(instance->can_ins, 1);
    }
}

void *CANCommGet(CANCommInstance *instance)
{
    instance->update_flag = 0;
    return instance->unpacked_recv_data;
}

uint8_t CANCommIsOnline(CANCommInstance *instance)
{
    return DaemonIsOnline(instance->comm_daemon);
}
