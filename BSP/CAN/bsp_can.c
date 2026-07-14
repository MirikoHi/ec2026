/**
 * @file bsp_can.c
 * @brief CAN总线抽象层 —— TI MSPM0 DriverLib MCAN 实现
 *
 * @note  移植自 Hero__2026_chasisis 项目（STM32 HAL → TI DriverLib）
 *        参考 TI SDK 例程: mcan_multi_message_tx
 *        使用 ClassicCAN 模式: fdMode=false, brsEnable=false
 *        单 MCAN0 外设，TX Buffer 模式
 */

#include "bsp_can.h"
#include "stdlib.h"
#include "string.h"
#include "dwt.h"
#include "bsp_log.h"
#include "FreeRTOS.h"
#include "task.h"


/** 全局CAN实例注册表 */
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx = 0;                          /**< 当前已注册实例数 */
static uint8_t service_initialized = 0;          /**< CAN服务初始化标志 */

/** 默认 TX Buffer 元素模板（经典CAN标准帧，8字节数据） */
static const DL_MCAN_TxBufElement TX_ELEM_TEMPLATE = {
    .id   = 0x000,
    .rtr  = 0,         /* 数据帧 */
    .xtd  = 0,         /* 标准ID (11-bit) */
    .esi  = 0,         /* 错误状态指示器 */
    .dlc  = 8,         /* 8字节数据长度 */
    .brs  = 0,         /* 无位速率切换 (Classic CAN) */
    .fdf  = 0,         /* 经典CAN格式 */
    .efc  = 0,         /* 无事件FIFO */
    .mm   = 0,         /* 消息标记 */
    .data = {0},
};


/**
 * @brief 为指定 rx_id 配置标准ID硬件过滤器
 * @note  使用标准ID双过滤器模式，将 rx_id 加入过滤列表
 *        奇数 rx_id → FIFO0，偶数 rx_id → FIFO1
 */
static void CANAddFilter(CANInstance *instance);

/* ========================== 公共API实现 ========================== */

/**
 * @brief 初始化CAN服务
 * @note  MCAN0硬件初始化已在 SYSCFG_DL_MCAN0_init() 中完成（SysConfig生成）
 *        此函数配置中断线并启用NVIC
 */
void CANServiceInit(void)
{
    if (service_initialized)
        return;

    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);

    service_initialized = 1;
    LOGINFO("[bsp_can] CAN Service Init OK");
}

/**
 * @brief 注册一个CAN实例
 *
 * @param config 初始化配置（tx_id, rx_id, tx_buf_idx, callback, id）
 * @return CANInstance* 实例指针，失败返回NULL
 */
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!service_initialized)
    {
        CANServiceInit();
    }

    if (idx >= CAN_MX_REGISTER_CNT)
    {
        LOGERROR("[bsp_can] CAN instance exceeded MAX num (%d)", CAN_MX_REGISTER_CNT);
        return NULL;
    }

    /* 检查重复注册 */
    for (uint8_t i = 0; i < idx; i++)
    {
        if (can_instance[i]->rx_id == config->rx_id)
        {
            LOGERROR("[bsp_can] CAN rx_id [0x%03X] already registered!", config->rx_id);
            return NULL;
        }
    }

    /* 检查 TX Buffer 索引合法性 */
    if (config->tx_buf_idx >= CAN_TX_BUF_CNT)
    {
        LOGERROR("[bsp_can] TX buf idx [%d] out of range (max %d)", config->tx_buf_idx, CAN_TX_BUF_CNT - 1);
        return NULL;
    }

    /* 分配CAN实例内存 */
    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    if (instance == NULL)
    {
        LOGERROR("[bsp_can] malloc failed for CANInstance");
        return NULL;
    }
    memset(instance, 0, sizeof(CANInstance));

    /* 初始化发送报文配置 */
    instance->tx_elem    = TX_ELEM_TEMPLATE;
    instance->tx_elem.id = CAN_STD_ID_TO_REG(config->tx_id);
    instance->tx_id      = config->tx_id;
    instance->rx_id      = config->rx_id;
    instance->tx_buf_idx = config->tx_buf_idx;
    instance->can_module_callback = config->can_module_callback;
    instance->id         = config->id;

    /* 添加硬件过滤器 */
    CANAddFilter(instance);

    /* 注册到全局实例表 */
    can_instance[idx++] = instance;

    LOGINFO("[bsp_can] Registered: tx_id=0x%03X, rx_id=0x%03X, buf=%d",
            instance->tx_id, instance->rx_id, instance->tx_buf_idx);

    return instance;
}

/**
 * @brief 设置CAN发送数据长度
 *
 * @param instance CAN实例
 * @param length   数据长度（1~8）
 */
void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if (length > 8 || length == 0)
    {
        LOGERROR("[bsp_can] CAN DLC error! length=%d", length);
        return;
    }
    /* 注意：DLC 值直接对应 0~8，MSB 为 0 表示经典CAN数据长度 */
    instance->tx_elem.dlc = length;
}

/**
 * @brief 通过CAN实例发送消息
 *
 * @note  发送前需要将数据写入 instance->tx_buff[]
 *        每次发送前将 tx_buff 同步到 tx_elem.data，然后写入消息RAM
 *        MCAN的TXBRP在帧实际发送完成前一直为1，等待其清除再写入下一帧
 *
 * @param instance CAN实例
 * @param timeout  超时时间（ms）
 * @return uint8_t  1=发送成功，0=超时
 */
uint8_t CANTransmit(CANInstance *instance, float timeout)
{
    float start = DWT_GetTimeline_ms();

    /* 等待TX Buffer空闲（TXBRP位清除 = 上一帧已实际发送完成） */
    while (DL_MCAN_getTxBufReqPend(MCAN0_INST) & ((uint32_t)1U << instance->tx_buf_idx))
    {
        taskYIELD();  //让出CPU，同优先级任务可在此期间运行
        if (DWT_GetTimeline_ms() - start > timeout)
        {
            LOGERROR("[bsp_can] CAN TX timeout! buf_idx=%d", instance->tx_buf_idx);
            return 0;
        }
    }

    memcpy(instance->tx_elem.data, instance->tx_buff, 8);

    /* 写入消息RAM（TX Buffer区） */
    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
                        instance->tx_buf_idx, &instance->tx_elem);

    /* 触发发送请求 */
    DL_MCAN_TXBufAddReq(MCAN0_INST, instance->tx_buf_idx);

    return 1; // 发送请求已提交
}


/**
 * @brief 为标准ID配置硬件过滤器
 *
 * @note  MSPM0 MCAN 标准ID过滤器在消息RAM中配置
 *        通过 DL_MCAN_addStandardMsgIDFilter 添加过滤条目
 *        当前策略：使用双过滤器模式，rx_id写入过滤列表
 *        奇数ID → FIFO0，偶数ID → FIFO1
 *
 * @param instance CAN实例
 */
static void CANAddFilter(CANInstance *instance)
{
    /*
     * 简化策略：对于小规模应用（≤8个CAN实例），
     * 依赖中断处理中的软件ID匹配，不配置严格的硬件过滤器。
     *
     * SysConfig 已配置全局过滤器为"全部拒绝，只接受匹配过滤器的ID"。
     * 如果未来需要硬件过滤优化，可以使用：
     *   DL_MCAN_addStandardMsgIDFilter(MCAN0_INST, DL_MCAN_MEM_TYPE_STD_MSG_ID_FILTER,
     *       filter_idx, &filterElem);
     *
     * 当前采用全局非匹配帧接收方案：在 SYSCFG_DL_MCAN0_init 中
     * 通过 DL_MCAN_config 的 ConfigParams 配置为接受所有帧进入FIFO0，
     * 然后在 ISR 中做软件ID匹配。
     */

    (void)instance; // 预留，当前使用软件过滤
}


/**
 * @brief MCAN0 中断服务程序
 *
 * @note  处理 RX FIFO 0 新消息中断
 *        从 FIFO 读取消息，匹配 rx_id，调用对应回调
 */
void MCAN0_INST_IRQHandler(void)
{
    uint32_t pendingInt = DL_MCAN_getPendingInterrupt(MCAN0_INST);

    if (pendingInt == DL_MCAN_IIDX_LINE1)
    {
        uint32_t intrStatus = DL_MCAN_getIntrStatus(MCAN0_INST);
        intrStatus &= MCAN0_INST_MCAN_INTERRUPTS;

        DL_MCAN_clearIntrStatus(MCAN0_INST, intrStatus, DL_MCAN_INTR_SRC_MCAN_LINE_1);

        /* RX FIFO 0 新消息 */
        if (intrStatus & DL_MCAN_INTERRUPT_RF0N)
        {
            DL_MCAN_RxBufElement  rxMsg;
            DL_MCAN_RxFIFOStatus  fifoStatus;

            /* 查询FIFO0状态 */
            fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
            DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);

            while (fifoStatus.fillLvl > 0)
            {
                /* 从FIFO0读取消息（bufNum=0, fifoNum=DL_MCAN_RX_FIFO_NUM_0） */
                DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO,
                                   0, DL_MCAN_RX_FIFO_NUM_0, &rxMsg);

                /* 只处理标准ID数据帧 */
                if (rxMsg.xtd == 0 && rxMsg.rtr == 0)
                {
                    uint32_t rx_id  = CAN_REG_ID_TO_STD(rxMsg.id);
                    uint8_t  rx_dlc = rxMsg.dlc;

                    for (uint8_t i = 0; i < idx; i++)
                    {
                        if (can_instance[i]->rx_id == rx_id)
                        {
                            can_instance[i]->rx_len = (rx_dlc > 8) ? 8 : rx_dlc;
                            memcpy(can_instance[i]->rx_buff, rxMsg.data, can_instance[i]->rx_len);

                            if (can_instance[i]->can_module_callback != NULL)
                            {
                                can_instance[i]->can_module_callback(can_instance[i]);
                            }
                            break;
                        }
                    }
                }

                /* 确认消息已读取，推进FIFO指针 */
                DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0, fifoStatus.getIdx);

                /* 刷新FIFO状态 */
                fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
                DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);
            }
        }

        /* RX FIFO 1 新消息（备用） */
        if (intrStatus & DL_MCAN_INTERRUPT_RF1N)
        {
            DL_MCAN_RxBufElement  rxMsg;
            DL_MCAN_RxFIFOStatus  fifoStatus;

            fifoStatus.num = DL_MCAN_RX_FIFO_NUM_1;
            DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);

            while (fifoStatus.fillLvl > 0)
            {
                DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO,
                                   0, DL_MCAN_RX_FIFO_NUM_1, &rxMsg);

                if (rxMsg.xtd == 0 && rxMsg.rtr == 0)
                {
                    uint32_t rx_id  = CAN_REG_ID_TO_STD(rxMsg.id);
                    uint8_t  rx_dlc = rxMsg.dlc;

                    for (uint8_t i = 0; i < idx; i++)
                    {
                        if (can_instance[i]->rx_id == rx_id)
                        {
                            can_instance[i]->rx_len = (rx_dlc > 8) ? 8 : rx_dlc;
                            memcpy(can_instance[i]->rx_buff, rxMsg.data, can_instance[i]->rx_len);

                            if (can_instance[i]->can_module_callback != NULL)
                            {
                                can_instance[i]->can_module_callback(can_instance[i]);
                            }
                            break;
                        }
                    }
                }

                DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_1, fifoStatus.getIdx);

                fifoStatus.num = DL_MCAN_RX_FIFO_NUM_1;
                DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifoStatus);
            }
        }
    }
}
