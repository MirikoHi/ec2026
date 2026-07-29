#ifndef __BSP_W25Q128JV_SPI_H__
#define __BSP_W25Q128JV_SPI_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

/**
  * @brief W25Q128JV 标准 SPI 驱动库用法介绍
  * @note 1. SysConfig 中需要先配置好 SPI_FLASH、CS 引脚以及 SPI 时钟参数
  * @note 2. 上电后调用 W25Q128JV_SpiInit()，保证 CS 处于空闲高电平
  * @note 3. 调用 W25Q128JV_ReadJedecId() 确认芯片通信正常，25Q128JVSQ 期望 ID 为 EF 4018
  * @note 4. 写入前必须先擦除对应扇区或块；Flash 只能把 1 写成 0，不能直接把 0 写回 1
  * @note 5. 普通跨页写入使用 W25Q128JV_WriteBuffer()，单页写入可使用 W25Q128JV_PageProgram()
  * @note 6. 参数存储服务占用最后两个 4KB 扇区，正常业务不要擦写 0xFFE000 和 0xFFF000
  */

/* W25Q128JV 标准 SPI 指令。 */
#define W25Q128JV_CMD_WRITE_ENABLE          0x06U
#define W25Q128JV_CMD_WRITE_DISABLE         0x04U
#define W25Q128JV_CMD_READ_STATUS1          0x05U
#define W25Q128JV_CMD_READ_STATUS2          0x35U
#define W25Q128JV_CMD_READ_STATUS3          0x15U
#define W25Q128JV_CMD_READ_JEDEC_ID         0x9FU
#define W25Q128JV_CMD_READ_DATA             0x03U
#define W25Q128JV_CMD_FAST_READ             0x0BU
#define W25Q128JV_CMD_PAGE_PROGRAM          0x02U
#define W25Q128JV_CMD_SECTOR_ERASE_4K       0x20U
#define W25Q128JV_CMD_BLOCK_ERASE_32K       0x52U
#define W25Q128JV_CMD_BLOCK_ERASE_64K       0xD8U
#define W25Q128JV_CMD_CHIP_ERASE1           0xC7U
#define W25Q128JV_CMD_CHIP_ERASE2           0x60U
#define W25Q128JV_CMD_ENABLE_RESET          0x66U
#define W25Q128JV_CMD_RESET                 0x99U

/* 芯片几何参数。 */
#define W25Q128JV_PAGE_SIZE                 256U
#define W25Q128JV_SECTOR_SIZE               4096U
#define W25Q128JV_BLOCK32_SIZE              32768U
#define W25Q128JV_BLOCK64_SIZE              65536U
#define W25Q128JV_TOTAL_SIZE                (16U * 1024U * 1024U)

/* 这颗料号对应的 JEDEC ID。 */
#define W25Q128JV_MANUFACTURER_ID           0xEFU
#define W25Q128JV_DEVICE_ID                 0x4018U

typedef enum
{
    W25Q128JV_OK = 0,
    W25Q128JV_ERR_NULL = -1,
    W25Q128JV_ERR_TIMEOUT = -2,
    W25Q128JV_ERR_ID_MISMATCH = -3,
    W25Q128JV_ERR_PARAM = -4,
} W25Q128JV_Status_e;

typedef struct
{
    uint8_t manufacturer_id;
    uint16_t device_id;
} W25Q128JV_JedecId_s;

/**
  * @brief 初始化 W25Q128JV SPI 片选状态
  * @return 无
  * @note 用法：系统初始化后调用一次，保证 CS 处于高电平空闲状态
  */
void W25Q128JV_SpiInit(void);

/**
  * @brief 读取并校验 JEDEC ID
  * @param id 输出 JEDEC ID，manufacturer_id 为厂商 ID，device_id 为容量/类型 ID
  * @return W25Q128JV_OK 表示 ID 正确，其他值表示空指针或型号不匹配
  * @note 用法：初始化后先调用本函数确认 SPI 通信和 Flash 型号正确
  */
W25Q128JV_Status_e W25Q128JV_ReadJedecId(W25Q128JV_JedecId_s *id);

/**
  * @brief 读取状态寄存器 1
  * @param value 输出状态寄存器 1 的值
  * @return W25Q128JV_OK 表示读取成功
  * @note 状态寄存器 1 的 bit0 为 WIP，1 表示芯片忙
  */
W25Q128JV_Status_e W25Q128JV_ReadStatus1(uint8_t *value);

/**
  * @brief 读取状态寄存器 2
  * @param value 输出状态寄存器 2 的值
  * @return W25Q128JV_OK 表示读取成功
  */
W25Q128JV_Status_e W25Q128JV_ReadStatus2(uint8_t *value);

/**
  * @brief 读取状态寄存器 3
  * @param value 输出状态寄存器 3 的值
  * @return W25Q128JV_OK 表示读取成功
  */
W25Q128JV_Status_e W25Q128JV_ReadStatus3(uint8_t *value);

/**
  * @brief 等待 Flash 空闲
  * @param timeout_ms 超时时间，单位 ms
  * @return W25Q128JV_OK 表示 Flash 已空闲，W25Q128JV_ERR_TIMEOUT 表示超时
  * @note 用法：擦除、写入后可调用本函数等待芯片内部操作完成
  */
W25Q128JV_Status_e W25Q128JV_WaitReady(uint32_t timeout_ms);

/**
  * @brief 从 Flash 读取任意长度数据
  * @param address 起始地址，范围 0 ~ W25Q128JV_TOTAL_SIZE-1
  * @param buf 读取数据输出缓冲区
  * @param len 读取长度，单位字节
  * @return W25Q128JV_OK 表示读取成功，其他值表示参数错误或等待超时
  * @note 用法：读取参数、日志或普通数据时调用，本函数不会修改 Flash 内容
  */
W25Q128JV_Status_e W25Q128JV_ReadData(uint32_t address, uint8_t *buf, uint32_t len);

/**
  * @brief 单页编程
  * @param address 写入起始地址
  * @param buf 待写入数据缓冲区
  * @param len 写入长度，不能跨越 256 字节页边界
  * @return W25Q128JV_OK 表示写入成功，其他值表示参数错误或写入超时
  * @note 写入前需要对应区域已经擦除；普通跨页写入建议使用 W25Q128JV_WriteBuffer()
  */
W25Q128JV_Status_e W25Q128JV_PageProgram(uint32_t address, const uint8_t *buf, uint32_t len);

/**
  * @brief 跨页连续写入数据
  * @param address 写入起始地址
  * @param buf 待写入数据缓冲区
  * @param len 写入长度，单位字节
  * @return W25Q128JV_OK 表示写入成功，其他值表示参数错误或写入失败
  * @note 函数内部会按 256 字节页自动拆分；写入前仍需先擦除对应区域
  */
W25Q128JV_Status_e W25Q128JV_WriteBuffer(uint32_t address, const uint8_t *buf, uint32_t len);

/**
  * @brief 擦除 4KB 扇区
  * @param address 扇区起始地址，必须 4KB 对齐
  * @return W25Q128JV_OK 表示擦除成功
  * @note 参数存储服务会使用本函数擦除参数槽
  */
W25Q128JV_Status_e W25Q128JV_Erase4K(uint32_t address);

/**
  * @brief 擦除 32KB 块
  * @param address 块起始地址，必须 32KB 对齐
  * @return W25Q128JV_OK 表示擦除成功
  */
W25Q128JV_Status_e W25Q128JV_Erase32K(uint32_t address);

/**
  * @brief 擦除 64KB 块
  * @param address 块起始地址，必须 64KB 对齐
  * @return W25Q128JV_OK 表示擦除成功
  */
W25Q128JV_Status_e W25Q128JV_Erase64K(uint32_t address);

/**
  * @brief 整片擦除
  * @return W25Q128JV_OK 表示整片擦除成功
  * @note 会清空整颗 Flash，包含参数槽，正常调参和比赛运行禁止调用
  */
W25Q128JV_Status_e W25Q128JV_ChipErase(void);

/**
  * @brief 软件复位 Flash
  * @return W25Q128JV_OK 表示复位命令已发送
  * @note 用法：通信异常或自测前可调用一次，让芯片回到标准 SPI 状态
  */
W25Q128JV_Status_e W25Q128JV_Reset(void);

/* 自测会擦除最后一个扇区 0xFFF000，该扇区已作为参数 slot1，禁止在正常启动中调用。 */
/**
  * @brief 运行 Flash 芯片基础自测
  * @return W25Q128JV_OK 表示 JEDEC、擦除、写入、读取校验全部通过
  * @note 自测会擦除最后一个扇区 0xFFF000，该扇区已作为参数 slot1，禁止在正常启动中调用
  */
W25Q128JV_Status_e W25Q128JV_RunSelfTest(void);

#endif
