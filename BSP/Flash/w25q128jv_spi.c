#include "w25q128jv_spi.h"

#include "bsp_log.h"
#include "dwt.h"

/* 这个版本按当前工程的 SPI1 实现，不依赖 STM32 QSPI 外设。 */
#define W25Q128JV_SPI_INST          SPI_FLASH_INST
#define W25Q128JV_SPI_CS_PORT       SPI_FLASH_CS_PORT
#define W25Q128JV_SPI_CS_PIN        SPI_FLASH_CS_CSN_PIN

/**
  * @brief SPI 交换一个字节
  * @param tx 要发送的一个字节数据，范围 0x00~0xFF
  * @return 同时接收到的一个字节数据，范围 0x00~0xFF
  * @note 用法：W25Q128JV 是全双工 SPI，发送指令、地址、dummy 或数据时都通过本函数完成
  */
static uint8_t w25q128jv_spi_xfer(uint8_t tx)
{
    DL_SPI_transmitData8(W25Q128JV_SPI_INST, tx);
    while (DL_SPI_isBusy(W25Q128JV_SPI_INST))
    {
    }
    return DL_SPI_receiveData8(W25Q128JV_SPI_INST);
}

/**
  * @brief 拉低 Flash 片选
  * @return 无
  * @note 用法：每条 SPI 指令开始前调用，使 W25Q128JV 进入本次命令接收状态
  */
static void w25q128jv_cs_low(void)
{
    DL_GPIO_clearPins(W25Q128JV_SPI_CS_PORT, W25Q128JV_SPI_CS_PIN);
}

/**
  * @brief 拉高 Flash 片选
  * @return 无
  * @note 用法：每条 SPI 指令发送完成后调用，通知 W25Q128JV 结束本次命令
  */
static void w25q128jv_cs_high(void)
{
    DL_GPIO_setPins(W25Q128JV_SPI_CS_PORT, W25Q128JV_SPI_CS_PIN);
}

/**
  * @brief 发送 24 位 Flash 地址
  * @param address Flash 地址，低 24 位有效
  * @return 无
  * @note 用法：读、页编程、扇区擦除等带地址命令发送指令后调用
  */
static void w25q128jv_send_addr24(uint32_t address)
{
    w25q128jv_spi_xfer((uint8_t)(address >> 16));
    w25q128jv_spi_xfer((uint8_t)(address >> 8));
    w25q128jv_spi_xfer((uint8_t)address);
}

/**
  * @brief 发送写使能命令
  * @return W25Q128JV_OK 表示命令已发送
  * @note 用法：页编程、扇区擦除、块擦除、整片擦除前必须先调用
  */
static W25Q128JV_Status_e w25q128jv_write_enable(void)
{
    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_WRITE_ENABLE);
    w25q128jv_cs_high();
    return W25Q128JV_OK;
}

/**
  * @brief 读取指定状态寄存器
  * @param cmd 状态寄存器读取命令，例如 0x05、0x35、0x15
  * @param value 输出状态寄存器值
  * @return W25Q128JV_OK 表示读取成功，W25Q128JV_ERR_NULL 表示输出指针为空
  * @note 用法：对外的 ReadStatus1/2/3 都调用本函数实现
  */
static W25Q128JV_Status_e w25q128jv_read_status(uint8_t cmd, uint8_t *value)
{
    if (value == NULL)
    {
        return W25Q128JV_ERR_NULL;
    }

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(cmd);
    *value = w25q128jv_spi_xfer(0xFFU);
    w25q128jv_cs_high();
    return W25Q128JV_OK;
}

/**
  * @brief 等待状态寄存器 1 的 WIP 位清零
  * @param timeout_ms 超时时间，单位 ms
  * @return W25Q128JV_OK 表示 Flash 空闲，W25Q128JV_ERR_TIMEOUT 表示等待超时
  * @note 用法：写入或擦除后等待芯片内部操作完成，避免下一条命令过早发送
  */
static W25Q128JV_Status_e w25q128jv_wait_wip_clear(uint32_t timeout_ms)
{
    const float start_ms = DWT_GetTimeline_ms();
    uint8_t status = 0U;

    do
    {
        if (w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS1, &status) != W25Q128JV_OK)
        {
            return W25Q128JV_ERR_TIMEOUT;
        }
        if ((status & 0x01U) == 0U)
        {
            return W25Q128JV_OK;
        }
    } while ((DWT_GetTimeline_ms() - start_ms) < (float)timeout_ms);

    return W25Q128JV_ERR_TIMEOUT;
}

/**
  * @brief 初始化 W25Q128JV SPI 片选状态
  * @return 无
  * @note 用法：系统初始化后调用一次，保证 CS 处于高电平空闲状态
  */
void W25Q128JV_SpiInit(void)
{
    w25q128jv_cs_high();
}

/**
  * @brief 读取并校验 JEDEC ID
  * @param id 输出 JEDEC ID
  * @return W25Q128JV_OK 表示 ID 正确，其他值表示空指针或型号不匹配
  * @note 25Q128JVSQ 期望读到 manufacturer_id=0xEF，device_id=0x4018
  */
W25Q128JV_Status_e W25Q128JV_ReadJedecId(W25Q128JV_JedecId_s *id)
{
    uint8_t raw[3] = {0};

    if (id == NULL)
    {
        return W25Q128JV_ERR_NULL;
    }

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_READ_JEDEC_ID);
    raw[0] = w25q128jv_spi_xfer(0xFFU);
    raw[1] = w25q128jv_spi_xfer(0xFFU);
    raw[2] = w25q128jv_spi_xfer(0xFFU);
    w25q128jv_cs_high();

    id->manufacturer_id = raw[0];
    id->device_id = (uint16_t)raw[1] << 8;
    id->device_id |= raw[2];

    if ((id->manufacturer_id != W25Q128JV_MANUFACTURER_ID) ||
        (id->device_id != W25Q128JV_DEVICE_ID))
    {
        return W25Q128JV_ERR_ID_MISMATCH;
    }

    return W25Q128JV_OK;
}

/**
  * @brief 读取状态寄存器 1
  * @param value 输出状态寄存器 1 的值
  * @return W25Q128JV_OK 表示读取成功
  * @note 状态寄存器 1 的 bit0 为 WIP，bit1 为 WEL
  */
W25Q128JV_Status_e W25Q128JV_ReadStatus1(uint8_t *value)
{
    return w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS1, value);
}

/**
  * @brief 读取状态寄存器 2
  * @param value 输出状态寄存器 2 的值
  * @return W25Q128JV_OK 表示读取成功
  * @note 可用于检查 QE 等配置位
  */
W25Q128JV_Status_e W25Q128JV_ReadStatus2(uint8_t *value)
{
    return w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS2, value);
}

/**
  * @brief 读取状态寄存器 3
  * @param value 输出状态寄存器 3 的值
  * @return W25Q128JV_OK 表示读取成功
  */
W25Q128JV_Status_e W25Q128JV_ReadStatus3(uint8_t *value)
{
    return w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS3, value);
}

/**
  * @brief 等待 Flash 空闲
  * @param timeout_ms 超时时间，单位 ms
  * @return W25Q128JV_OK 表示 Flash 已空闲，W25Q128JV_ERR_TIMEOUT 表示超时
  * @note 对外封装 WIP 轮询，供上层在必要时主动等待
  */
W25Q128JV_Status_e W25Q128JV_WaitReady(uint32_t timeout_ms)
{
    return w25q128jv_wait_wip_clear(timeout_ms);
}

/**
  * @brief 从 Flash 读取任意长度数据
  * @param address 起始地址，范围 0 ~ W25Q128JV_TOTAL_SIZE-1
  * @param buf 读取数据输出缓冲区
  * @param len 读取长度，单位字节
  * @return W25Q128JV_OK 表示读取成功，其他值表示参数错误或等待超时
  * @note 本函数发送 0x03 普通读命令，不需要写使能，不会修改 Flash 内容
  */
W25Q128JV_Status_e W25Q128JV_ReadData(uint32_t address, uint8_t *buf, uint32_t len)
{
    uint32_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return W25Q128JV_ERR_NULL;
    }

    if ((address >= W25Q128JV_TOTAL_SIZE) || (len > (W25Q128JV_TOTAL_SIZE - address)))
    {
        return W25Q128JV_ERR_PARAM;
    }

    if (w25q128jv_wait_wip_clear(200U) != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_READ_DATA);
    w25q128jv_send_addr24(address);
    for (i = 0U; i < len; i++)
    {
        buf[i] = w25q128jv_spi_xfer(0xFFU);
    }
    w25q128jv_cs_high();
    return W25Q128JV_OK;
}

/**
  * @brief 单页编程
  * @param address 写入起始地址
  * @param buf 待写入数据缓冲区
  * @param len 写入长度，不能跨越 256 字节页边界
  * @return W25Q128JV_OK 表示写入成功，其他值表示参数错误或写入超时
  * @note 写入前需要对应区域已经擦除；Flash 编程只能把 1 写成 0
  */
W25Q128JV_Status_e W25Q128JV_PageProgram(uint32_t address, const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t chunk;

    if ((buf == NULL) || (len == 0U))
    {
        return W25Q128JV_ERR_NULL;
    }

    if ((address >= W25Q128JV_TOTAL_SIZE) || (len > (W25Q128JV_TOTAL_SIZE - address)))
    {
        return W25Q128JV_ERR_PARAM;
    }

    chunk = W25Q128JV_PAGE_SIZE - (address % W25Q128JV_PAGE_SIZE);
    if (len > chunk)
    {
        return W25Q128JV_ERR_PARAM;
    }

    if (chunk > len)
    {
        chunk = len;
    }

    if (w25q128jv_wait_wip_clear(200U) != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    if (w25q128jv_write_enable() != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_PAGE_PROGRAM);
    w25q128jv_send_addr24(address);
    for (i = 0U; i < chunk; i++)
    {
        w25q128jv_spi_xfer(buf[i]);
    }
    w25q128jv_cs_high();

    return w25q128jv_wait_wip_clear(15U);
}

/**
  * @brief 跨页连续写入数据
  * @param address 写入起始地址
  * @param buf 待写入数据缓冲区
  * @param len 写入长度，单位字节
  * @return W25Q128JV_OK 表示写入成功，其他值表示参数错误或写入失败
  * @note 内部按页边界拆分，多次调用 W25Q128JV_PageProgram()
  */
W25Q128JV_Status_e W25Q128JV_WriteBuffer(uint32_t address, const uint8_t *buf, uint32_t len)
{
    W25Q128JV_Status_e ret;
    uint32_t offset = 0U;

    if ((buf == NULL) || (len == 0U))
    {
        return W25Q128JV_ERR_NULL;
    }

    if ((address >= W25Q128JV_TOTAL_SIZE) || (len > (W25Q128JV_TOTAL_SIZE - address)))
    {
        return W25Q128JV_ERR_PARAM;
    }

    while (offset < len)
    {
        uint32_t page_left = W25Q128JV_PAGE_SIZE - ((address + offset) % W25Q128JV_PAGE_SIZE);
        uint32_t chunk = len - offset;

        if (chunk > page_left)
        {
            chunk = page_left;
        }

        ret = W25Q128JV_PageProgram(address + offset, &buf[offset], chunk);
        if (ret != W25Q128JV_OK)
        {
            return ret;
        }

        offset += chunk;
    }

    return W25Q128JV_OK;
}

/**
  * @brief 擦除公共实现
  * @param cmd 擦除命令
  * @param address 擦除起始地址，整片擦除时该参数无效
  * @param timeout_ms 等待擦除完成的超时时间，单位 ms
  * @param mask 地址对齐掩码
  * @return W25Q128JV_OK 表示擦除成功，其他值表示地址未对齐或擦除超时
  * @note 4K、32K、64K 和整片擦除都通过本函数或相同流程实现
  */
static W25Q128JV_Status_e w25q128jv_erase_common(uint8_t cmd, uint32_t address, uint32_t timeout_ms, uint32_t mask)
{
    if ((address & mask) != 0U)
    {
        return W25Q128JV_ERR_PARAM;
    }

    if (w25q128jv_wait_wip_clear(200U) != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    if (w25q128jv_write_enable() != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(cmd);
    if (cmd != W25Q128JV_CMD_CHIP_ERASE1)
    {
        w25q128jv_send_addr24(address);
    }
    w25q128jv_cs_high();

    return w25q128jv_wait_wip_clear(timeout_ms);
}

/**
  * @brief 擦除 4KB 扇区
  * @param address 扇区起始地址，必须 4KB 对齐
  * @return W25Q128JV_OK 表示擦除成功
  * @note 参数双槽机制使用 4KB 扇区作为一个槽
  */
W25Q128JV_Status_e W25Q128JV_Erase4K(uint32_t address)
{
    return w25q128jv_erase_common(W25Q128JV_CMD_SECTOR_ERASE_4K,
                                  address,
                                  400U,
                                  W25Q128JV_SECTOR_SIZE - 1U);
}

/**
  * @brief 擦除 32KB 块
  * @param address 块起始地址，必须 32KB 对齐
  * @return W25Q128JV_OK 表示擦除成功
  * @note 适合清理较大连续区域，比逐个 4KB 扇区擦除更快
  */
W25Q128JV_Status_e W25Q128JV_Erase32K(uint32_t address)
{
    return w25q128jv_erase_common(W25Q128JV_CMD_BLOCK_ERASE_32K,
                                  address,
                                  1600U,
                                  W25Q128JV_BLOCK32_SIZE - 1U);
}

/**
  * @brief 擦除 64KB 块
  * @param address 块起始地址，必须 64KB 对齐
  * @return W25Q128JV_OK 表示擦除成功
  * @note 适合清理较大连续区域，比 32KB 或 4KB 擦除更快
  */
W25Q128JV_Status_e W25Q128JV_Erase64K(uint32_t address)
{
    return w25q128jv_erase_common(W25Q128JV_CMD_BLOCK_ERASE_64K,
                                  address,
                                  2000U,
                                  W25Q128JV_BLOCK64_SIZE - 1U);
}

/**
  * @brief 整片擦除
  * @return W25Q128JV_OK 表示整片擦除成功
  * @note 会清空整颗 Flash，包含参数槽，正常调参和比赛运行禁止调用
  */
W25Q128JV_Status_e W25Q128JV_ChipErase(void)
{
    if (w25q128jv_wait_wip_clear(200U) != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    if (w25q128jv_write_enable() != W25Q128JV_OK)
    {
        return W25Q128JV_ERR_TIMEOUT;
    }

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_CHIP_ERASE1);
    w25q128jv_cs_high();

    return w25q128jv_wait_wip_clear(200000U);
}

/**
  * @brief 软件复位 Flash
  * @return W25Q128JV_OK 表示复位命令已发送
  * @note 依次发送 0x66 和 0x99，复位后延时等待芯片恢复
  */
W25Q128JV_Status_e W25Q128JV_Reset(void)
{
    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_ENABLE_RESET);
    w25q128jv_cs_high();

    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_RESET);
    w25q128jv_cs_high();

    DWT_Delay(0.00003f);
    return W25Q128JV_OK;
}

/**
  * @brief 运行 Flash 芯片基础自测
  * @return W25Q128JV_OK 表示 JEDEC、擦除、写入、读取校验全部通过
  * @note 会擦除最后一个 4KB 扇区 0xFFF000，该扇区已作为参数 slot1，正常启动禁止调用
  */
W25Q128JV_Status_e W25Q128JV_RunSelfTest(void)
{
    static uint8_t write_buf[W25Q128JV_PAGE_SIZE];
    static uint8_t read_buf[W25Q128JV_PAGE_SIZE];
    const uint32_t test_addr = W25Q128JV_TOTAL_SIZE - W25Q128JV_SECTOR_SIZE;
    W25Q128JV_JedecId_s id;
    W25Q128JV_Status_e ret;
    uint32_t i;

    LOGINFO("[flash] self test start, test sector=0x%06X", test_addr);

    W25Q128JV_SpiInit();
    DWT_Delay(0.005f);

    ret = W25Q128JV_Reset();
    if (ret != W25Q128JV_OK)
    {
        LOGERROR("[flash] reset failed, ret=%d", ret);
        return ret;
    }

    ret = W25Q128JV_ReadJedecId(&id);
    if (ret != W25Q128JV_OK)
    {
        LOGERROR("[flash] JEDEC failed, mf=0x%02X, dev=0x%04X, ret=%d",
                 id.manufacturer_id,
                 id.device_id,
                 ret);
        return ret;
    }
    LOGINFO("[flash] JEDEC OK, mf=0x%02X, dev=0x%04X", id.manufacturer_id, id.device_id);

    /* 自检使用最后一个 4KB 扇区，测试会擦除该扇区原内容。 */
    ret = W25Q128JV_Erase4K(test_addr);
    if (ret != W25Q128JV_OK)
    {
        LOGERROR("[flash] erase test sector failed, ret=%d", ret);
        return ret;
    }

    ret = W25Q128JV_ReadData(test_addr, read_buf, sizeof(read_buf));
    if (ret != W25Q128JV_OK)
    {
        LOGERROR("[flash] erase verify read failed, ret=%d", ret);
        return ret;
    }
    for (i = 0U; i < sizeof(read_buf); i++)
    {
        if (read_buf[i] != 0xFFU)
        {
            LOGERROR("[flash] erase verify failed, offset=%u, value=0x%02X", i, read_buf[i]);
            return W25Q128JV_ERR_PARAM;
        }
    }
    LOGINFO("[flash] erase verify OK");

    for (i = 0U; i < sizeof(write_buf); i++)
    {
        write_buf[i] = (uint8_t)(0xA5U ^ (uint8_t)i);
    }

    ret = W25Q128JV_PageProgram(test_addr, write_buf, sizeof(write_buf));
    if (ret != W25Q128JV_OK)
    {
        LOGERROR("[flash] page program failed, ret=%d", ret);
        return ret;
    }

    ret = W25Q128JV_ReadData(test_addr, read_buf, sizeof(read_buf));
    if (ret != W25Q128JV_OK)
    {
        LOGERROR("[flash] readback failed, ret=%d", ret);
        return ret;
    }
    for (i = 0U; i < sizeof(write_buf); i++)
    {
        if (read_buf[i] != write_buf[i])
        {
            LOGERROR("[flash] verify failed, offset=%u, wr=0x%02X, rd=0x%02X",
                     i,
                     write_buf[i],
                     read_buf[i]);
            return W25Q128JV_ERR_PARAM;
        }
    }
    LOGINFO("[flash] program/read verify OK");

    LOGINFO("[flash] self test PASS");
    return W25Q128JV_OK;
}
