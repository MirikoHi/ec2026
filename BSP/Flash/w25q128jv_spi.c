#include "w25q128jv_spi.h"

#include "bsp_log.h"
#include "dwt.h"

/* 这个版本按当前工程的 SPI1 实现，不依赖 STM32 QSPI 外设。 */
#define W25Q128JV_SPI_INST          SPI_FLASH_INST
#define W25Q128JV_SPI_CS_PORT       SPI_FLASH_CS_PORT
#define W25Q128JV_SPI_CS_PIN        SPI_FLASH_CS_CSN_PIN

static uint8_t w25q128jv_spi_xfer(uint8_t tx)
{
    DL_SPI_transmitData8(W25Q128JV_SPI_INST, tx);
    while (DL_SPI_isBusy(W25Q128JV_SPI_INST))
    {
    }
    return DL_SPI_receiveData8(W25Q128JV_SPI_INST);
}

static void w25q128jv_cs_low(void)
{
    DL_GPIO_clearPins(W25Q128JV_SPI_CS_PORT, W25Q128JV_SPI_CS_PIN);
}

static void w25q128jv_cs_high(void)
{
    DL_GPIO_setPins(W25Q128JV_SPI_CS_PORT, W25Q128JV_SPI_CS_PIN);
}

static void w25q128jv_send_addr24(uint32_t address)
{
    w25q128jv_spi_xfer((uint8_t)(address >> 16));
    w25q128jv_spi_xfer((uint8_t)(address >> 8));
    w25q128jv_spi_xfer((uint8_t)address);
}

static W25Q128JV_Status_e w25q128jv_write_enable(void)
{
    w25q128jv_cs_low();
    w25q128jv_spi_xfer(W25Q128JV_CMD_WRITE_ENABLE);
    w25q128jv_cs_high();
    return W25Q128JV_OK;
}

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

void W25Q128JV_SpiInit(void)
{
    w25q128jv_cs_high();
}

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

W25Q128JV_Status_e W25Q128JV_ReadStatus1(uint8_t *value)
{
    return w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS1, value);
}

W25Q128JV_Status_e W25Q128JV_ReadStatus2(uint8_t *value)
{
    return w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS2, value);
}

W25Q128JV_Status_e W25Q128JV_ReadStatus3(uint8_t *value)
{
    return w25q128jv_read_status(W25Q128JV_CMD_READ_STATUS3, value);
}

W25Q128JV_Status_e W25Q128JV_WaitReady(uint32_t timeout_ms)
{
    return w25q128jv_wait_wip_clear(timeout_ms);
}

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

W25Q128JV_Status_e W25Q128JV_Erase4K(uint32_t address)
{
    return w25q128jv_erase_common(W25Q128JV_CMD_SECTOR_ERASE_4K,
                                  address,
                                  400U,
                                  W25Q128JV_SECTOR_SIZE - 1U);
}

W25Q128JV_Status_e W25Q128JV_Erase32K(uint32_t address)
{
    return w25q128jv_erase_common(W25Q128JV_CMD_BLOCK_ERASE_32K,
                                  address,
                                  1600U,
                                  W25Q128JV_BLOCK32_SIZE - 1U);
}

W25Q128JV_Status_e W25Q128JV_Erase64K(uint32_t address)
{
    return w25q128jv_erase_common(W25Q128JV_CMD_BLOCK_ERASE_64K,
                                  address,
                                  2000U,
                                  W25Q128JV_BLOCK64_SIZE - 1U);
}

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
