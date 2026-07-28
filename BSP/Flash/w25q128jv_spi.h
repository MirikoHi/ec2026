#ifndef __BSP_W25Q128JV_SPI_H__
#define __BSP_W25Q128JV_SPI_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

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

void W25Q128JV_SpiInit(void);
W25Q128JV_Status_e W25Q128JV_ReadJedecId(W25Q128JV_JedecId_s *id);
W25Q128JV_Status_e W25Q128JV_ReadStatus1(uint8_t *value);
W25Q128JV_Status_e W25Q128JV_ReadStatus2(uint8_t *value);
W25Q128JV_Status_e W25Q128JV_ReadStatus3(uint8_t *value);
W25Q128JV_Status_e W25Q128JV_WaitReady(uint32_t timeout_ms);
W25Q128JV_Status_e W25Q128JV_ReadData(uint32_t address, uint8_t *buf, uint32_t len);
W25Q128JV_Status_e W25Q128JV_PageProgram(uint32_t address, const uint8_t *buf, uint32_t len);
W25Q128JV_Status_e W25Q128JV_WriteBuffer(uint32_t address, const uint8_t *buf, uint32_t len);
W25Q128JV_Status_e W25Q128JV_Erase4K(uint32_t address);
W25Q128JV_Status_e W25Q128JV_Erase32K(uint32_t address);
W25Q128JV_Status_e W25Q128JV_Erase64K(uint32_t address);
W25Q128JV_Status_e W25Q128JV_ChipErase(void);
W25Q128JV_Status_e W25Q128JV_Reset(void);
/* 自测会擦除最后一个扇区 0xFFF000，该扇区已作为参数 slot1，禁止在正常启动中调用。 */
W25Q128JV_Status_e W25Q128JV_RunSelfTest(void);

#endif
