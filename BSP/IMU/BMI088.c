#include "BMI088.h"
#include "dwt.h"

#define BMI088DelayMs(_nms)  DWT_Delay(_nms * 0.001)

/**
 * @brief TI MSPM0 硬件 SPI 单字节收发
 */
static uint8_t spi_read_write_byte(uint8_t dat)
{
    uint8_t data = 0;
    DL_SPI_transmitData8(BMI088_SPI_INST, dat);
    while (DL_SPI_isBusy(BMI088_SPI_INST));
    data = DL_SPI_receiveData8(BMI088_SPI_INST);
    while (DL_SPI_isBusy(BMI088_SPI_INST));
    return data;
}

/* ---- 加速度计读写 (带 1 字节 Dummy) ---- */
static uint8_t bmi088_acc_read_reg(uint8_t reg)
{
    uint8_t regval;
    BMI088_ACC_CS(0);
    spi_read_write_byte(reg | 0x80);  // 地址字节, MISO 为高阻态 → 丢弃
    regval = spi_read_write_byte(0x00); // dummy 字节, MISO 返回 REG[addr] → 保留!
    BMI088_ACC_CS(1);
    return regval;
}

static void bmi088_acc_write_reg(uint8_t reg, uint8_t value)
{
    BMI088_ACC_CS(0);
    spi_read_write_byte(reg & 0x7F);
    spi_read_write_byte(value);
    BMI088_ACC_CS(1);
}

/* ---- 陀螺仪读写 (不带 Dummy) ---- */
static uint8_t bmi088_gyro_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    BMI088_GYRO_CS(0);
    spi_read_write_byte(reg | 0x80);
    regval = spi_read_write_byte(0xFF);
    BMI088_GYRO_CS(1);
    return regval;
}

static void bmi088_gyro_write_reg(uint8_t reg, uint8_t value)
{
    BMI088_GYRO_CS(0);
    spi_read_write_byte(reg & 0x7F);
    spi_read_write_byte(value);
    BMI088_GYRO_CS(1);
}

/**
 * @brief BMI088 初始化
 */

/* 加速度计错误状态标志 (初始化时检查一次) */
static uint8_t bmi088_acc_error_flag = 0;

int8_t bsp_Bmi088Init(void)
{
    uint8_t reg_val;

    BMI088_ACC_CS(1);
    BMI088_GYRO_CS(1);
    BMI088DelayMs(50);

    /* ── 1. 加速度计软复位 ── */
    bmi088_acc_write_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088DelayMs(10);  /* 复位后等待 10ms */

    /* ── 2. 陀螺仪软复位 ── */
    bmi088_gyro_write_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088DelayMs(30);  /* 陀螺仪复位后需等待 30ms */

    /* ── 3. 加速度计 SPI 模式确认：读两次 CHIP_ID
     *    第一次读触发内部状态机切换，第二次读验证 ── */
    bmi088_acc_read_reg(BMI088_ACC_CHIP_ID);           /* 触发 SPI 模式 */
    BMI088DelayMs(1);
    reg_val = bmi088_acc_read_reg(BMI088_ACC_CHIP_ID); /* 正式读取 */
    if (reg_val != BMI088_ACC_CHIP_ID_VALUE)
    {
        return -2;  /* 加速度计 CHIP_ID 异常 (期望 0x1E) */
    }

    /* ── 4. 校验陀螺仪 CHIP_ID (应为 0x0F) ── */
    reg_val = bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID);
    if (reg_val != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return -1;  /* 陀螺仪 CHIP_ID 异常 (期望 0x0F) */
    }

    /* ── 5. 配置加速度计 (顺序: RANGE → CONF → PWR_CONF → PWR_CTRL)
     *    注意: PWR_CTRL 必须最后写入，否则传感器以未定义状态启动 ── */
    bmi088_acc_write_reg(BMI088_ACC_RANGE,    0x02); /* ±12g */
    bmi088_acc_write_reg(BMI088_ACC_CONF,     0x8C); /* ODR=1600Hz, OSR2 */
    bmi088_acc_write_reg(BMI088_ACC_PWR_CONF, 0x00); /* Active 模式 */
    BMI088DelayMs(1);                                  /* 模式切换稳定时间 */
    bmi088_acc_write_reg(BMI088_ACC_PWR_CTRL, 0x04); /* ★ 最后使能加速度计 */

    /* ── 6. 配置陀螺仪 ── */
    bmi088_gyro_write_reg(BMI088_GYRO_RANGE,     0x00); /* ±2000dps */
    bmi088_gyro_write_reg(BMI088_GYRO_BANDWIDTH, 0x81); /* ODR=2000Hz, BW=230Hz, bit7 MUST be 1 */
    bmi088_gyro_write_reg(BMI088_GYRO_LPM1,      0x00); /* Normal mode */

    BMI088DelayMs(30);  /* 等待传感器稳定输出数据 */

    /* ── 7. 检查加速度计致命错误标志 ── */
    if (bmi088_acc_read_reg(BMI088_ACC_ERR_REG) & 0x01)
    {
        bmi088_acc_error_flag = 1;
        return -3;  /* 加速度计致命错误 */
    }

    return 0;
}

/**
 * @brief 获取 BMI088 转换后的真实数据
 */
void bsp_Bmi088GetRawData(bmi088RealData_t* accData, bmi088RealData_t* GyroData)
{
    uint8_t i;
    uint8_t accBuf[6];
    uint8_t gyroBuf[6];

    int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;

    /* 初始化时已检测到传感器致命错误，直接返回 0 */
    if (bmi088_acc_error_flag)
    {
        accData->x = accData->y = accData->z = 0.0f;
        GyroData->x = GyroData->y = GyroData->z = 0.0f;
        return;
    }

    /* 连续读加速度 6 字节
     * BMI088 加速度计 SPI 协议: 地址字节后第一个 dummy 返回寄存器数据
     * 后续 dummy 字节依次返回地址递增的寄存器数据 */
    BMI088_ACC_CS(0);
    spi_read_write_byte(BMI088_ACC_X_LSB | 0x80);  /* 发地址, SDO 返回的是上一次读的数据 → 丢弃 */
    for (i = 0; i < 6; i++)
    {
        accBuf[i] = spi_read_write_byte(0x00);
    }
    BMI088_ACC_CS(1);

    /* 连续读陀螺仪 6 字节
     * BMI088 陀螺仪 SPI 协议: 地址字节后立即返回数据 (不需要 dummy) */
    BMI088_GYRO_CS(0);
    spi_read_write_byte(BMI088_GYRO_RATE_X_LSB | 0x80);
    for (i = 0; i < 6; i++)
    {
        gyroBuf[i] = spi_read_write_byte(0xFF);
    }
    BMI088_GYRO_CS(1);

    /* BMI088 小端 (Little-Endian) 转换 */
    rawAccX = (int16_t)((uint16_t)accBuf[0] | ((uint16_t)accBuf[1] << 8));
    rawAccY = (int16_t)((uint16_t)accBuf[2] | ((uint16_t)accBuf[3] << 8));
    rawAccZ = (int16_t)((uint16_t)accBuf[4] | ((uint16_t)accBuf[5] << 8));

    rawGyroX = (int16_t)((uint16_t)gyroBuf[0] | ((uint16_t)gyroBuf[1] << 8));
    rawGyroY = (int16_t)((uint16_t)gyroBuf[2] | ((uint16_t)gyroBuf[3] << 8));
    rawGyroZ = (int16_t)((uint16_t)gyroBuf[4] | ((uint16_t)gyroBuf[5] << 8));

    /* 单位换算 */
    accData->x = (float)rawAccX * BMI088_ACCEL_SENSITIVITY_G;
    accData->y = (float)rawAccY * BMI088_ACCEL_SENSITIVITY_G;
    accData->z = (float)rawAccZ * BMI088_ACCEL_SENSITIVITY_G;

    GyroData->x = (float)rawGyroX * BMI088_GYRO_SENSITIVITY_DPS;
    GyroData->y = (float)rawGyroY * BMI088_GYRO_SENSITIVITY_DPS;
    GyroData->z = (float)rawGyroZ * BMI088_GYRO_SENSITIVITY_DPS;
}