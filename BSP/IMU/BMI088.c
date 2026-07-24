#include "BMI088.h"
#include "dwt.h"

#define BMI088DelayMs(_nms)  DWT_Delay(_nms * 0.001)

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
    uint8_t regval = 0;
    BMI088_ACC_CS(0);
    spi_read_write_byte(reg | 0x80);
    spi_read_write_byte(0x00); // dummy byte
    regval = spi_read_write_byte(0xFF);
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
int8_t bsp_Bmi088Init(void)
{
    BMI088_ACC_CS(1);
    BMI088_GYRO_CS(1);
    BMI088DelayMs(50);

    /* 1. 加速度计假读，触发硬件切到 SPI 模式 */
    BMI088_ACC_CS(0);
    spi_read_write_byte(BMI088_ACC_CHIP_ID | 0x80);
    spi_read_write_byte(0x00);
    spi_read_write_byte(0x00);
    BMI088_ACC_CS(1);

    /* 2. 校验陀螺仪 CHIP_ID (应为 0x0F) */
    if (bmi088_gyro_read_reg(BMI088_GYRO_CHIP_ID) != 0x0F)
    {
        return -1;
    }

    /* 3. 配置加速度计 */
    bmi088_acc_write_reg(BMI088_ACC_PWR_CTRL, 0x04); /* 使能加速度计 */
    BMI088DelayMs(1);
    bmi088_acc_write_reg(BMI088_ACC_PWR_CONF, 0x00); /* 开启 Active 模式 */
    bmi088_acc_write_reg(BMI088_ACC_CONF,     0x8C); /* ODR=1600Hz */
    bmi088_acc_write_reg(BMI088_ACC_RANGE,    0x02); /* ±12g */

    /* 4. 配置陀螺仪 */
    bmi088_gyro_write_reg(BMI088_GYRO_RANGE,     0x00); /* ±2000dps */
    bmi088_gyro_write_reg(BMI088_GYRO_BANDWIDTH, 0x01); /* ODR=2000Hz, BW=216Hz */

    BMI088DelayMs(30);
    return 0;
}

/**
 * @brief 获取 BMI088 转换后的真实数据
 */
int8_t bsp_Bmi088GetRawData(bmi088RealData_t* accData, bmi088RealData_t* GyroData)
{
    uint8_t i;
    uint8_t accBuf[6];
    uint8_t gyroBuf[6];

    int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;

    /* 连续读加速度 6 字节 */
    BMI088_ACC_CS(0);
    spi_read_write_byte(BMI088_ACC_X_LSB | 0x80);
    spi_read_write_byte(0x00); // dummy byte
    for (i = 0; i < 6; i++)
    {
        accBuf[i] = spi_read_write_byte(0xFF);
    }
    BMI088_ACC_CS(1);

    /* 连续读陀螺仪 6 字节 */
    BMI088_GYRO_CS(0);
    spi_read_write_byte(BMI088_GYRO_RATE_X_LSB | 0x80);
    for (i = 0; i < 6; i++)
    {
        gyroBuf[i] = spi_read_write_byte(0xFF);
    }
    BMI088_GYRO_CS(1);

    /* BMI088 小端转换 */
    rawAccX = (int16_t)(accBuf[0] | (accBuf[1] << 8));
    rawAccY = (int16_t)(accBuf[2] | (accBuf[3] << 8));
    rawAccZ = (int16_t)(accBuf[4] | (accBuf[5] << 8));

    rawGyroX = (int16_t)(gyroBuf[0] | (gyroBuf[1] << 8));
    rawGyroY = (int16_t)(gyroBuf[2] | (gyroBuf[3] << 8));
    rawGyroZ = (int16_t)(gyroBuf[4] | (gyroBuf[5] << 8));

    /* 单位换算 (转换为 g 和 deg/s) */
    accData->x = (float)(rawAccX * BMI088_ACCEL_SENSITIVITY_G);
    accData->y = (float)(rawAccY * BMI088_ACCEL_SENSITIVITY_G);
    accData->z = (float)(rawAccZ * BMI088_ACCEL_SENSITIVITY_G);

    GyroData->x = (float)(rawGyroX * BMI088_GYRO_SENSITIVITY_DPS);
    GyroData->y = (float)(rawGyroY * BMI088_GYRO_SENSITIVITY_DPS);
    GyroData->z = (float)(rawGyroZ * BMI088_GYRO_SENSITIVITY_DPS);

    return 0;
}