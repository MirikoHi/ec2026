/**
 ******************************************************************************
 * @file    icm42688.c
 * @author  Geng LX (ported from Legacy verified driver)
 * @brief   ICM42688 6-axis IMU SPI driver for MSPM0 platform
 *
 * @note    Ported from Legacy/icm42688.c (STM32 HAL) to MSPM0 DL SPI API.
 *          Fixed: signedness bug, unit conversions (accel→m/s², gyro→rad/s),
 *          bank switching, register configuration.
 ******************************************************************************
 */

#include "icm42688.h"
#include "dwt.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Static / Module-level Variables
 * ═══════════════════════════════════════════════════════════════════════ */

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

/* ═══════════════════════════════════════════════════════════════════════
 * SPI Low-level Helpers (MSPM0 DL API)
 * ═══════════════════════════════════════════════════════════════════════ */

/** CS pin control */
#define ICM_CS_LOW()   DL_GPIO_clearPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN)
#define ICM_CS_HIGH()  DL_GPIO_setPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN)

/** Blocking SPI byte exchange */
static uint8_t spi_transfer(uint8_t tx)
{
    uint8_t rx;
    DL_SPI_transmitData8(ICM42688_INST, tx);
    while (DL_SPI_isBusy(ICM42688_INST));
    rx = DL_SPI_receiveData8(ICM42688_INST);
    while (DL_SPI_isBusy(ICM42688_INST));
    return rx;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Bank Switching
 *
 * ICM42688 has multiple register banks. You MUST switch banks before
 * accessing registers in non-zero banks. REG_BANK_SEL (0x76) controls this.
 * ═══════════════════════════════════════════════════════════════════════ */

static void icm_select_bank(uint8_t bank)
{
    ICM_CS_LOW();
    spi_transfer(0x76);              // REG_BANK_SEL
    spi_transfer(bank & 0x07);       // only bits [2:0] used
    ICM_CS_HIGH();
}

/* ═══════════════════════════════════════════════════════════════════════
 * Register Read / Write
 * ═══════════════════════════════════════════════════════════════════════ */

static void icm_write_reg(uint8_t reg, uint8_t value)
{
    ICM_CS_LOW();
    spi_transfer(reg & 0x7F);        // bit7=0: write
    spi_transfer(value);
    ICM_CS_HIGH();
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t val;
    ICM_CS_LOW();
    spi_transfer(reg | 0x80);        // bit7=1: read
    val = spi_transfer(0x00);        // dummy byte to clock out data
    ICM_CS_HIGH();
    return val;
}

/** Multi-byte burst read */
static void icm_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM_CS_LOW();
    spi_transfer(reg | 0x80);        // bit7=1: read
    while (len--) {
        *buf++ = spi_transfer(0x00);
    }
    ICM_CS_HIGH();
}

/* ═══════════════════════════════════════════════════════════════════════
 * Data Conversion Helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/** Big-endian two-byte → signed int16 */
static inline int16_t be16_to_i16(const uint8_t *b)
{
    return (int16_t)((b[0] << 8) | b[1]);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Raw Data Parsing
 *
 * buf layout (14 bytes):
 *   [ 0: 1] TEMP_DATA1:TEMP_DATA0
 *   [ 2: 3] ACCEL_DATA_X1:X0
 *   [ 4: 5] ACCEL_DATA_Y1:Y0
 *   [ 6: 7] ACCEL_DATA_Z1:Z0
 *   [ 8: 9] GYRO_DATA_X1:X0
 *   [10:11] GYRO_DATA_Y1:Y0
 *   [12:13] GYRO_DATA_Z1:Z0
 *
 * Output:
 *   acc[0..2] — acceleration [m/s²]
 *   gyro[0..2] — angular velocity [rad/s]
 *   temp       — temperature [°C] (can be NULL)
 * ═══════════════════════════════════════════════════════════════════════ */

static void icm42688_parse(const uint8_t *buf, float *acc, float *gyro, float *temp)
{
    if (temp) {
        *temp = (float)be16_to_i16(&buf[0]) * 0.007548309f + 25.0f;
    }
    /* Accel: raw * (16/32768) [g] * 9.8 [m/s² per g] = [m/s²] */
    acc[0]  = (float)be16_to_i16(&buf[2])  * ICM_ACCEL_SENSITIVITY_16G * g_gravity;
    acc[1]  = (float)be16_to_i16(&buf[4])  * ICM_ACCEL_SENSITIVITY_16G * g_gravity;
    acc[2]  = (float)be16_to_i16(&buf[6])  * ICM_ACCEL_SENSITIVITY_16G * g_gravity;
    /* Gyro: raw * (2000/32768) [dps] * (pi/180) [rad/s per dps] = [rad/s] */
    gyro[0] = (float)be16_to_i16(&buf[8])  * ICM_GYRO_SENSITIVITY_2000 * pidivide180;
    gyro[1] = (float)be16_to_i16(&buf[10]) * ICM_GYRO_SENSITIVITY_2000 * pidivide180;
    gyro[2] = (float)be16_to_i16(&buf[12]) * ICM_GYRO_SENSITIVITY_2000 * pidivide180;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Gyro Bias Calibration
 *
 * Averages 500 samples while the robot is stationary.
 * Call once at startup.
 * ═══════════════════════════════════════════════════════════════════════ */

static void icm_calibrate_gyro(void)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint8_t buf[14];

    for (int i = 0; i < 500; i++) {
        icm_read_regs(ICM_BURST_START, buf, ICM_BURST_LEN);
        float acc[3], gyro[3];
        icm42688_parse(buf, acc, gyro, NULL);
        sum[0] += gyro[0];
        sum[1] += gyro[1];
        sum[2] += gyro[2];
        DWT_Delay(0.002f);  // 2ms between samples
    }
    gyro_bias[0] = sum[0] / 500.0f;
    gyro_bias[1] = sum[1] / 500.0f;
    gyro_bias[2] = sum[2] / 500.0f;
}

void icm42688_correct_gyro_bias(float *gyro)
{
    gyro[0] -= gyro_bias[0];
    gyro[1] -= gyro_bias[1];
    gyro[2] -= gyro_bias[2];
}

/* ═══════════════════════════════════════════════════════════════════════
 * Register Configuration
 * ═══════════════════════════════════════════════════════════════════════ */

static int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val;

    /* Check WHO_AM_I */
    reg_val = icm_read_reg(ICM42688_WHO_AM_I);
    if (reg_val != ICM42688_ID) {
        return -1;
    }

    /* ── Software reset ── */
    icm_write_reg(ICM42688_DEVICE_CONFIG, 0x01);
    DWT_Delay(0.05f);  // 50ms

    /* Re-check WHO_AM_I after reset */
    reg_val = icm_read_reg(ICM42688_WHO_AM_I);
    if (reg_val != ICM42688_ID) {
        return -1;
    }

    /* ── Bank 0: configure sensor ── */
    icm_select_bank(0);

    /* GYRO_CONFIG0: 2000dps, 1kHz ODR
     *   bits [7:5] = GFS_2000DPS (0b000)
     *   bits [3:0] = GODR_1000Hz (0b0110) */
    icm_write_reg(ICM42688_GYRO_CONFIG0, (GFS_2000DPS << 5) | GODR_1000Hz);

    /* ACCEL_CONFIG0: ±16g, 1kHz ODR
     *   bits [7:5] = AFS_16G (0b000)
     *   bits [3:0] = AODR_1000Hz (0b0110) */
    icm_write_reg(ICM42688_ACCEL_CONFIG0, (AFS_16G << 5) | AODR_1000Hz);

    /* PWR_MGMT0: Gyro LN mode + Accel LN mode + temp enabled
     *   bit 5  = 0 (TEMP_DIS = 0, temperature enabled)
     *   bits [3:2] = 0b11 (GYRO_MODE = Low Noise)
     *   bits [1:0] = 0b11 (ACCEL_MODE = Low Noise) */
    icm_write_reg(ICM42688_PWR_MGMT0, 0x0F);

    /* GYRO_ACCEL_CONFIG0: default anti-aliasing filter settings */
    icm_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, 0x55);

    DWT_Delay(0.03f);  // 30ms stabilization

    /* ── Gyro bias calibration ── */
    icm_calibrate_gyro();  // Uncomment to enable auto-calibration

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

int8_t bsp_Icm42688Init(void)
{
    return bsp_Icm42688RegCfg();
}

int8_t bsp_IcmGetTemperature(int16_t *pTemp)
{
    uint8_t buf[2];
    icm_read_regs(ICM42688_TEMP_DATA1, buf, 2);
    *pTemp = (int16_t)(((int16_t)((buf[0] << 8) | buf[1])) / 132.48f + 25.0f);
    return 0;
}

/**
 * @brief  Burst-read accelerometer + gyroscope data
 * @param  accData  [out] acceleration [m/s²]
 * @param  gyroData [out] angular velocity [rad/s]
 * @retval 0 = success
 */
int8_t bsp_IcmGetRawData(icm42688RealData_t *accData, icm42688RealData_t *gyroData)
{
    uint8_t buf[14];

    /* Burst read: TEMP(2) + ACCEL_XYZ(6) + GYRO_XYZ(6) = 14 bytes from 0x1D */
    icm_read_regs(ICM_BURST_START, buf, ICM_BURST_LEN);

    /* Parse: acc → m/s², gyro → rad/s */
    float acc[3], gyro[3];
    icm42688_parse(buf, acc, gyro, NULL);

    /* Apply gyro bias correction */
    icm42688_correct_gyro_bias(gyro);

    accData->x  = acc[0];
    accData->y  = acc[1];
    accData->z  = acc[2];
    gyroData->x = gyro[0];
    gyroData->y = gyro[1];
    gyroData->z = gyro[2];

    return 0;
}
