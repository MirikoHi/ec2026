#include "icm42688_improved.h"
#include "dwt.h"

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static uint8_t gyro_calibrated = 0U;

#define ICM_IMP_CS_LOW()   DL_GPIO_clearPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN)
#define ICM_IMP_CS_HIGH()  DL_GPIO_setPins(ICM42688_CS_PORT, ICM42688_CS_CS_PIN)

static uint8_t icm_imp_spi_transfer(uint8_t tx)
{
    uint8_t rx;

    DL_SPI_transmitData8(ICM42688_INST, tx);
    while (DL_SPI_isBusy(ICM42688_INST)) {
    }
    rx = DL_SPI_receiveData8(ICM42688_INST);
    while (DL_SPI_isBusy(ICM42688_INST)) {
    }

    return rx;
}

static void icm_imp_select_bank(uint8_t bank)
{
    ICM_IMP_CS_LOW();
    icm_imp_spi_transfer(ICM42688_IMP_REG_BANK_SEL & 0x7FU);
    icm_imp_spi_transfer(bank & 0x07U);
    ICM_IMP_CS_HIGH();
}

static void icm_imp_write_reg(uint8_t reg, uint8_t value)
{
    ICM_IMP_CS_LOW();
    icm_imp_spi_transfer(reg & 0x7FU);
    icm_imp_spi_transfer(value);
    ICM_IMP_CS_HIGH();
}

static uint8_t icm_imp_read_reg(uint8_t reg)
{
    uint8_t value;

    ICM_IMP_CS_LOW();
    icm_imp_spi_transfer(reg | 0x80U);
    value = icm_imp_spi_transfer(0x00U);
    ICM_IMP_CS_HIGH();

    return value;
}

static void icm_imp_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM_IMP_CS_LOW();
    icm_imp_spi_transfer(reg | 0x80U);
    while (len-- > 0U) {
        *buf++ = icm_imp_spi_transfer(0x00U);
    }
    ICM_IMP_CS_HIGH();
}

static int16_t icm_imp_be16_to_i16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static void icm_imp_parse_burst(const uint8_t *buf, float *acc, float *gyro, float *temp)
{
    if (temp != 0) {
        *temp = (float)icm_imp_be16_to_i16(&buf[0]) * 0.007548309f + 25.0f;
    }

    acc[0] = (float)icm_imp_be16_to_i16(&buf[2]) *
             ICM42688_IMP_ACCEL_SENS_4G * ICM42688_IMP_GRAVITY;
    acc[1] = (float)icm_imp_be16_to_i16(&buf[4]) *
             ICM42688_IMP_ACCEL_SENS_4G * ICM42688_IMP_GRAVITY;
    acc[2] = (float)icm_imp_be16_to_i16(&buf[6]) *
             ICM42688_IMP_ACCEL_SENS_4G * ICM42688_IMP_GRAVITY;

    gyro[0] = (float)icm_imp_be16_to_i16(&buf[8]) *
              ICM42688_IMP_GYRO_SENS_1000 * ICM42688_IMP_PI_DIV_180;
    gyro[1] = (float)icm_imp_be16_to_i16(&buf[10]) *
              ICM42688_IMP_GYRO_SENS_1000 * ICM42688_IMP_PI_DIV_180;
    gyro[2] = (float)icm_imp_be16_to_i16(&buf[12]) *
              ICM42688_IMP_GYRO_SENS_1000 * ICM42688_IMP_PI_DIV_180;
}

static void icm_imp_calibrate_gyro(void)
{
    enum {
        ICM_IMP_CAL_WARMUP_SAMPLES = 60,
        ICM_IMP_CAL_SAMPLES = 400
    };
    uint8_t buf[ICM42688_IMP_BURST_LEN];
    float acc[3];
    float gyro[3];
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float sum_sq[3] = {0.0f, 0.0f, 0.0f};
    float acc_norm_sum = 0.0f;
    float mean[3];
    float variance[3];

    gyro_calibrated = 0U;
    gyro_bias[0] = 0.0f;
    gyro_bias[1] = 0.0f;
    gyro_bias[2] = 0.0f;

    for (uint16_t i = 0U; i < ICM_IMP_CAL_WARMUP_SAMPLES; i++) {
        icm_imp_read_regs(ICM42688_IMP_BURST_START, buf, ICM42688_IMP_BURST_LEN);
        DWT_Delay(0.005f);
    }

    for (uint16_t i = 0U; i < ICM_IMP_CAL_SAMPLES; i++) {
        icm_imp_read_regs(ICM42688_IMP_BURST_START, buf, ICM42688_IMP_BURST_LEN);
        icm_imp_parse_burst(buf, acc, gyro, 0);
        sum[0] += gyro[0];
        sum[1] += gyro[1];
        sum[2] += gyro[2];
        sum_sq[0] += gyro[0] * gyro[0];
        sum_sq[1] += gyro[1] * gyro[1];
        sum_sq[2] += gyro[2] * gyro[2];
        acc_norm_sum += sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        DWT_Delay(0.005f);
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        mean[axis] = sum[axis] / (float)ICM_IMP_CAL_SAMPLES;
        variance[axis] = (sum_sq[axis] / (float)ICM_IMP_CAL_SAMPLES) -
                         (mean[axis] * mean[axis]);
        if (variance[axis] < 0.0f) {
            variance[axis] = 0.0f;
        }
    }

    const float acc_norm_avg = acc_norm_sum / (float)ICM_IMP_CAL_SAMPLES;
    const float gyro_var_limit = 0.000015f;
    const float acc_norm_err_limit = 0.8f;

    if ((variance[0] < gyro_var_limit) &&
        (variance[1] < gyro_var_limit) &&
        (variance[2] < gyro_var_limit) &&
        (fabsf(acc_norm_avg - ICM42688_IMP_GRAVITY) < acc_norm_err_limit)) {
        gyro_bias[0] = mean[0];
        gyro_bias[1] = mean[1];
        gyro_bias[2] = mean[2];
        gyro_calibrated = 1U;
    }
}

void ICM42688_Improved_CorrectGyroBias(float *gyro)
{
    gyro[0] -= gyro_bias[0];
    gyro[1] -= gyro_bias[1];
    gyro[2] -= gyro_bias[2];
}

uint8_t ICM42688_Improved_IsGyroCalibrated(void)
{
    return gyro_calibrated;
}

static int8_t icm_imp_config(void)
{
    uint8_t reg_value;

    reg_value = icm_imp_read_reg(ICM42688_IMP_WHO_AM_I);
    if (reg_value != ICM42688_IMP_ID) {
        return -1;
    }

    icm_imp_write_reg(ICM42688_IMP_DEVICE_CONFIG, 0x01U);
    DWT_Delay(0.05f);

    reg_value = icm_imp_read_reg(ICM42688_IMP_WHO_AM_I);
    if (reg_value != ICM42688_IMP_ID) {
        return -1;
    }

    icm_imp_select_bank(0U);
    icm_imp_write_reg(ICM42688_IMP_GYRO_CONFIG0,
                      (ICM42688_IMP_GFS_1000DPS << 5) | ICM42688_IMP_GODR_200HZ);
    icm_imp_write_reg(ICM42688_IMP_ACCEL_CONFIG0,
                      (ICM42688_IMP_AFS_4G << 5) | ICM42688_IMP_AODR_200HZ);
    icm_imp_write_reg(ICM42688_IMP_PWR_MGMT0, 0x0FU);
    icm_imp_write_reg(ICM42688_IMP_GYRO_ACCEL_CONFIG0, 0x55U);
    DWT_Delay(0.03f);

    icm_imp_calibrate_gyro();

    return 0;
}

int8_t ICM42688_Improved_Init(void)
{
    return icm_imp_config();
}

int8_t ICM42688_Improved_GetTemperature(int16_t *temp)
{
    uint8_t buf[2];

    icm_imp_read_regs(ICM42688_IMP_TEMP_DATA1, buf, 2U);
    *temp = (int16_t)(((float)icm_imp_be16_to_i16(buf)) / 132.48f + 25.0f);

    return 0;
}

int8_t ICM42688_Improved_GetRawData(icm42688_imp_data_t *acc,
                                    icm42688_imp_data_t *gyro)
{
    uint8_t buf[ICM42688_IMP_BURST_LEN];
    float acc_raw[3];
    float gyro_raw[3];

    icm_imp_read_regs(ICM42688_IMP_BURST_START, buf, ICM42688_IMP_BURST_LEN);
    icm_imp_parse_burst(buf, acc_raw, gyro_raw, 0);
    ICM42688_Improved_CorrectGyroBias(gyro_raw);

    acc->x = acc_raw[0];
    acc->y = acc_raw[1];
    acc->z = acc_raw[2];
    gyro->x = gyro_raw[0];
    gyro->y = gyro_raw[1];
    gyro->z = gyro_raw[2];

    return 0;
}
