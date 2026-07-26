#include "icm42688.h"
#include <string.h>


  static SPI_HandleTypeDef *icm_hspi = NULL;
  static float gyro_bias[3] = {0};

  /* ---- Bank 切换 ---- */
//因为这玩意寄存器太多，所以有bank机制
  static void icm_select_bank(uint8_t bank)
  {
      uint8_t tx[2] = { 0x76, bank & 0x07 };  //REG_BANK_SEL = 0x76(手册p62),就是切换bank这个寄存器的地址，这个东西bit0——bit2，所以其他位置0
      ICM_CS_LOW();
      HAL_SPI_Transmit(icm_hspi, tx, 2, HAL_MAX_DELAY);
      ICM_CS_HIGH();
  }

  /* ---- 写寄存器 ---- */
  static void icm_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
  {
      uint8_t tx[2] = { reg & 0x7F, value };//设置bit7=0
      ICM_CS_LOW();
      HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
      ICM_CS_HIGH();
  }

  /* ---- 读寄存器 ---- */
  static uint8_t icm_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg)
  {
      uint8_t tx[2] = { reg | 0x80, 0x00 };//设置bit7=1,后面一位是空，用来提供时钟，因为只有主机才能提供时钟
      uint8_t rx[2] = {0};
      ICM_CS_LOW();
      HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
      ICM_CS_HIGH();
      return rx[1];//rx[0]是无意义的量，此时寄存器还是别的值，没有移到对应寄存器上，只有接收完tx的值，才知道是哪个寄存器
  }

  /* ---- 大端 → int16 ---- */
//用来拼接数据，一个值由两个寄存器存着
  static inline int16_t be16_to_i16(const uint8_t *b)
  {
      return (int16_t)((b[0] << 8) | b[1]);
  }

  /* ---- 解析 ---- */
  void icm42688_parse(const uint8_t *buf, float *acc, float *gyro, float *temp)
  {
      if (temp) {
          *temp = (float)be16_to_i16(&buf[0]) * 0.007548309f + 25.0f;//温度计算公式见手册p66
      }
      acc[0]  = (float)be16_to_i16(&buf[2])  * ICM_ACCEL_SENSITIVITY_16G * g_gravity;//该imu性能参数2000dps,16g
      acc[1]  = (float)be16_to_i16(&buf[4])  * ICM_ACCEL_SENSITIVITY_16G * g_gravity;
      acc[2]  = (float)be16_to_i16(&buf[6])  * ICM_ACCEL_SENSITIVITY_16G * g_gravity;
      gyro[0] = (float)be16_to_i16(&buf[8])  * ICM_GYRO_SENSITIVITY_2000 * pidivide180;
      gyro[1] = (float)be16_to_i16(&buf[10]) * ICM_GYRO_SENSITIVITY_2000 * pidivide180;
      gyro[2] = (float)be16_to_i16(&buf[12]) * ICM_GYRO_SENSITIVITY_2000 * pidivide180;
  }

  /* ---- 去零偏 ---- */
  void icm42688_correct_gyro_bias(float *gyro)
  {
      gyro[0] -= gyro_bias[0];
      gyro[1] -= gyro_bias[1];
      gyro[2] -= gyro_bias[2];
  }

  /* ---- 零偏校准 ---- */
  static void icm_calibrate_gyro(SPI_HandleTypeDef *hspi)
  {
      float sum[3] = {0};
      uint8_t buf[14], dummy[14];
      memset(dummy, 0, 14);
      uint8_t tx_addr = 0x1D | 0x80;

      for (int i = 0; i < 500; i++) {
          ICM_CS_LOW();
          HAL_SPI_Transmit(hspi, &tx_addr, 1, HAL_MAX_DELAY);
          HAL_SPI_TransmitReceive(hspi, dummy, buf, 14, HAL_MAX_DELAY);
          ICM_CS_HIGH();
          float acc[3], gyro[3];
          icm42688_parse(buf, acc, gyro, NULL);
          sum[0] += gyro[0]; sum[1] += gyro[1]; sum[2] += gyro[2];
          HAL_Delay(2);
      }
      gyro_bias[0] = sum[0] / 500.0f;
      gyro_bias[1] = sum[1] / 500.0f;
      gyro_bias[2] = sum[2] / 500.0f;
  }

  /* ---- 初始化 ---- */
  int icm42688_init(SPI_HandleTypeDef *hspi)
  {
      icm_hspi = hspi;
      HAL_Delay(50);

      icm_write_reg(hspi, 0x11, 0x01);   /* DEVICE_CONFIG: soft reset */
      HAL_Delay(50);

      if (icm_read_reg(hspi, 0x75) != 0x47) return -1;  /* WHO_AM_I */

      icm_write_reg(hspi, 0x76, 0x00);   /* Bank 0 */
      icm_write_reg(hspi, 0x4F, 0x06);   /* GYRO_CONFIG0: 2000dps, 1kHz */
      icm_write_reg(hspi, 0x50, 0x06);   /* ACCEL_CONFIG0: ±16g, 1kHz */
      icm_write_reg(hspi, 0x4E, 0x0F);   /* PWR_MGMT0: Gyro LN + Accel LN */
      icm_write_reg(hspi, 0x52, 0x55);   /* GYRO_ACCEL_CONFIG0 */
      HAL_Delay(30);

      // icm_calibrate_gyro(hspi);
      return 0;
  }