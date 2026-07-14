/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for Motor */
#define Motor_INST                                                         TIMG0
#define Motor_INST_IRQHandler                                   TIMG0_IRQHandler
#define Motor_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define Motor_INST_CLK_FREQ                                             40000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_C0_PORT                                                 GPIOB
#define GPIO_Motor_C0_PIN                                         DL_GPIO_PIN_10
#define GPIO_Motor_C0_IOMUX                                      (IOMUX_PINCM27)
#define GPIO_Motor_C0_IOMUX_FUNC                     IOMUX_PINCM27_PF_TIMG0_CCP0
#define GPIO_Motor_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_C1_PORT                                                 GPIOB
#define GPIO_Motor_C1_PIN                                         DL_GPIO_PIN_11
#define GPIO_Motor_C1_IOMUX                                      (IOMUX_PINCM28)
#define GPIO_Motor_C1_IOMUX_FUNC                     IOMUX_PINCM28_PF_TIMG0_CCP1
#define GPIO_Motor_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_TICK */
#define TIMER_TICK_INST                                                  (TIMA0)
#define TIMER_TICK_INST_IRQHandler                              TIMA0_IRQHandler
#define TIMER_TICK_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define TIMER_TICK_INST_LOAD_VALUE                                       (9999U)
/* Defines for dwt */
#define dwt_INST                                                        (TIMG12)
#define dwt_INST_IRQHandler                                    TIMG12_IRQHandler
#define dwt_INST_INT_IRQN                                      (TIMG12_INT_IRQn)
#define dwt_INST_LOAD_VALUE                                        (4294959999U)
/* Defines for ZDT_MOTOR_TICK */
#define ZDT_MOTOR_TICK_INST                                              (TIMG6)
#define ZDT_MOTOR_TICK_INST_IRQHandler                          TIMG6_IRQHandler
#define ZDT_MOTOR_TICK_INST_INT_IRQN                            (TIMG6_INT_IRQn)
#define ZDT_MOTOR_TICK_INST_LOAD_VALUE                                   (2399U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_0 */
#define UART_0_INST                                                        UART3
#define UART_0_INST_FREQUENCY                                           80000000
#define UART_0_INST_IRQHandler                                  UART3_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOB
#define GPIO_UART_0_TX_PORT                                                GPIOB
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_12
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM30)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM29)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM30_PF_UART3_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM29_PF_UART3_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_0_FBRD_80_MHZ_115200_BAUD                                      (26)
/* Defines for K230 */
#define K230_INST                                                          UART2
#define K230_INST_FREQUENCY                                             40000000
#define K230_INST_IRQHandler                                    UART2_IRQHandler
#define K230_INST_INT_IRQN                                        UART2_INT_IRQn
#define GPIO_K230_RX_PORT                                                  GPIOB
#define GPIO_K230_TX_PORT                                                  GPIOB
#define GPIO_K230_RX_PIN                                          DL_GPIO_PIN_16
#define GPIO_K230_TX_PIN                                          DL_GPIO_PIN_15
#define GPIO_K230_IOMUX_RX                                       (IOMUX_PINCM33)
#define GPIO_K230_IOMUX_TX                                       (IOMUX_PINCM32)
#define GPIO_K230_IOMUX_RX_FUNC                        IOMUX_PINCM33_PF_UART2_RX
#define GPIO_K230_IOMUX_TX_FUNC                        IOMUX_PINCM32_PF_UART2_TX
#define K230_BAUD_RATE                                                  (115200)
#define K230_IBRD_40_MHZ_115200_BAUD                                        (21)
#define K230_FBRD_40_MHZ_115200_BAUD                                        (45)




/* Defines for ICM42688 */
#define ICM42688_INST                                                      SPI0
#define ICM42688_INST_IRQHandler                                SPI0_IRQHandler
#define ICM42688_INST_INT_IRQN                                    SPI0_INT_IRQn
#define GPIO_ICM42688_PICO_PORT                                           GPIOB
#define GPIO_ICM42688_PICO_PIN                                   DL_GPIO_PIN_17
#define GPIO_ICM42688_IOMUX_PICO                                (IOMUX_PINCM43)
#define GPIO_ICM42688_IOMUX_PICO_FUNC                IOMUX_PINCM43_PF_SPI0_PICO
#define GPIO_ICM42688_POCI_PORT                                           GPIOA
#define GPIO_ICM42688_POCI_PIN                                   DL_GPIO_PIN_10
#define GPIO_ICM42688_IOMUX_POCI                                (IOMUX_PINCM21)
#define GPIO_ICM42688_IOMUX_POCI_FUNC                IOMUX_PINCM21_PF_SPI0_POCI
/* GPIO configuration for ICM42688 */
#define GPIO_ICM42688_SCLK_PORT                                           GPIOA
#define GPIO_ICM42688_SCLK_PIN                                   DL_GPIO_PIN_11
#define GPIO_ICM42688_IOMUX_SCLK                                (IOMUX_PINCM22)
#define GPIO_ICM42688_IOMUX_SCLK_FUNC                IOMUX_PINCM22_PF_SPI0_SCLK
/* Defines for SPI_OLED */
#define SPI_OLED_INST                                                      SPI1
#define SPI_OLED_INST_IRQHandler                                SPI1_IRQHandler
#define SPI_OLED_INST_INT_IRQN                                    SPI1_INT_IRQn
#define GPIO_SPI_OLED_PICO_PORT                                           GPIOB
#define GPIO_SPI_OLED_PICO_PIN                                    DL_GPIO_PIN_8
#define GPIO_SPI_OLED_IOMUX_PICO                                (IOMUX_PINCM25)
#define GPIO_SPI_OLED_IOMUX_PICO_FUNC                IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_OLED_POCI_PORT                                           GPIOA
#define GPIO_SPI_OLED_POCI_PIN                                   DL_GPIO_PIN_16
#define GPIO_SPI_OLED_IOMUX_POCI                                (IOMUX_PINCM38)
#define GPIO_SPI_OLED_IOMUX_POCI_FUNC                IOMUX_PINCM38_PF_SPI1_POCI
/* GPIO configuration for SPI_OLED */
#define GPIO_SPI_OLED_SCLK_PORT                                           GPIOB
#define GPIO_SPI_OLED_SCLK_PIN                                    DL_GPIO_PIN_9
#define GPIO_SPI_OLED_IOMUX_SCLK                                (IOMUX_PINCM26)
#define GPIO_SPI_OLED_IOMUX_SCLK_FUNC                IOMUX_PINCM26_PF_SPI1_SCLK



/* Defines for ADC1 */
#define ADC1_INST                                                           ADC1
#define ADC1_INST_IRQHandler                                     ADC1_IRQHandler
#define ADC1_INST_INT_IRQN                                       (ADC1_INT_IRQn)
#define ADC1_ADCMEM_ADC_Channel8                              DL_ADC12_MEM_IDX_0
#define ADC1_ADCMEM_ADC_Channel8_REF             DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC1_ADCMEM_ADC_Channel8_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC1_C8_PORT                                                  GPIOA
#define GPIO_ADC1_C8_PIN                                          DL_GPIO_PIN_22
#define GPIO_ADC1_IOMUX_C8                                       (IOMUX_PINCM47)
#define GPIO_ADC1_IOMUX_C8_FUNC                   (IOMUX_PINCM47_PF_UNCONNECTED)



/* Port definition for Pin Group BEEP */
#define BEEP_PORT                                                        (GPIOA)

/* Defines for PIN_14: GPIOA.14 with pinCMx 36 on package pin 7 */
#define BEEP_PIN_14_PIN                                         (DL_GPIO_PIN_14)
#define BEEP_PIN_14_IOMUX                                        (IOMUX_PINCM36)
/* Port definition for Pin Group ICM42688_CS */
#define ICM42688_CS_PORT                                                 (GPIOA)

/* Defines for CS: GPIOA.29 with pinCMx 4 on package pin 36 */
#define ICM42688_CS_CS_PIN                                      (DL_GPIO_PIN_29)
#define ICM42688_CS_CS_IOMUX                                      (IOMUX_PINCM4)
/* Port definition for Pin Group RELAY */
#define RELAY_PORT                                                       (GPIOB)

/* Defines for Control: GPIOB.4 with pinCMx 17 on package pin 52 */
#define RELAY_Control_PIN                                        (DL_GPIO_PIN_4)
#define RELAY_Control_IOMUX                                      (IOMUX_PINCM17)
/* Port definition for Pin Group User_LED */
#define User_LED_PORT                                                    (GPIOB)

/* Defines for User_led: GPIOB.22 with pinCMx 50 on package pin 21 */
#define User_LED_User_led_PIN                                   (DL_GPIO_PIN_22)
#define User_LED_User_led_IOMUX                                  (IOMUX_PINCM50)
/* Defines for key1: GPIOA.30 with pinCMx 5 on package pin 37 */
#define KEY_key1_PORT                                                    (GPIOA)
#define KEY_key1_PIN                                            (DL_GPIO_PIN_30)
#define KEY_key1_IOMUX                                            (IOMUX_PINCM5)
/* Defines for key2: GPIOA.31 with pinCMx 6 on package pin 39 */
#define KEY_key2_PORT                                                    (GPIOA)
#define KEY_key2_PIN                                            (DL_GPIO_PIN_31)
#define KEY_key2_IOMUX                                            (IOMUX_PINCM6)
/* Defines for key3: GPIOB.0 with pinCMx 12 on package pin 47 */
#define KEY_key3_PORT                                                    (GPIOB)
#define KEY_key3_PIN                                             (DL_GPIO_PIN_0)
#define KEY_key3_IOMUX                                           (IOMUX_PINCM12)
/* Defines for key4: GPIOB.1 with pinCMx 13 on package pin 48 */
#define KEY_key4_PORT                                                    (GPIOB)
#define KEY_key4_PIN                                             (DL_GPIO_PIN_1)
#define KEY_key4_IOMUX                                           (IOMUX_PINCM13)
/* Defines for EN1_A: GPIOA.7 with pinCMx 14 on package pin 49 */
#define Motor_dir_EN1_A_PORT                                             (GPIOA)
#define Motor_dir_EN1_A_PIN                                      (DL_GPIO_PIN_7)
#define Motor_dir_EN1_A_IOMUX                                    (IOMUX_PINCM14)
/* Defines for EN1_B: GPIOA.21 with pinCMx 46 on package pin 17 */
#define Motor_dir_EN1_B_PORT                                             (GPIOA)
#define Motor_dir_EN1_B_PIN                                     (DL_GPIO_PIN_21)
#define Motor_dir_EN1_B_IOMUX                                    (IOMUX_PINCM46)
/* Defines for EN2_A: GPIOA.18 with pinCMx 40 on package pin 11 */
#define Motor_dir_EN2_A_PORT                                             (GPIOA)
#define Motor_dir_EN2_A_PIN                                     (DL_GPIO_PIN_18)
#define Motor_dir_EN2_A_IOMUX                                    (IOMUX_PINCM40)
/* Defines for EN2_B: GPIOB.14 with pinCMx 31 on package pin 2 */
#define Motor_dir_EN2_B_PORT                                             (GPIOB)
#define Motor_dir_EN2_B_PIN                                     (DL_GPIO_PIN_14)
#define Motor_dir_EN2_B_IOMUX                                    (IOMUX_PINCM31)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOB)

/* Defines for ENC_A2: GPIOB.2 with pinCMx 15 on package pin 50 */
// pins affected by this interrupt request:["ENC_A2","ENC_B2","ENC_A1","ENC_B1"]
#define ENCODER_INT_IRQN                                        (GPIOB_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODER_ENC_A2_IIDX                                  (DL_GPIO_IIDX_DIO2)
#define ENCODER_ENC_A2_PIN                                       (DL_GPIO_PIN_2)
#define ENCODER_ENC_A2_IOMUX                                     (IOMUX_PINCM15)
/* Defines for ENC_B2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define ENCODER_ENC_B2_IIDX                                  (DL_GPIO_IIDX_DIO3)
#define ENCODER_ENC_B2_PIN                                       (DL_GPIO_PIN_3)
#define ENCODER_ENC_B2_IOMUX                                     (IOMUX_PINCM16)
/* Defines for ENC_A1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define ENCODER_ENC_A1_IIDX                                 (DL_GPIO_IIDX_DIO18)
#define ENCODER_ENC_A1_PIN                                      (DL_GPIO_PIN_18)
#define ENCODER_ENC_A1_IOMUX                                     (IOMUX_PINCM44)
/* Defines for ENC_B1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define ENCODER_ENC_B1_IIDX                                 (DL_GPIO_IIDX_DIO19)
#define ENCODER_ENC_B1_PIN                                      (DL_GPIO_PIN_19)
#define ENCODER_ENC_B1_IOMUX                                     (IOMUX_PINCM45)
/* Port definition for Pin Group ZDT_Motor */
#define ZDT_Motor_PORT                                                   (GPIOA)

/* Defines for Yaw_Dir: GPIOA.15 with pinCMx 37 on package pin 8 */
#define ZDT_Motor_Yaw_Dir_PIN                                   (DL_GPIO_PIN_15)
#define ZDT_Motor_Yaw_Dir_IOMUX                                  (IOMUX_PINCM37)
/* Defines for Pitch_Dir: GPIOA.8 with pinCMx 19 on package pin 54 */
#define ZDT_Motor_Pitch_Dir_PIN                                  (DL_GPIO_PIN_8)
#define ZDT_Motor_Pitch_Dir_IOMUX                                (IOMUX_PINCM19)
/* Defines for Yaw_Stp: GPIOA.17 with pinCMx 39 on package pin 10 */
#define ZDT_Motor_Yaw_Stp_PIN                                   (DL_GPIO_PIN_17)
#define ZDT_Motor_Yaw_Stp_IOMUX                                  (IOMUX_PINCM39)
/* Defines for Pitch_Stp: GPIOA.9 with pinCMx 20 on package pin 55 */
#define ZDT_Motor_Pitch_Stp_PIN                                  (DL_GPIO_PIN_9)
#define ZDT_Motor_Pitch_Stp_IOMUX                                (IOMUX_PINCM20)
/* Defines for PIN_0: GPIOA.24 with pinCMx 54 on package pin 25 */
#define Gray_Address_PIN_0_PORT                                          (GPIOA)
#define Gray_Address_PIN_0_PIN                                  (DL_GPIO_PIN_24)
#define Gray_Address_PIN_0_IOMUX                                 (IOMUX_PINCM54)
/* Defines for PIN_1: GPIOB.25 with pinCMx 56 on package pin 27 */
#define Gray_Address_PIN_1_PORT                                          (GPIOB)
#define Gray_Address_PIN_1_PIN                                  (DL_GPIO_PIN_25)
#define Gray_Address_PIN_1_IOMUX                                 (IOMUX_PINCM56)
/* Defines for PIN_2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define Gray_Address_PIN_2_PORT                                          (GPIOB)
#define Gray_Address_PIN_2_PIN                                  (DL_GPIO_PIN_24)
#define Gray_Address_PIN_2_IOMUX                                 (IOMUX_PINCM52)
/* Port definition for Pin Group Gray_Serial */
#define Gray_Serial_PORT                                                 (GPIOA)

/* Defines for DAT: GPIOA.26 with pinCMx 59 on package pin 30 */
#define Gray_Serial_DAT_PIN                                     (DL_GPIO_PIN_26)
#define Gray_Serial_DAT_IOMUX                                    (IOMUX_PINCM59)
/* Defines for CLK: GPIOA.27 with pinCMx 60 on package pin 31 */
#define Gray_Serial_CLK_PIN                                     (DL_GPIO_PIN_27)
#define Gray_Serial_CLK_IOMUX                                    (IOMUX_PINCM60)


/* Defines for MCAN0 */
#define MCAN0_INST                                                        CANFD0
#define GPIO_MCAN0_CAN_TX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_TX_PIN                                     DL_GPIO_PIN_12
#define GPIO_MCAN0_IOMUX_CAN_TX                                  (IOMUX_PINCM34)
#define GPIO_MCAN0_IOMUX_CAN_TX_FUNC               IOMUX_PINCM34_PF_CANFD0_CANTX
#define GPIO_MCAN0_CAN_RX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_RX_PIN                                     DL_GPIO_PIN_13
#define GPIO_MCAN0_IOMUX_CAN_RX                                  (IOMUX_PINCM35)
#define GPIO_MCAN0_IOMUX_CAN_RX_FUNC               IOMUX_PINCM35_PF_CANFD0_CANRX
#define MCAN0_INST_IRQHandler                                 CANFD0_IRQHandler
#define MCAN0_INST_INT_IRQN                                     CANFD0_INT_IRQn


/* Defines for MCAN0 MCAN RAM configuration */
#define MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_STD_ID_FILTER_NUM          (1)
#define MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR     (48)
#define MCAN0_INST_MCAN_EXT_ID_FILTER_NUM          (1)
#define MCAN0_INST_MCAN_TX_BUFF_START_ADDR         (148)
#define MCAN0_INST_MCAN_TX_BUFF_SIZE               (2)
#define MCAN0_INST_MCAN_FIFO_1_START_ADDR          (192)
#define MCAN0_INST_MCAN_FIFO_1_NUM                 (2)
#define MCAN0_INST_MCAN_TX_EVENT_START_ADDR        (164)
#define MCAN0_INST_MCAN_TX_EVENT_SIZE              (2)
#define MCAN0_INST_MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
#define MCAN0_INST_MCAN_RX_BUFF_START_ADDR         (208)
#define MCAN0_INST_MCAN_FIFO_0_START_ADDR          (172)
#define MCAN0_INST_MCAN_FIFO_0_NUM                 (3)

#define MCAN0_INST_MCAN_INTERRUPTS (DL_MCAN_INTERRUPT_RF0N)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_Motor_init(void);
void SYSCFG_DL_TIMER_TICK_init(void);
void SYSCFG_DL_dwt_init(void);
void SYSCFG_DL_ZDT_MOTOR_TICK_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_K230_init(void);
void SYSCFG_DL_ICM42688_init(void);
void SYSCFG_DL_SPI_OLED_init(void);
void SYSCFG_DL_ADC1_init(void);

void SYSCFG_DL_MCAN0_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
