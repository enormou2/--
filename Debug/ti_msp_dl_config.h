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


#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG0
#define PWM_MOTOR_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                           312500
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX

/* Defines for PWM_STEP */
#define PWM_STEP_INST                                                      TIMG7
#define PWM_STEP_INST_IRQHandler                                TIMG7_IRQHandler
#define PWM_STEP_INST_INT_IRQN                                  (TIMG7_INT_IRQn)
#define PWM_STEP_INST_CLK_FREQ                                          10000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_STEP_C0_PORT                                              GPIOA
#define GPIO_PWM_STEP_C0_PIN                                      DL_GPIO_PIN_28
#define GPIO_PWM_STEP_C0_IOMUX                                    (IOMUX_PINCM3)
#define GPIO_PWM_STEP_C0_IOMUX_FUNC                   IOMUX_PINCM3_PF_TIMG7_CCP0
#define GPIO_PWM_STEP_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_STEP_C1_PORT                                              GPIOA
#define GPIO_PWM_STEP_C1_PIN                                      DL_GPIO_PIN_18
#define GPIO_PWM_STEP_C1_IOMUX                                   (IOMUX_PINCM40)
#define GPIO_PWM_STEP_C1_IOMUX_FUNC                  IOMUX_PINCM40_PF_TIMG7_CCP1
#define GPIO_PWM_STEP_C1_IDX                                 DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG6)
#define TIMER_0_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (2019U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                     DL_GPIO_PIN_16
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM38)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                     DL_GPIO_PIN_15
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM37)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM37_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           80000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_0_FBRD_80_MHZ_115200_BAUD                                      (26)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                           80000000
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOA
#define GPIO_UART_2_TX_PORT                                                GPIOA
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_22
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_21
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM47)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM46)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM46_PF_UART2_TX
#define UART_2_BAUD_RATE                                                  (9600)
#define UART_2_IBRD_80_MHZ_9600_BAUD                                       (520)
#define UART_2_FBRD_80_MHZ_9600_BAUD                                        (53)





/* Port definition for Pin Group TRACK1 */
#define TRACK1_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK1: GPIOB.2 with pinCMx 15 on package pin 50 */
#define TRACK1_PIN_TRACK1_PIN                                    (DL_GPIO_PIN_2)
#define TRACK1_PIN_TRACK1_IOMUX                                  (IOMUX_PINCM15)
/* Port definition for Pin Group TRACK2 */
#define TRACK2_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define TRACK2_PIN_TRACK2_PIN                                    (DL_GPIO_PIN_3)
#define TRACK2_PIN_TRACK2_IOMUX                                  (IOMUX_PINCM16)
/* Port definition for Pin Group TRACK3 */
#define TRACK3_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK3: GPIOB.4 with pinCMx 17 on package pin 52 */
#define TRACK3_PIN_TRACK3_PIN                                    (DL_GPIO_PIN_4)
#define TRACK3_PIN_TRACK3_IOMUX                                  (IOMUX_PINCM17)
/* Port definition for Pin Group TRACK4 */
#define TRACK4_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK4: GPIOB.5 with pinCMx 18 on package pin 53 */
#define TRACK4_PIN_TRACK4_PIN                                    (DL_GPIO_PIN_5)
#define TRACK4_PIN_TRACK4_IOMUX                                  (IOMUX_PINCM18)
/* Port definition for Pin Group TRACK5 */
#define TRACK5_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK5: GPIOB.6 with pinCMx 23 on package pin 58 */
#define TRACK5_PIN_TRACK5_PIN                                    (DL_GPIO_PIN_6)
#define TRACK5_PIN_TRACK5_IOMUX                                  (IOMUX_PINCM23)
/* Port definition for Pin Group TRACK6 */
#define TRACK6_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK6: GPIOB.7 with pinCMx 24 on package pin 59 */
#define TRACK6_PIN_TRACK6_PIN                                    (DL_GPIO_PIN_7)
#define TRACK6_PIN_TRACK6_IOMUX                                  (IOMUX_PINCM24)
/* Port definition for Pin Group TRACK7 */
#define TRACK7_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK7: GPIOB.8 with pinCMx 25 on package pin 60 */
#define TRACK7_PIN_TRACK7_PIN                                    (DL_GPIO_PIN_8)
#define TRACK7_PIN_TRACK7_IOMUX                                  (IOMUX_PINCM25)
/* Port definition for Pin Group TRACK8 */
#define TRACK8_PORT                                                      (GPIOB)

/* Defines for PIN_TRACK8: GPIOB.9 with pinCMx 26 on package pin 61 */
#define TRACK8_PIN_TRACK8_PIN                                    (DL_GPIO_PIN_9)
#define TRACK8_PIN_TRACK8_IOMUX                                  (IOMUX_PINCM26)
/* Port definition for Pin Group AIN1 */
#define AIN1_PORT                                                        (GPIOB)

/* Defines for PIN_AIN1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define AIN1_PIN_AIN1_PIN                                       (DL_GPIO_PIN_13)
#define AIN1_PIN_AIN1_IOMUX                                      (IOMUX_PINCM30)
/* Port definition for Pin Group AIN2 */
#define AIN2_PORT                                                        (GPIOB)

/* Defines for PIN_AIN2: GPIOB.14 with pinCMx 31 on package pin 2 */
#define AIN2_PIN_AIN2_PIN                                       (DL_GPIO_PIN_14)
#define AIN2_PIN_AIN2_IOMUX                                      (IOMUX_PINCM31)
/* Port definition for Pin Group BIN1 */
#define BIN1_PORT                                                        (GPIOB)

/* Defines for PIN_BIN1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define BIN1_PIN_BIN1_PIN                                       (DL_GPIO_PIN_15)
#define BIN1_PIN_BIN1_IOMUX                                      (IOMUX_PINCM32)
/* Port definition for Pin Group BIN2 */
#define BIN2_PORT                                                        (GPIOB)

/* Defines for PIN_BIN2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define BIN2_PIN_BIN2_PIN                                       (DL_GPIO_PIN_16)
#define BIN2_PIN_BIN2_IOMUX                                      (IOMUX_PINCM33)
/* Port definition for Pin Group ENC_R_A */
#define ENC_R_A_PORT                                                     (GPIOA)

/* Defines for PIN_ENC_R_A: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ENC_R_A_PIN_ENC_R_A_PIN                                 (DL_GPIO_PIN_24)
#define ENC_R_A_PIN_ENC_R_A_IOMUX                                (IOMUX_PINCM54)
/* Port definition for Pin Group ENC_R_B */
#define ENC_R_B_PORT                                                     (GPIOA)

/* Defines for PIN_ENC_R_B: GPIOA.17 with pinCMx 39 on package pin 10 */
#define ENC_R_B_PIN_ENC_R_B_PIN                                 (DL_GPIO_PIN_17)
#define ENC_R_B_PIN_ENC_R_B_IOMUX                                (IOMUX_PINCM39)
/* Port definition for Pin Group GPIO_GRP_0 */
#define GPIO_GRP_0_PORT                                                  (GPIOB)

/* Defines for BTN: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_GRP_0_BTN_PIN                                      (DL_GPIO_PIN_21)
#define GPIO_GRP_0_BTN_IOMUX                                     (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_GRP_1 */
#define GPIO_GRP_1_PORT                                                  (GPIOA)

/* Defines for ENC_L_A: GPIOA.6 with pinCMx 11 on package pin 46 */
#define GPIO_GRP_1_ENC_L_A_PIN                                   (DL_GPIO_PIN_6)
#define GPIO_GRP_1_ENC_L_A_IOMUX                                 (IOMUX_PINCM11)
/* Port definition for Pin Group GPIO_GRP_2 */
#define GPIO_GRP_2_PORT                                                  (GPIOA)

/* Defines for ENC_L_B: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_GRP_2_ENC_L_B_PIN                                   (DL_GPIO_PIN_7)
#define GPIO_GRP_2_ENC_L_B_IOMUX                                 (IOMUX_PINCM14)
/* Port definition for Pin Group STEP_DIR */
#define STEP_DIR_PORT                                                    (GPIOA)

/* Defines for PIN_0: GPIOA.29 with pinCMx 4 on package pin 36 */
#define STEP_DIR_PIN_0_PIN                                      (DL_GPIO_PIN_29)
#define STEP_DIR_PIN_0_IOMUX                                      (IOMUX_PINCM4)
/* Port definition for Pin Group STEP_EN */
#define STEP_EN_PORT                                                     (GPIOA)

/* Defines for PIN_1: GPIOA.4 with pinCMx 9 on package pin 44 */
#define STEP_EN_PIN_1_PIN                                        (DL_GPIO_PIN_4)
#define STEP_EN_PIN_1_IOMUX                                       (IOMUX_PINCM9)
/* Port definition for Pin Group STEP_EN2 */
#define STEP_EN2_PORT                                                    (GPIOA)

/* Defines for PIN_2: GPIOA.31 with pinCMx 6 on package pin 39 */
#define STEP_EN2_PIN_2_PIN                                      (DL_GPIO_PIN_31)
#define STEP_EN2_PIN_2_IOMUX                                      (IOMUX_PINCM6)
/* Port definition for Pin Group GPIO_GRP_3 */
#define GPIO_GRP_3_PORT                                                  (GPIOB)

/* Defines for PIN_3: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_GRP_3_PIN_3_PIN                                    (DL_GPIO_PIN_11)
#define GPIO_GRP_3_PIN_3_IOMUX                                   (IOMUX_PINCM28)
/* Port definition for Pin Group GPIO_GRP_4 */
#define GPIO_GRP_4_PORT                                                  (GPIOA)

/* Defines for CHALLENGE_BTN: GPIOA.30 with pinCMx 5 on package pin 37 */
#define GPIO_GRP_4_CHALLENGE_BTN_PIN                            (DL_GPIO_PIN_30)
#define GPIO_GRP_4_CHALLENGE_BTN_IOMUX                            (IOMUX_PINCM5)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_PWM_STEP_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_2_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
