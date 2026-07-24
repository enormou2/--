/*
 * icm45686.c
 * ICM45686 IMU chip driver for MSPM0 with I2C interface
 *
 * This file provides:
 *  - I2C transport callbacks (read/write register)
 *  - Device initialization and configuration (setup_imu)
 *  - Sensor data reading with unit conversion (bsp_IcmGetRawData)
 */

#include <stdio.h>
#include <string.h>
#include "icm45686.h"
#include "inv_imu_driver.h"
#include "IIC/I2C_communication.h"

/* ========================================================================
 * Hardware configuration
 * ======================================================================== */

#define ICM_I2C_ADDR    0x69    /* 7-bit I2C address of ICM45686 */

/* ========================================================================
 * Global driver device structure
 * ======================================================================== */

static inv_imu_device_t imu_dev;

/* ========================================================================
 * Delay utilities (CPUCLK_FREQ defined in ti_msp_dl_config.h)
 * ======================================================================== */

static void delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000) * us);
}

static void delay_ms(uint32_t ms)
{
    delay_cycles((CPUCLK_FREQ / 1000) * ms);
}

/* ========================================================================
 * Error helper
 * ======================================================================== */

static int print_error(int rc)
{
    if (rc != 0) {
        switch (rc) {
        case INV_IMU_ERROR:
            printf("IMU: Unspecified error (%d)\r\n", rc);
            break;
        case INV_IMU_ERROR_TRANSPORT:
            printf("IMU: Transport error (%d)\r\n", rc);
            break;
        case INV_IMU_ERROR_TIMEOUT:
            printf("IMU: Timeout (%d)\r\n", rc);
            break;
        case INV_IMU_ERROR_BAD_ARG:
            printf("IMU: Bad argument (%d)\r\n", rc);
            break;
        default:
            printf("IMU: Unknown error (%d)\r\n", rc);
            break;
        }
    }
    return rc;
}

#define SI_CHECK_RC(rc)                                   \
    do {                                                  \
        if (print_error(rc)) {                            \
            printf("Error at %s:%d\r\n", __FILE__, __LINE__); \
            delay_ms(100);                                \
            return rc;                                    \
        }                                                 \
    } while (0)

/* ========================================================================
 * I2C transport callbacks — called by inv_imu_transport layer
 * ======================================================================== */

static int icm45686_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    I2C_ReadReg(ICM_I2C_ADDR, reg, buf, (uint8_t)len);
    return 0;
}

static int icm45686_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        I2C_WriteReg(ICM_I2C_ADDR, (uint8_t)(reg + i), &buf[i], 1);
    }
    return 0;
}

/* ========================================================================
 * setup_imu — initialize and configure ICM45686
 * ======================================================================== */

int setup_imu(int use_ln, int accel_en, int gyro_en)
{
    int                      rc     = 0;
    uint8_t                  whoami = 0;
    inv_imu_int_pin_config_t int_pin_config;
    inv_imu_int_state_t      int_config;

    /* Init transport layer */
    imu_dev.transport.read_reg   = icm45686_read_reg;
    imu_dev.transport.write_reg  = icm45686_write_reg;
    imu_dev.transport.serif_type = UI_I2C;
    imu_dev.transport.sleep_us   = delay_us;

    /* Wait 3 ms for device to power up */
    delay_us(3000);

    /* Check WHOAMI */
    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
    SI_CHECK_RC(rc);
    if (whoami != INV_IMU_WHOAMI) {
        printf("IMU: Wrong WHOAMI! Read 0x%02X, expected 0x%02X\r\n",
               whoami, INV_IMU_WHOAMI);
        return -1;
    }
    printf("ICM45686 detected (WHOAMI=0x%02X)\r\n", whoami);

    rc |= inv_imu_soft_reset(&imu_dev);
    SI_CHECK_RC(rc);

    /* Configure interrupt pin (INT1): active-high, pulse, push-pull */
    int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
    int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
    SI_CHECK_RC(rc);

    /* Enable data-ready interrupt on INT1 */
    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_UI_DRDY = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
    SI_CHECK_RC(rc);

    /* Set full-scale range: Accel +/-4g, Gyro +/-1000dps */
    rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G);
    rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS);
    SI_CHECK_RC(rc);

    /* Set output data rate: 200 Hz */
    rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
    rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
    SI_CHECK_RC(rc);

    /* Set bandwidth = ODR/4 */
    rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
    rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
    SI_CHECK_RC(rc);

    /* Use RCOSC for LP mode (not ULP, so registers remain accessible) */
    rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
    SI_CHECK_RC(rc);

    /* Set power modes */
    if (use_ln) {
        if (accel_en)
            rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
        if (gyro_en)
            rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
    } else {
        if (accel_en)
            rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
        if (gyro_en)
            rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
    }

    SI_CHECK_RC(rc);
    return rc;
}

/* ========================================================================
 * bsp_IcmGetRawData — read sensor data and convert to physical units
 * ======================================================================== */

int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc)
{
    int                 rc = 0;
    inv_imu_sensor_data_t d;

    rc |= inv_imu_get_register_data(&imu_dev, &d);
    SI_CHECK_RC(rc);

    /* Convert accel: LSB to mg  (FSR=4g => 32768 LSB/g => 1 LSB = 4/32.768 mg) */
    accel_mg[0] = (float)d.accel_data[0] * 4.0f / 32.768f;
    accel_mg[1] = (float)d.accel_data[1] * 4.0f / 32.768f;
    accel_mg[2] = (float)d.accel_data[2] * 4.0f / 32.768f;

    /* Convert gyro: LSB to dps (FSR=1000dps => 32768 LSB/(1000dps) => 1 LSB = 1000/32768 dps) */
    gyro_dps[0] = (float)d.gyro_data[0] * 1000.0f / 32768.0f;
    gyro_dps[1] = (float)d.gyro_data[1] * 1000.0f / 32768.0f;
    gyro_dps[2] = (float)d.gyro_data[2] * 1000.0f / 32768.0f;

    /* Temperature: 25°C + raw/128 */
    *temp_degc = 25.0f + ((float)d.temp_data / 128.0f);

    return 0;
}
