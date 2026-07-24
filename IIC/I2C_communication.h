/*
 * I2C_communication.h
 * Simplified I2C communication layer for MSPM0 + ICM45686
 */

#ifndef I2C_COMMUNICATION_H_
#define I2C_COMMUNICATION_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/**
 * @brief Write data to an I2C device register.
 * @param addr     7-bit I2C target address
 * @param reg_addr Register address to write to
 * @param reg_data Pointer to data to write
 * @param count    Number of bytes to write
 */
void I2C_WriteReg(uint8_t addr, uint8_t reg_addr, const uint8_t *reg_data, uint8_t count);

/**
 * @brief Read data from an I2C device register.
 * @param addr     7-bit I2C target address
 * @param reg_addr Register address to read from
 * @param reg_data Pointer to buffer for received data
 * @param count    Number of bytes to read
 */
void I2C_ReadReg(uint8_t addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

#endif /* I2C_COMMUNICATION_H_ */
