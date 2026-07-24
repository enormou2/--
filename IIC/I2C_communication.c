/*
 * I2C_communication.c
 * Simplified I2C polling-mode read/write for MSPM0 + ICM45686
 */

#include "I2C_communication.h"

/**
 * @brief Write register(s) via I2C (polling mode).
 *        Sends: [reg_addr] [data0] [data1] ...
 */
void I2C_WriteReg(uint8_t addr, uint8_t reg_addr, const uint8_t *reg_data, uint8_t count)
{
    uint8_t tx_buf[8];
    uint8_t i;

    tx_buf[0] = reg_addr;
    for (i = 0; i < count; i++) {
        tx_buf[1 + i] = reg_data[i];
    }

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &tx_buf[0], count + 1);

    /* Wait for bus idle */
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    DL_I2C_startControllerTransfer(I2C_0_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, count + 1);

    /* Wait for transfer to complete */
    while (DL_I2C_getControllerStatus(I2C_0_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}

/**
 * @brief Read register(s) via I2C (polling mode).
 *        Writes reg_addr, then re-starts as read.
 */
void I2C_ReadReg(uint8_t addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Fill TX FIFO with just the register address (1 byte) */
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg_addr, 1);

    /* Wait for bus idle */
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    /* Send register address */
    DL_I2C_startControllerTransfer(
        I2C_0_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (DL_I2C_getControllerStatus(I2C_0_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    /* Read data from target */
    DL_I2C_startControllerTransfer(
        I2C_0_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, count);

    for (uint8_t i = 0; i < count; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST))
            ;
        reg_data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
}
