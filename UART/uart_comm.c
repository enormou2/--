/*
 * uart_comm.c
 * UART0: 115200 RX 接收外部MCU数据
 * UART1: 115200 TX+RX 蓝牙PID调试
 */
#include "ti_msp_dl_config.h"
#include "UART/uart_comm.h"
#include <stdarg.h>
#include <stdio.h>

/* ---- UART0 接收缓冲区 ---- */
uint8_t  g_uart0_rx_buf[USART_REC_LEN];
uint16_t g_uart0_rx_sta;

/* ---- UART1 接收缓冲区 (蓝牙PID) ---- */
uint8_t  g_uart1_rx_buf[USART_REC_LEN];
uint16_t g_uart1_rx_sta;

/* ========================================================================
 * uart_printf — 通过 UART1 发送 (蓝牙)
 * ======================================================================== */
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len < 0) return;

    for (const char *p = buf; *p; p++) {
        DL_UART_Main_transmitDataBlocking(UART_1_INST, *p);
    }
}

/* ========================================================================
 * UART0 ISR — 接收外部MCU数据 (小球位置, 暂不解析)
 * ======================================================================== */
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);

        if ((g_uart0_rx_sta & 0x8000) == 0) {
            if (ch == '\r') {
                g_uart0_rx_sta |= 0x4000;
            } else if (ch == '\n') {
                if (g_uart0_rx_sta & 0x4000) {
                    g_uart0_rx_sta |= 0x8000;
                }
            } else {
                uint16_t idx = g_uart0_rx_sta & 0x3FFF;
                g_uart0_rx_buf[idx] = ch;
                g_uart0_rx_sta = (g_uart0_rx_sta & 0xC000) | ((idx + 1) & 0x3FFF);
                if (idx >= USART_REC_LEN - 1) g_uart0_rx_sta = 0;
            }
        }
        break;
    }
    default: break;
    }
}

/* ========================================================================
 * UART1 ISR — 接收蓝牙PID调参命令
 * ======================================================================== */
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);

        if ((g_uart1_rx_sta & 0x8000) == 0) {
            if (ch == '\r') {
                g_uart1_rx_sta |= 0x4000;
            } else if (ch == '\n') {
                if (g_uart1_rx_sta & 0x4000) {
                    g_uart1_rx_sta |= 0x8000;
                }
            } else {
                uint16_t idx = g_uart1_rx_sta & 0x3FFF;
                g_uart1_rx_buf[idx] = ch;
                g_uart1_rx_sta = (g_uart1_rx_sta & 0xC000) | ((idx + 1) & 0x3FFF);
                if (idx >= USART_REC_LEN - 1) g_uart1_rx_sta = 0;
            }
        }
        break;
    }
    default: break;
    }
}
