/*
 * uart_comm.c
 * UART0: 115200 RX 接收外部MCU数据
 * UART2: 9600 TX+RX 蓝牙PID调试
 */
#include "ti_msp_dl_config.h"
#include "UART/uart_comm.h"
#include <stdarg.h>
#include <stdio.h>

uint8_t  g_uart0_rx_buf[USART_REC_LEN];
uint16_t g_uart0_rx_sta;
uint8_t  g_uart2_rx_buf[USART_REC_LEN];
uint16_t g_uart2_rx_sta;

/* ---- uart_printf → UART2 TX (蓝牙, 9600) ---- */
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len < 0) return;

    for (const char *p = buf; *p; p++) {
        DL_UART_Main_transmitData(UART_2_INST, *p);
        while (DL_UART_Main_isBusy(UART_2_INST));  /* 等 TX 完成 */
    }
}

/* ---- UART0 ISR: 接收外部MCU数据 ---- */
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);
        if ((g_uart0_rx_sta & 0x8000) == 0) {
            if (ch == '\r') {
                g_uart0_rx_sta |= 0x4000;
            } else if (ch == '\n') {
                g_uart0_rx_sta |= 0x8000;     /* \n 或 \r\n 都完成 */
            } else {
                uint16_t i = g_uart0_rx_sta & 0x3FFF;
                g_uart0_rx_buf[i] = ch;
                g_uart0_rx_sta = (g_uart0_rx_sta & 0xC000) | ((i + 1) & 0x3FFF);
                if (i >= USART_REC_LEN - 1) g_uart0_rx_sta = 0;
            }
        }
        break;
    }
    default: break;
    }
}

/* ---- UART2 ISR: 接收蓝牙PID命令 ---- */
void UART_2_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t ch = DL_UART_Main_receiveData(UART_2_INST);
        if ((g_uart2_rx_sta & 0x8000) == 0) {
            if (ch == '\r' || ch == '\n') {
                g_uart2_rx_sta |= 0x8000;     /* \r / \n / \r\n 都完成 */
            } else {
                uint16_t i = g_uart2_rx_sta & 0x3FFF;
                g_uart2_rx_buf[i] = ch;
                g_uart2_rx_sta = (g_uart2_rx_sta & 0xC000) | ((i + 1) & 0x3FFF);
                if (i >= USART_REC_LEN - 1) g_uart2_rx_sta = 0;
            }
        }
        break;
    }
    default: break;
    }
}
