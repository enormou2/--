/*
 * uart_comm.h
 * UART0: 115200 RX 接收其他MCU数据 (小球位置)
 * UART2: 115200 TX+RX 蓝牙PID调试
 */
#ifndef UART_COMM_H_
#define UART_COMM_H_

#include <stdint.h>

#define USART_REC_LEN  200

/* UART0 接收缓冲区 (外部MCU数据) */
extern uint8_t  g_uart0_rx_buf[USART_REC_LEN];
extern uint16_t g_uart0_rx_sta;

/* UART2 接收缓冲区 (蓝牙PID调试) */
extern uint8_t  g_uart2_rx_buf[USART_REC_LEN];
extern uint16_t g_uart2_rx_sta;

/* 兼容旧代码: 指向 UART2 蓝牙 */
#define g_usart_rx_buf  g_uart2_rx_buf
#define g_usart_rx_sta  g_uart2_rx_sta

/* API */
void uart_printf(const char *fmt, ...);  /* → UART2 (蓝牙) */

#endif
