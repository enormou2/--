/*
 * uart_comm.h
 * UART 通信模块 — 格式化打印 + 中断接收
 */
#ifndef UART_COMM_H_
#define UART_COMM_H_

#include <stdint.h>

/* ---- 接收缓冲区长度 ---- */
#define USART_REC_LEN  200

/* ---- 接收缓冲区及状态（供 main 或其他模块读取已接收的行） ---- */
extern uint8_t  g_usart_rx_buf[USART_REC_LEN];
extern uint16_t g_usart_rx_sta;   /* bit15:接收完成, bit14:收到\r, bit13-0:已接收字节数 */

/* ---- API ---- */

/**
 * @brief 通过 UART0 发送格式化字符串（非阻塞，轮询方式逐字节发送）。
 */
void uart_printf(const char *fmt, ...);

#endif /* UART_COMM_H_ */
