/*
 * uart_comm.c
 * UART 通信模块实现
 *   - uart_printf: 格式化字符串通过 UART0 发送
 *   - UART_0_INST_IRQHandler: 中断接收，带 \r\n 换行检测
 */

#include "ti_msp_dl_config.h"
#include "UART/uart_comm.h"
#include <stdarg.h>
#include <stdio.h>

/* ---- 接收缓冲区 ---- */
uint8_t  g_usart_rx_buf[USART_REC_LEN];
uint16_t g_usart_rx_sta = 0;  /* bit15:接收完成, bit14:收到\r, bit13-0:已接收字节数 */

static uint8_t g_rx_byte;     /* 中断接收单字节缓冲 */

/* ========================================================================
 * uart_printf — 格式化输出
 * ======================================================================== */
void uart_printf(const char *fmt, ...)
{
    char    buf[256];
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0)
        return;

    const char *p = buf;
    while (*p) {
        DL_UART_Main_transmitData(UART_0_INST, *p);
        delay_cycles(3200);   /* 32MHz 下约 100us，匹配 115200 波特率 */
        p++;
    }
}

/* ========================================================================
 * UART_0_INST_IRQHandler — 中断接收，\r\n 换行检测
 * ======================================================================== */
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        g_rx_byte = DL_UART_Main_receiveData(UART_0_INST);

        if ((g_usart_rx_sta & 0x8000) == 0) {
            if (g_rx_byte == 0x0d) {          /* \r: 标记 */
                g_usart_rx_sta |= 0x4000;
            } else if (g_rx_byte == 0x0a) {   /* \n: 完成 (兼容 \n 和 \r\n) */
                g_usart_rx_sta |= 0x8000;
            } else {
                if (g_usart_rx_sta & 0x4000) {
                    g_usart_rx_sta = 0;       /* \r 后不是 \n, 丢弃 */
                }
                g_usart_rx_buf[g_usart_rx_sta & 0x3FFF] = g_rx_byte;
                g_usart_rx_sta++;
                if ((g_usart_rx_sta & 0x3FFF) > (USART_REC_LEN - 1)) {
                    g_usart_rx_sta = 0;
                }
            }
        }
        break;

    default:
        break;
    }
}
