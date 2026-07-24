/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"
#include "UART/uart_comm.h"
#include "IMU/IMU.h"
#include "OLED/OLED.h"
#include "PID/PID.h"
#include "Motor/Motor.h"
#include <stdio.h>
#include <string.h>


float ypr[3];

int main(void)
{
    SYSCFG_DL_init();

    /* ---- 使能 NVIC，UART 中断已在 SYSCFG_DL_init 中配置 ---- */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    OLED_Init();

    uart_printf("UART0 initialized.\r\n");
    uart_printf("PA10=TX, PA11=RX, Baud=115200\r\n\r\n");

    /* ---- IMU 初始化 ---- */
    //IMU_init();

    OLED_Printf(30, 30, OLED_8X16, "d");
    OLED_Printf(1, 1, OLED_8X16, "IMU Ready");
    OLED_Update();

    uart_printf("fesfsdf");
    while (1) {
        /* 检查是否收到一行数据 */
        if (g_usart_rx_sta & 0x8000) {
            uint16_t len = g_usart_rx_sta & 0x3FFF;
            g_usart_rx_buf[len] = '\0';
            uart_printf("收到: %s\r\n", g_usart_rx_buf);

            /* 回显 */
            uart_printf("回显: ");
            const char *p = (const char *)g_usart_rx_buf;
            while (*p) {
                DL_UART_Main_transmitData(UART_0_INST, *p);
                delay_cycles(3200);
                p++;
            }
            uart_printf("\r\n");

            g_usart_rx_sta = 0;
        }

        /* ---- 周期性输出 IMU 姿态角度 ---- */
        {
            static uint32_t tick = 0;
            if (++tick >= 100000) {
                tick = 0;
                IMU_getYawPitchRoll(ypr);
                uart_printf("YPR: %.2f  %.2f  %.2f\r\n", ypr[0], ypr[1], ypr[2]);
            }
        }
    }
}
