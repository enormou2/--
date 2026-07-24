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
 *
 * === 小车控制架构 ===
 * SysTick 100Hz → Control_Update() → 速度环 + 转向环 → 电机PWM
 * TIMG6  50Hz  → IMU_getYawPitchRoll() → 姿态更新
 */

#include "ti_msp_dl_config.h"
#include "UART/uart_comm.h"
#include "IMU/IMU.h"
#include "OLED/OLED.h"
#include "Motor/Motor.h"
#include "Track/Track.h"
#include "Control/Control.h"
#include <stdio.h>
#include <string.h>

/* ---- 全局 ---- */
float ypr[3];

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000);
    }
}

/* ---- SysTick 中断处理：100Hz 控制循环 ---- */
void SysTick_Handler(void)
{
    Control_Update();
    /* IMU 姿态更新 (与主循环配合, 约 50-100Hz) */
    IMU_getYawPitchRoll(ypr);
}

/* ---- UART 命令解析 ---- */
static void ParseCommand(const char *cmd)
{
    float val;

    if (strncmp(cmd, "MODE TRACK", 10) == 0) {
        Control_SetMode(STEER_MODE_TRACK);
        uart_printf("→ 切换为循迹模式\r\n");
    }
    else if (strncmp(cmd, "MODE ANGLE", 10) == 0) {
        Control_SetMode(STEER_MODE_ANGLE);
        uart_printf("→ 切换为角度模式\r\n");
    }
    else if (sscanf(cmd, "SPEED %f", &val) == 1) {
        Control_SetTargetSpeed(val);
        uart_printf("→ 目标速度 = %.0f\r\n", val);
    }
    else if (sscanf(cmd, "YAW %f", &val) == 1) {
        Control_SetTargetYaw(val);
        uart_printf("→ 目标角度 = %.2f\r\n", val);
    }
    else if (strcmp(cmd, "STAT") == 0) {
        motor_speed_t spd;
        Motor_GetSpeed(&spd);
        uart_printf("Mode=%s TargetSpd=%.0f Avg=%.1f Diff=%.1f "
                    "Yaw=%.1f Track=%.1f\r\n",
                    g_steer_mode == STEER_MODE_TRACK ? "TRACK" : "ANGLE",
                    g_target_speed, spd.avg, spd.diff,
                    ypr[0], Track_Err(0));
    }
    else {
        uart_printf("Commands:\r\n");
        uart_printf("  MODE TRACK  - 循迹环模式\r\n");
        uart_printf("  MODE ANGLE  - 角度环模式\r\n");
        uart_printf("  SPEED <n>   - 设置目标速度 (0-1000)\r\n");
        uart_printf("  YAW <deg>   - 设置目标角度\r\n");
        uart_printf("  STAT        - 显示状态\r\n");
    }
}

int main(void)
{
    SYSCFG_DL_init();

    /* ---- 使能 UART 中断 ---- */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    /* ---- 初始化 ---- */
    OLED_Init();
    delay_ms(100);

    Motor_Init();
    Control_Init();

    /* ---- IMU 初始化（必须在定时器中断使能之前完成，避免 I2C 冲突）---- */
    IMU_init();
    delay_ms(100);

    /* ---- 配置 SysTick 100Hz (CPUCLK=32MHz, period=320000) ---- */
    SysTick_Config(CPUCLK_FREQ / CTRL_LOOP_FREQ_HZ);

    /* ---- 启动 ---- */
    OLED_Printf(30, 30, OLED_8X16, "d");
    OLED_Printf(1, 1, OLED_8X16, "CAR Ready");
    OLED_Update();

    uart_printf("\r\n=== 小车控制架构 ===\r\n");
    uart_printf("SysTick 100Hz 控制循环已启动\r\n");
    uart_printf("默认: 循迹模式 | 基础速度=%d | PWM周期=%d\r\n",
                CTRL_BASE_SPEED, CTRL_PWM_PERIOD);
    uart_printf("命令: MODE/SPEED/YAW/STAT\r\n\r\n");

    while (1) {
        /* 高频轮询右编码器软件解码（补充 SysTick 10ms 间隔） */
        Motor_UpdateRightEncoder();

        /* UART 命令处理 */
        if (g_usart_rx_sta & 0x8000) {
            uint16_t len = g_usart_rx_sta & 0x3FFF;
            g_usart_rx_buf[len] = '\0';
            ParseCommand((const char *)g_usart_rx_buf);
            g_usart_rx_sta = 0;
        }

        /* 周期性输出状态 */
        {
            static uint32_t tick = 0;
            if (++tick >= 200000) {
                tick = 0;
                motor_speed_t spd;
                Motor_GetSpeed(&spd);
                uart_printf("[%.1f,%.1f,%.1f] Enc=(%ld,%ld) "
                            "Avg=%.1f Diff=%.1f Mode=%s\r\n",
                            ypr[0], ypr[1], ypr[2],
                            (long)spd.left_delta, (long)spd.right_delta,
                            spd.avg, spd.diff,
                            g_steer_mode == STEER_MODE_TRACK ? "T" : "A");
            }
        }
    }
}
