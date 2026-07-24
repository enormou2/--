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
 * TIMG6  50Hz  → IMU_getYawPitchRoll() → 姿态更新
 * SysTick 100Hz → Control_Update() → 速度环 + 转向环 → 电机PWM
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
volatile uint32_t g_imu_cnt = 0;  /* IMU ISR 计数器，用于调试 */

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000);
    }
}

/* ---- TIMER_0 (TIMG6) 中断处理：50Hz 驱动 IMU 姿态融合 ---- */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            g_imu_cnt++;
            IMU_getYawPitchRoll(ypr);
            break;
        default:
            break;
    }
}

/* ---- SysTick 中断处理：100Hz 控制循环 ---- */
void SysTick_Handler(void)
{
    Control_Update();
}

/* ---- UART 命令解析 ---- */
static void ParseCommand(const char *cmd)
{
    float val;

    if (strncmp(cmd, "MODE TRACK", 10) == 0) {
        Control_SetMode(STEER_MODE_TRACK);
        uart_printf("-> TRACK mode\r\n");
    }
    else if (strncmp(cmd, "MODE ANGLE", 10) == 0) {
        Control_SetMode(STEER_MODE_ANGLE);
        uart_printf("-> ANGLE mode\r\n");
    }
    else if (sscanf(cmd, "SPEED %f", &val) == 1) {
        Control_SetTargetSpeed(val);
        uart_printf("-> Target speed = %.0f\r\n", val);
    }
    else if (sscanf(cmd, "YAW %f", &val) == 1) {
        Control_SetTargetYaw(val);
        uart_printf("-> Target yaw = %.2f\r\n", val);
    }
    else if (strcmp(cmd, "STAT") == 0) {
        motor_speed_t spd;
        Motor_GetSpeed(&spd);
        uart_printf("M=%s Spd=%.0f Avg=%.1f Diff=%.1f "
                    "Yaw=%.1f Trk=%.1f ImuCnt=%lu\r\n",
                    g_steer_mode == STEER_MODE_TRACK ? "T" : "A",
                    g_target_speed, spd.avg, spd.diff,
                    ypr[0], Track_Err(0), g_imu_cnt);
    }
    else {
        uart_printf("CMD: MODE TRACK|ANGLE, SPEED N, YAW N, STAT\r\n");
    }
}

/* ---- OLED 状态显示 ---- */
static void Display_Update(void)
{
    motor_speed_t spd;
    Motor_GetSpeed(&spd);

    OLED_Clear();

    /* 第1行: 模式 */
    OLED_Printf(0, 0, OLED_8X16,
                g_steer_mode == STEER_MODE_TRACK ? "MODE: TRACK" : "MODE: ANGLE");

    /* 第2行: 航向角 + 目标速度 */
    OLED_Printf(0, 16, OLED_8X16, "Yaw:%6.1f", ypr[0]);

    /* 第3行: 编码器速度 */
    OLED_Printf(0, 32, OLED_8X16, "Spd:%5.0f L:%4ld R:%4ld",
                g_target_speed,
                (long)spd.left_delta, (long)spd.right_delta);

    /* 第4行: 循迹偏移 */
    OLED_Printf(0, 48, OLED_8X16, "Trk:%5.1f", Track_Err(0));

    OLED_Update();
}

int main(void)
{
    SYSCFG_DL_init();

    /* ---- 1. UART 中断使能 ---- */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    uart_printf("\r\n=== CAR START ===\r\n");

    /* ---- 2. IMU 最先初始化 (参照 ICM45686 工程) ---- */
    IMU_init();
    delay_ms(100);

    /* ---- 3. 使能 TIMG6 50Hz 中断 ---- */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    uart_printf("TIMG6 50Hz IMU started\r\n");

    /* ---- 4. OLED ---- */
    OLED_Init();
    delay_ms(50);

    /* ---- 5. 电机 + 控制 ---- */
    Motor_Init();
    Control_Init();

    /* ---- 6. SysTick 100Hz 控制循环 ---- */
    SysTick_Config(CPUCLK_FREQ / CTRL_LOOP_FREQ_HZ);
    uart_printf("SysTick 100Hz Ctrl started\r\n");

    OLED_Printf(0, 0, OLED_8X16, "CAR CONTROL");
    OLED_Printf(0, 16, OLED_8X16, "IMU OK");
    OLED_Update();

    uart_printf("Ready. CMD: MODE/SPEED/YAW/STAT\r\n\r\n");

    while (1) {
        /* 高频轮询右编码器软件解码 */
        Motor_UpdateRightEncoder();

        /* UART 命令处理 */
        if (g_usart_rx_sta & 0x8000) {
            uint16_t len = g_usart_rx_sta & 0x3FFF;
            g_usart_rx_buf[len] = '\0';
            ParseCommand((const char *)g_usart_rx_buf);
            g_usart_rx_sta = 0;
        }

        /* OLED 刷新 ~5Hz + 串口输出 */
        {
            static uint32_t tick = 0;
            if (++tick >= 100000) {
                tick = 0;
                Display_Update();
                uart_printf("%.2f,%.2f,%.2f\r\n",
                            ypr[0], ypr[1], ypr[2]);
            }
        }
    }
}
