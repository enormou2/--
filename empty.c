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
#include "Button/Button.h"
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
    Button_Poll();      /* 按键消抖 */
    Control_Update();   /* 控制循环 */
}

/* ---- UART PID 调参: speed.kp:0.79 ---- */
static void ParseCommand(const char *cmd)
{
    float val;
    char  pid[8];
    char  param[4];

    if (sscanf(cmd, "%7[^.].%3[^:]:%f", pid, param, &val) == 3) {
        if      (strcmp(pid, "speed") == 0 && strcmp(param, "kp") == 0) g_speed_kp = val;
        else if (strcmp(pid, "speed") == 0 && strcmp(param, "ki") == 0) g_speed_ki = val;
        else if (strcmp(pid, "speed") == 0 && strcmp(param, "kd") == 0) g_speed_kd = val;
        else if (strcmp(pid, "track") == 0 && strcmp(param, "kp") == 0) g_track_kp = val;
        else if (strcmp(pid, "track") == 0 && strcmp(param, "ki") == 0) g_track_ki = val;
        else if (strcmp(pid, "track") == 0 && strcmp(param, "kd") == 0) g_track_kd = val;
        else if (strcmp(pid, "angle") == 0 && strcmp(param, "kp") == 0) g_angle_kp = val;
        else if (strcmp(pid, "angle") == 0 && strcmp(param, "ki") == 0) g_angle_ki = val;
        else if (strcmp(pid, "angle") == 0 && strcmp(param, "kd") == 0) g_angle_kd = val;
        else if (strcmp(pid, "angle") == 0 && strcmp(param, "num") == 0) {
            Control_SetTargetYaw(val);
        }
        else if (strcmp(pid, "angle") == 0 && strcmp(param, "avs") == 0) {
            Control_SetTargetSpeed(val);
        }
    }
}

/* ---- OLED 状态显示 ---- */
static void Display_Update(void)
{
    motor_speed_t spd;
    float trk_err;
    const char *mode_str;

    Motor_GetSpeed(&spd);
    trk_err = Track_Err(0);

    if (g_steer_mode == STEER_MODE_IDLE)      mode_str = "IDLE";
    else if (g_steer_mode == STEER_MODE_TRACK) mode_str = "TRACK";
    else                                       mode_str = "ANGLE";

    OLED_Clear();

    /* Y=0:  模式 + Yaw/Pitch/Roll */
    OLED_Printf(0, 0, OLED_6X8, "%-5s Y:%-5.1f P:%-5.1f R:%-5.1f",
                mode_str, ypr[0], ypr[1], ypr[2]);

    /* Y=8:  循迹err + 左右编码器速度 */
    OLED_Printf(0, 8, OLED_6X8, "Trk:%-5.1f L:%-2ld R:%-2ld",
                trk_err, (long)spd.left_delta, (long)spd.right_delta);

    /* Y=16: 平均速度 + 差分速度 */
    OLED_Printf(0, 16, OLED_6X8, "Avg:%5.1f Diff:%5.1f",
                spd.avg, spd.diff);

    /* Y=24: Speed PID */
    OLED_Printf(0, 24, OLED_6X8, "Sp Kp:%-4.2f Ki:%-4.2f Kd:%-4.2f",
                g_speed_kp, g_speed_ki, g_speed_kd);

    /* Y=32: Track PID */
    OLED_Printf(0, 32, OLED_6X8, "Tr Kp:%-4.2f Ki:%-4.2f Kd:%-4.2f",
                g_track_kp, g_track_ki, g_track_kd);

    /* Y=40: Angle PID */
    OLED_Printf(0, 40, OLED_6X8, "An Kp:%-4.2f Ki:%-4.2f Kd:%-4.2f",
                g_angle_kp, g_angle_ki, g_angle_kd);

    OLED_Update();
}

int main(void)
{
    SYSCFG_DL_init();

    /* ---- 1. UART 中断使能 ---- */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    /* ---- 2. IMU 最先初始化 ---- */
    IMU_init();
    delay_ms(100);
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_14);
    /* ---- 3. 使能 TIMG6 50Hz 中断 ---- */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ---- 4. OLED ---- */
    OLED_Init();
    delay_ms(50);

    /* ---- 5. 电机 + 控制 ---- */
    Motor_Init();
    Control_Init();

    /* ---- 6. SysTick 100Hz 控制循环 (优先级2=低, 不阻塞UART/TIMG6) ---- */
    SysTick_Config(CPUCLK_FREQ / CTRL_LOOP_FREQ_HZ);
    NVIC_SetPriority(SysTick_IRQn, 2);

    OLED_Printf(0, 0, OLED_6X8, "CAR CONTROL");
    OLED_Printf(0, 16, OLED_6X8, "BTN: IDLE TRACK ANGLE");
    OLED_Update();
    
    while (1) {
        /* 高频轮询编码器软件解码 */
        Motor_UpdateLeftEncoder();
        Motor_UpdateRightEncoder();

        /* 按键模式切换 */
        if (Button_IsPressed()) {
            switch (g_steer_mode) {
            case STEER_MODE_IDLE:  Control_SetMode(STEER_MODE_TRACK); break;
            case STEER_MODE_TRACK: Control_SetMode(STEER_MODE_ANGLE); break;
            case STEER_MODE_ANGLE: Control_SetMode(STEER_MODE_IDLE); break;
            }
        }

        /* UART PID 调参 */
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
                
                //uart_printf("%.2f,%.2f,%.2f\r\n", ypr[0], ypr[1], ypr[2]);
            }
        }
        
    }
}
