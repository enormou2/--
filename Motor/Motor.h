/*
 * Motor.h
 * 双电机驱动模块 — TB6612 + 编码器
 *
 * 硬件:
 *   左电机: PWM=PA12(TIMG0_C0), AIN1=PB13, AIN2=PB14, 编码器=PA6/PA7(TIMG8 QEI)
 *   右电机: PWM=PA13(TIMG0_C1), BIN1=PB15, BIN2=PB16, 编码器=PA17/PA24(软件解码)
 *
 * 速度测量:
 *   编码器脉冲增量 / 采样周期 = 速度 (单位: counts/sample)
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* ---- 速度测量结构体 ---- */
typedef struct {
    int32_t left_delta;   /* 左轮编码器增量 (counts/采样周期) */
    int32_t right_delta;  /* 右轮编码器增量 (counts/采样周期) */
    float   avg;          /* 平均速度 = (left_delta + right_delta) / 2 */
    float   diff;         /* 差分速度 = (left_delta - right_delta) / 2 */
} motor_speed_t;

/* ---- API ---- */

void Motor_Init(void);

/** @brief 左电机速度  -1000..1000 (正=前进, 负=后退, 0=停止) */
void Motor_SetLeftSpeed(int16_t speed);

/** @brief 右电机速度  -1000..1000 (正=前进, 负=后退, 0=停止) */
void Motor_SetRightSpeed(int16_t speed);

/** @brief 左编码器 (软件解码计数值) */
int32_t Motor_GetLeftEncoder(void);

/** @brief 右编码器 (软件解码计数值) */
int32_t Motor_GetRightEncoder(void);

/** @brief 左编码器软件解码 — 主循环或定时中断中周期性调用 (≥1kHz) */
void Motor_UpdateLeftEncoder(void);

/** @brief 右编码器软件解码 — 主循环或定时中断中周期性调用 (≥1kHz) */
void Motor_UpdateRightEncoder(void);

/** @brief 采样编码器并计算速度 — 控制循环定时调用 (e.g. 100Hz) */
void Motor_UpdateSpeed(void);

/** @brief 获取最近一次速度测量结果 */
void Motor_GetSpeed(motor_speed_t *s);

#endif
