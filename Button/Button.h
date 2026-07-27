/*
 * Button.h
 * 按键模块 — PB21 模式切换按键
 *
 * 模式循环: IDLE → TRACK → ANGLE → IDLE → ...
 *   IDLE:  电机停止，方便调试
 *   TRACK: 循迹环
 *   ANGLE: 角度环 (陀螺仪)
 */

#ifndef __BUTTON_H
#define __BUTTON_H

#include <stdint.h>

/* ---- 按键状态 ---- */
typedef enum {
    BTN_STATE_IDLE = 0,   /* 等待按下 */
    BTN_STATE_PRESS = 1,  /* 按下中 (消抖) */
    BTN_STATE_WAIT = 2,   /* 等待释放 */
} btn_state_t;

/* ---- API ---- */

/** @brief 初始化按键 (GPIO 已在 SysConfig 中配置) */
void Button_Init(void);

/** @brief 周期性轮询按键 (每 10ms 调用一次) */
void Button_Poll(void);

/** @brief 获取当前按键是否刚被按下 (读后自动清零) */
uint8_t Button_IsPressed(void);

#endif
