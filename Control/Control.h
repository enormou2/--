/*
 * Control.h
 * 小车控制架构 — 平均/差分 PWM + 双模式转向环
 *
 * 架构:
 *   LeftPWM  = clamp(AvgPWM + DiffPWM, 0, PWM_PERIOD)
 *   RightPWM = clamp(AvgPWM - DiffPWM, 0, PWM_PERIOD)
 *
 *   速度环 (Speed PID):  AvgPWM  = PID(target_speed,  avg_speed )
 *   转向环 (Steer PID):  DiffPWM = PID(target_steer,  steer_err )
 *     - TRACK 模式: steer_err = Track_Err()  (目标=0)
 *     - ANGLE 模式: steer_err = yaw          (目标=target_yaw)
 */

#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

/* ---- 控制模式 ---- */
typedef enum {
    STEER_MODE_TRACK = 0,  /* 循迹环: Track_Err() → 转向 PID → DiffPWM */
    STEER_MODE_ANGLE = 1,  /* 角度环: yaw_error   → 转向 PID → DiffPWM */
} steer_mode_t;

/* ---- 可配置参数 ---- */
#define CTRL_PWM_PERIOD      1000      /* PWM 周期 (与 SysConfig 一致) */
#define CTRL_LOOP_FREQ_HZ    100       /* 控制循环频率 Hz */
#define CTRL_BASE_SPEED      400       /* 基础速度 (循迹模式最大速度) */
#define CTRL_MIN_SPEED       100       /* 循迹模式最小速度 */
#define CTRL_MAX_OFFSET_MM   35.0f     /* 循迹传感器最大偏移 mm */
#define CTRL_TARGET_YAW      0.0f      /* 角度模式默认目标角度 */

/* ---- 外露变量 (用于 UART 调试/调参) ---- */
extern steer_mode_t g_steer_mode;
extern float        g_target_speed;    /* 角度模式下固定目标速度 */
extern float        g_target_yaw;      /* 角度模式下目标角度 */

/* ---- API ---- */

/** @brief 初始化控制模块 (PID参数、默认模式) */
void Control_Init(void);

/** @brief 切换转向环模式 */
void Control_SetMode(steer_mode_t mode);

/** @brief 设置目标速度 (角度模式用) */
void Control_SetTargetSpeed(float speed);

/** @brief 设置目标角度 (角度模式用) */
void Control_SetTargetYaw(float yaw);

/** @brief 主控制迭代 — 由定时中断周期性调用 (100Hz) */
void Control_Update(void);

#endif
