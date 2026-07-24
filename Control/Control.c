/*
 * Control.c
 * 小车控制架构实现
 *
 * 控制循环 (100Hz):
 *   1. 读取编码器 → avg_speed / diff_speed
 *   2. 速度环: target_speed vs avg_speed  → PID → AvgPWM
 *   3. 转向环: steer_err  vs 0/target_yaw → PID → DiffPWM
 *   4. 合成: Left = Avg+Diff, Right = Avg-Diff
 *   5. 输出电机
 */

#include "ti_msp_dl_config.h"
#include "Control/Control.h"
#include "PID/PID.h"
#include "Motor/Motor.h"
#include "Track/Track.h"
#include <math.h>

/* ---- 全局变量 ---- */
steer_mode_t g_steer_mode   = STEER_MODE_TRACK;
float        g_target_speed = 300.0f;     /* 角度模式默认目标速度 (0~PWM_PERIOD) */
float        g_target_yaw   = CTRL_TARGET_YAW;

/* ---- PID 对象 ---- */
static PID_t speed_pid;   /* 速度环 → AvgPWM */
static PID_t steer_pid;   /* 转向环 → DiffPWM */

/* ---- 外部引用 ---- */
extern float ypr[3];  /* ypr[0]=yaw, 由 IMU 50Hz 更新 */

/* ========================================================================
 * Control_Init
 * ======================================================================== */
void Control_Init(void)
{
    /* ---- 速度环 PID 参数 ---- */
    PID_Init(&speed_pid);
    speed_pid.Kp = 1.0f;
    speed_pid.Ki = 0.05f;
    speed_pid.Kd = 0.0f;
    speed_pid.OutMax      =  CTRL_PWM_PERIOD;
    speed_pid.OutMin      =  0;
    speed_pid.ErrorIntMax =  200.0f;
    speed_pid.ErrorIntMin = -200.0f;

    /* ---- 转向环 PID 参数 ---- */
    PID_Init(&steer_pid);
    steer_pid.Kp = 15.0f;
    steer_pid.Ki = 0.1f;
    steer_pid.Kd = 5.0f;
    steer_pid.OutMax      =  500;    /* 差分 PWM 限幅 ±500 */
    steer_pid.OutMin      = -500;
    steer_pid.ErrorIntMax =  100.0f;
    steer_pid.ErrorIntMin = -100.0f;

    /* 默认模式 */
    g_steer_mode = STEER_MODE_TRACK;
}

/* ========================================================================
 * Control_SetMode
 * ======================================================================== */
void Control_SetMode(steer_mode_t mode)
{
    g_steer_mode = mode;
    /* 切换模式时重置转向 PID 积分 */
    steer_pid.ErrorInt = 0.0f;
}

/* ========================================================================
 * Control_SetTargetSpeed / Control_SetTargetYaw
 * ======================================================================== */
void Control_SetTargetSpeed(float speed)
{
    if (speed < 0) speed = 0;
    if (speed > CTRL_PWM_PERIOD) speed = CTRL_PWM_PERIOD;
    g_target_speed = speed;
}

void Control_SetTargetYaw(float yaw)
{
    g_target_yaw = yaw;
}

/* ========================================================================
 * 工具函数: 限幅
 * ======================================================================== */
static float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ========================================================================
 * Control_Update — 主控制迭代 (100Hz 调用)
 * ======================================================================== */
void Control_Update(void)
{
    motor_speed_t spd;
    float track_err = 0.0f, target_spd;
    float avg_pwm, diff_pwm;
    int16_t left_pwm, right_pwm;

    /* ---- 1. 读取速度 & 循迹 ---- */
    Motor_UpdateSpeed();
    Motor_GetSpeed(&spd);
    Read_Track_DATA(&TrackN);            /* 刷新循迹传感器 */
    track_err = Track_Err(0);            /* 统一读取，两种模式都可能用到 */

    /* ---- 2. 速度环 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        /* 循迹模式: 偏移越大速度越慢，中心线最快 */
        float ratio = 1.0f - fabsf(track_err) / CTRL_MAX_OFFSET_MM;
        if (ratio < 0.0f) ratio = 0.0f;
        target_spd = CTRL_MIN_SPEED + (CTRL_BASE_SPEED - CTRL_MIN_SPEED) * ratio;
    } else {
        /* 角度模式: 固定目标速度 */
        target_spd = g_target_speed;
    }

    speed_pid.Target = target_spd;
    speed_pid.Actual = spd.avg;
    PID_Update(&speed_pid);
    avg_pwm = speed_pid.Out;

    /* ---- 3. 转向环 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        /* 循迹环: Target=0, Actual=Track_Err() */
        steer_pid.Target = 0.0f;
        steer_pid.Actual = track_err;
    } else {
        /* 角度环: Target=目标角度, Actual=当前yaw */
        steer_pid.Target = g_target_yaw;
        steer_pid.Actual = ypr[0];
    }
    PID_Update(&steer_pid);
    diff_pwm = steer_pid.Out;

    /* ---- 4. 合成左右轮 PWM ---- */
    float left  = avg_pwm + diff_pwm;
    float right = avg_pwm - diff_pwm;

    left  = clampf(left,  0, CTRL_PWM_PERIOD);
    right = clampf(right, 0, CTRL_PWM_PERIOD);

    left_pwm  = (int16_t)left;
    right_pwm = (int16_t)right;

    /* ---- 5. 输出电机 ---- */
    Motor_SetLeftSpeed(left_pwm);
    Motor_SetRightSpeed(right_pwm);
}
