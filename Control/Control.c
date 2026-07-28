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
steer_mode_t g_steer_mode   = STEER_MODE_IDLE;
float        g_target_speed = 0.0f;
float        g_target_yaw   = CTRL_TARGET_YAW;

/* ---- PID 参数 (UART 可调) ---- */
float g_speed_kp = 1.0f,  g_speed_ki = 0.0f,  g_speed_kd = 0.0f;
float g_track_kp = 15.0f, g_track_ki = 0.0f, g_track_kd = 0.0f;
float g_angle_kp = 10.0f, g_angle_ki = 0.0f,g_angle_kd = 0.0f;

/* ---- PID 对象 ---- */
static PID_t speed_pid;   /* 速度环 → AvgPWM */
static PID_t track_pid;   /* 循迹环 → DiffPWM */
static PID_t angle_pid;   /* 角度环 → DiffPWM */

/* ---- 外部引用 ---- */
extern float ypr[3];  /* ypr[0]=yaw, 由 IMU 50Hz 更新 */

/* ========================================================================
 * Control_Init
 * ======================================================================== */
void Control_Init(void)
{
    /* ---- 速度环 PID ---- */
    PID_Init(&speed_pid);
    speed_pid.OutMax      =  CTRL_PWM_PERIOD;
    speed_pid.OutMin      =  0;
    speed_pid.ErrorIntMax =  200.0f;
    speed_pid.ErrorIntMin = -200.0f;

    /* ---- 循迹环 PID ---- */
    PID_Init(&track_pid);
    track_pid.OutMax      =  500;
    track_pid.OutMin      = -500;
    track_pid.ErrorIntMax =  100.0f;
    track_pid.ErrorIntMin = -100.0f;

    /* ---- 角度环 PID ---- */
    PID_Init(&angle_pid);
    angle_pid.OutMax      =  500;
    angle_pid.OutMin      = -500;
    angle_pid.ErrorIntMax =  100.0f;
    angle_pid.ErrorIntMin = -100.0f;

    /* 默认模式: 空闲 */
    g_steer_mode = STEER_MODE_IDLE;
}

/* ========================================================================
 * Control_SyncPID — 把外露参数同步到 PID 结构体
 * ======================================================================== */
static void Control_SyncPID(void)
{
    speed_pid.Kp = g_speed_kp; speed_pid.Ki = g_speed_ki; speed_pid.Kd = g_speed_kd;
    track_pid.Kp = g_track_kp; track_pid.Ki = g_track_ki; track_pid.Kd = g_track_kd;
    angle_pid.Kp = g_angle_kp; angle_pid.Ki = g_angle_ki; angle_pid.Kd = g_angle_kd;
}

/* ========================================================================
 * Control_SetMode
 * ======================================================================== */
void Control_SetMode(steer_mode_t mode)
{
    g_steer_mode = mode;
    /* 切换模式时重置对应 PID 积分 */
    if (mode == STEER_MODE_TRACK) {
        track_pid.ErrorInt = 0.0f;
    } else if (mode == STEER_MODE_ANGLE) {
        angle_pid.ErrorInt = 0.0f;
    } else {
        Motor_SetLeftSpeed(0);
        Motor_SetRightSpeed(0);
    }
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

    /* ---- 始终读取速度 & 循迹 (IDLE 也需要更新 OLED 显示) ---- */
    Motor_UpdateSpeed();
    Motor_GetSpeed(&spd);
    Read_Track_DATA(&TrackN);
    track_err = Track_Err(0);

    /* ---- IDLE 模式: 不控制电机 ---- */
    if (g_steer_mode == STEER_MODE_IDLE) {
        return;
    }

    /* ---- 同步 PID 参数 ---- */
    Control_SyncPID();

    /* ---- 2. 速度环 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        float ratio = 1.0f - fabsf(track_err) / CTRL_MAX_OFFSET_MM;
        if (ratio < 0.0f) ratio = 0.0f;
        target_spd = CTRL_MIN_SPEED + (CTRL_BASE_SPEED - CTRL_MIN_SPEED) * ratio;
    } else {
        target_spd = g_target_speed;
    }

    speed_pid.Target = target_spd;
    speed_pid.Actual = (fabsf((float)spd.left_delta) + fabsf((float)spd.right_delta)) * 0.5f;
    PID_Update(&speed_pid);
    avg_pwm = speed_pid.Out;

    /* ---- 3. 转向环 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        track_pid.Target = 0.0f;
        track_pid.Actual = -track_err;
        PID_Update(&track_pid);
        diff_pwm = track_pid.Out;
    } else {
        angle_pid.Target = g_target_yaw;
        angle_pid.Actual = ypr[0];
        PID_Update(&angle_pid);
        diff_pwm = angle_pid.Out;
    }

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