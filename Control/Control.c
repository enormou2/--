/*
 * Control.c — 速度环 + 转向环 (Track_Err → PID)
 *
 * 控制循环 (100Hz SysTick):
 *   1. 读取编码器速度 + 循迹传感器
 *   2. 速度环: target_speed vs avg_speed → PID → AvgPWM
 *   3. 转向环: Track_Err → PID → DiffPWM
 *   4. 合成: Left=Avg+Diff, Right=Avg-Diff → 电机
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
float g_speed_kp = 1.79f, g_speed_ki = 0.13f,  g_speed_kd = 0.0f;
float g_track_kp = 9.5f,  g_track_ki = 0.05f,  g_track_kd = 2.0f;
float g_angle_kp = 10.0f, g_angle_ki = 0.0f,  g_angle_kd = 0.0f;

/* ---- 内部状态 ---- */
static PID_t speed_pid, track_pid, angle_pid;
extern float ypr[3];

/* ---- 秒表 ---- */
volatile uint32_t g_lap_ticks  = 0;   /* 100Hz = 每tick 10ms */
volatile uint8_t  g_lap_active = 0;   /* 0=停止, 1=计时中 */

/* ========================================================================
 * Control_Init
 * ======================================================================== */
void Control_Init(void)
{
    PID_Init(&speed_pid);
    speed_pid.OutMax      = CTRL_PWM_PERIOD;
    speed_pid.OutMin      = 0;
    speed_pid.ErrorIntMax =  020.0f;
    speed_pid.ErrorIntMin = -200.0f;

    PID_Init(&track_pid);
    track_pid.OutMax      =  500;
    track_pid.OutMin      = -500;
    track_pid.ErrorIntMax =  100.0f;
    track_pid.ErrorIntMin = -100.0f;

    PID_Init(&angle_pid);
    angle_pid.OutMax      =  500;
    angle_pid.OutMin      = -500;
    angle_pid.ErrorIntMax =  100.0f;
    angle_pid.ErrorIntMin = -100.0f;

    g_steer_mode = STEER_MODE_IDLE;
}

/* ========================================================================
 * Control_SyncPID — 外露参数同步到 PID 结构体
 * ======================================================================== */
static void Control_SyncPID(void)
{
    speed_pid.Kp = g_speed_kp;
    speed_pid.Ki = g_speed_ki;
    speed_pid.Kd = g_speed_kd;

    track_pid.Kp = g_track_kp;
    track_pid.Ki = g_track_ki;
    track_pid.Kd = g_track_kd;

    angle_pid.Kp = g_angle_kp;
    angle_pid.Ki = g_angle_ki;
    angle_pid.Kd = g_angle_kd;
}

/* ========================================================================
 * Control_SetMode / SetTargetSpeed / SetTargetYaw
 * ======================================================================== */
void Control_SetMode(steer_mode_t m)
{
    g_steer_mode = m;

    if (m == STEER_MODE_TRACK) {
        g_lap_ticks  = 0;
        g_lap_active = 1;
    } else {
        g_lap_active = 0;
    }

    if (m == STEER_MODE_ANGLE) {
        angle_pid.ErrorInt = 0.0f;
    } else {
        Motor_SetLeftSpeed(0);
        Motor_SetRightSpeed(0);
    }
}

void Control_SetTargetSpeed(float s)
{
    if (s < 0)   s = 0;
    if (s > CTRL_PWM_PERIOD) s = CTRL_PWM_PERIOD;
    g_target_speed = s;
}

void Control_SetTargetYaw(float y)
{
    g_target_yaw = y;
}

/* ========================================================================
 * 工具函数
 * ======================================================================== */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ========================================================================
 * Control_Update — 主控制迭代 (100Hz SysTick 调用)
 * ======================================================================== */
void Control_Update(void)
{
    motor_speed_t spd;
    float err, target_spd, avg_pwm, diff_pwm;
    int16_t left_pwm, right_pwm;

    /* ---- 1. 传感器读取 ---- */
    Motor_UpdateSpeed();
    Motor_GetSpeed(&spd);
    Read_Track_DATA(&TrackN);
    err = Track_Err(0);

    /* ---- 秒表: TRACK模式下计时 ---- */
    if (g_lap_active) g_lap_ticks++;

    /* ---- 停止标记: 停车线18mm宽, ≥5路黑线连续≥3帧 ---- */
    /* 检测到停车线后滑行 STOP_COAST_TICKS 个周期再刹停 */
    #define STOP_COAST_TICKS  15   /* 100Hz, 15 ticks = 150ms */
    {
        static uint8_t stop_cnt  = 0;
        static uint8_t coasting  = 0;  /* 0=正常, 1=滑行中 */
        static uint8_t coast_cnt = 0;

        uint8_t black_cnt = 0;
        for (int i = 0; i < 8; i++) {
            if (((TrackN >> i) & 1) == 0) black_cnt++;
        }

        if (coasting) {
            /* 滑行阶段: 继续正常控制, 倒计时结束后停车 */
            if (--coast_cnt == 0) {
                Control_SetMode(STEER_MODE_IDLE);
                coasting  = 0;
                stop_cnt  = 0;
                return;
            }
        } else if (g_steer_mode == STEER_MODE_TRACK && black_cnt >= 5) {
            if (++stop_cnt >= 3) {
                g_lap_active = 0;                 /* 停秒表 */
                coasting     = 1;
                coast_cnt    = STOP_COAST_TICKS;   /* 进入滑行阶段 */
                stop_cnt     = 0;
            }
        } else {
            stop_cnt = 0;
        }
    }

    /* ---- IDLE 模式: 不控制电机 ---- */
    if (g_steer_mode == STEER_MODE_IDLE) {
        return;
    }

    Control_SyncPID();

    /* ---- 2. 速度环: 固定目标速度, 不随循迹误差缩放 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        target_spd = CTRL_BASE_SPEED;
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
        track_pid.Actual = -err;
        PID_Update(&track_pid);
        diff_pwm = track_pid.Out;
    } else {
        angle_pid.Target = g_target_yaw;
        angle_pid.Actual = ypr[0];
        PID_Update(&angle_pid);
        diff_pwm = angle_pid.Out;
    }

    /* ---- 4. 合成 + 输出: 限制差速, 防止单轮夹死到0 ---- */
    #define CTRL_MIN_WHEEL_PWM  30   /* 单轮最低PWM, 防止骤停 */
    {
        float max_diff = avg_pwm - CTRL_MIN_WHEEL_PWM;
        if (max_diff < 0.0f) max_diff = 0.0f;
        if (diff_pwm >  max_diff) diff_pwm =  max_diff;
        if (diff_pwm < -max_diff) diff_pwm = -max_diff;
    }

    float left  = clampf(avg_pwm + diff_pwm, 0.0f, (float)CTRL_PWM_PERIOD);
    float right = clampf(avg_pwm - diff_pwm, 0.0f, (float)CTRL_PWM_PERIOD);

    left_pwm  = (int16_t)left;
    right_pwm = (int16_t)right;

    Motor_SetLeftSpeed(left_pwm);
    Motor_SetRightSpeed(right_pwm);
}
