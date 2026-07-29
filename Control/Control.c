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
/* Kp scales the discrete steering levels; Ki/Kd remain for UART compatibility. */
float g_track_kp = 8.0f,  g_track_ki = 0.0f, g_track_kd = 0.0f;
float g_angle_kp = 10.0f, g_angle_ki = 0.0f,g_angle_kd = 0.0f;

/* ---- PID 对象 ---- */
static PID_t speed_pid;   /* 速度环 → AvgPWM */
static PID_t track_pid;   /* 循迹转向环 → DiffPWM (连续 Track_Err 驱动) */
static PID_t angle_pid;   /* 角度环 → DiffPWM */

typedef enum {
    TRACK_RECOVERY_NONE = 0,
    TRACK_RECOVERY_LEFT,
    TRACK_RECOVERY_RIGHT,
    TRACK_RECOVERY_STOPPED,
} track_recovery_t;

static track_recovery_t s_track_recovery = TRACK_RECOVERY_NONE;
static float            s_track_diff_pwm = 0.0f;
static uint16_t         s_track_lost_ticks = 0;

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

    /* ---- 循迹转向 PID (连续 Track_Err → DiffPWM) ---- */
    PID_Init(&track_pid);
    track_pid.OutMax      =  400;
    track_pid.OutMin      = -400;
    track_pid.ErrorIntMax =  150.0f;
    track_pid.ErrorIntMin = -150.0f;

    /* ---- 角度环 PID ---- */
    PID_Init(&angle_pid);
    angle_pid.OutMax      =  500;
    angle_pid.OutMin      = -500;
    angle_pid.ErrorIntMax =  100.0f;
    angle_pid.ErrorIntMin = -100.0f;

    /* 默认模式: 空闲 */
    g_steer_mode = STEER_MODE_IDLE;
    s_track_recovery = TRACK_RECOVERY_NONE;
    s_track_diff_pwm = 0.0f;
    s_track_lost_ticks = 0;
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
    s_track_recovery = TRACK_RECOVERY_NONE;
    s_track_diff_pwm = 0.0f;
    s_track_lost_ticks = 0;

    /* 切换模式时重置对应 PID 积分 */
    if (mode == STEER_MODE_ANGLE) {
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

static float Control_Slew(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

/*
 * Hybrid steering: feed-forward (immediate) + PID feedback (fine-tuning + integral).
 *
 * With 25mm sensor spacing, only one sensor fires at a time.
 * Track_Err() returns EMA-smoothed discrete levels: 0, ±25, ±45 mm.
 *
 * Feed-forward gives instant strong correction the moment a sensor triggers.
 * PID adds proportional fine-tuning and integral to kill steady-state drift.
 * Integral resets on state change to prevent windup across sensor transitions.
 */
static void Control_UpdateTrackSteering(track_state_t state, float track_err, float *diff_pwm)
{
    float ff = 0.0f;  /* feed-forward base */

    /* ---- Determine feed-forward from discrete sensor state ---- */
    switch (state) {
    case TRACK_STATE_CENTER:
        ff = 0.0f;
        s_track_recovery = TRACK_RECOVERY_NONE;
        s_track_lost_ticks = 0;
        break;

    case TRACK_STATE_LEFT_INNER:
        ff = -CTRL_TRACK_FF_INNER;
        s_track_recovery = TRACK_RECOVERY_LEFT;
        s_track_lost_ticks = 0;
        break;

    case TRACK_STATE_RIGHT_INNER:
        ff =  CTRL_TRACK_FF_INNER;
        s_track_recovery = TRACK_RECOVERY_RIGHT;
        s_track_lost_ticks = 0;
        break;

    case TRACK_STATE_LEFT_EDGE:
        ff = -CTRL_TRACK_FF_EDGE;
        s_track_recovery = TRACK_RECOVERY_LEFT;
        s_track_lost_ticks = 0;
        break;

    case TRACK_STATE_RIGHT_EDGE:
        ff =  CTRL_TRACK_FF_EDGE;
        s_track_recovery = TRACK_RECOVERY_RIGHT;
        s_track_lost_ticks = 0;
        break;

    case TRACK_STATE_LOST:
    case TRACK_STATE_UNKNOWN:
    default:
        /* Lost: search in last known direction, then timeout → straight */
        if (s_track_recovery == TRACK_RECOVERY_LEFT) {
            ff = -CTRL_TRACK_SEARCH_DIFF_PWM;
        } else if (s_track_recovery == TRACK_RECOVERY_RIGHT) {
            ff =  CTRL_TRACK_SEARCH_DIFF_PWM;
        } else {
            s_track_recovery = TRACK_RECOVERY_STOPPED;
            *diff_pwm = 0.0f;
            return;
        }

        if (++s_track_lost_ticks >= CTRL_TRACK_LOST_STOP_TICKS) {
            s_track_recovery = TRACK_RECOVERY_STOPPED;
            *diff_pwm = 0.0f;
            return;
        }

        s_track_diff_pwm = Control_Slew(s_track_diff_pwm, ff, CTRL_TRACK_SEARCH_SLEW_PWM);
        *diff_pwm = s_track_diff_pwm;
        return;
    }

    /* ---- Reset PID integral on state change to prevent windup ---- */
    {
        static track_state_t last_state = TRACK_STATE_CENTER;
        if (state != last_state) {
            track_pid.ErrorInt = 0.0f;
            last_state = state;
        }
    }

    /* ---- PID on EMA-smoothed Track_Err for proportional + integral trim ---- */
    track_pid.Target = 0.0f;
    track_pid.Actual = -track_err;
    PID_Update(&track_pid);

    /* ---- Slew-rate limit total output to prevent sudden jerks ---- */
    {
        static float s_normal_out = 0.0f;
        float target = ff + track_pid.Out;
        s_normal_out = Control_Slew(s_normal_out, target, CTRL_TRACK_SLEW_NORMAL);
        *diff_pwm = s_normal_out;
    }
}

/* ========================================================================
 * Control_Update — 主控制迭代 (100Hz 调用)
 * ======================================================================== */
void Control_Update(void)
{
    motor_speed_t spd;
    float track_err = 0.0f, target_spd;
    track_state_t track_state;
    float avg_pwm, diff_pwm;
    int16_t left_pwm, right_pwm;

    /* ---- 始终读取速度 & 循迹 (IDLE 也需要更新 OLED 显示) ---- */
    Motor_UpdateSpeed();
    Motor_GetSpeed(&spd);
    Read_Track_DATA(&TrackN);
    track_err = Track_Err(0);
    track_state = Track_GetState();

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
        if (track_state == TRACK_STATE_LEFT_EDGE ||
            track_state == TRACK_STATE_RIGHT_EDGE) {
            target_spd = CTRL_MIN_SPEED;
        }
    } else {
        target_spd = g_target_speed;
    }

    speed_pid.Target = target_spd;
    speed_pid.Actual = (fabsf((float)spd.left_delta) + fabsf((float)spd.right_delta)) * 0.5f;
    PID_Update(&speed_pid);
    avg_pwm = speed_pid.Out;

    /* ---- 3. 转向环 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        Control_UpdateTrackSteering(track_state, track_err, &diff_pwm);
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
