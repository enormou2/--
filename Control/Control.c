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
float        g_base_speed   = CTRL_BASE_SPEED;   /* 循迹基础速度, 运行时可变 */
float        g_min_speed    = CTRL_MIN_SPEED;     /* 循迹最低速度, 运行时可变 */

/* ---- PID 参数 (UART 可调) ---- */
float g_speed_kp = 1.40f, g_speed_ki = 0.05f,  g_speed_kd = 0.0f;
float g_track_kp = 12.0f,  g_track_ki = 0.15f,  g_track_kd = 2.0f;
float g_angle_kp = 10.0f, g_angle_ki = 0.0f,  g_angle_kd = 0.0f;

/* ---- 内部状态 ---- */
static PID_t speed_pid, track_pid, angle_pid;
static uint8_t g_ramp_cnt = 0;    /* 起步斜坡计数 */
extern float ypr[3];

/* ---- 秒表 ---- */
volatile uint32_t g_lap_ticks       = 0;   /* 100Hz = 每tick 10ms */
volatile uint8_t  g_lap_active      = 0;   /* 0=停止, 1=计时中 */
uint32_t          g_auto_stop_ticks = 0;   /* >0 时 lap_ticks 达到后自动停车 */
uint8_t           g_stop_black_min  = 5;   /* 停车线黑线最少路数 */
uint8_t           g_stop_frames     = 3;   /* 停车线连续帧数 */

/* ---- 模式参数影子存储: 切走保存, 切回恢复 ---- */
typedef struct {
    float base_speed, min_speed;
    float spd_kp, spd_ki, spd_kd;
    float trk_kp, trk_ki, trk_kd;
} mode_params_t;

static mode_params_t g_mp[4];  /* index = bal_mode enum */

/* ========================================================================
 * Control_Init
 * ======================================================================== */
void Control_Init(void)
{
    PID_Init(&speed_pid);
    speed_pid.OutMax      = CTRL_PWM_PERIOD;
    speed_pid.OutMin      = 0;
    speed_pid.ErrorIntMax =  200.0f;
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
        g_ramp_cnt   = 0;    /* 重新开始斜坡 */
    } else {
        g_lap_active = 0;
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

    /* ---- 停止标记: 参数可配置 (MODE2=8路平行, MODE4=5路) ---- */
    {
        static uint8_t stop_cnt = 0;

        uint8_t black_cnt = 0;
        for (int i = 0; i < 8; i++) {
            if (((TrackN >> i) & 1) == 0) black_cnt++;
        }

        if (g_steer_mode == STEER_MODE_TRACK && black_cnt >= g_stop_black_min) {
            if (++stop_cnt >= g_stop_frames) {
                g_lap_active = 0;  /* 停秒表 */
                Control_SetMode(STEER_MODE_IDLE);
                stop_cnt = 0;
                return;
            }
        } else {
            stop_cnt = 0;
        }
    }

    /* ---- 定时停止: MODE4 8s 自动停车 ---- */
    if (g_auto_stop_ticks > 0 && g_lap_ticks >= g_auto_stop_ticks) {
        g_auto_stop_ticks = 0;
        Control_SetMode(STEER_MODE_IDLE);
        return;
    }

    /* ---- IDLE 模式: 不控制电机 ---- */
    if (g_steer_mode == STEER_MODE_IDLE) {
        return;
    }

    Control_SyncPID();

    /* ---- 2. 速度环 ---- */
    if (g_steer_mode == STEER_MODE_TRACK) {
        float ratio = 1.0f - fabsf(err) / CTRL_MAX_OFFSET_MM;
        if (ratio < 0.0f) ratio = 0.0f;
        target_spd = g_min_speed + (g_base_speed - g_min_speed) * ratio;
    } else {
        target_spd = g_target_speed;
    }

    /* ---- 起步斜坡: 前 CTRL_RAMP_TICKS tick 内线性缓加速 ---- */
    if (g_ramp_cnt < CTRL_RAMP_TICKS) {
        g_ramp_cnt++;
        float ramp = (float)g_ramp_cnt / CTRL_RAMP_TICKS;
        if (ramp < 0.25f) ramp = 0.25f;  /* 最低25%, 防止原地不动 */
        target_spd *= ramp;
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
    
    /* ---- 4. 合成 + 输出 ---- */
    float left  = clampf(avg_pwm + diff_pwm, 0.0f, (float)CTRL_PWM_PERIOD);
    float right = clampf(avg_pwm - diff_pwm, 0.0f, (float)CTRL_PWM_PERIOD);

    left_pwm  = (int16_t)left;
    right_pwm = (int16_t)right;

    Motor_SetLeftSpeed(left_pwm);
    Motor_SetRightSpeed(right_pwm);
}

/* ========================================================================
 * Control_SaveParams — 把活跃参数保存到指定模式的影子存储
 * ======================================================================== */
void Control_SaveParams(int mode)
{
    if (mode < 0 || mode > 3) return;
    g_mp[mode].base_speed = g_base_speed;
    g_mp[mode].min_speed  = g_min_speed;
    g_mp[mode].spd_kp     = g_speed_kp;
    g_mp[mode].spd_ki     = g_speed_ki;
    g_mp[mode].spd_kd     = g_speed_kd;
    g_mp[mode].trk_kp     = g_track_kp;
    g_mp[mode].trk_ki     = g_track_ki;
    g_mp[mode].trk_kd     = g_track_kd;
}

/* ========================================================================
 * Control_LoadParams — 从指定模式的影子存储恢复到活跃参数
 * ======================================================================== */
void Control_LoadParams(int mode)
{
    if (mode < 0 || mode > 3) return;
    g_base_speed = g_mp[mode].base_speed;
    g_min_speed  = g_mp[mode].min_speed;
    g_speed_kp   = g_mp[mode].spd_kp;
    g_speed_ki   = g_mp[mode].spd_ki;
    g_speed_kd   = g_mp[mode].spd_kd;
    g_track_kp   = g_mp[mode].trk_kp;
    g_track_ki   = g_mp[mode].trk_ki;
    g_track_kd   = g_mp[mode].trk_kd;
}

/* ========================================================================
 * Control_InitParams — 用 #define 默认值初始化影子存储 (main中调用一次)
 * ======================================================================== */
void Control_InitParams(void)
{
    /* MODE2 */
    g_mp[1].base_speed = SPD2_BASE;
    g_mp[1].min_speed  = SPD2_MIN;
    g_mp[1].spd_kp     = SPD2_KP;
    g_mp[1].spd_ki     = SPD2_KI;
    g_mp[1].spd_kd     = SPD2_KD;
    g_mp[1].trk_kp     = TRK2_KP;
    g_mp[1].trk_ki     = TRK2_KI;
    g_mp[1].trk_kd     = TRK2_KD;

    /* MODE4 */
    g_mp[3].base_speed = SPD4_BASE;
    g_mp[3].min_speed  = SPD4_MIN;
    g_mp[3].spd_kp     = SPD4_KP;
    g_mp[3].spd_ki     = SPD4_KI;
    g_mp[3].spd_kd     = SPD4_KD;
    g_mp[3].trk_kp     = TRK4_KP;
    g_mp[3].trk_ki     = TRK4_KI;
    g_mp[3].trk_kd     = TRK4_KD;
}
