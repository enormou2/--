/*
 * Balance.c — 小球平衡控制 (UART0 接收K230数据, 增量式PID + 步进电机)
 */
#include "ti_msp_dl_config.h"
#include "Motor_stp/StepMotor.h"
#include "Motor_stp/Balance.h"
#include "UART/uart_comm.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---- PID 参数 ---- */
float g_bal_kp = BAL_KP_DEFAULT;
float g_bal_ki = BAL_KI_DEFAULT;
float g_bal_kd = BAL_KD_DEFAULT;

/* ---- 内部状态 ---- */
static float   g_ball_pos;
static bool    g_cam_valid;
static float   g_target_pos;
static float   g_error, g_error_prev, g_error_prev2;
static int32_t g_step_target;
static uint8_t g_settle_cnt;

/* ---- 解析纯数字 "3.2" / "-1.5" 格式 ---- */
static bool parse_camera_data(const char *buf, float *pos)
{
    char *end;
    float val = strtof(buf, &end);
    if (end == buf) return false;
    *pos = val;
    return true;
}

/* ---- Balance_Init ---- */
void Balance_Init(void)
{
    /* UART0 已在 SysConfig 配置 (115200, PA10/PA11), ISR 在 uart_comm.c */
    g_ball_pos    = 0.0f;
    g_cam_valid   = false;
    g_target_pos  = 0.0f;
    g_error       = 0.0f;
    g_error_prev  = 0.0f;
    g_error_prev2 = 0.0f;
    g_step_target = 0;
    g_settle_cnt  = 0;
}

/* ---- 检查 UART0 是否有新摄像头帧 ---- */
static bool balance_check_frame(float *pos)
{
    if (!(g_uart0_rx_sta & 0x8000)) return false;

    uint16_t len = g_uart0_rx_sta & 0x3FFF;
    g_uart0_rx_buf[len] = '\0';
    bool ok = parse_camera_data((const char *)g_uart0_rx_buf, pos);
    g_uart0_rx_sta = 0;
    return ok;
}

/* ---- Balance_Update ---- */
void Balance_Update(void)
{
    static uint8_t lost_cnt = 0;

    /* 检查是否有新帧 */
    if (!balance_check_frame(&g_ball_pos)) {
        /* 丢球超时 ~500ms → 停止电机, 回到中心 */
        if (++lost_cnt >= 12) {
            StepMotor_Stop();
            g_step_target = StepMotor_GetPosition();  /* 同步位置 */
        }
        return;
    }

    lost_cnt = 0;
    g_cam_valid = true;
    float error = g_target_pos - g_ball_pos;

    /* 增量式 PID */
    g_error_prev2 = g_error_prev;
    g_error_prev  = g_error;
    g_error       = error;

    float delta = g_bal_kp * (g_error - g_error_prev)
                + g_bal_ki * g_error
                + g_bal_kd * (g_error - 2.0f * g_error_prev + g_error_prev2);

    if (delta > BAL_DELTA_MAX)  delta = BAL_DELTA_MAX;
    if (delta < BAL_DELTA_MIN)  delta = BAL_DELTA_MIN;

    g_step_target += (int32_t)delta;
    if (g_step_target > STEP_SOFT_LIMIT_MAX) g_step_target = STEP_SOFT_LIMIT_MAX;
    if (g_step_target < STEP_SOFT_LIMIT_MIN) g_step_target = STEP_SOFT_LIMIT_MIN;

    StepMotor_SetTarget(g_step_target);

    /* 到位判断 */
    float ae = (error > 0) ? error : -error;
    if (ae < BAL_SETTLE_THRESHOLD) {
        if (g_settle_cnt < BAL_SETTLE_COUNT) g_settle_cnt++;
    } else {
        g_settle_cnt = 0;
    }
}

void Balance_SetTarget(float pos_cm) { g_target_pos = pos_cm; g_settle_cnt = 0; }
bool   Balance_IsSettled(void)       { return (g_settle_cnt >= BAL_SETTLE_COUNT); }
float  Balance_GetPosition(void)     { return g_ball_pos; }
int32_t Balance_GetMotorSteps(void)  { return g_step_target; }
void Balance_ParseChar(uint8_t ch)   { (void)ch; }
