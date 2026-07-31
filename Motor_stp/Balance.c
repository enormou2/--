/*
 * Balance.c — 小球平衡控制 + MODE1/MODE2 模式切换
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

/* ---- 模式 ---- */
volatile bal_mode_t g_bal_mode   = BAL_MODE1_NORMAL;
volatile uint8_t    g_chal_state = CHAL_STATE_OFF;
volatile uint32_t   g_chal_tick  = 0;

/* ---- 内部状态 ---- */
static float   g_ball_pos;
static bool    g_cam_valid;
static float   g_target_pos;
static float   g_error, g_error_prev, g_error_prev2;
static int32_t g_step_target;
static uint8_t g_settle_cnt;

/* 挑战赛内部 */
static uint8_t  g_chal_phase;     /* 0=去14.5, 1=去35.7 */
static uint8_t  g_chal_btn_last = 1;

/* ---- 解析纯数字 ---- */
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
    g_ball_pos    = 0.0f;
    g_cam_valid   = false;
    g_target_pos  = 25.0f;   /* MODE1 默认中心 */
    g_error       = 0.0f;
    g_error_prev  = 0.0f;
    g_error_prev2 = 0.0f;
    g_step_target = 0;
    g_settle_cnt  = 0;
    g_bal_mode    = BAL_MODE1_NORMAL;
    g_chal_state  = CHAL_STATE_OFF;
    g_chal_tick   = 0;
    g_chal_phase  = 0;
    g_chal_btn_last = 1;
}

/* ---- 检查 UART0 新帧 ---- */
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

    if (!balance_check_frame(&g_ball_pos)) {
        if (++lost_cnt >= 12) {
            StepMotor_Stop();
            g_step_target = StepMotor_GetPosition();
        }
        return;
    }

    lost_cnt = 0;
    g_cam_valid = true;
    float error = g_target_pos - g_ball_pos;

    float ae = (error > 0) ? error : -error;
    if (ae < BAL_SETTLE_THRESHOLD) {
        g_settle_cnt++;
        /* 死区内仍保持电机位置, 防止球漂移 */
        StepMotor_SetTarget(g_step_target);
        return;
    }

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
    g_settle_cnt = 0;
}

/* ---- Balance API ---- */
void Balance_SetTarget(float pos_cm) { g_target_pos = pos_cm; g_settle_cnt = 0; }
bool Balance_IsSettled(void)         { return (g_settle_cnt >= BAL_SETTLE_COUNT); }
float Balance_GetPosition(void)      { return g_ball_pos; }
int32_t Balance_GetMotorSteps(void)  { return g_step_target; }
void Balance_ParseChar(uint8_t ch)   { (void)ch; }

/* ========================================================================
 * Balance_SwitchMode — PA30 按键 MODE1/MODE2 切换
 * 主循环中调用
 * ======================================================================== */
void Balance_SwitchMode(void)
{
    uint8_t btn = DL_GPIO_readPins(GPIO_GRP_4_PORT, GPIO_GRP_4_CHALLENGE_BTN_PIN) ? 1 : 0;
    uint8_t pressed = (g_chal_btn_last == 1 && btn == 0);
    g_chal_btn_last = btn;

    if (!pressed) return;

    if (g_bal_mode == BAL_MODE1_NORMAL) {
        /* → MODE2: 挑战赛 */
        g_bal_mode   = BAL_MODE2_CHALLENGE;
        g_chal_state = CHAL_STATE_RUN;
        g_chal_tick  = 0;
        g_chal_phase = 0;
        Balance_SetTarget(14.5f);
    } else {
        /* → MODE1: 正常 */
        g_bal_mode   = BAL_MODE1_NORMAL;
        g_chal_state = CHAL_STATE_OFF;
        Balance_SetTarget(25.0f);
    }
}

/* ========================================================================
 * 挑战赛状态机 (SysTick 中调用)
 * ======================================================================== */
void Balance_ChallengeUpdate(void)
{
    /* 计时 */
    if (g_chal_state == CHAL_STATE_RUN) {
        g_chal_tick++;
    }

    if (g_bal_mode != BAL_MODE2_CHALLENGE || g_chal_state != CHAL_STATE_RUN)
        return;

    float pos = Balance_GetPosition();
    float ae;

    switch (g_chal_phase) {
    case 0: /* 去 14.5cm: 进入1cm范围就立刻去35.7 */
        ae = pos - 14.5f;
        if (ae < 0) ae = -ae;
        if (ae < 1.0f) {
            g_chal_phase = 1;
            Balance_SetTarget(35.7f);
        }
        break;
    case 1: /* 去 35.7cm: 需要停稳 */
        if (Balance_IsSettled()) {
            g_chal_state = CHAL_STATE_DONE;
        }
        break;
    }
}
