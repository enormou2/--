/*
 * StepMotor.c — 步进电机驱动 (SysConfig 配置引脚, TIMG7 PWM)
 *
 * SysConfig 实例:
 *   PWM_STEP = TIMG7, PA3(CCP0), BUSCLK/8 → 10MHz
 *   STEP_DIR = PA2 (GPIO 输出)
 *   STEP_EN  = PA4 (GPIO 输出)
 */
#include "ti_msp_dl_config.h"
#include "Motor_stp/StepMotor.h"
#include <math.h>

/* TIMG7 定时器时钟 (BUSCLK / 8 = 10MHz) */
#define PWM_STEP_CLK             PWM_STEP_INST_CLK_FREQ
#define PWM_STEP_PERIOD_MAX      65535
#define PWM_STEP_PERIOD_MIN      10
#define UPDATE_DT                0.01f

/* ---- 内部状态 ---- */
static int32_t  g_position;
static int32_t  g_target;
static bool     g_done;
static bool     g_running;
static bool     g_dir_fwd;
static uint32_t g_move_total;
static uint32_t g_move_count;
static uint32_t g_max_speed   = STEP_DEFAULT_MAX_SPEED;
static uint32_t g_accel       = STEP_DEFAULT_ACCEL;
static uint32_t g_start_speed = STEP_DEFAULT_START_SPEED;
static float    g_current_speed;

/* ---- 频率→period 转换 ---- */
static uint32_t freq_to_period(uint32_t hz)
{
    if (hz == 0) return PWM_STEP_PERIOD_MAX;
    uint32_t p = PWM_STEP_CLK / hz;
    if (p < PWM_STEP_PERIOD_MIN) p = PWM_STEP_PERIOD_MIN;
    if (p > PWM_STEP_PERIOD_MAX) p = PWM_STEP_PERIOD_MAX;
    return (p > 1) ? (p - 1) : p;
}

static void set_pwm_freq(uint32_t hz)
{
    uint32_t period = freq_to_period(hz);
    uint32_t ccr    = (period + 1) / 2;
    DL_TimerG_setLoadValue(PWM_STEP_INST, period);
    DL_TimerG_setCaptureCompareValue(PWM_STEP_INST, ccr, DL_TIMER_CC_0_INDEX);
}

static void start_pulse(void)
{
    g_move_count = 0;
    g_running    = true;
    g_done       = false;

    if (g_dir_fwd)
        DL_GPIO_setPins(DIR_PORT, DIR_PIN);
    else
        DL_GPIO_clearPins(DIR_PORT, DIR_PIN);

    DL_GPIO_setPins(EN_PORT, EN_PIN);

    g_current_speed = (float)g_start_speed;
    set_pwm_freq((uint32_t)g_current_speed);
    DL_TimerG_startCounter(PWM_STEP_INST);
}

static void stop_pulse(void)
{
    DL_TimerG_stopCounter(PWM_STEP_INST);
    g_running = false;
    g_done    = true;
    if (g_dir_fwd)
        g_position += (int32_t)g_move_count;
    else
        g_position -= (int32_t)g_move_count;
}

/* ---- TIMG7 ZERO 中断: 脉冲计数 ---- */
void PWM_STEP_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(PWM_STEP_INST)) {
    case DL_TIMER_IIDX_ZERO:
        if (++g_move_count >= g_move_total) stop_pulse();
        break;
    default: break;
    }
}

/* ---- StepMotor_Init ---- */
void StepMotor_Init(void)
{
    /* EN→PA31(PINCM6), DIR→PA29(PINCM4) */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM6);    /* PA31 → EN */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM4);    /* PA29 → DIR */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_31);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_29);

    /* 初始 PWM 周期 (无脉冲) */
    DL_TimerG_setLoadValue(PWM_STEP_INST, PWM_STEP_PERIOD_MAX);
    DL_TimerG_setCaptureCompareValue(PWM_STEP_INST, 0, DL_TIMER_CC_0_INDEX);

    /* 使能 ZERO 中断 */
    DL_TimerG_enableInterrupt(PWM_STEP_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);

    /* 状态 */
    g_position   = 0;
    g_target     = 0;
    g_done       = true;
    g_running    = false;
    g_move_total = 0;
    g_move_count = 0;

    /* NVIC */
    NVIC_ClearPendingIRQ(PWM_STEP_INST_INT_IRQN);
    NVIC_SetPriority(PWM_STEP_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(PWM_STEP_INST_INT_IRQN);
}

/* ---- 位置控制 ---- */
void StepMotor_SetTarget(int32_t target)
{
    if (target > STEP_SOFT_LIMIT_MAX)  target = STEP_SOFT_LIMIT_MAX;
    if (target < STEP_SOFT_LIMIT_MIN)  target = STEP_SOFT_LIMIT_MIN;
    g_target = target;

    if (g_running) {
        DL_TimerG_stopCounter(PWM_STEP_INST);
        g_running = false;
        if (g_dir_fwd) g_position += (int32_t)g_move_count;
        else           g_position -= (int32_t)g_move_count;
    }

    int32_t delta = g_target - g_position;
    if (delta == 0) { g_done = true; return; }

    g_move_total = (uint32_t)((delta > 0) ? delta : -delta);
    g_dir_fwd = (delta > 0) ? (STEP_DIR_SIGN > 0) : (STEP_DIR_SIGN < 0);
    start_pulse();
}

void StepMotor_Stop(void)
{
    if (!g_running) return;
    DL_TimerG_stopCounter(PWM_STEP_INST);
    g_running = false;
    g_done    = true;
    if (g_dir_fwd) g_position += (int32_t)g_move_count;
    else           g_position -= (int32_t)g_move_count;
}

bool StepMotor_IsDone(void) { return g_done; }

int32_t StepMotor_GetPosition(void)
{
    if (g_running)
        return g_position + (g_dir_fwd ? (int32_t)g_move_count : -(int32_t)g_move_count);
    return g_position;
}

void StepMotor_SetSpeed(uint32_t max_hz, uint32_t accel_hz_per_s)
{
    if (max_hz < 200) max_hz = 200;
    if (max_hz > 10000) max_hz = 10000;
    if (accel_hz_per_s < 1000) accel_hz_per_s = 1000;
    if (accel_hz_per_s > 50000) accel_hz_per_s = 50000;
    g_max_speed = max_hz;
    g_accel     = accel_hz_per_s;
}

void StepMotor_Jog(int32_t delta)
{
    if (delta == 0) return;
    StepMotor_SetTarget(StepMotor_GetPosition() + delta);
}

/* ---- 梯形加减速 (SysTick 100Hz) ---- */
void StepMotor_Update(void)
{
    if (!g_running || g_move_total == 0) return;

    uint32_t remaining = g_move_total - g_move_count;
    if (remaining == 0) { stop_pulse(); return; }

    float decel_dist = (g_current_speed * g_current_speed) / (2.0f * g_accel);

    if (g_current_speed < g_start_speed)
        g_current_speed = (float)g_start_speed;

    if ((float)remaining <= decel_dist + 1.0f) {
        float vt = sqrtf(2.0f * g_accel * (float)remaining);
        if (vt < g_start_speed) vt = (float)g_start_speed;
        if (g_current_speed > vt) {
            g_current_speed -= g_accel * UPDATE_DT;
            if (g_current_speed < vt) g_current_speed = vt;
        }
    } else if (g_current_speed < g_max_speed) {
        g_current_speed += g_accel * UPDATE_DT;
        if (g_current_speed > g_max_speed) g_current_speed = (float)g_max_speed;
    }

    set_pwm_freq((uint32_t)g_current_speed);
}
