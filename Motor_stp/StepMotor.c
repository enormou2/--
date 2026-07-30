/*
 * StepMotor.c — 步进电机底层驱动实现
 *
 * ============================ 驱动原理 ============================
 *
 * 1. 脉冲产生:
 *    TIMG7 配置为 Edge-Aligned PWM 模式, CCP0 输出到 PA3
 *    每个 PWM 周期 = 一个 STEP 脉冲 (上升沿有效, 占空比 50%)
 *    修改 TIMG7 period 寄存器 → 改变脉冲频率 → 实现调速
 *
 * 2. 脉冲计数:
 *    TIMG7 ZERO 中断: 每个 PWM 周期溢出一次 → 计数 +1
 *    到达目标脉冲数 → 关闭 PWM → 电机停止
 *
 * 3. 梯形加减速:
 *    在 StepMotor_Update() (SysTick 100Hz) 中计算当前速度
 *    加速段: v += a*dt  (线性加速)
 *    匀速段: v = v_max
 *    减速段: v = sqrt(2*a*remaining)  (确保精确停在目标)
 *
 * 4. DIR 引脚:
 *    delta = target - position
 *    forward = (delta > 0) XOR (STEP_DIR_SIGN < 0)
 *    DIR = forward ? HIGH : LOW
 *
 * 5. EN 引脚:
 *    SetTarget/Jog 时自动拉高 (使能)
 *    Stop 时保持高 (不掉电, 保持力矩)
 *    如需休眠可手动拉低
 *
 * ============================ 时钟参数 ============================
 *
 * BUSCLK = 80MHz (CPUCLK_FREQ)
 * TIMG7 预分频 = /8 → TIMG7 clock = 10MHz
 * 目标频率范围: 200Hz ~ 10kHz
 *   f=200Hz  → period = 10M/200-1 = 49999 (16-bit 安全)
 *   f=10kHz  → period = 10M/10k-1 = 999
 *   全范围均在 16-bit period (65535) 内
 *
 * ============================ 引脚映射 ============================
 *
 * PA3 → IOMUX_PINCM8  → TIMG7 CCP0 (STEP PWM 输出)
 * PA2 → IOMUX_PINCM5  → GPIO 输出 (DIR)
 * PA4 → IOMUX_PINCM7  → GPIO 输出 (EN)
 *
 * ⚠️ MSPM0G3507 没有 TIMG1_CCP0, 选用 TIMG7
 * 以上全部在代码中手动配置, 不经过 SysConfig
 */

#include "ti_msp_dl_config.h"
#include "Motor_stp/StepMotor.h"
#include <math.h>

/* ========================================================================
 * 硬件常量
 * ======================================================================== */

/* TIMG7 定时器时钟 (BUSCLK / prescaler) */
#define TIMG7_CLK_FREQ          (CPUCLK_FREQ / 8)    /* 10MHz */
#define TIMG7_PWM_PERIOD_MAX    65535                 /* 16-bit 上限 */
#define TIMG7_PWM_PERIOD_MIN    10                    /* 防止频率过高 */

/* 更新周期 (s), 与 SysTick 100Hz 一致 */
#define UPDATE_DT               0.01f

/* ========================================================================
 * 静态变量 (模块内部状态)
 * ======================================================================== */

static int32_t  g_position;          /* 当前绝对位置 (步数), 上电=0 */
static int32_t  g_target;            /* 目标绝对位置 */
static bool     g_done;              /* 到位标志 */
static bool     g_running;           /* 电机是否在发脉冲 */

/* 本次运动的参数 */
static bool     g_dir_fwd;           /* 本次运动方向 (考虑了 DIR_SIGN 后的 DIR 引脚电平) */
static uint32_t g_move_total;        /* 本次运动需要发出的总脉冲数 (绝对值) */
static uint32_t g_move_count;        /* 本次运动已发出的脉冲数 */

/* 速度参数 (运行时可变) */
static uint32_t g_max_speed   = STEP_DEFAULT_MAX_SPEED;     /* Hz */
static uint32_t g_accel       = STEP_DEFAULT_ACCEL;         /* Hz/s */
static uint32_t g_start_speed = STEP_DEFAULT_START_SPEED;   /* Hz */
static float    g_current_speed;     /* Hz, 浮点用于平滑过渡 */

/* ========================================================================
 * 内部辅助

/**
 * @brief 设置 TIMG7 PWM 频率
 */
static void set_pwm_freq(uint32_t freq_hz)
{
    uint32_t period = freq_to_period(freq_hz);
    uint32_t ccr    = (period + 1) / 2;  /* 50% 占空比 */

    DL_TimerG_setPeriod(TIMG7, period);
    DL_TimerG_setCaptureCompareValue(TIMG7, ccr, DL_TIMER_CC_0_INDEX);
}函数
 * ======================================================================== */

/**
 * @brief 将频率 (Hz) 转换为 TIMG7 period 寄存器的值
 */
static uint32_t freq_to_period(uint32_t freq_hz)
{
    uint32_t period;
    if (freq_hz == 0) return TIMG7_PWM_PERIOD_MAX;  /* 停止 */
    period = TIMG7_CLK_FREQ / freq_hz;
    if (period < TIMG7_PWM_PERIOD_MIN)  period = TIMG7_PWM_PERIOD_MIN;
    if (period > TIMG7_PWM_PERIOD_MAX)  period = TIMG7_PWM_PERIOD_MAX;
    if (period > 1) period -= 1;  /* period = clk/freq - 1 */
    return period;
}

/**
 * @brief 启动脉冲输出 (使能 PWM 并从起始速度开始)
 */
static void start_pulse(void)
{
    g_move_count = 0;
    g_running    = true;
    g_done       = false;

    /* 设置 DIR */
    if (g_dir_fwd) {
        DL_GPIO_setPins(DIR_PORT, DIR_PIN);
    } else {
        DL_GPIO_clearPins(DIR_PORT, DIR_PIN);
    }

    /* EN 拉高 (使能电机) */
    DL_GPIO_setPins(EN_PORT, EN_PIN);

    /* 从起始速度开始 */
    g_current_speed = (float)g_start_speed;
    set_pwm_freq((uint32_t)g_current_speed);

    /* 清空中断计数, 启动定时器 */
    DL_TimerG_startCounter(TIMG7);
}

/**
 * @brief 停止脉冲输出
 */
static void stop_pulse(void)
{
    DL_TimerG_stopCounter(TIMG7);
    g_running = false;
    g_done    = true;

    /* 更新绝对位置 */
    if (g_dir_fwd) {
        g_position += (int32_t)g_move_count;
    } else {
        g_position -= (int32_t)g_move_count;
    }

    /* EN 保持高 (不掉电, 保持力矩) */
}

/* ========================================================================
 * TIMG7 中断处理 (ZERO 溢出 → 脉冲计数)
 * ======================================================================== */
void TIMG7_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMG7)) {
    case DL_TIMER_IIDX_ZERO:
        g_move_count++;

        if (g_move_count >= g_move_total) {
            /* 到达目标脉冲数 → 停止 */
            stop_pulse();
        }
        break;

    default:
        break;
    }
}

/* ========================================================================
 * StepMotor_Init
 * ======================================================================== */
void StepMotor_Init(void)
{
    /*
     * 引脚定义 (手动配置, 不走 SysConfig):
     *   PA3 → TIMG7 CCP0 (STEP)
     *   PA2 → GPIO 输出 (DIR)
     *   PA4 → GPIO 输出 (EN)
     */

    /* ---- 1. 配置 PA3 → TIMG7 CCP0 (STEP 脉冲) ---- */
    DL_GPIO_initPeripheralFunction(STEP_IOMUX, STEP_IOMUX_FUNC);

    /* ---- 2. 配置 PA2 → GPIO 输出 (DIR) ---- */
    DL_GPIO_initDigitalOutput(DIR_IOMUX);
    DL_GPIO_clearPins(DIR_PORT, DIR_PIN);

    /* ---- 3. 配置 PA4 → GPIO 输出 (EN) ---- */
    DL_GPIO_initDigitalOutput(EN_IOMUX);
    DL_GPIO_clearPins(EN_PORT, EN_PIN);  /* 初始=休眠 */

    /* ---- 4. 配置 TIMG7 PWM 模式 ---- */
    /*
     * TIMG7 没有在 SysConfig 中配置, 需要手动设置。
     */

    /* 时钟配置: BUSCLK, /8 分频 */
    DL_TimerG_setClockConfig(TIMG7,
        DL_TIMER_CLOCK_BUSCLK,          /* 80MHz */
        DL_TIMER_CLOCK_DIVIDE_8);       /* /8 → 10MHz */

    /* PWM 模式: Edge-Aligned, CCP0 输出 */
    DL_TimerG_initPWMMode(TIMG7, DL_TIMER_PWM_EDGE_ALIGN);

    /* 初始 period (无脉冲输出, 相当于停止) */
    DL_TimerG_setPeriod(TIMG7, TIMG7_PWM_PERIOD_MAX);
    DL_TimerG_setCaptureCompareValue(TIMG7, 0,
        DL_TIMER_CC_0_INDEX);           /* 占空比 0%, 无脉冲 */

    /* 使能 ZERO 中断 */
    DL_TimerG_enableInterrupt(TIMG7, DL_TIMER_INTERRUPT_ZERO_EVENT);

    /* ---- 5. 初始化内部状态 ---- */
    g_position    = 0;
    g_target      = 0;
    g_done        = true;
    g_running     = false;
    g_move_total  = 0;
    g_move_count  = 0;

    /* ---- 6. 使能 TIMG7 NVIC 中断 (优先级=1, 与 TIMG6 相同) ---- */
    NVIC_ClearPendingIRQ(TIMG7_INT_IRQn);
    NVIC_SetPriority(TIMG7_INT_IRQn, 1);
    NVIC_EnableIRQ(TIMG7_INT_IRQn);
}

/* ========================================================================
 * StepMotor_SetTarget
 * ======================================================================== */
void StepMotor_SetTarget(int32_t target)
{
    int32_t delta;

    /* 安全限幅 */
    if (target > STEP_SOFT_LIMIT_MAX)  target = STEP_SOFT_LIMIT_MAX;
    if (target < STEP_SOFT_LIMIT_MIN)  target = STEP_SOFT_LIMIT_MIN;

    g_target = target;

    /* 如果正在运动, 先停 */
    if (g_running) {
        DL_TimerG_stopCounter(TIMG7);
        g_running = false;
        /* 更新位置 (已走的部分) */
        if (g_dir_fwd) {
            g_position += (int32_t)g_move_count;
        } else {
            g_position -= (int32_t)g_move_count;
        }
    }

    /* 计算 delta  目标值 - 当前的位置值*/
    delta = g_target - g_position;

    /* 已在目标位置 → 无需运动 */
    if (delta == 0) {
        g_done = true;
        return;
    }

    /* 确定方向和脉冲数 */
    g_move_total = (uint32_t)((delta > 0) ? delta : -delta);

    if (delta > 0) {
        g_dir_fwd = (STEP_DIR_SIGN > 0);
    } else {
        g_dir_fwd = (STEP_DIR_SIGN < 0);
    }

    /* 启动 */
    start_pulse();
}

/* ========================================================================
 * StepMotor_Stop
 * ======================================================================== */
void StepMotor_Stop(void)
{
    if (!g_running) return;

    DL_TimerG_stopCounter(TIMG7);
    g_running = false;
    g_done    = true;

    /* 更新位置 */
    if (g_dir_fwd) {
        g_position += (int32_t)g_move_count;
    } else {
        g_position -= (int32_t)g_move_count;
    }

    /* EN 保持高, 不掉电 */
}

/* ========================================================================
 * StepMotor_IsDone
 * ======================================================================== */
bool StepMotor_IsDone(void)
{
    return g_done;
}

/* ========================================================================
 * StepMotor_GetPosition
 * ======================================================================== */
int32_t StepMotor_GetPosition(void)
{
    /* 如果正在运动, 返回中间位置 */
    if (g_running) {
        if (g_dir_fwd) {
            return g_position + (int32_t)g_move_count;
        } else {
            return g_position - (int32_t)g_move_count;
        }
    }
    return g_position;
}

/* ========================================================================
 * StepMotor_SetSpeed
 * ======================================================================== */
void StepMotor_SetSpeed(uint32_t max_hz, uint32_t accel_hz_per_s)
{
    if (max_hz < 200)   max_hz = 200;
    if (max_hz > 10000) max_hz = 10000;
    if (accel_hz_per_s < 1000)  accel_hz_per_s = 1000;
    if (accel_hz_per_s > 50000) accel_hz_per_s = 50000;

    g_max_speed = max_hz;
    g_accel     = accel_hz_per_s;
}

/* ========================================================================
 * StepMotor_Jog
 * ======================================================================== */
void StepMotor_Jog(int32_t delta)
{
    if (delta == 0) return;

    /* 基于当前位置 + delta 设置目标 */
    int32_t cur = StepMotor_GetPosition();
    StepMotor_SetTarget(cur + delta);
}

/* ========================================================================
 * StepMotor_Update — 梯形加减速 (SysTick 100Hz)
 * ======================================================================== */
void StepMotor_Update(void)
{
    uint32_t remaining;
    float    decel_dist;

    if (!g_running) return;
    if (g_move_total == 0) return;

    /* 剩余步数 */
    remaining = g_move_total - g_move_count;

    /* 计算从当前速度减速到 0 需要的距离  加速度 牛顿第二定律 */
    decel_dist = (g_current_speed * g_current_speed) / (2.0f * g_accel);

    /* ---- 梯形加减速三段逻辑 ---- */

    if (remaining <= 0) {
        /* 已经到位了 (ISR 应该已经停了, 这里是保险) */
        stop_pulse();
        return;
    }

    if (g_current_speed < g_start_speed) {
        /* 确保不低于起始速度 */
        g_current_speed = (float)g_start_speed;
    }

    if ((float)remaining <= decel_dist + 1.0f) {
        /* 减速段: 用剩余距离反算所需速度, 确保精确停在目标 */
        /* v = sqrt(2 * a * s), 但不低于起始速度 */
        float v_target = sqrtf(2.0f * g_accel * (float)remaining);
        if (v_target < g_start_speed) {
            v_target = (float)g_start_speed;
        }
        /* 平滑降速: 不要跳变 */
        if (g_current_speed > v_target) {
            g_current_speed -= g_accel * UPDATE_DT;
            if (g_current_speed < v_target) g_current_speed = v_target;
        }
    } else if (g_current_speed < g_max_speed) {
        /* 加速段: 线性加速 */
        g_current_speed += g_accel * UPDATE_DT;
        if (g_current_speed > g_max_speed) {
            g_current_speed = (float)g_max_speed;
        }
    } else {
        /* 匀速段 */
        g_current_speed = (float)g_max_speed;
    }

    /* 更新 TIMG7 PWM 频率 */
    set_pwm_freq((uint32_t)g_current_speed);
}
