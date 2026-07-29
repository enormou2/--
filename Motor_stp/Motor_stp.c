/**
 * motor.c - 步进电机 PWM 驱动实现
 *
 * 适配目标工程（双路共用一个定时器 TIMA0）：
 *   - X ?= CC3 (PB24), Y ?= CC0 (PA8)
 *   - 两路共用定时器周期，因此两路速度相同
 *   - 同一定时器上两路独立 CC 通道，互不干 ?
 *
 * 本层只关 ?怎么输出 STEP 脉冲" ?
 *   - 根据 RPM 计算 PWM 周期
 *   - 根据速度方向设置 DIR
 *   - 停止 STEP 输出
 *   - 使能/关闭 PWM 中断用于位置计步
 *
 * "要转多少角度" ?control.c 的职责 ?
 */
#include "motor.h"

volatile uint16_t Step_Period_X = 0U;
volatile uint16_t Step_Period_Y = 0U;

float Target_Speed_X = 0.0f;
float Target_Speed_Y = 0.0f;

/*
 * Motor_Init() — ?重配置步进电机定时器时钟 ?1MHz
 *
 * SysConfig 默认配置 TIMA0 时钟 ?32MHz（无预分频） ?
 * 为了与原工程的速度计算公式兼容，将预分频设 ?31 ?
 * 得到 32MHz / (31+1) = 1MHz ?
 *
 * 注意：须 ?SYSCFG_DL_init() 之后调用 ?
 */
void Motor_Init(void)
{
    DL_TimerA_ClockConfig clkCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 31U,   /* 32MHz / 32 = 1MHz */
    };

    DL_TimerA_stopCounter(GIMBAL_STEP_TIMER);
    DL_TimerA_setClockConfig(GIMBAL_STEP_TIMER, &clkCfg);

    /* PWM 模式重初始化，使用当前时钟配 ?*/
    DL_TimerA_initPWMMode(GIMBAL_STEP_TIMER,
        &(DL_TimerA_PWMConfig){
            .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
            .period            = 1000,
            .isTimerWithFourCC = true,
            .startTimer        = DL_TIMER_STOP,
        });

    /* 恢复 CCP 输出方向 */
    DL_TimerA_setCCPDirection(GIMBAL_STEP_TIMER,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC3_OUTPUT);
}

uint16_t Motor_RpmToPeriodTicks(float abs_rpm)
{
    uint32_t period;

    if (abs_rpm < 0.5f) {
        return 0U;
    }

    period = (uint32_t)(SPEED_CONST_FACTOR /
        (abs_rpm * (float)MOTOR_STEPS_PER_REV));

    if (period < MIN_PWM_PERIOD) {
        period = MIN_PWM_PERIOD;
    }
    if (period > MAX_PWM_PERIOD) {
        period = MAX_PWM_PERIOD;
    }

    return (uint16_t)period;
}

/* 恢复 CCP 输出 ?OCTL 控制 */
static void Motor_UsePWMOutput(GPTIMER_Regs *timer)
{
    DL_TimerA_setCCPOutputDisabled(timer,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

/*
 * 设置 PWM 周期和指定通道的占空比 (50%)
 *
 * ★ 关键适配：两路 STEP 共用 TIMA0，不能 stop/start 整个定时器，
 *    否则另一路会短暂丢失脉冲。直接更新寄存器，新值立即生效。
 */
static void Motor_SetPWMTicks(GPTIMER_Regs *timer,
    DL_TIMER_CC_INDEX step_cc_index, uint16_t period_ticks)
{
    uint32_t load_value = (uint32_t)period_ticks - 1U;
    uint32_t duty_ticks = (uint32_t)period_ticks >> 1;

    /* 清除之前可能由 Motor_StopPWM 设置的 SW 强制输出，恢复 PWM 控制 */
    DL_Timer_overrideCCPOut(timer,
        DL_TIMER_FORCE_OUT_DISABLED,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED,
        step_cc_index);

    Motor_UsePWMOutput(timer);
    DL_TimerA_setLoadValue(timer, load_value);
    DL_TimerA_setTimerCount(timer, load_value);
    DL_TimerA_setCaptureCompareValue(timer, duty_ticks, step_cc_index);
}

/*
 * 停止指定通道 ?PWM 输出（仅停止该通道，不影响另一路）
 *
 *  ?关键适配：原工程用独立定时器可以停整 ?timer ?
 *    现在两路共用，必须使 ?SW override 单独拉低指定通道 ?
 *
 *    DL_Timer_overrideCCPOut 强制指定 CCP 引脚 ?LOW ?
 *    D36A  ?STEP 是上升沿有效，强 ?LOW 后不再产 ?STEP 脉冲 ?
 */
static void Motor_StopPWM(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX step_cc_index)
{
    DL_Timer_overrideCCPOut(timer,
        DL_TIMER_FORCE_OUT_LOW,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED,
        step_cc_index);
}

void Motor_EnableStepInterrupt(uint8_t axis)
{
    if (axis == GIMBAL_STEP1) {
        DL_TimerA_clearInterruptStatus(GIMBAL_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_enableInterrupt(GIMBAL_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        NVIC_ClearPendingIRQ(STEPPER_MOTOR_INST_INT_IRQN);
        NVIC_EnableIRQ(STEPPER_MOTOR_INST_INT_IRQN);
    } else if (axis == GIMBAL_STEP2) {
        /* Y 轴也使用同一定时器中断，X/Y 共用一 ?ISR */
        DL_TimerA_clearInterruptStatus(GIMBAL_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_enableInterrupt(GIMBAL_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        NVIC_ClearPendingIRQ(STEPPER_MOTOR_INST_INT_IRQN);
        NVIC_EnableIRQ(STEPPER_MOTOR_INST_INT_IRQN);
    }
}

/*
 * 禁用中断（仅当两路都停时才真正关闭硬件中断）
 * 共用中断，停一路时不能关，否则另一路无法计步。
 */
void Motor_DisableStepInterrupt(uint8_t axis)
{
    (void)axis;
    /* 两路都停了才关 */
    if ((Step_Period_X == 0U) && (Step_Period_Y == 0U)) {
        DL_TimerA_disableInterrupt(GIMBAL_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        DL_TimerA_clearInterruptStatus(GIMBAL_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    }
}

void Motor_StopAxis(uint8_t axis)
{
    if (axis == GIMBAL_STEP1) {
        Step_Period_X = 0U;
        Target_Speed_X = 0.0f;
        Motor_StopPWM(GIMBAL_STEP_TIMER, GIMBAL_X_STEP_CC_INDEX);
        Motor_DisableStepInterrupt(GIMBAL_STEP1);
    } else if (axis == GIMBAL_STEP2) {
        Step_Period_Y = 0U;
        Target_Speed_Y = 0.0f;
        Motor_StopPWM(GIMBAL_STEP_TIMER, GIMBAL_Y_STEP_CC_INDEX);
        Motor_DisableStepInterrupt(GIMBAL_STEP2);
    }
}

void Motor_SetAxisSpeed_RPM(uint8_t axis, float rpm)
{
    float abs_speed;
    uint16_t new_period;

    if (rpm > SPEED_LIMIT) {
        rpm = SPEED_LIMIT;
    }
    if (rpm < -SPEED_LIMIT) {
        rpm = -SPEED_LIMIT;
    }

    abs_speed = (rpm >= 0.0f) ? rpm : -rpm;
    if (abs_speed < 0.5f) {
        Motor_StopAxis(axis);
        return;
    }

    new_period = Motor_RpmToPeriodTicks(abs_speed);

    if (axis == GIMBAL_STEP1) {
        if (rpm > 0.0f) {
            DL_GPIO_setPins(GIMBAL_DIR_PORT, GIMBAL_DIR_X_PIN);
        } else {
            DL_GPIO_clearPins(GIMBAL_DIR_PORT, GIMBAL_DIR_X_PIN);
        }
        Motor_SetPWMTicks(GIMBAL_STEP_TIMER, GIMBAL_X_STEP_CC_INDEX, new_period);
        Step_Period_X = new_period;
        Target_Speed_X = rpm;
    } else if (axis == GIMBAL_STEP2) {
        if (rpm > 0.0f) {
            DL_GPIO_setPins(GIMBAL_DIR_PORT, GIMBAL_DIR_Y_PIN);
        } else {
            DL_GPIO_clearPins(GIMBAL_DIR_PORT, GIMBAL_DIR_Y_PIN);
        }
        Motor_SetPWMTicks(GIMBAL_STEP_TIMER, GIMBAL_Y_STEP_CC_INDEX, new_period);
        Step_Period_Y = new_period;
        Target_Speed_Y = rpm;
    }
}

/* 统一设置两路速度，位置模式内部使 ?Motor_SetAxisSpeed_RPM */
void Set_Motor_Speed_RPM(float rpm_x, float rpm_y)
{
    Motor_SetAxisSpeed_RPM(GIMBAL_STEP1, rpm_x);
    Motor_SetAxisSpeed_RPM(GIMBAL_STEP2, rpm_y);
}
