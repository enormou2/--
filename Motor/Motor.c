/*
 * Motor.c
 * 双电机驱动 — TB6612 控制 + 双编码器读取
 *
 * TB6612 真值表:
 *   IN1=0, IN2=0 → Stop
 *   IN1=1, IN2=0 → Forward (CW)
 *   IN1=0, IN2=1 → Reverse (CCW)
 *   IN1=1, IN2=1 → Brake
 */

#include "ti_msp_dl_config.h"
#include "Motor/Motor.h"

/* PWM 周期（与 SysConfig 中 PWM Period Count 一致） */
#define PWM_PERIOD  1000

/* ========================================================================
 * Motor_Init
 * ======================================================================== */
void Motor_Init(void)
{
    /* PWM 和 GPIO 的初始化已在 SYSCFG_DL_init() 中完成 */

    /* 确保电机初始停止 */
    DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_AIN1_PIN);
    DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_BIN1_PIN);
    DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_BIN2_PIN);

    /* 启动 PWM 定时器 */
    DL_TimerG_startCounter(PWM_MOTOR_INST);

    /* 启动左编码器 QEI */
    DL_TimerG_startCounter(ENC_L_INST);
}

/* ========================================================================
 * 左电机速度控制
 * ======================================================================== */
void Motor_SetLeftSpeed(int16_t speed)
{
    uint16_t duty;
    bool     forward;

    if (speed > 0) {
        forward = true;
        duty = (speed > PWM_PERIOD) ? PWM_PERIOD : (uint16_t)speed;
    } else if (speed < 0) {
        forward = false;
        duty = (-speed > PWM_PERIOD) ? PWM_PERIOD : (uint16_t)(-speed);
    } else {
        /* Stop */
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
        return;
    }

    /* 方向 */
    if (forward) {
        DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_AIN1_PIN);
        DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_AIN2_PIN);
    }

    /* 占空比 */
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, duty, DL_TIMER_CC_0_INDEX);
}

/* ========================================================================
 * 右电机速度控制
 * ======================================================================== */
void Motor_SetRightSpeed(int16_t speed)
{
    uint16_t duty;
    bool     forward;

    if (speed > 0) {
        forward = true;
        duty = (speed > PWM_PERIOD) ? PWM_PERIOD : (uint16_t)speed;
    } else if (speed < 0) {
        forward = false;
        duty = (-speed > PWM_PERIOD) ? PWM_PERIOD : (uint16_t)(-speed);
    } else {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
        return;
    }

    if (forward) {
        DL_GPIO_setPins(BIN1_PORT, BIN1_PIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_BIN1_PIN);
        DL_GPIO_setPins(BIN2_PORT, BIN2_PIN_BIN2_PIN);
    }

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, duty, DL_TIMER_CC_1_INDEX);
}

/* ========================================================================
 * 左编码器 — 硬件 QEI
 * ======================================================================== */
int32_t Motor_GetLeftEncoder(void)
{
    return (int32_t)(int16_t)DL_TimerG_getCount(ENC_L_INST);
}

/* ========================================================================
 * 右编码器 — 软件解码
 * ======================================================================== */

static int32_t  enc_r_count = 0;
static uint8_t  enc_r_last;   /* 上一次 AB 状态 (bit1=A, bit0=B) */

/*
 * 正交解码状态机:
 *   AB: 00 → 01 → 11 → 10 → 00  (正向 +1)
 *   AB: 00 → 10 → 11 → 01 → 00  (反向 -1)
 */
static const int8_t enc_table[4][4] = {
    /* old\new:  00   01   10   11  */
    /*  00  */ {  0,  +1,  -1,   0 },
    /*  01  */ { -1,   0,   0,  +1 },
    /*  10  */ { +1,   0,   0,  -1 },
    /*  11  */ {  0,  -1,  +1,   0 },
};

int32_t Motor_GetRightEncoder(void)
{
    return enc_r_count;
}

void Motor_UpdateRightEncoder(void)
{
    uint32_t a = DL_GPIO_readPins(ENC_R_A_PORT, ENC_R_A_PIN_ENC_R_A_PIN) ? 1 : 0;
    uint32_t b = DL_GPIO_readPins(ENC_R_B_PORT, ENC_R_B_PIN_ENC_R_B_PIN) ? 1 : 0;
    uint8_t  curr = (uint8_t)((a << 1) | b);

    if (curr != enc_r_last) {
        enc_r_count += enc_table[enc_r_last][curr];
        enc_r_last = curr;
    }
}
