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

/* ---- 速度测量内部变量 ---- */
static int32_t  enc_l_prev;      /* 上一次左编码器计数值 */
static int32_t  enc_r_prev;      /* 上一次右编码器计数值 */
static motor_speed_t g_speed;    /* 当前速度测量结果 */
static uint8_t  speed_valid;     /* 首次采样标志 (0=无效) */

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

    /* 左编码器 PB22(A相)+PA7(B相) — 手动配置 (PA6损坏, 未在SysConfig中) */
    DL_GPIO_initDigitalInput(IOMUX_PINCM50);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM50, DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInput(IOMUX_PINCM10);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM10, DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
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
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_PERIOD, DL_TIMER_CC_0_INDEX);
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
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_PERIOD, DL_TIMER_CC_1_INDEX);
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
 * 编码器 — 双路软件解码
 * 正交解码状态机:
 *   AB: 00 → 01 → 11 → 10 → 00  (正向 +1)
 *   AB: 00 → 10 → 11 → 01 → 00  (反向 -1)
 * ======================================================================== */

static const int8_t enc_table[4][4] = {
    /* old\new:  00   01   10   11  */
    /*  00  */ {  0,  +1,  -1,   0 },
    /*  01  */ { -1,   0,   0,  +1 },
    /*  10  */ { +1,   0,   0,  -1 },
    /*  11  */ {  0,  -1,  +1,   0 },
};

/* ---- 左编码器 (原硬件 QEI 可能被 5V 损坏，改用软件解码) ---- */

static int32_t  enc_l_count = 0;
static uint8_t  enc_l_last;

int32_t Motor_GetLeftEncoder(void)
{
    return enc_l_count;
}

void Motor_UpdateLeftEncoder(void)
{
    /* A=PB22(GPIOB) B=PA7(GPIOA) — 跨端口分别读 */
    uint32_t b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_22) ? 1 : 0;
    uint32_t a = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_7)  ? 1 : 0;
    uint8_t  curr = (uint8_t)((a << 1) | b);

    if (curr != enc_l_last) {
        enc_l_count += enc_table[enc_l_last][curr];
        enc_l_last = curr;
    }
}

/* ---- 右编码器 (软件解码) ---- */

static int32_t  enc_r_count = 0;
static uint8_t  enc_r_last;

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

/* ========================================================================
 * Motor_UpdateSpeed — 采样编码器并计算速度
 * 应由控制定时器周期性调用 (e.g. 100Hz)
 * ======================================================================== */
void Motor_UpdateSpeed(void)
{
    int32_t enc_l_now, enc_r_now;
    int32_t dl, dr;

    /* 刷新两个编码器软件解码 */
    Motor_UpdateLeftEncoder();
    Motor_UpdateRightEncoder();
    enc_l_now = Motor_GetLeftEncoder();
    enc_r_now = Motor_GetRightEncoder();

    if (!speed_valid) {
        /* 首次采样，仅记录基准值 */
        enc_l_prev   = enc_l_now;
        enc_r_prev   = enc_r_now;
        speed_valid  = 1;
        g_speed.left_delta  = 0;
        g_speed.right_delta = 0;
        g_speed.avg   = 0.0f;
        g_speed.diff  = 0.0f;
        return;
    }

    /* 计算差值（int32 自动处理 16 位硬件 QEI 翻转） */
    dl = enc_l_now - enc_l_prev;
    dr = enc_r_now - enc_r_prev;

    enc_l_prev = enc_l_now;
    enc_r_prev = enc_r_now;

    g_speed.left_delta  = dl;
    g_speed.right_delta = dr;
    g_speed.avg   = (float)(dl + dr) * 0.5f;
    g_speed.diff  = (float)(dl - dr) * 0.5f;
}

/* ========================================================================
 * Motor_GetSpeed — 获取最近一次速度测量结果
 * ======================================================================== */
void Motor_GetSpeed(motor_speed_t *s)
{
    if (s != 0) {
        *s = g_speed;
    }
}
