/*
 * Button.c
 * 按键模块实现 — 消抖 + 短按检测
 *
 * 硬件: PB21, 上拉输入, 按下 = GND (低电平)
 */

#include "ti_msp_dl_config.h"
#include "Button/Button.h"

/* ---- 消抖时间 (ms) ---- */
#define BTN_DEBOUNCE_MS  50

static btn_state_t g_btn_state = BTN_STATE_IDLE;
static uint8_t     g_btn_pressed = 0;   /* 按下标志 (读后清零) */
static uint32_t    g_btn_timer = 0;     /* 消抖计时器 (ms) */

/* ========================================================================
 * Button_Init
 * ======================================================================== */
void Button_Init(void)
{
    g_btn_state   = BTN_STATE_IDLE;
    g_btn_pressed = 0;
    g_btn_timer   = 0;
}

/* ========================================================================
 * Button_Poll — 每 10ms 调用一次 (由 SysTick 驱动)
 * ======================================================================== */
void Button_Poll(void)
{
    uint8_t level = DL_GPIO_readPins(GPIO_GRP_0_PORT, GPIO_GRP_0_BTN_PIN) ? 1 : 0;
    /* level=1: 未按下 (上拉), level=0: 按下 (GND) */

    switch (g_btn_state) {
    case BTN_STATE_IDLE:
        if (level == 0) {                          /* 检测到按下 */
            g_btn_state = BTN_STATE_PRESS;
            g_btn_timer = 0;
        }
        break;

    case BTN_STATE_PRESS:
        g_btn_timer += 10;                         /* 每次 Poll 10ms */
        if (level == 1) {                          /* 提前释放，回到空闲 */
            g_btn_state = BTN_STATE_IDLE;
        } else if (g_btn_timer >= BTN_DEBOUNCE_MS) { /* 消抖完成 */
            g_btn_pressed = 1;
            g_btn_state   = BTN_STATE_WAIT;
        }
        break;

    case BTN_STATE_WAIT:
        if (level == 1) {                          /* 按键释放 */
            g_btn_state = BTN_STATE_IDLE;
        }
        break;
    }
}

/* ========================================================================
 * Button_IsPressed — 读取并清零按下标志
 * ======================================================================== */
uint8_t Button_IsPressed(void)
{
    uint8_t ret = g_btn_pressed;
    g_btn_pressed = 0;
    return ret;
}
