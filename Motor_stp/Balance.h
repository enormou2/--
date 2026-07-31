/*
 * Balance.h — 小球平衡控制模块
 *
 * 增量式 PID + 步进电机 + K230 摄像头
 * MODE1 正常模式(中心25) | MODE2 挑战赛(15.8→35.7, 5s)
 */

#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 默认 PID 参数 ---- */
#define BAL_KP_DEFAULT       1.0f
#define BAL_KI_DEFAULT       0.44f
#define BAL_KD_DEFAULT       14.0f

#define BAL_DELTA_MAX        15.0f
#define BAL_DELTA_MIN       -15.0f

#define BAL_SETTLE_THRESHOLD   0.3f
#define BAL_SETTLE_COUNT       3

/* ---- 模式 ---- */
typedef enum {
    BAL_MODE1_NORMAL  = 0,  /* 正常模式, 中心 25cm */
    BAL_MODE2_CHALLENGE = 1, /* 挑战赛: 25→15.8→35.7, 5秒内 */
} bal_mode_t;

/* ---- 挑战赛状态 ---- */
#define CHAL_STATE_OFF    0
#define CHAL_STATE_RUN    1
#define CHAL_STATE_DONE   2

/* ---- 公开接口 ---- */
void Balance_Init(void);
void Balance_Update(void);
void Balance_SetTarget(float pos_cm);
bool Balance_IsSettled(void);
float Balance_GetPosition(void);
int32_t Balance_GetMotorSteps(void);
void Balance_ParseChar(uint8_t ch);

/* ---- 模式控制 ---- */
extern volatile bal_mode_t g_bal_mode;
void Balance_SwitchMode(void);         /* PA30 切换 MODE1/MODE2 */
void Balance_ChallengeUpdate(void);    /* 挑战赛状态机, SysTick调用 */

/* ---- 挑战赛计时 ---- */
extern volatile uint8_t  g_chal_state;
extern volatile uint32_t g_chal_tick;   /* 10ms/tick */

/* ---- PID 参数 (UART 调参) ---- */
extern float g_bal_kp;
extern float g_bal_ki;
extern float g_bal_kd;

#endif
