/*
 * Balance.h — 小球平衡控制模块
 *
 * 增量式 PID + 步进电机 + K230 摄像头
 * MODE1 正常(中心25) | MODE2 循迹(中心25) | MODE3 挑战赛(19→15→32→35.7)
 */

#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 默认 PID 参数 ---- */
#define BAL_KP_DEFAULT       1.0f
#define BAL_KI_DEFAULT       0.57f
#define BAL_KD_DEFAULT       12.0f

#define BAL_DELTA_MAX        15.0f
#define BAL_DELTA_MIN       -15.0f

#define BAL_SETTLE_THRESHOLD   0.3f
#define BAL_SETTLE_COUNT       3

/* ---- 挑战赛参数 ---- */
#define CHAL_TARGET_INIT     15.6f    /* 初始目标: 引球向左 */
#define CHAL_TRIGGER_15      15.0f    /* 球到达15±1 触发 */
#define CHAL_TARGET_MID      32.0f    /* 中途目标: 甩向右侧 */
#define CHAL_TRIGGER_357     35.7f    /* 球到达35.7±1 触发 */
#define CHAL_TARGET_FINAL    35.7f    /* 最终稳定目标 */
#define CHAL_TRIGGER_TOL     1.0f     /* 触发容差 */

/* ---- 模式 ---- */
typedef enum {
    BAL_MODE1_NORMAL    = 0,  /* 正常模式, 中心 25cm */
    BAL_MODE2_TRACK     = 1,  /* 循迹模式, 中心 25cm */
    BAL_MODE3_CHALLENGE = 2,  /* 挑战赛: 19→(15触发)→32→(35.7触发)→35.7稳定 */
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
void Balance_SwitchMode(void);         /* PA30 切换 */
void Balance_NextMode(void);           /* 切换下一模式 (PB21 调用) */
void Balance_ChallengeUpdate(void);    /* MODE3 挑战赛状态机, SysTick调用 */

/* ---- 挑战赛计时 ---- */
extern volatile uint8_t  g_chal_state;
extern volatile uint32_t g_chal_tick;   /* 10ms/tick */

/* ---- PID 参数 (UART 调参) ---- */
extern float g_bal_kp;
extern float g_bal_ki;
extern float g_bal_kd;

#endif
