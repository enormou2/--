/**
 * motor.h - 步进电机 PWM 驱动接口
 *
 * 适配目标工程硬件：
 *   STEP1 (X轴) = TIMA0_CCP3 / PB24
 *   STEP2 (Y轴) = TIMA0_CCP0 / PA8
 *   DIR1 (X轴)  = PA28
 *   DIR2 (Y轴)  = PA31
 *   EN1+EN2     = PA9 (共用，低电平有效)
 *
 * 提供速度/位置控制上层逻辑调用的底层接口：
 *   - Motor_Init():            初始化定时器时钟为 1MHz
 *   - Motor_SetAxisSpeed_RPM():以 RPM 设定某一轴 STEP 频率
 *   - Motor_StopAxis():        停止某一轴 STEP，置位 EN 禁用输出
 *   - Motor_EnableStepInterrupt():  位置模式启动时使能 PWM 计数中断
 *   - Motor_DisableStepInterrupt(): 位置停止时关闭计数中断
 */
#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_STEP1           1U
#define GIMBAL_STEP2           2U

/* ========== 引脚映射（适配目标工程） ========== */

/* 共用定时器：STEP1(PB24/CC3) + STEP2(PA8/CC0) 都在 TIMA0 */
#define GIMBAL_STEP_TIMER         STEPPER_MOTOR_INST

/* X轴 = STEP1 = PB24 = TIMA0_CCP3 */
#define GIMBAL_X_STEP_CC_INDEX   DL_TIMER_CC_3_INDEX
#define GIMBAL_X_STEP_CC_OUTPUT  DL_TIMER_CC3_OUTPUT

/* Y轴 = STEP2 = PA8  = TIMA0_CCP0 */
#define GIMBAL_Y_STEP_CC_INDEX   DL_TIMER_CC_0_INDEX
#define GIMBAL_Y_STEP_CC_OUTPUT  DL_TIMER_CC0_OUTPUT

/* DIR 引脚 */
#define GIMBAL_DIR_PORT          GPIO_DIR_PORT
#define GIMBAL_DIR_X_PIN         GPIO_DIR_DIR1_PIN    /* PA28 */
#define GIMBAL_DIR_Y_PIN         GPIO_DIR_DIR2_PIN    /* PA31 */

/* EN 引脚（共用 PA9，低电平有效=使能） */
#define GIMBAL_EN_PORT           GPIO_STEP_EN_PORT
#define GIMBAL_EN_PIN            GPIO_STEP_EN_EN_PIN  /* PA9 */

/* ========== 速度计算常量 ========== */
/*
 * 3200 STEP/圈 = 200 步进/圈 * 16 细分
 * 定时器时钟经 Motor_Init() 重配置为 1MHz
 * PWM 周期 = 1MHz * 60 / (RPM * 3200)
 */
#define MOTOR_STEPS_PER_REV  3200U
#define PWM_TIMER_CLK_HZ     1000000U
#define SPEED_CONST_FACTOR   60000000.0f
#define MIN_PWM_PERIOD       100U
#define MAX_PWM_PERIOD       65535U
#define STOP_PWM_PERIOD      1000U       /* 停车时置 1kHz 伪频率，强制 STEP 为低 */
#define SPEED_LIMIT          160.0f

/* ========== 全局变量 ========== */

extern volatile uint16_t Step_Period_X;
extern volatile uint16_t Step_Period_Y;

extern float Target_Speed_X;
extern float Target_Speed_Y;

/* ========== 函数声明 ========== */

void Motor_Init(void);
uint16_t Motor_RpmToPeriodTicks(float abs_rpm);
void Motor_SetAxisSpeed_RPM(uint8_t axis, float rpm);
void Motor_StopAxis(uint8_t axis);
void Motor_EnableStepInterrupt(uint8_t axis);
void Motor_DisableStepInterrupt(uint8_t axis);
void Set_Motor_Speed_RPM(float rpm_x, float rpm_y);

#ifdef __cplusplus
}
#endif

#endif
