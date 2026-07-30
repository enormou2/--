/*
 * Balance.h — 小球平衡控制模块
 *
 * ============================ 控制策略 ============================
 *
 * 【主力方案】增量式 PID:
 *   每周期(有新摄像头数据时):
 *     error = target - actual  (cm)
 *     delta = Kp*(error-last_error) + Ki*error + Kd*(error-2*last_error+last_last_error)
 *     current_target_steps += (int32_t)delta
 *     StepMotor_SetTarget(current_target_steps)
 *
 *   优点: 输出平滑, 不会因误差缩小而回退, 天然匹配步进电机增量执行器
 *
 * 【备用方案】位置式 PID + 死区 (代码中注释保留, 实测后可 A/B 对比):
 *   每周期:
 *     PID_Update() → output (步数绝对值)
 *     if |output - last_output| > deadband → StepMotor_SetTarget(output)
 *
 * ============================ 反馈链路 ============================
 *
 * K230 摄像头 (22-24fps)
 *   → UART1 发送: "x:+3.2\r\n" 或 "x:-1.5\r\n" (球位置, cm)
 *   → MSPM0 UART1 RX (PA9) 中断接收, 检测 \n 换行
 *   → Balance_Update() (SysTick 100Hz) 中检测新数据标志
 *   → 仅在新帧到达时计算 PID
 *   → 摄像头帧率 ~23fps → 实际控制频率 ~23Hz
 *
 * ============================ 硬件连接 ============================
 *
 * UART1 RX = PA9  (IOMUX_PINCM20, 接收 K230 摄像头数据)
 * UART1 TX = PA8  (IOMUX_PINCM19, 可选同步信号, 暂未使用)
 *
 * ⚠️ MSPM0G3507: PA8=UART1_TX, PA9=UART1_RX (与直觉相反)
 * 引脚在代码中手动配置, 不走 SysConfig
 */

#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 * 默认 PID 参数 —【待实测】从小往大调
 * ======================================================================== */

/* 增量式 PID (主力) */
#define BAL_KP_DEFAULT      20.0f   /* P 增益 (steps/cm) */
#define BAL_KI_DEFAULT       0.0f   /* I 增益 (初期不加) */
#define BAL_KD_DEFAULT      40.0f   /* D 增益 (steps/(cm·s)), 阻尼防振荡 */

/* 增量式 PID 输出限幅 (每周期最大增量) */
#define BAL_DELTA_MAX       100.0f  /* 步/周期 */
#define BAL_DELTA_MIN      -100.0f

/* 到位判断: 连续 N 帧误差 < 阈值 → 判定为 settled */
#define BAL_SETTLE_THRESHOLD   0.5f   /* cm, 误差小于此值认为到位 */
#define BAL_SETTLE_COUNT       5      /* 连续帧数 */

/* UART1 接收缓冲区 */
#define BAL_UART_BUF_LEN   32

/* ========================================================================
 * 公开接口
 * ======================================================================== */

/**
 * @brief 初始化平衡模块
 *   - UART1: 115200-8-N-1, RX 中断 (NVIC pri=0)
 *   - PID 参数初始化为默认值
 *   - 目标位置初始化为 0.0cm (中心)
 *
 * @note 在 StepMotor_Init() 之后调用 (因为 Balance 需要调用 StepMotor API)
 */
void Balance_Init(void);

/**
 * @brief 平衡控制更新 (必须在 SysTick 100Hz 中每周期调用)
 *
 * 内部仅在有新摄像头数据时执行 PID 计算
 * 无新数据时立即返回 (不浪费 CPU)
 */
void Balance_Update(void);

/**
 * @brief 设置目标球位置
 * @param pos_cm  目标位置 (cm), 0=中心, 正=右侧, 负=左侧
 */
void Balance_SetTarget(float pos_cm);

/**
 * @brief 查询球是否稳定在目标位置
 * @return true=连续多帧误差在阈值内
 */
bool Balance_IsSettled(void);

/**
 * @brief 获取球当前实际位置 (最近一帧摄像头数据)
 * @return 球位置 (cm)
 */
float Balance_GetPosition(void);

/**
 * @brief 获取步进电机目标步数 (用于 OLED 显示/调试)
 */
int32_t Balance_GetMotorSteps(void);

/**
 * @brief 解析 UART1 接收的单字节 (在 UART1 ISR 中调用)
 * @param ch 接收到的字节
 */
void Balance_ParseChar(uint8_t ch);

/* ---- PID 参数 (运行时可通过 UART 调整) ---- */
extern float g_bal_kp;
extern float g_bal_ki;
extern float g_bal_kd;

#endif /* __BALANCE_H */
