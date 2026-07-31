/*
 * StepMotor.h — 步进电机底层驱动模块
 *
 * 硬件连接:
 *   STEP = PA3 (TIMG7 CCP0, PWM 模式, 硬件自动输出脉冲)
 *   DIR  = PA2 (GPIO 推挽输出)
 *   EN   = PA4 (GPIO 推挽输出, 高电平=使能, 低电平=休眠)
 *
 * ⚠️ 注意: MSPM0G3507 没有 TIMG1_CCP0, 选用 TIMG7 (PA3→PINCM8)
 *
 * 驱动板: WHEELTEC D36A 双路步进电机驱动模块
 *   供电: 5.5–17V (推荐 12V)
 *   细分: 16 细分 (拨码开关设置) → 3200 脉冲/转
 *   电流: ~1.2A (拨码开关设置, 电机额定 1.5A, D36A 最大 1.44A)
 *
 * 步进电机: 42 型两相 (34mm 机身, 0.29 N·m 静力矩, 1.5A 额定)
 *
 * 驱动方式:
 *   - TIMG7 PWM 模式 → CCP0 输出到 PA3, 硬件产生 STEP 脉冲
 *   - TIMG7 ZERO 中断 → 每个周期计数一个脉冲
 *   - 梯形加减速: 在 StepMotor_Update() 中每 10ms 更新速度
 *
 * 引脚配置: 全部在代码中手动配置, 不走 SysConfig
 *   (遵循现有工程 PB22+PA7 左编码器的手动配置模式)
 */

#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 * 引脚宏定义 (手动配置, 不走 SysConfig)
 * ⚠️ MSPM0G3507 没有 TIMG1_CCP0, 选用 TIMG7 (PA3→PINCM8)
 * ======================================================================== */

/* STEP — PA3 → TIMG7 CCP0 (SysConfig: PWM_STEP) */
#define STEP_IOMUX         (GPIO_PWM_STEP_C0_IOMUX)
#define STEP_IOMUX_FUNC    (GPIO_PWM_STEP_C0_IOMUX_FUNC)

/* DIR — PA29 → GPIO 输出 (SysConfig: STEP_DIR) */
#define DIR_PORT            (STEP_DIR_PORT)
#define DIR_PIN             (STEP_DIR_PIN_0_PIN)
#define DIR_IOMUX           (STEP_DIR_PIN_0_IOMUX)

/* EN — PA31 → GPIO 输出 (SysConfig: STEP_EN2) */
#define EN_PORT             (STEP_EN2_PORT)
#define EN_PIN              (STEP_EN2_PIN_2_PIN)
#define EN_IOMUX            (STEP_EN2_PIN_2_IOMUX)

/* ========================================================================
 * 可调参数 (调试时根据需要修改)
 * ======================================================================== */

/*
 * 方向符号 —【待实测】
 * 上电后调用 StepMotor_Jog(100), 观察摄像头读数变化:
 *   如果球往正方向(cm增大)移动 → STEP_DIR_SIGN = +1 (不变)
 *   如果球往负方向(cm减小)移动 → STEP_DIR_SIGN = -1 (改为 -1)
 */
#define STEP_DIR_SIGN   1

/*
 * 安全软限位 (单位: 步数)
 * 小角度摆动场景不需要精确标定行程极限, 但保留宽松限位
 * 防止 PID 异常飞车或逻辑错误导致电机撞机械结构
 */
#define STEP_SOFT_LIMIT_MAX    3000
#define STEP_SOFT_LIMIT_MIN   -3000

/*
 * 梯形加减速默认参数 —【待实测】
 * 34mm 电机扭矩偏小 (0.29 N·m), 速度不宜过高
 * 实测时从小往大加, 听到啸叫或丢步就降
 */
#define STEP_DEFAULT_START_SPEED    150     /* 起始速度 (Hz) */
#define STEP_DEFAULT_MAX_SPEED      800     /* 最高速度 (Hz) */
#define STEP_DEFAULT_ACCEL          2000    /* 加速度 (Hz/s) */

/* ========================================================================
 * 公开接口
 * ======================================================================== */

/**
 * @brief 初始化步进电机
 *   - 配置 PA3→TIMG7_CCP0 (PWM), PA2→DIR (GPIO), PA4→EN (GPIO)
 *   - TIMG7: BUSCLK / 8 = 10MHz, 初始 PWM 占空比 50%
 *   - 初始状态: 电机未使能 (EN=低), 位置=0
 *   - 使能 TIMG7 ZERO 中断 (NVIC pri=1)
 *
 * @note 在 main() 中 SYSCFG_DL_init() 之后调用
 * @note 引脚不走 SysConfig, 全部在代码中手动配置 IOMUX
 */
void StepMotor_Init(void);

/**
 * @brief 设置目标绝对位置 (步数), 自动梯形加减速
 *
 * @param target  目标步数 (相对于上电零点, 受 STEP_SOFT_LIMIT 约束)
 *
 * 行为:
 *   1. clamp target 到 [STEP_SOFT_LIMIT_MIN, STEP_SOFT_LIMIT_MAX]
 *   2. 计算 delta = target - current_position
 *   3. 设置 DIR 引脚方向 (考虑 STEP_DIR_SIGN)
 *   4. 使能电机 (EN=高)
 *   5. 启动 TIMG1 PWM, 开始发脉冲
 *   6. 后续在 StepMotor_Update() 中做梯形加减速
 */
void StepMotor_SetTarget(int32_t target);

/**
 * @brief 紧急停止
 *   - 立即关闭 TIMG1 PWM
 *   - 保持当前绝对位置不丢失
 *   - EN 引脚保持高 (不掉电, 保持力矩)
 */
void StepMotor_Stop(void);

/**
 * @brief 查询是否到达目标
 * @return true=当前步数≈目标步数 (无脉冲发送中), false=运动中
 */
bool StepMotor_IsDone(void);

/**
 * @brief 读取当前绝对位置 (步数)
 * @note 上电时为 0; 正数=正方向, 负数=反方向
 */
int32_t StepMotor_GetPosition(void);

/**
 * @brief 运行时修改速度/加速度参数 (UART 调参用)
 * @param max_hz          最高速度 (Hz), 范围建议 200–10000
 * @param accel_hz_per_s  加速度 (Hz/s), 范围建议 1000–50000
 */
void StepMotor_SetSpeed(uint32_t max_hz, uint32_t accel_hz_per_s);

/**
 * @brief 手动点动 (标定/调试用)
 * @param delta  相对步数: 正=正方向, 负=反方向
 *
 * 走完 delta 步后自动停止。阻塞式? 否, 通过 StepMotor_IsDone() 判断完成。
 */
void StepMotor_Jog(int32_t delta);

/**
 * @brief 梯形加减速更新 (必须在 SysTick 100Hz 中每周期调用)
 *
 * 每 10ms 执行一次速度曲线计算:
 *   加速段: current_speed += accel * 0.01
 *   匀速段: current_speed = max_speed
 *   减速段: current_speed = sqrt(2 * accel * remaining)
 *   更新 TIMG1 period 寄存器以改变脉冲频率
 */
void StepMotor_Update(void);

#endif /* __STEP_MOTOR_H */
