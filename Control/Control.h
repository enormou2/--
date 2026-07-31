/*
 * Control.h
 */
#ifndef __CONTROL_H
#define __CONTROL_H
#include <stdint.h>

typedef enum { STEER_MODE_IDLE=0, STEER_MODE_TRACK=1 } steer_mode_t;

#define CTRL_PWM_PERIOD    1000
#define CTRL_LOOP_FREQ_HZ  100
#define CTRL_BASE_SPEED    150
#define CTRL_MIN_SPEED     100
#define CTRL_MAX_OFFSET_MM 35.0f
#define CTRL_TARGET_YAW    0.0f
#define CTRL_RAMP_TICKS    40       /* 起步斜坡 tick数 (100Hz, 40=400ms) */

/* ---- MODE2 循迹 PID 默认值 (待调) ---- */
#define SPD2_BASE  250
#define SPD2_MIN   200
#define SPD2_KP    2.0f
#define SPD2_KI    0.13f
#define SPD2_KD    0.0f
#define TRK2_KP    13.0f
#define TRK2_KI    0.0f
#define TRK2_KD    0.0f

/* ---- MODE4 定时循迹 PID 默认值 (当前调好的一套) ---- */
#define SPD4_BASE  150
#define SPD4_MIN   100
#define SPD4_KP    1.71f
#define SPD4_KI    0.13f
#define SPD4_KD    0.0f
#define TRK4_KP    8.0f
#define TRK4_KI    0.0f
#define TRK4_KD    1.0f

/* ---- 定时停止: >0 则 lap_ticks 到达后自动停车 ---- */
extern uint32_t g_auto_stop_ticks;

/* ---- 停车线检测参数 (模式切换时配置) ---- */
extern uint8_t g_stop_black_min;   /* 触发黑线最小路数 (MODE2=8平行, MODE4=5) */
extern uint8_t g_stop_frames;      /* 连续帧数 (MODE2=2高速, MODE4=3) */

extern steer_mode_t g_steer_mode;
extern float g_target_speed, g_target_yaw;
extern float g_speed_kp, g_speed_ki, g_speed_kd;
extern float g_track_kp, g_track_ki, g_track_kd;
extern float g_angle_kp, g_angle_ki, g_angle_kd;
extern float g_base_speed, g_min_speed;  /* 循迹速度, 模式切换时改写 */

/* 秒表: 100Hz ticks, g_lap_ticks/100 = 秒 */
extern volatile uint32_t g_lap_ticks;
extern volatile uint8_t  g_lap_active;

void Control_Init(void);
void Control_SetMode(steer_mode_t m);
void Control_SetTargetSpeed(float s);
void Control_SetTargetYaw(float y);
void Control_Update(void);

/* 模式参数持久化: 切走时保存, 切回时恢复 (mode=bal_mode enum值) */
void Control_SaveParams(int mode);
void Control_LoadParams(int mode);
void Control_InitParams(void);   /* 用 #define 初始化影子存储, main中调用一次 */
#endif
