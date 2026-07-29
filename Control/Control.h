/*
 * Control.h
 */
#ifndef __CONTROL_H
#define __CONTROL_H
#include <stdint.h>

typedef enum { STEER_MODE_IDLE=0, STEER_MODE_TRACK=1, STEER_MODE_ANGLE=2 } steer_mode_t;

#define CTRL_PWM_PERIOD    1000
#define CTRL_LOOP_FREQ_HZ  100
#define CTRL_BASE_SPEED    300
#define CTRL_MIN_SPEED     200
#define CTRL_MAX_OFFSET_MM 35.0f
#define CTRL_TARGET_YAW    0.0f

extern steer_mode_t g_steer_mode;
extern float g_target_speed, g_target_yaw;
extern float g_speed_kp, g_speed_ki, g_speed_kd;
extern float g_track_kp, g_track_ki, g_track_kd;
extern float g_angle_kp, g_angle_ki, g_angle_kd;

void Control_Init(void);
void Control_SetMode(steer_mode_t m);
void Control_SetTargetSpeed(float s);
void Control_SetTargetYaw(float y);
void Control_Update(void);
#endif
