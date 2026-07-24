/*
 * IMU.h
 * AHRS姿态解算模块 — 6轴 (加速度计+陀螺仪), 无磁力计
 */
#ifndef __IMU_H
#define __IMU_H

#include "ti_msp_dl_config.h"
#include <math.h>

#undef M_PI
#define M_PI  (float)3.1415926535

/* 三维向量结构体 */
typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

extern xyz_f_t north, west;
extern volatile float yaw[5];

/* ---- API ---- */

/** @brief 初始化IMU（传感器初始化 + 四元数初始化） */
void IMU_init(void);

/** @brief 获取当前 Yaw/Pitch/Roll 角度（度） */
void IMU_getYawPitchRoll(float *ypr);

/** @brief 获取原始传感器数据（加速度+陀螺仪+温度） */
void IMU_TT_getgyro(float *data);

#endif
