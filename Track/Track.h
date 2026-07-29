/*
 * Track.h
 * 8路循迹传感器模块 — 10mm间距
 *
 * 传感器布局:  T1=-35  T2=-25  T3=-15  T4=-5  T5=5  T6=15  T7=25  T8=35 (mm)
 * 引脚映射:   TRACK1=PB2 ... TRACK8=PB9
 * 上拉输入:   黑线 → 低电平(0), 白底 → 高电平(1)
 */
#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

typedef enum {
    TRACK_STATE_LEFT_EDGE   = -2,
    TRACK_STATE_LEFT_INNER  = -1,
    TRACK_STATE_CENTER      =  0,
    TRACK_STATE_RIGHT_INNER =  1,
    TRACK_STATE_RIGHT_EDGE  =  2,
    TRACK_STATE_LOST        =  3,
    TRACK_STATE_UNKNOWN     =  4,
} track_state_t;

/** @brief 8bit传感器状态: bit7=Track1(最左) ... bit0=Track8(最右), 1=黑线 */
extern uint8_t TrackN;

/** @brief 读取8路传感器数据并做3帧逐位多数表决 */
void    Read_Track_DATA(uint8_t *arr);

/** @brief 计算黑线中心偏移（mm），正值偏右（带EMA滤波） */
float   Track_Err(uint16_t car_state);

/** @brief 优先级解码：最外侧激活传感器决定状态 */
track_state_t Track_GetState(void);

/** @brief 返回 1 表示至少有一个传感器检测到黑线 */
uint8_t Is_Line_Detected(void);

#endif /* __TRACK_H */
