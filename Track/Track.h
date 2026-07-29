/*
 * Track.h
 * 5路循迹传感器模块
 *
 * 传感器布局: L1=-45mm  L2=-25mm  Center=0  R2=25mm  R1=45mm
 * 引脚映射:   TRACK1=PB2  TRACK2=PB3  TRACK3=PB4  TRACK4=PB5  TRACK5=PB6
 */
#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

typedef enum {
    TRACK_STATE_LEFT_EDGE  = -2,
    TRACK_STATE_LEFT_INNER = -1,
    TRACK_STATE_CENTER     = 0,
    TRACK_STATE_RIGHT_INNER = 1,
    TRACK_STATE_RIGHT_EDGE  = 2,
    TRACK_STATE_LOST        = 3,
    TRACK_STATE_UNKNOWN     = 4,
} track_state_t;

/** @brief 5bit传感器状态: bit4=Track1(最左) ... bit0=Track5(最右), 1=黑线 */
extern uint8_t TrackN;

/** @brief 读取5路传感器数据并做逐位多数表决，结果存入 *arr */
void    Read_Track_DATA(uint8_t *arr);

/** @brief 计算黑线中心偏移（mm），正值偏右 */
float   Track_Err(uint16_t car_state);

/** @brief 将单探头循迹状态解码为离散方向等级或丢线状态 */
track_state_t Track_GetState(void);

/** @brief 返回 1 表示至少有一个传感器检测到黑线 */
uint8_t Is_Line_Detected(void);

#endif /* __TRACK_H */
