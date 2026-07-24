/*
 * Track.h
 * 8路循迹传感器模块
 */
#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

/** @brief 8bit传感器状态: bit7=Track1(最左) ... bit0=Track8(最右), 0=黑线 */
extern uint8_t TrackN;

/** @brief 读取8路传感器数据并做均值滤波，结果存入 *arr */
void    Read_Track_DATA(uint8_t *arr);

/** @brief 计算黑线中心偏移（mm），正值偏右 */
float   Track_Err(uint16_t car_state);

/** @brief 返回 1 表示至少有一个传感器检测到黑线 */
uint8_t Is_Line_Detected(void);

#endif /* __TRACK_H */
