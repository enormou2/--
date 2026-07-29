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

/** @brief 5bit传感器状态: bit4=Track1(最左) ... bit0=Track5(最右), 0=黑线 */
extern uint8_t TrackN;

/** @brief 读取5路传感器数据并做均值滤波，结果存入 *arr */
void    Read_Track_DATA(uint8_t *arr);

/** @brief 计算黑线中心偏移（mm），正值偏右 */
float   Track_Err(uint16_t car_state);

/** @brief 返回 1 表示至少有一个传感器检测到黑线 */
uint8_t Is_Line_Detected(void);

#endif /* __TRACK_H */
