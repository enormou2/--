/*
 * Track.c
 * 5路循迹传感器模块（MSPM0 版本）
 *
 * 传感器布局: L1=-45mm  L2=-25mm  Center=0  R2=25mm  R1=45mm
 * 引脚映射:   TRACK1=PB2  TRACK2=PB3  TRACK3=PB4  TRACK4=PB5  TRACK5=PB6
 * 上拉输入:   黑线 → 低电平(0), 白底 → 高电平(1)
 */

#include "ti_msp_dl_config.h"
#include "Track/Track.h"

uint8_t TrackN;  /* 5bit传感器状态: bit4=Track1(最左) ... bit0=Track5(最右), 0=黑线 */

/*
 *  ======== Read_Track_DATA ========
 *  读取 5 路传感器并做均值滤波
 */
void Read_Track_DATA(uint8_t *arr)
{
    static uint8_t last_track = 0;
    uint8_t        strackarr[5];
    uint8_t        current_track;

    /* 读取每个传感器（1=检测到黑线, 0=白底） */
    strackarr[0] = DL_GPIO_readPins(TRACK1_PORT, TRACK1_PIN_TRACK1_PIN) ? 0 : 1;
    strackarr[1] = DL_GPIO_readPins(TRACK2_PORT, TRACK2_PIN_TRACK2_PIN) ? 0 : 1;
    strackarr[2] = DL_GPIO_readPins(TRACK3_PORT, TRACK3_PIN_TRACK3_PIN) ? 0 : 1;
    strackarr[3] = DL_GPIO_readPins(TRACK4_PORT, TRACK4_PIN_TRACK4_PIN) ? 0 : 1;
    strackarr[4] = DL_GPIO_readPins(TRACK5_PORT, TRACK5_PIN_TRACK5_PIN) ? 0 : 1;

    /* 5 位数据合并: bit4=Track1(最左) ... bit0=Track5(最右) */
    current_track = (uint8_t)(strackarr[4]       | (strackarr[3] << 1) |
                              (strackarr[2] << 2) | (strackarr[1] << 3) |
                              (strackarr[0] << 4));

    /* 均值滤波 */
    TrackN = (uint8_t)((current_track + last_track) / 2);
    last_track = current_track;
    *arr = TrackN;
}

/*
 *  ======== Track_Err ========
 *  根据传感器状态计算黑线中心偏移（mm）
 *  car_state: 小车状态码（保留，可用于特殊状态修正）
 *  返回值: 黑线中心相对于模块中心的偏移，正值偏右、负值偏左
 */
float Track_Err(uint16_t car_state)
{
    /* 5个探头物理位置（mm）: L1=-45, L2=-25, Center=0, R2=25, R1=45 */
    static const float pos[5] = {-45.0f, -25.0f, 0.0f, 25.0f, 45.0f};

    float   sum   = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 5; i++) {
        /* bit4=Track1(最左) ... bit0=Track5(最右) */
        uint8_t bit_val = (TrackN >> (4 - i)) & 0x01;
        if (bit_val == 1) {  /* 1 = 检测到黑线 */
            sum += pos[i];
            count++;
        }
    }

    if (count == 0 || count == 5)
        return 0.0f;  /* 全白或全黑，无有效偏移 */

    (void)car_state;  /* 保留参数，后续可扩展状态修正 */
    return sum / (float)count;  /* 黑线中心偏移（mm） */
}

/*
 *  ======== Is_Line_Detected ========
 *  检查是否检测到黑线（至少一个传感器检测到）
 */
uint8_t Is_Line_Detected(void)
{
    return (TrackN != 0x00);
}
