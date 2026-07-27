/*
 * Track.c
 * 8路循迹传感器模块（MSPM0 版本）
 *
 * 硬件: PB2=Track1(最左) ... PB9=Track8(最右)，上拉输入
 *       黑线 → 低电平(0), 白底 → 高电平(1)
 */

#include "ti_msp_dl_config.h"
#include "Track/Track.h"

uint8_t TrackN;  /* 8bit传感器状态: bit7=Track1(左) ... bit0=Track8(右), 0=黑线 */

/*
 *  ======== Read_Track_DATA ========
 *  读取 8 路传感器并做均值滤波
 */
void Read_Track_DATA(uint8_t *arr)
{
    static uint8_t last_track = 0;
    uint8_t        strackarr[8];
    uint8_t        current_track;

    /* 读取每个传感器（0=低/黑线, 1=高/白底） */
    strackarr[0] = DL_GPIO_readPins(TRACK1_PORT, TRACK1_PIN_TRACK1_PIN) ? 0 : 1;
    strackarr[1] = DL_GPIO_readPins(TRACK2_PORT, TRACK2_PIN_TRACK2_PIN) ? 0 : 1;
    strackarr[2] = DL_GPIO_readPins(TRACK3_PORT, TRACK3_PIN_TRACK3_PIN) ? 0 : 1;
    strackarr[3] = DL_GPIO_readPins(TRACK4_PORT, TRACK4_PIN_TRACK4_PIN) ? 0 : 1;
    strackarr[4] = DL_GPIO_readPins(TRACK5_PORT, TRACK5_PIN_TRACK5_PIN) ? 0 : 1;
    strackarr[5] = DL_GPIO_readPins(TRACK6_PORT, TRACK6_PIN_TRACK6_PIN) ? 0 : 1;
    strackarr[6] = DL_GPIO_readPins(TRACK7_PORT, TRACK7_PIN_TRACK7_PIN) ? 0 : 1;
    strackarr[7] = DL_GPIO_readPins(TRACK8_PORT, TRACK8_PIN_TRACK8_PIN) ? 0 : 1;

    /* 8 位数据合并为字节: bit7=Track1 ... bit0=Track8 */
    current_track = (uint8_t)(strackarr[7]       | (strackarr[6] << 1) |
                              (strackarr[5] << 2) | (strackarr[4] << 3) |
                              (strackarr[3] << 4) | (strackarr[2] << 5) |
                              (strackarr[1] << 6) | (strackarr[0] << 7));

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
    /* 8个探头相对于模块中心的物理位置（mm），间距 10mm */
    static const float pos[8] = {-35.0f, -25.0f, -15.0f, -5.0f,
                                   5.0f,  15.0f,  25.0f, 35.0f};

    float   sum   = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        /* bit7=Track1 ... bit0=Track8 */
        uint8_t bit_val = (TrackN >> (7 - i)) & 0x01;
        if (bit_val == 0) {  /* 0 = 检测到黑线 */
            sum += pos[i];
            count++;
        }
    }

    if (count == 0 || count == 8)
        return 0.0f;  /* 全白或全黑，无有效偏移 */

    (void)car_state;  /* 保留参数，后续可扩展状态修正 */
    return sum / (float)count;  /* 黑线中心偏移（mm） */
}

/*
 *  ======== Is_Line_Detected ========
 *  检查是否检测到黑线（至少一个传感器为低电平）
 */
uint8_t Is_Line_Detected(void)
{
    return (TrackN != 0xFF);
}
