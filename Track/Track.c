/*
 * Track.c
 * 5路循迹传感器模块（MSPM0 版本）
 *
 * 传感器布局: L1=-45mm  L2=-25mm  Center=0  R2=25mm  R1=45mm
 * 引脚映射:   TRACK1=PB2  TRACK2=PB3  TRACK3=PB4  TRACK4=PB5  TRACK5=PB6
 * 上拉输入:   黑线 → 低电平(0), 白底 → 高电平(1)
 *
 * 滤波策略:
 *   Read_Track_DATA: 3帧逐位多数表决 — 消除单次读取毛刺
 *   Track_Err:       EMA低通 + 中心死区 — 抑制二值量化跳变和直线抖动
 */

#include "ti_msp_dl_config.h"
#include "Track/Track.h"

/* EMA 平滑系数: 越大响应越快, 越小越平滑 (范围 0.1~0.5) */
#define TRACK_EMA_ALPHA  0.25f
#define TRACK_CENTER_DEADBAND_MM  8.0f

uint8_t TrackN;  /* 5bit传感器状态: bit4=Track1(最左) ... bit0=Track5(最右), 1=黑线 */

/*
 *  ======== Read_Track_DATA ========
 *  读取 5 路传感器并做 3 帧逐位多数表决
 */
void Read_Track_DATA(uint8_t *arr)
{
    static uint8_t last_track_1 = 0;
    static uint8_t last_track_2 = 0;
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

    /*
     * Do not average the packed bit field. For example, averaging 0b00100
     * and 0b00010 produces 0b00011, which invents a sensor state.
     * Vote on each bit across three control cycles instead.
     */
    TrackN = (uint8_t)((current_track & last_track_1) |
                       (current_track & last_track_2) |
                       (last_track_1 & last_track_2));
    last_track_2 = last_track_1;
    last_track_1 = current_track;
    *arr = TrackN;
}

/*
 *  ======== Track_Err ========
 *  计算黑线中心偏移（mm），带 EMA 低通与中心死区
 *
 *  传感器布局:  L1=-45  L2=-25  C=0  R2=25  R1=45 (mm)
 *  黑线 ~18mm 宽，最多覆盖相邻 2 个传感器
 *
 *  EMA 滤波效果（alpha=0.25, 100Hz 调用）:
 *    传感器跳变 0→25mm → 10ms后=6.3mm, 40ms后=17mm, 100ms后≈24mm
 *    消除瞬时尖峰，保留真实偏移趋势
 *
 *  car_state: 保留
 *  返回值:  滤波后的偏移（mm），正值偏右
 */
float Track_Err(uint16_t car_state)
{
    static const float pos[5] = {-45.0f, -25.0f, 0.0f, 25.0f, 45.0f};
    static float       s_filt = 0.0f;  /* EMA 滤波状态 */

    float   sum   = 0.0f;
    uint8_t count = 0;

    /* 计算激活传感器的质心 */
    for (uint8_t i = 0; i < 5; i++) {
        if ((TrackN >> (4 - i)) & 0x01) {
            sum += pos[i];
            count++;
        }
    }

    /* 全白(丢线) 或 全黑(路口) → 直行 + 复位滤波器 */
    if (count == 0 || count == 5) {
        s_filt = 0.0f;
        (void)car_state;
        return 0.0f;
    }

    float raw = sum / (float)count;

    /* EMA 低通滤波: 消除二值量化跳变 */
    s_filt = s_filt * (1.0f - TRACK_EMA_ALPHA) + raw * TRACK_EMA_ALPHA;

    /*
     * Avoid steering for small errors caused by overlapping wide sensors.
     * Keep the filter state intact so a persistent real offset can still
     * accumulate beyond the deadband.
     */
    if (s_filt > -TRACK_CENTER_DEADBAND_MM &&
        s_filt <  TRACK_CENTER_DEADBAND_MM) {
        (void)car_state;
        return 0.0f;
    }

    (void)car_state;
    return s_filt;
}

/*
 * Five narrow-line sensors normally report one active bit at a time.
 * Keep this discrete decoder separate from Track_Err(): the control layer
 * can use it for gentle normal steering and bounded lost-line recovery.
 */
track_state_t Track_GetState(void)
{
    switch (TrackN) {
    case 0x10: return TRACK_STATE_LEFT_EDGE;
    case 0x08: return TRACK_STATE_LEFT_INNER;
    case 0x04: return TRACK_STATE_CENTER;
    case 0x02: return TRACK_STATE_RIGHT_INNER;
    case 0x01: return TRACK_STATE_RIGHT_EDGE;
    case 0x00: return TRACK_STATE_LOST;
    default:   return TRACK_STATE_UNKNOWN;
    }
}

/*
 *  ======== Is_Line_Detected ========
 *  检查是否检测到黑线（至少一个传感器检测到）
 */
uint8_t Is_Line_Detected(void)
{
    return (TrackN != 0x00);
}
