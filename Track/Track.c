/*
 * Track.c
 * 8路循迹传感器模块（MSPM0 版本）— 10mm间距
 *
 * 传感器布局:  T1=-35 T2=-25 T3=-15 T4=-5 T5=5 T6=15 T7=25 T8=35 (mm)
 * 引脚映射:   TRACK1=PB2 TRACK2=PB3 TRACK3=PB4 TRACK4=PB5
 *             TRACK5=PB6 TRACK6=PB7 TRACK7=PB8 TRACK8=PB9
 * 上拉输入:   黑线 → 低电平(0), 白底 → 高电平(1)
 *
 * 滤波策略:
 *   Read_Track_DATA: 3帧逐位多数表决 — 消除单次读取毛刺
 *   Track_Err:       EMA低通 — 消除传感器量化跳变 (10mm间距下跳变仅5mm级别)
 */

#include "ti_msp_dl_config.h"
#include "Track/Track.h"

/* EMA 平滑系数 */
#define TRACK_EMA_ALPHA  0.3f

uint8_t TrackN;  /* 8bit: bit7=Track1(最左) ... bit0=Track8(最右), 1=黑线 */

/*
 *  ======== Read_Track_DATA ========
 *  读取 8 路传感器并做 3 帧逐位多数表决
 */
void Read_Track_DATA(uint8_t *arr)
{
    static uint8_t last_1 = 0, last_2 = 0;
    uint8_t        strackarr[8];
    uint8_t        current_track;

    /* 读取每个传感器（1=检测到黑线, 0=白底） */
    strackarr[0] = DL_GPIO_readPins(TRACK1_PORT, TRACK1_PIN_TRACK1_PIN) ? 0 : 1;
    strackarr[1] = DL_GPIO_readPins(TRACK2_PORT, TRACK2_PIN_TRACK2_PIN) ? 0 : 1;
    strackarr[2] = DL_GPIO_readPins(TRACK3_PORT, TRACK3_PIN_TRACK3_PIN) ? 0 : 1;
    strackarr[3] = DL_GPIO_readPins(TRACK4_PORT, TRACK4_PIN_TRACK4_PIN) ? 0 : 1;
    strackarr[4] = DL_GPIO_readPins(TRACK5_PORT, TRACK5_PIN_TRACK5_PIN) ? 0 : 1;
    strackarr[5] = DL_GPIO_readPins(TRACK6_PORT, TRACK6_PIN_TRACK6_PIN) ? 0 : 1;
    strackarr[6] = DL_GPIO_readPins(TRACK7_PORT, TRACK7_PIN_TRACK7_PIN) ? 0 : 1;
    strackarr[7] = DL_GPIO_readPins(TRACK8_PORT, TRACK8_PIN_TRACK8_PIN) ? 0 : 1;

    /* 8 位合并: bit7=Track1 ... bit0=Track8 */
    current_track = (uint8_t)(strackarr[7]       | (strackarr[6] << 1) |
                              (strackarr[5] << 2) | (strackarr[4] << 3) |
                              (strackarr[3] << 4) | (strackarr[2] << 5) |
                              (strackarr[1] << 6) | (strackarr[0] << 7));

    /* 3帧逐位多数表决 (至少2票才有效, 消除单帧毛刺) */
    TrackN = (uint8_t)((current_track & last_1) |
                       (current_track & last_2) |
                       (last_1 & last_2));
    last_2 = last_1;
    last_1 = current_track;
    *arr = TrackN;
}

/*
 *  ======== Track_Err ========
 *  质心法计算黑线中心偏移（mm），带 EMA 低通滤波
 *
 *  8探头位置: -35 -25 -15 -5  5  15  25  35 (mm), 10mm间距
 *  黑线 ~15-18mm 宽 → 覆盖 1~2 个传感器 → 量化跳变仅 5mm 级别
 *
 *  EMA 滤波 (alpha=0.3): 跳变 0→10mm → 10ms后 3mm, 30ms后 6.6mm
 *  返回值: 滤波后偏移（mm），正值偏右
 */
float Track_Err(uint16_t car_state)
{
    static const float pos[8] = {-35.0f, -25.0f, -15.0f, -5.0f,
                                    5.0f,  15.0f,  25.0f, 35.0f};
    static float s_filt = 0.0f;

    float   sum   = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if ((TrackN >> (7 - i)) & 0x01) {
            sum += pos[i];
            count++;
        }
    }

    /* 全白(丢线) 或 全黑(路口) → 复位滤波器 */
    if (count == 0 || count == 8) {
        s_filt = 0.0f;
        (void)car_state;
        return 0.0f;
    }

    float raw = sum / (float)count;

    /* EMA 低通滤波 */
    s_filt = s_filt * (1.0f - TRACK_EMA_ALPHA) + raw * TRACK_EMA_ALPHA;

    (void)car_state;
    return s_filt;
}

/*
 *  ======== Track_GetState ========
 *  优先级解码：最外侧激活传感器决定状态 (EDGE > INNER > CENTER)
 *
 *  目的: 前馈基础量选择 + 速度降档判断
 */
track_state_t Track_GetState(void)
{
    if (TrackN == 0x00) return TRACK_STATE_LOST;

    /* 优先级从外到内: EDGE → INNER → CENTER */
    if (TrackN & 0xC0) return TRACK_STATE_LEFT_EDGE;    /* Track1(-35) or Track2(-25) */
    if (TrackN & 0x03) return TRACK_STATE_RIGHT_EDGE;   /* Track7(25) or Track8(35) */
    if (TrackN & 0x20) return TRACK_STATE_LEFT_INNER;   /* Track3(-15) */
    if (TrackN & 0x04) return TRACK_STATE_RIGHT_INNER;  /* Track6(15) */
    if (TrackN & 0x18) return TRACK_STATE_CENTER;       /* Track4(-5) or Track5(5) */

    return TRACK_STATE_UNKNOWN;  /* unexpected pattern */
}

/*
 *  ======== Is_Line_Detected ========
 */
uint8_t Is_Line_Detected(void)
{
    return (TrackN != 0x00);
}
