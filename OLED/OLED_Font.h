#ifndef __OLED_FONT_H
#define __OLED_FONT_H

#include <stdint.h>

/* 字符集选择（二选一） */
#define OLED_CHARSET_UTF8
//#define OLED_CHARSET_GB2312

/* 中文字符结构体 */
typedef struct {
    char     Index[5]; /* 字符编码（UTF8 最多 4 字节 + '\0'） */
    uint8_t  Data[32]; /* 16x16 点阵数据 */
} ChineseCell_t;

/* ASCII 字模 */
extern const uint8_t OLED_F8x16[][16];   /* 宽8 高16 */
extern const uint8_t OLED_F6x8[][6];     /* 宽6 高8  */

/* 中文字模 */
extern const ChineseCell_t OLED_CF16x16[];

/* 图像 */
extern const uint8_t Diode[];

#endif
