#ifndef __OLED_H__
#define __OLED_H__

#include "sys.h"

typedef enum {
    OLED_ZH_MENU = 0,
    OLED_ZH_Q3,
    OLED_ZH_HOLD,
    OLED_ZH_CENTER_HOLD,
    OLED_ZH_CAPTURE,
    OLED_ZH_CAPTURED,
    OLED_ZH_FREE_HOLD,
    OLED_ZH_RETURN_ZERO,
    OLED_ZH_BALL_LOST,
    OLED_ZH_ESTOP,
    OLED_ZH_FAULT
} OLED_ZhText;

/* SSD1306 128x64, I2C address 0x3C, PB6=SCL, PB7=SDA. */
u8 OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowLine(u8 row, const char *text);
void OLED_ShowChineseLine(u8 row16, OLED_ZhText text);
u8 OLED_Refresh(void);

#endif
