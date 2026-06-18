#ifndef __OLED_SSD1306_H__
#define __OLED_SSD1306_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* OLED 尺寸 */
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_PAGES     8   /* HEIGHT / 8 */

/* SSD1306 I2C 地址 (7-bit: 0x3C, 8-bit: 0x78) */
#define SSD1306_I2C_ADDR  0x78

/* ---- 初始化 ---- */
void OLED_Init(I2C_HandleTypeDef *hi2c);

/* ---- 基础绘图 ---- */
void OLED_Clear(void);
void OLED_Refresh(void);          /* 全屏刷新 (写入 framebuffer)     */
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);
void OLED_DrawChar(uint8_t x, uint8_t page, char c);
void OLED_DrawString(uint8_t x, uint8_t page, const char *str);
void OLED_DrawBarH(uint8_t x, uint8_t page, uint8_t w, uint8_t h_px);

/* ---- 高级显示 (由 ChassisTask 调用) ---- */
void OLED_ShowStatus(uint8_t  state,
                     uint16_t sensors,
                     uint16_t distance,
                     uint16_t calibPulse,
                     uint8_t  calibFailed,
                     uint32_t runTimeSec);

#endif /* __OLED_SSD1306_H__ */
