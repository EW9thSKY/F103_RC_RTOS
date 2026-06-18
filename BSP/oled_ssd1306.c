#include "oled_ssd1306.h"
#include <string.h>
#include <stdio.h>

/* ---- DWT 微秒延时 (本地副本, freertos.c 中为 static) ---- */
static void DWT_DelayUs(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);
  while ((DWT->CYCCNT - start) < ticks);
}

/* ================================================================
 *  SSD1306 OLED 驱动 (I2C, 128×64)
 *  芯片: HS96L03W2C03 (SSD1306 兼容)
 *  接口: PB10=SCL, PB11=SDA (I2C2, 100kHz)
 * ================================================================ */

static I2C_HandleTypeDef *oled_i2c;

/* 全屏帧缓冲 (128×64 = 1024 字节, 8 页 × 128 列) */
static uint8_t framebuf[OLED_PAGES][OLED_WIDTH];

/* ================================================================
 *  6×8 字符字库 (ASCII 32-127, 每字符 6 字节列数据)
 * ================================================================ */
static const uint8_t font_6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /*   空格 */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F */
    {0x3E,0x41,0x41,0x49,0x7A,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* \ */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x08,0x04,0x08,0x10,0x08,0x00}, /* ~ */
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 127 */
};

/* ================================================================
 *  I2C 底层操作 (含超时恢复)
 * ================================================================ */
static uint8_t i2c_error_count = 0;   /* 连续 I2C 错误计数         */
#define I2C_MAX_ERRORS  5             /* 连续错误上限 → 总线复位    */
#define I2C_TX_TIMEOUT  20            /* 单次传输超时 (ms)         */

/* I2C 总线复位: 时钟脉冲 9 次释放 SDA */
static void OLED_I2C_Reset(void) {
    GPIO_InitTypeDef GPIO_Init = {0};

    /* SCL (PB10) + SDA (PB11) 切为 GPIO 开漏输出 */
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    /* 发 9 个时钟脉冲释放 SDA */
    for (uint8_t i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        DWT_DelayUs(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        DWT_DelayUs(10);
    }
    /* STOP 条件: SCL=H, SDA L→H */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    DWT_DelayUs(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    DWT_DelayUs(10);

    /* 恢复 I2C2 外设, MX_I2C2_Init 会重配 AF */
    HAL_I2C_DeInit(oled_i2c);
    /* 重新初始化 */
    {
        oled_i2c->Instance = I2C2;
        oled_i2c->Init.ClockSpeed = 100000;
        oled_i2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
        oled_i2c->Init.OwnAddress1 = 0;
        oled_i2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        oled_i2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        oled_i2c->Init.OwnAddress2 = 0;
        oled_i2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        oled_i2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        HAL_I2C_Init(oled_i2c);
    }
    i2c_error_count = 0;
}

static HAL_StatusTypeDef OLED_I2C_Tx(uint8_t *buf, uint16_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
        oled_i2c, SSD1306_I2C_ADDR, buf, len, I2C_TX_TIMEOUT);
    if (ret != HAL_OK) {
        i2c_error_count++;
        if (i2c_error_count >= I2C_MAX_ERRORS) {
            OLED_I2C_Reset();  /* 连续失败 → 总线复位 */
        }
    } else {
        i2c_error_count = 0;
    }
    return ret;
}

static void OLED_WriteCmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    OLED_I2C_Tx(buf, 2);
}

static void OLED_WriteData(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    OLED_I2C_Tx(buf, 2);
}

/* ================================================================
 *  初始化序列
 * ================================================================ */
void OLED_Init(I2C_HandleTypeDef *hi2c) {
    oled_i2c = hi2c;

    HAL_Delay(100);  /* 等待 OLED 上电稳定 */

    OLED_WriteCmd(0xAE); /* Display OFF */

    OLED_WriteCmd(0xD5); /* Set Oscillator Frequency */
    OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); /* Set Multiplex Ratio */
    OLED_WriteCmd(0x3F); /* 64 lines */
    OLED_WriteCmd(0xD3); /* Set Display Offset */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40); /* Set Display Start Line = 0 */
    OLED_WriteCmd(0x8D); /* Charge Pump */
    OLED_WriteCmd(0x14); /* Enable */
    OLED_WriteCmd(0x20); /* Memory Addressing Mode */
    OLED_WriteCmd(0x00); /* Horizontal */
    OLED_WriteCmd(0xA1); /* Segment Remap (左右翻转) */
    OLED_WriteCmd(0xC8); /* COM Scan Direction (上下翻转) */
    OLED_WriteCmd(0xDA); /* COM Pins */
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81); /* Set Contrast */
    OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9); /* Set Pre-charge */
    OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB); /* Set VCOMH */
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4); /* Resume to RAM content */
    OLED_WriteCmd(0xA6); /* Normal display (非反相) */
    OLED_WriteCmd(0x2E); /* Deactivate scroll */

    OLED_WriteCmd(0xAF); /* Display ON */

    OLED_Clear();
    OLED_Refresh();
}

/* ================================================================
 *  绘图操作 (操作 framebuffer)
 * ================================================================ */
void OLED_Clear(void) {
    memset(framebuf, 0x00, sizeof(framebuf));
}

void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    if (on)
        framebuf[y / 8][x] |=  (1 << (y & 7));
    else
        framebuf[y / 8][x] &= ~(1 << (y & 7));
}

/*
 * 在 (x, page) 处绘制 6×8 字符 (不刷新)
 *   x: 0~122 (超出截断)
 *   page: 0~7
 */
void OLED_DrawChar(uint8_t x, uint8_t page, char c) {
    if (page >= OLED_PAGES) return;
    uint8_t idx = (c >= ' ' && c <= 127) ? (uint8_t)(c - ' ') : 0;
    for (uint8_t col = 0; col < 6 && (x + col) < OLED_WIDTH; col++) {
        framebuf[page][x + col] = font_6x8[idx][col];
    }
}

void OLED_DrawString(uint8_t x, uint8_t page, const char *str) {
    while (*str && x <= OLED_WIDTH - 6) {
        OLED_DrawChar(x, page, *str);
        x += 6;
        str++;
    }
}

/*
 * 水平进度条
 *   (x, page) 起点, w 像素宽, h_px 像素高 (≤8)
 */
void OLED_DrawBarH(uint8_t x, uint8_t page, uint8_t w, uint8_t h_px) {
    if (page >= OLED_PAGES || h_px > 8) return;
    uint8_t mask = (uint8_t)((1 << h_px) - 1);
    for (uint8_t col = 0; col < w && (x + col) < OLED_WIDTH; col++) {
        framebuf[page][x + col] |= mask;
    }
}

/* ================================================================
 *  全屏刷新: 写 framebuffer → SSD1306 GDDRAM
 * ================================================================ */
void OLED_Refresh(void) {
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        OLED_WriteCmd(0xB0 + page);  /* Set Page Address */
        OLED_WriteCmd(0x00);         /* Set Column Low  = 0 */
        OLED_WriteCmd(0x10);         /* Set Column High = 0 */
        for (uint16_t col = 0; col < OLED_WIDTH; col++) {
            OLED_WriteData(framebuf[page][col]);
        }
    }
}

/* ================================================================
 *  高级显示: 信息布局 (由 ChassisTask 调用)
 * ================================================================ */
void OLED_ShowStatus(uint8_t  state,
                     uint16_t sensors,
                     uint16_t distance,
                     uint16_t calibPulse,
                     uint8_t  calibFailed,
                     uint32_t runTimeSec)
{
    char buf[22];  /* 128/6 = 21 字符 + null */

    OLED_Clear();

    /* ---- Line 0 (page 0): 系统状态 + 运行计时 ---- */
    switch (state) {
    case 0:  OLED_DrawString(0, 0, "STMWheels [IDLE]");      break;
    case 1:  OLED_DrawString(0, 0, "STMWheels [CALIB...]");  break;
    case 2:  OLED_DrawString(0, 0, "CAL FAILED! Retry?");    break;
    case 3:  OLED_DrawString(0, 0, "STMWheels [RUN]");       break;
    default: OLED_DrawString(0, 0, "STMWheels v1.0");        break;
    }
    /* 右上方显示运行秒数 (若 @2022/06/18 23:59:59 → 溢出回绕) */
    snprintf(buf, sizeof(buf), "%5lus", (unsigned long)runTimeSec);
    OLED_DrawString(90, 0, buf);

    /* ---- Line 1 (page 1): 循迹传感器位图 ---- */
    OLED_DrawString(0, 1, "TRK:");
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t on = (sensors >> i) & 1;
        OLED_DrawBarH(12 + i * 12 + 2, 1, 8, on ? 8 : 2);
    }
    OLED_DrawString(12, 2, "L2  L1  M  R1  R2");

    /* ---- Line 3 (page 3): 超声波距离 ---- */
    if (distance > 0 && distance <= 450) {
        snprintf(buf, sizeof(buf), "Dist: %3u cm", distance);
    } else if (distance == 0) {
        snprintf(buf, sizeof(buf), "Dist: --- cm");
    } else {
        snprintf(buf, sizeof(buf), "Dist: >450cm");
    }
    OLED_DrawString(0, 3, buf);
    if (distance > 0 && distance <= 5) {
        OLED_DrawString(90, 3, "STOP");
    }

    /* ---- Line 4 (page 4): IR PWM ---- */
    snprintf(buf, sizeof(buf), "IR PWM: %5u", calibPulse);
    OLED_DrawString(0, 4, buf);
    {
        uint8_t barW = (uint8_t)((uint32_t)calibPulse * 128 / 65535);
        OLED_DrawBarH(0, 5, barW, 6);
    }

    /* ---- Line 6 (page 6): 校准失败 / 故障 ---- */
    if (calibFailed) {
        OLED_DrawString(0, 6, "CAL FAILED!");
    }

    /* ---- Line 7 (page 7): 底部 ---- */
    OLED_DrawString(0, 7, "Btn:Sta/Stp");

    OLED_Refresh();
}
