/* SH1106 OLED driver and TTJW screen rendering. */
#include "main.h"
#include "ttjw_config.h"
#include "ttjw_state.h"
#include "oled_sh1106.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;

/* 1-bit framebuffer: one bit per pixel, grouped in 8-pixel pages. */
static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8U];

static const uint8_t Font5x7[][5] =
{
    /* 0 - Space */
    {0x00, 0x00, 0x00, 0x00, 0x00},

    /* 1 - 1 */
    {0x00, 0x06, 0x3F, 0x00, 0x00},

    /* 2 - 2 */
    {0x32, 0x29, 0x29, 0x29, 0x26},

    /* 3 - 3 */
    {0x22, 0x21, 0x29, 0x29, 0x36},

    /* 4 - 4 */
    {0x0C, 0x0A, 0x09, 0x3F, 0x08},

    /* 5 - 5 */
    {0x2F, 0x29, 0x29, 0x29, 0x31},

    /* 6 - 6 */
    {0x1E, 0x29, 0x29, 0x29, 0x10},

    /* 7 - 7 */
    {0x01, 0x01, 0x39, 0x05, 0x03},

    /* 8 - 8 */
    {0x36, 0x29, 0x29, 0x29, 0x36},

    /* 9 - 9 */
    {0x06, 0x29, 0x29, 0x29, 0x1E},

    /* 10 - 0 */
    {0x1E, 0x21, 0x21, 0x21, 0x1E}
};


static void OLED_GetGlyph(char ch, uint8_t *font);
static void OLED_WriteCommand(uint8_t command);
static void OLED_WriteData(uint8_t *data, uint16_t size);
static void OLED_Clear(void);
static void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t state);
static void OLED_UpdateScreen(void);
static void OLED_DrawSmallStringAt(uint8_t page, uint8_t column, const char *str);
static void OLED_DrawBigStringAt(uint8_t page, uint8_t column, const char *str);
static void OLED_DrawIndicatorBox(uint8_t x, uint8_t y, uint8_t filled);


static void OLED_GetGlyph(char ch, uint8_t *font)
{
    uint8_t index;

    memset(font, 0x00, 5);

    /* Numbers */
    if ((ch >= '0') && (ch <= '9'))
    {
        if (ch == '0')
        {
            index = 10;
        }
        else
        {
            index = (uint8_t)(ch - '0');
        }

        memcpy(
            font,
            Font5x7[index],
            5
        );
    }

    /* R */
    else if (ch == 'R')
    {
        font[0] = 0x3F;
        font[1] = 0x09;
        font[2] = 0x19;
        font[3] = 0x29;
        font[4] = 0x06;
    }

    /* S */
    else if (ch == 'S')
    {
        font[0] = 0x26;
        font[1] = 0x29;
        font[2] = 0x29;
        font[3] = 0x29;
        font[4] = 0x12;
    }

    /* V */
    else if (ch == 'V')
    {
        font[0] = 0x1F;
        font[1] = 0x20;
        font[2] = 0x20;
        font[3] = 0x20;
        font[4] = 0x1F;
    }

    /* P */
    else if (ch == 'P')
    {
        font[0] = 0x3F;
        font[1] = 0x09;
        font[2] = 0x09;
        font[3] = 0x09;
        font[4] = 0x06;
    }

    /* C */
    else if (ch == 'C')
    {
        font[0] = 0x1E;
        font[1] = 0x21;
        font[2] = 0x21;
        font[3] = 0x21;
        font[4] = 0x12;
    }

    /* J */
    else if (ch == 'J')
    {
        font[0] = 0x20;
        font[1] = 0x20;
        font[2] = 0x21;
        font[3] = 0x1F;
        font[4] = 0x00;
    }

    /* A */
    else if (ch == 'A')
    {
        font[0] = 0x3E;
        font[1] = 0x09;
        font[2] = 0x09;
        font[3] = 0x09;
        font[4] = 0x3E;
    }

    /* K */
    else if (ch == 'K')
    {
        font[0] = 0x3F;
        font[1] = 0x08;
        font[2] = 0x14;
        font[3] = 0x22;
        font[4] = 0x00;
    }

    /* E */
    else if (ch == 'E')
    {
        font[0] = 0x3F;
        font[1] = 0x29;
        font[2] = 0x29;
        font[3] = 0x29;
        font[4] = 0x21;
    }

    /* T */
    else if (ch == 'T')
    {
        font[0] = 0x01;
        font[1] = 0x01;
        font[2] = 0x3F;
        font[3] = 0x01;
        font[4] = 0x01;
    }

    /* W */
    else if (ch == 'W')
    {
        font[0] = 0x1F;
        font[1] = 0x20;
        font[2] = 0x18;
        font[3] = 0x20;
        font[4] = 0x1F;
    }

    /* M */
    else if (ch == 'M')
    {
        font[0] = 0x3F;
        font[1] = 0x02;
        font[2] = 0x0C;
        font[3] = 0x02;
        font[4] = 0x3F;
    }

    /* G */
    else if (ch == 'G')
    {
        font[0] = 0x1E;
        font[1] = 0x21;
        font[2] = 0x29;
        font[3] = 0x2D;
        font[4] = 0x1A;
    }

    /* F */
    else if (ch == 'F')
    {
        font[0] = 0x3F;
        font[1] = 0x09;
        font[2] = 0x09;
        font[3] = 0x01;
        font[4] = 0x01;
    }

    /* I */
    else if (ch == 'I')
    {
        font[0] = 0x21;
        font[1] = 0x21;
        font[2] = 0x3F;
        font[3] = 0x21;
        font[4] = 0x21;
    }

    /* N */
    else if (ch == 'N')
    {
        font[0] = 0x3F;
        font[1] = 0x02;
        font[2] = 0x0C;
        font[3] = 0x10;
        font[4] = 0x3F;
    }

    /* O */
    else if (ch == 'O')
    {
        font[0] = 0x1E;
        font[1] = 0x21;
        font[2] = 0x21;
        font[3] = 0x21;
        font[4] = 0x1E;
    }

    /* U */
    else if (ch == 'U')
    {
        font[0] = 0x1F;
        font[1] = 0x20;
        font[2] = 0x20;
        font[3] = 0x20;
        font[4] = 0x1F;
    }

    /* D */
    else if (ch == 'D')
    {
        font[0] = 0x3F;
        font[1] = 0x21;
        font[2] = 0x21;
        font[3] = 0x12;
        font[4] = 0x0C;
    }

    /* H */
    else if (ch == 'H')
    {
        font[0] = 0x3F;
        font[1] = 0x08;
        font[2] = 0x08;
        font[3] = 0x08;
        font[4] = 0x3F;
    }

    /* L */
    else if (ch == 'L')
    {
        font[0] = 0x3F;
        font[1] = 0x20;
        font[2] = 0x20;
        font[3] = 0x20;
        font[4] = 0x20;
    }

    /* Y */
    else if (ch == 'Y')
    {
        font[0] = 0x03;
        font[1] = 0x04;
        font[2] = 0x38;
        font[3] = 0x04;
        font[4] = 0x03;
    }

    /* Colon */
    else if (ch == ':')
    {
        font[0] = 0x00;
        font[1] = 0x14;
        font[2] = 0x00;
        font[3] = 0x14;
        font[4] = 0x00;
    }

    /* Decimal point */
    else if (ch == '.')
    {
        font[0] = 0x00;
        font[1] = 0x20;
        font[2] = 0x00;
        font[3] = 0x00;
        font[4] = 0x00;
    }

    /* Hyphen */
    else if (ch == '-')
    {
        font[0] = 0x00;
        font[1] = 0x08;
        font[2] = 0x08;
        font[3] = 0x08;
        font[4] = 0x00;
    }

    /* Degree symbol substitute / C is handled separately */

    /* Space */
    else if (ch == ' ')
    {
        font[0] = 0x00;
        font[1] = 0x00;
        font[2] = 0x00;
        font[3] = 0x00;
        font[4] = 0x00;
    }

    /* --------------------------------------------------------------------- */
    /* Lowercase letters used by startup screen                             */
    /* --------------------------------------------------------------------- */

    /* a */
    else if (ch == 'a')
    {
        font[0] = 0x20;
        font[1] = 0x54;
        font[2] = 0x54;
        font[3] = 0x54;
        font[4] = 0x78;
    }

    /* c */
    else if (ch == 'c')
    {
        font[0] = 0x38;
        font[1] = 0x44;
        font[2] = 0x44;
        font[3] = 0x44;
        font[4] = 0x20;
    }

    /* e */
    else if (ch == 'e')
    {
        font[0] = 0x38;
        font[1] = 0x54;
        font[2] = 0x54;
        font[3] = 0x54;
        font[4] = 0x18;
    }

    /* f */
    else if (ch == 'f')
    {
        font[0] = 0x08;
        font[1] = 0x7E;
        font[2] = 0x09;
        font[3] = 0x01;
        font[4] = 0x02;
    }

    /* i */
    else if (ch == 'i')
    {
        font[0] = 0x00;
        font[1] = 0x44;
        font[2] = 0x7D;
        font[3] = 0x40;
        font[4] = 0x00;
    }

    /* n */
    else if (ch == 'n')
    {
        font[0] = 0x7C;
        font[1] = 0x08;
        font[2] = 0x04;
        font[3] = 0x04;
        font[4] = 0x78;
    }

    /* t */
    else if (ch == 't')
    {
        font[0] = 0x04;
        font[1] = 0x3F;
        font[2] = 0x44;
        font[3] = 0x40;
        font[4] = 0x20;
    }
}

static void OLED_WriteCommand(uint8_t command)
{
    uint8_t buffer[2];


    buffer[0] = 0x00;
    buffer[1] = command;


    HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        buffer,
        2,
        HAL_MAX_DELAY
    );
}

static void OLED_WriteData(uint8_t *data, uint16_t size)
{
    uint8_t buffer[129];


    if (size > 128)
    {
        size = 128;
    }


    buffer[0] = 0x40;


    memcpy(
        &buffer[1],
        data,
        size
    );


    HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        buffer,
        size + 1,
        HAL_MAX_DELAY
    );
}

void OLED_Init(void)
{
    HAL_Delay(100);


    /*
     * SH1106 initialization.
     */

    OLED_WriteCommand(0xAE);

    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);

    OLED_WriteCommand(0xAD);
    OLED_WriteCommand(0x8B);

    OLED_WriteCommand(0xA1);

    OLED_WriteCommand(0xC8);

    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0x22);

    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x35);

    OLED_WriteCommand(0xA4);

    OLED_WriteCommand(0xA6);

    OLED_WriteCommand(0xAF);


    OLED_Clear();

    OLED_UpdateScreen();
}

static void OLED_Clear(void)
{
    memset(
        OLED_Buffer,
        0x00,
        sizeof(OLED_Buffer)
    );
}



static void OLED_SetPixel(
    uint8_t x,
    uint8_t y,
    uint8_t state)
{
    uint16_t index;
    uint8_t bit;


    if (x >= OLED_WIDTH ||
        y >= OLED_HEIGHT)
    {
        return;
    }


    index =
        (uint16_t)x +
        ((uint16_t)(y / 8U) * OLED_WIDTH);


    bit =
        (uint8_t)(1U << (y % 8U));


    if (state != 0U)
    {
        OLED_Buffer[index] |= bit;
    }
    else
    {
        OLED_Buffer[index] &= (uint8_t)~bit;
    }
}

static void OLED_UpdateScreen(void)
{
    uint8_t page;


    for (page = 0; page < 8; page++)
    {
        /*
         * SH1106 page.
         */
        OLED_WriteCommand(
            (uint8_t)(0xB0 + page)
        );


        /*
         * SH1106 column address offset.
         */
        OLED_WriteCommand(0x02);

        OLED_WriteCommand(0x10);


        OLED_WriteData(
            &OLED_Buffer[OLED_WIDTH * page],
            OLED_WIDTH
        );
    }
}











static void OLED_DrawSmallStringAt(
    uint8_t page,
    uint8_t column,
    const char *str)
{
    uint8_t font[5];
    uint8_t col;
    uint8_t x = column;


    if (page > 7)
    {
        page = 7;
    }


    while (*str &&
           (x + 6 <= OLED_WIDTH))
    {
        OLED_GetGlyph(
            *str,
            font
        );


        for (col = 0; col < 5; col++)
        {
            OLED_Buffer[
                (page * OLED_WIDTH) +
                x +
                col
            ] = font[col];
        }


        OLED_Buffer[
            (page * OLED_WIDTH) +
            x +
            5
        ] = 0x00;


        x += 6;

        str++;
    }
}

static void OLED_DrawBigStringAt(
    uint8_t page,
    uint8_t column,
    const char *str)
{
    uint8_t font[5];

    uint8_t x = column;

    uint8_t col;
    uint8_t row;

    uint8_t sx;
    uint8_t sy;

    uint8_t y =
        (uint8_t)(page * 8U);


    /*
     * 2x font.
     *
     * One source pixel becomes a 2x2 pixel block.
     */


    while (*str &&
           (x + 12 <= OLED_WIDTH))
    {
        OLED_GetGlyph(
            *str,
            font
        );


        for (col = 0; col < 5; col++)
        {
            for (row = 0; row < 7; row++)
            {
                if (font[col] &
                    (1U << row))
                {
                    for (sx = 0; sx < 2; sx++)
                    {
                        for (sy = 0; sy < 2; sy++)
                        {
                            uint8_t px;
                            uint8_t py;


                            px =
                                x +
                                (col * 2U) +
                                sx;


                            py =
                                y +
                                (row * 2U) +
                                sy;


                            if (px < OLED_WIDTH &&
                                py < OLED_HEIGHT)
                            {
                                OLED_Buffer[
                                    px +
                                    ((py / 8U) *
                                     OLED_WIDTH)
                                ] |=
                                    (uint8_t)(
                                        1U <<
                                        (py % 8U)
                                    );
                            }
                        }
                    }
                }
            }
        }


        /*
         * Blank 2-pixel column
         * between characters.
         */
        x += 12;


        str++;
    }
}







static void OLED_DrawIndicatorBox(uint8_t x, uint8_t y, uint8_t filled)
{
    uint8_t dx;
    uint8_t dy;

    /* 8x8 pixel box. Filled means the indicator is ON. */
    for (dy = 0U; dy < 8U; dy++)
    {
        for (dx = 0U; dx < 8U; dx++)
        {
            uint8_t edge = (uint8_t)(
                (dx == 0U) || (dx == 7U) ||
                (dy == 0U) || (dy == 7U));

            if ((filled != 0U) || (edge != 0U))
            {
                OLED_SetPixel((uint8_t)(x + dx),
                              (uint8_t)(y + dy), 1U);
            }
        }
    }
}

void OLED_DisplayTemperature(
    float temperature)
{
    char svText[20], pvText[20], valueText[24];
    int svWhole, svDecimal, pvWhole, pvDecimal;
    uint32_t pwmCounter;
    uint32_t pwmCompare;
    uint8_t outIndicatorOn;

    if (g_displayMode == DISPLAY_MODE_ALARM1)
    {
        int tenths = (int)(g_alarm1Value * 10.0f +
                           ((g_alarm1Value >= 0.0f) ? 0.5f : -0.5f));
        int absTenths = (tenths < 0) ? -tenths : tenths;
        if (tenths < 0)
            (void)snprintf(valueText, sizeof(valueText), "-%d.%d",
                           absTenths / 10, absTenths % 10);
        else
            (void)snprintf(valueText, sizeof(valueText), "%d.%d",
                           absTenths / 10, absTenths % 10);
        OLED_Clear();
        OLED_DrawBigStringAt(0U, 10U, "ALARM - 1");
        OLED_DrawBigStringAt(4U, (tenths < 0) ? 16U : 28U, valueText);
        OLED_UpdateScreen();
        return;
    }

    if (g_displayMode == DISPLAY_MODE_ADJUSTMENT_MENU)
    {
        OLED_Clear();
        /* Two large lines: each 10-character word fits the 128-pixel width. */
        OLED_DrawBigStringAt(0U, 4U, "ADJUSTMENT");
        OLED_DrawBigStringAt(4U, 4U, "PARAMETERS");
        OLED_UpdateScreen();
        return;
    }

    if (g_displayMode == DISPLAY_MODE_ERROR_ADJUSTMENT)
    {
        int tenths = (int)(g_errorAdjustment * 10.0f +
                           ((g_errorAdjustment >= 0.0f) ? 0.5f : -0.5f));
        int absTenths = (tenths < 0) ? -tenths : tenths;
        if (tenths < 0)
            (void)snprintf(valueText, sizeof(valueText), "-%d.%d",
                           absTenths / 10, absTenths % 10);
        else
            (void)snprintf(valueText, sizeof(valueText), "%d.%d",
                           absTenths / 10, absTenths % 10);
        OLED_Clear();
        OLED_DrawBigStringAt(0U, 38U, "ERROR");
        OLED_DrawBigStringAt(4U, (tenths < 0) ? 16U : 28U, valueText);
        OLED_UpdateScreen();
        return;
    }

    if (g_displayMode == DISPLAY_MODE_PID_P ||
        g_displayMode == DISPLAY_MODE_PID_I ||
        g_displayMode == DISPLAY_MODE_PID_D)
    {
        const char *label = (g_displayMode == DISPLAY_MODE_PID_P) ? "P" :
                            (g_displayMode == DISPLAY_MODE_PID_I) ? "I" : "D";
        float pidValue = (g_displayMode == DISPLAY_MODE_PID_P) ? g_pidPValue :
                         (g_displayMode == DISPLAY_MODE_PID_I) ? g_pidIValue :
                         g_pidDValue;
        /*
         * Display the shared float control variable with one decimal place.
         * Convert to tenths and use integer formatting so this works even
         * when STM32 newlib-nano floating-point printf is not enabled.
         * The value shown here is the same g_pidPValue/g_pidIValue/g_pidDValue
         * variable used by Heater_CalculateDuty().
         */
        {
            int pidTenths = (int)(pidValue * 10.0f + 0.5f);
            (void)snprintf(valueText, sizeof(valueText), "%d.%d",
                           pidTenths / 10, pidTenths % 10);
        }
        OLED_Clear();
        OLED_DrawBigStringAt(0U, 58U, label);
        OLED_DrawBigStringAt(4U, (strlen(valueText) > 3U) ? 16U : 28U, valueText);
        OLED_UpdateScreen();
        return;
    }

    /* Operation screen: retain existing title and large SV/PV layout. */
    {
        int svTenths = (int)(g_setValue * 10.0f + 0.5f);
        svWhole = svTenths / 10;
        svDecimal = svTenths % 10;
    }
    pvWhole = (int)temperature;
    pvDecimal = (int)((temperature - (float)pvWhole) * 100.0f);
    if (pvDecimal >= 100) { pvWhole++; pvDecimal = 0; }

    (void)snprintf(svText, sizeof(svText), "SV:%d.%01d C", svWhole, svDecimal);
    (void)snprintf(pvText, sizeof(pvText), "PV:%d.%02d C", pvWhole, pvDecimal);
    OLED_Clear();
    OLED_DrawSmallStringAt(0U, 22U, "JACKET WARMER");
    OLED_DrawBigStringAt(2U, 4U, svText);
    OLED_DrawBigStringAt(4U, 4U, pvText);

    /* ALM is a steady threshold indicator. */
    OLED_DrawSmallStringAt(6U, 8U, "ALM");
    OLED_DrawIndicatorBox(32U, 48U, g_alarmActive);

    /*
     * OUT follows the live TIM1 PWM phase, not merely nonzero duty demand.
     * PWM mode 1 with active-high polarity is ON while CNT < CCR.
     * Explicit 0% and 100% handling avoids edge-case display flicker.
     */
    pwmCounter = __HAL_TIM_GET_COUNTER(&htim1);
    pwmCompare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);

    if (g_heaterDutyPercent == 0U)
    {
        outIndicatorOn = 0U;
    }
    else if (g_heaterDutyPercent >= 100U)
    {
        outIndicatorOn = 1U;
    }
    else
    {
        outIndicatorOn = (pwmCounter < pwmCompare) ? 1U : 0U;
    }

    OLED_DrawSmallStringAt(6U, 72U, "OUT");
    OLED_DrawIndicatorBox(96U, 48U, outIndicatorOn);

    OLED_UpdateScreen();
}
