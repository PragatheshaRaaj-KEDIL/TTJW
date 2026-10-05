/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TTJW - 3-Wire PT100 Temperature Measurement
  *                   STM32L476RG + ADS1220 + SH1106 OLED
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

/* Private defines -----------------------------------------------------------*/

/* ========================= OLED ========================================== */

#define OLED_ADDR               (0x3C << 1)

#define OLED_WIDTH              128
#define OLED_HEIGHT             64

static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];

/* ========================= ADS1220 ======================================= */

#define ADS1220_CMD_RESET       0x06
#define ADS1220_CMD_START       0x08

/*
 * WREG:
 *
 * 0100 rrnn
 *
 * rr = starting register
 * nn = number of registers - 1
 *
 * Starting at register 0
 * Writing 4 registers:
 *
 * 0100 0011 = 0x43
 */
#define ADS1220_CMD_WREG_4REG   0x43

/* ========================= ADS1220 CONFIG ================================ */

/*
 * REG0 = 0x66
 *
 * AINP = AIN1
 * AINN = AIN0
 * Gain = 8
 * PGA enabled
 */
#define ADS1220_REG0            0x66

/*
 * REG1 = 0x04
 *
 * 20 SPS
 * Normal mode
 * Continuous conversion
 */
#define ADS1220_REG1            0x04

/*
 * REG2 = 0x55
 *
 * External reference REFP0 / REFN0
 * 50/60 Hz rejection
 * IDAC current = 500 uA
 */
#define ADS1220_REG2            0x55

/*
 * REG3 = 0x70
 *
 * IDAC1 -> AIN2
 * IDAC2 -> AIN3
 */
#define ADS1220_REG3            0x70

/* ========================= RTD =========================================== */

/*
 * Measured RREF:
 *
 * Two 3.3k resistors in parallel
 * Actual measured resistance = 1.624k
 */
#define RREF_OHMS               1624.0f

#define RTD_GAIN                8.0f

#define ADC_FULL_SCALE          8388608.0f

/* ========================= PT100 ========================================= */

/*
 * PT100 nominal resistance at 0 deg C.
 */
#define PT100_R0                100.0f

/*
 * Callendar-Van Dusen coefficients for the
 * positive temperature region of a standard PT100.
 */
#define PT100_A                 0.0039083f
#define PT100_B                -0.0000005775f

/* ========================= FIXED SV ====================================== */

/*
 * Set Value is fixed during this development phase.
 *
 * Later this will be changed using the matrix keypad.
 */
#define SV_FIXED_C              37.00f

/* ========================= TIMEOUT ======================================= */

#define ADS_DRDY_TIMEOUT_MS     2000U

/* Private function prototypes ----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* OLED */
static void OLED_WriteCommand(uint8_t command);
static void OLED_WriteData(uint8_t *data, uint16_t size);
static void OLED_Init(void);
static void OLED_Clear(void);
static void OLED_UpdateScreen(void);
static void OLED_SetCursor(uint8_t page, uint8_t column);
static void OLED_WriteChar(char ch);
static void OLED_WriteString(const char *str);

static void OLED_DisplayTemperature(float temperature);

static void OLED_DrawCharScaled(char ch, uint8_t scale);
static void OLED_DrawStringScaled(const char *str, uint8_t scale);

static void OLED_DrawSmallStringAt(
    uint8_t page,
    uint8_t column,
    const char *str
);

static void OLED_DrawBigStringAt(
    uint8_t page,
    uint8_t column,
    const char *str
);

/* ADS1220 */
static void ADS1220_Select(void);
static void ADS1220_Deselect(void);
static uint8_t ADS1220_SPI_Transfer(uint8_t data);
static void ADS1220_Reset(void);
static void ADS1220_WriteRegisters(void);
static void ADS1220_StartConversion(void);
static int32_t ADS1220_ReadConversion(void);

/* RTD */
static float CalculateResistance(int32_t raw);
static float ResistanceToTemperature(float resistance);

/* ========================================================================= */
/* SIMPLE FONT                                                               */
/* ========================================================================= */

static const uint8_t Font5x7[][5] =
{
    /* Space */
    {0x00, 0x00, 0x00, 0x00, 0x00},

    /* 1 */
    {0x00, 0x06, 0x3F, 0x00, 0x00},

    /* 2 */
    {0x32, 0x29, 0x29, 0x29, 0x26},

    /* 3 */
    {0x22, 0x21, 0x29, 0x29, 0x36},

    /* 4 */
    {0x0C, 0x0A, 0x09, 0x3F, 0x08},

    /* 5 */
    {0x2F, 0x29, 0x29, 0x29, 0x31},

    /* 6 */
    {0x1E, 0x29, 0x29, 0x29, 0x10},

    /* 7 */
    {0x01, 0x01, 0x39, 0x05, 0x03},

    /* 8 */
    {0x36, 0x29, 0x29, 0x29, 0x36},

    /* 9 */
    {0x06, 0x29, 0x29, 0x29, 0x1E},

    /* 0 */
    {0x1E, 0x21, 0x21, 0x21, 0x1E}
};

/* ========================================================================= */
/* FONT GLYPH                                                                */
/* ========================================================================= */

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

    /* S */
    else if (ch == 'S')
    {
        font[0] = 0x26;
        font[1] = 0x29;
        font[2] = 0x29;
        font[3] = 0x29;
        font[4] = 0x12;
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

    /* Minus */
    else if (ch == '-')
    {
        font[0] = 0x08;
        font[1] = 0x08;
        font[2] = 0x08;
        font[3] = 0x08;
        font[4] = 0x08;
    }

    /* Space */
    else if (ch == ' ')
    {
        font[0] = 0x00;
        font[1] = 0x00;
        font[2] = 0x00;
        font[3] = 0x00;
        font[4] = 0x00;
    }
}

/* ========================================================================= */
/* MAIN                                                                      */
/* ========================================================================= */

int main(void)
{
    int32_t raw;
    float resistance;
    float temperature;

    /* HAL initialization */
    HAL_Init();

    /* System clock */
    SystemClock_Config();

    /* GPIO */
    MX_GPIO_Init();

    /* I2C */
    MX_I2C1_Init();

    /* SPI */
    MX_SPI1_Init();

    /* --------------------------------------------------------------------- */
    /* OLED                                                                  */
    /* --------------------------------------------------------------------- */

    OLED_Init();

    /*
     * Startup / welcome screen.
     */
    OLED_Clear();

    OLED_SetCursor(1, 42);
    OLED_WriteString("SAFE");

    OLED_SetCursor(2, 24);
    OLED_WriteString("SCIENTIFIC");

    OLED_SetCursor(4, 36);
    OLED_WriteString("JACKET");

    OLED_SetCursor(5, 36);
    OLED_WriteString("WARMER");

    OLED_UpdateScreen();

    HAL_Delay(1000);

    /* --------------------------------------------------------------------- */
    /* ADS1220                                                               */
    /* --------------------------------------------------------------------- */

    ADS1220_Reset();

    /*
     * Datasheet requires a delay after RESET before another command.
     */
    HAL_Delay(5);

    /*
     * Configure:
     *
     * REG0 = 66
     * REG1 = 04
     * REG2 = 55
     * REG3 = 70
     */
    ADS1220_WriteRegisters();

    HAL_Delay(5);

    /*
     * Start continuous conversion.
     */
    ADS1220_StartConversion();

    HAL_Delay(100);

    /* --------------------------------------------------------------------- */
    /* MAIN LOOP                                                             */
    /* --------------------------------------------------------------------- */

    while (1)
    {
        raw = ADS1220_ReadConversion();

        /*
         * INT32_MIN means DRDY timeout.
         */
        if (raw != INT32_MIN)
        {
            /*
             * KEEP THE EXISTING RESISTANCE CALCULATION UNCHANGED.
             */
            resistance = CalculateResistance(raw);

            /*
             * Convert PT100 resistance to temperature.
             */
            temperature = ResistanceToTemperature(resistance);

            /*
             * Display:
             *
             * JACKET WARMER
             *
             * SV:37.00 C
             *
             * PV:36.82 C
             *
             * No software delay here.
             */
            OLED_DisplayTemperature(temperature);
        }
    }
}

/* ========================================================================= */
/* ADS1220                                                                  */
/* ========================================================================= */

/* ------------------------------------------------------------------------- */
/* CS LOW                                                                   */
/* ------------------------------------------------------------------------- */

static void ADS1220_Select(void)
{
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_RESET
    );
}

/* ------------------------------------------------------------------------- */
/* CS HIGH                                                                  */
/* ------------------------------------------------------------------------- */

static void ADS1220_Deselect(void)
{
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_SET
    );
}

/* ------------------------------------------------------------------------- */
/* SPI Transfer                                                              */
/* ------------------------------------------------------------------------- */

static uint8_t ADS1220_SPI_Transfer(uint8_t data)
{
    uint8_t rxData = 0;

    if (HAL_SPI_TransmitReceive(
            &hspi1,
            &data,
            &rxData,
            1,
            HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }

    return rxData;
}

/* ------------------------------------------------------------------------- */
/* ADS1220 RESET                                                             */
/* ------------------------------------------------------------------------- */

static void ADS1220_Reset(void)
{
    ADS1220_Select();

    ADS1220_SPI_Transfer(ADS1220_CMD_RESET);

    ADS1220_Deselect();

    /*
     * Give ADS1220 time to complete reset.
     */
    HAL_Delay(5);
}

/* ------------------------------------------------------------------------- */
/* ADS1220 WRITE REGISTERS                                                   */
/* ------------------------------------------------------------------------- */

static void ADS1220_WriteRegisters(void)
{
    /*
     * IMPORTANT:
     *
     * Correct WREG command for registers 0-3 is:
     *
     * 0x43
     *
     * Complete SPI sequence:
     *
     * 43 66 04 55 70
     */

    ADS1220_Select();

    /* WREG command */
    ADS1220_SPI_Transfer(ADS1220_CMD_WREG_4REG);

    /* Register 0 */
    ADS1220_SPI_Transfer(ADS1220_REG0);

    /* Register 1 */
    ADS1220_SPI_Transfer(ADS1220_REG1);

    /* Register 2 */
    ADS1220_SPI_Transfer(ADS1220_REG2);

    /* Register 3 */
    ADS1220_SPI_Transfer(ADS1220_REG3);

    ADS1220_Deselect();

    /*
     * Configuration registers are updated on the final
     * SCLK falling edge.
     */
    HAL_Delay(2);
}

/* ------------------------------------------------------------------------- */
/* START CONVERSION                                                          */
/* ------------------------------------------------------------------------- */

static void ADS1220_StartConversion(void)
{
    ADS1220_Select();

    ADS1220_SPI_Transfer(ADS1220_CMD_START);

    ADS1220_Deselect();
}

/* ------------------------------------------------------------------------- */
/* READ CONVERSION                                                           */
/* ------------------------------------------------------------------------- */

static int32_t ADS1220_ReadConversion(void)
{
    uint32_t startTime;

    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;

    int32_t raw;

    /*
     * Wait for DRDY to go LOW.
     *
     * DRDY LOW = conversion result available.
     */
    startTime = HAL_GetTick();

    while (HAL_GPIO_ReadPin(
               ADS_DRDY_GPIO_Port,
               ADS_DRDY_Pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - startTime) >
            ADS_DRDY_TIMEOUT_MS)
        {
            /*
             * No valid conversion.
             */
            return INT32_MIN;
        }
    }

    /*
     * DRDY is LOW.
     *
     * Read 24-bit conversion result.
     */
    ADS1220_Select();

    /* Read MSB */
    byte1 = ADS1220_SPI_Transfer(0x00);

    /* Read middle byte */
    byte2 = ADS1220_SPI_Transfer(0x00);

    /* Read LSB */
    byte3 = ADS1220_SPI_Transfer(0x00);

    ADS1220_Deselect();

    /*
     * Build 24-bit signed result.
     */
    raw =
        ((int32_t)byte1 << 16) |
        ((int32_t)byte2 << 8) |
        ((int32_t)byte3);

    /*
     * Sign extension.
     */
    if (raw & 0x00800000)
    {
        raw |= 0xFF000000;
    }

    return raw;
}

/* ========================================================================= */
/* RTD RESISTANCE                                                            */
/* ========================================================================= */

static float CalculateResistance(int32_t raw)
{
    float resistance;

    /*
     * EXISTING RESISTANCE CALCULATION.
     *
     * RRTD =
     *
     * RAW x (2 x RREF)
     * -----------------
     * Gain x 2^23
     *
     * IDAC1 = 500 uA
     * IDAC2 = 500 uA
     *
     * Total reference current = 1 mA.
     */

    resistance =
        ((float)raw * (2.0f * RREF_OHMS)) /
        (RTD_GAIN * ADC_FULL_SCALE);

    /*
     * Negative resistance is not physically valid.
     */
    if (resistance < 0.0f)
    {
        resistance = 0.0f;
    }

    return resistance;
}

/* ========================================================================= */
/* OLED                                                                      */
/* ========================================================================= */

/* ------------------------------------------------------------------------- */
/* OLED COMMAND                                                              */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* OLED DATA                                                                 */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* OLED INIT                                                                 */
/* ------------------------------------------------------------------------- */

static void OLED_Init(void)
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

/* ------------------------------------------------------------------------- */
/* OLED CLEAR                                                                */
/* ------------------------------------------------------------------------- */

static void OLED_Clear(void)
{
    memset(
        OLED_Buffer,
        0x00,
        sizeof(OLED_Buffer)
    );
}

/* ------------------------------------------------------------------------- */
/* OLED UPDATE                                                               */
/* ------------------------------------------------------------------------- */

static void OLED_UpdateScreen(void)
{
    uint8_t page;

    for (page = 0; page < 8; page++)
    {
        OLED_WriteCommand(0xB0 + page);

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

/* ------------------------------------------------------------------------- */
/* OLED SET CURSOR                                                           */
/* ------------------------------------------------------------------------- */

static void OLED_SetCursor(
    uint8_t page,
    uint8_t column)
{
    uint8_t lower;
    uint8_t upper;

    if (page > 7)
    {
        page = 7;
    }

    if (column > 127)
    {
        column = 127;
    }

    lower = column & 0x0F;
    upper = (column >> 4) & 0x0F;

    OLED_WriteCommand(
        0xB0 + page
    );

    OLED_WriteCommand(
        0x00 | lower
    );

    OLED_WriteCommand(
        0x10 | upper
    );
}

/* ------------------------------------------------------------------------- */
/* OLED CHARACTER                                                            */
/* ------------------------------------------------------------------------- */

static void OLED_WriteChar(char ch)
{
    uint8_t font[5];
    uint8_t data[6];

    OLED_GetGlyph(ch, font);

    data[0] = font[0];
    data[1] = font[1];
    data[2] = font[2];
    data[3] = font[3];
    data[4] = font[4];
    data[5] = 0x00;

    OLED_WriteData(
        data,
        6
    );
}

/* ------------------------------------------------------------------------- */
/* OLED STRING                                                               */
/* ------------------------------------------------------------------------- */

static void OLED_WriteString(const char *str)
{
    while (*str)
    {
        OLED_WriteChar(*str);
        str++;
    }
}

/* ------------------------------------------------------------------------- */
/* OLED SCALED CHARACTER                                                     */
/* ------------------------------------------------------------------------- */

static void OLED_DrawCharScaled(char ch, uint8_t scale)
{
    uint8_t font[5];
    uint8_t col;
    uint8_t row;
    uint8_t sx;
    uint8_t sy;

    if (scale == 0)
    {
        scale = 1;
    }

    OLED_GetGlyph(ch, font);

    /*
     * This function is retained for future display expansion.
     * The main PV display uses OLED_DrawBigStringAt().
     */

    for (col = 0; col < 5; col++)
    {
        for (row = 0; row < 7; row++)
        {
            if (font[col] & (1U << row))
            {
                for (sx = 0; sx < scale; sx++)
                {
                    for (sy = 0; sy < scale; sy++)
                    {
                        uint8_t px;
                        uint8_t py;

                        px = (col * scale) + sx;
                        py = (row * scale) + sy;

                        if (px < OLED_WIDTH &&
                            py < OLED_HEIGHT)
                        {
                            OLED_Buffer[
                                px +
                                ((py / 8U) * OLED_WIDTH)
                            ] |=
                                (uint8_t)(1U << (py % 8U));
                        }
                    }
                }
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* OLED SCALED STRING                                                        */
/* ------------------------------------------------------------------------- */

static void OLED_DrawStringScaled(
    const char *str,
    uint8_t scale)
{
    uint8_t x = 0;
    uint8_t y = 0;

    if (scale == 0)
    {
        scale = 1;
    }

    while (*str)
    {
        uint8_t font[5];
        uint8_t col;
        uint8_t row;
        uint8_t sx;
        uint8_t sy;

        if ((x + (6U * scale)) > OLED_WIDTH)
        {
            break;
        }

        OLED_GetGlyph(*str, font);

        for (col = 0; col < 5; col++)
        {
            for (row = 0; row < 7; row++)
            {
                if (font[col] & (1U << row))
                {
                    for (sx = 0; sx < scale; sx++)
                    {
                        for (sy = 0; sy < scale; sy++)
                        {
                            uint8_t px;
                            uint8_t py;

                            px =
                                x +
                                (col * scale) +
                                sx;

                            py =
                                y +
                                (row * scale) +
                                sy;

                            if (px < OLED_WIDTH &&
                                py < OLED_HEIGHT)
                            {
                                OLED_Buffer[
                                    px +
                                    ((py / 8U) * OLED_WIDTH)
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

        x += (6U * scale);
        str++;
    }
}

/* ------------------------------------------------------------------------- */
/* OLED SMALL STRING AT                                                      */
/* ------------------------------------------------------------------------- */

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
        OLED_GetGlyph(*str, font);

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

/* ------------------------------------------------------------------------- */
/* OLED BIG STRING AT                                                         */
/* ------------------------------------------------------------------------- */

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

    /*
     * 2x font.
     *
     * One source pixel becomes a 2x2 pixel block.
     */
    uint8_t y = page * 8U;

    while (*str &&
           (x + 12 <= OLED_WIDTH))
    {
        OLED_GetGlyph(*str, font);

        for (col = 0; col < 5; col++)
        {
            for (row = 0; row < 7; row++)
            {
                if (font[col] & (1U << row))
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
         * Blank 2-pixel column between characters.
         */
        x += 12;

        str++;
    }
}

/* ------------------------------------------------------------------------- */
/* OLED DISPLAY TEMPERATURE                                                  */
/* ------------------------------------------------------------------------- */

static void OLED_DisplayTemperature(float temperature)
{
    char svText[20];
    char pvText[20];

    int svWhole;
    int svDecimal;

    int pvWhole;
    int pvDecimal;

    /*
     * Protect against negative temperature.
     */
    if (temperature < 0.0f)
    {
        temperature = 0.0f;
    }

    /*
     * Fixed SV = 37.00 C.
     */
    svWhole = (int)SV_FIXED_C;

    svDecimal =
        (int)(
            (SV_FIXED_C -
             (float)svWhole) *
            100.0f
        );

    /*
     * PV = actual measured temperature.
     */
    pvWhole = (int)temperature;

    pvDecimal =
        (int)(
            (temperature -
             (float)pvWhole) *
            100.0f
        );

    /*
     * Protect decimal rollover.
     */
    if (svDecimal >= 100)
    {
        svWhole++;
        svDecimal = 0;
    }

    if (pvDecimal >= 100)
    {
        pvWhole++;
        pvDecimal = 0;
    }

    /*
     * Prepare SV string.
     */
    snprintf(
        svText,
        sizeof(svText),
        "SV:%d.%02d C",
        svWhole,
        svDecimal
    );

    /*
     * Prepare PV string.
     */
    snprintf(
        pvText,
        sizeof(pvText),
        "PV:%d.%02d C",
        pvWhole,
        pvDecimal
    );

    /*
     * IMPORTANT:
     *
     * The framebuffer is cleared in RAM only.
     *
     * The OLED is NOT cleared physically before the
     * new frame is transmitted.
     *
     * Therefore there is no visible blank screen
     * between refreshes.
     */
    OLED_Clear();

    /*
     * Small title.
     */
    OLED_DrawSmallStringAt(
        0,
        22,
        "JACKET WARMER"
    );

    /*
     * SV in smaller font.
     */
    OLED_DrawSmallStringAt(
        2,
        0,
        svText
    );

    /*
     * PV in large 2x font.
     */
    OLED_DrawBigStringAt(
        4,
        4,
        pvText
    );

    /*
     * Send complete framebuffer.
     *
     * NO HAL_Delay() here.
     */
    OLED_UpdateScreen();
}

/* ------------------------------------------------------------------------- */
/* RESISTANCE TO TEMPERATURE                                                 */
/* ------------------------------------------------------------------------- */

static float ResistanceToTemperature(float resistance)
{
    float ratio;
    float discriminant;
    float temperature;

    /*
     * Positive-temperature Callendar-Van Dusen equation:
     *
     * R = R0 * (1 + A*T + B*T^2)
     *
     * Solve the quadratic for T.
     *
     * IMPORTANT:
     *
     * This function is added after the existing resistance
     * calculation. The resistance calculation itself is
     * not changed.
     */

    if (resistance < 0.0f)
    {
        resistance = 0.0f;
    }

    ratio =
        resistance /
        PT100_R0;

    discriminant =
        (PT100_A * PT100_A) -
        (4.0f *
         PT100_B *
         (1.0f - ratio));

    if (discriminant < 0.0f)
    {
        return 0.0f;
    }

    temperature =
        (-PT100_A +
         sqrtf(discriminant)) /
        (2.0f * PT100_B);

    if (temperature < 0.0f)
    {
        temperature = 0.0f;
    }

    return temperature;
}

/* ========================================================================= */
/* SYSTEM CLOCK                                                              */
/* ========================================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*
     * Configure regulator.
     */
    if (HAL_PWREx_ControlVoltageScaling(
            PWR_REGULATOR_VOLTAGE_SCALE1)
        != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * HSI oscillator.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 1;

    RCC_OscInitStruct.PLL.PLLN = 10;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV7;

    RCC_OscInitStruct.PLL.PLLQ =
        RCC_PLLQ_DIV2;

    RCC_OscInitStruct.PLL.PLLR =
        RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Clock configuration.
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_4)
        != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/* I2C1                                                                      */
/* ========================================================================= */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.Timing =
        0x10D19CE4;

    hi2c1.Init.OwnAddress1 =
        0;

    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c1.Init.OwnAddress2 =
        0;

    hi2c1.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;

    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1)
        != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(
            &hi2c1,
            I2C_ANALOGFILTER_ENABLE)
        != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(
            &hi2c1,
            0)
        != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/* SPI1                                                                      */
/* ========================================================================= */

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode =
        SPI_MODE_MASTER;

    hspi1.Init.Direction =
        SPI_DIRECTION_2LINES;

    hspi1.Init.DataSize =
        SPI_DATASIZE_8BIT;

    /*
     * SPI Mode 1:
     *
     * CPOL = 0
     * CPHA = 1
     */
    hspi1.Init.CLKPolarity =
        SPI_POLARITY_LOW;

    hspi1.Init.CLKPhase =
        SPI_PHASE_2EDGE;

    hspi1.Init.NSS =
        SPI_NSS_SOFT;

    hspi1.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_64;

    hspi1.Init.FirstBit =
        SPI_FIRSTBIT_MSB;

    hspi1.Init.TIMode =
        SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial =
        7;

    hspi1.Init.CRCLength =
        SPI_CRC_LENGTH_DATASIZE;

    hspi1.Init.NSSPMode =
        SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1)
        != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/* GPIO                                                                      */
/* ========================================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*
     * Enable GPIO clocks.
     */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * Initial K100 outputs.
     */
    HAL_GPIO_WritePin(
        GPIOC,
        K100_R1_Pin | K100_R2_Pin,
        GPIO_PIN_SET
    );

    /*
     * ADS1220 CS HIGH.
     */
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_SET
    );

    /*
     * B1.
     */
    GPIO_InitStruct.Pin =
        B1_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_RISING_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        B1_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * K100 pins.
     */
    GPIO_InitStruct.Pin =
        K100_R1_Pin |
        K100_R2_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );

    /*
     * PC2 / PC3.
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_2 |
        GPIO_PIN_3;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );

    /*
     * ADS1220 CS.
     */
    GPIO_InitStruct.Pin =
        ADS_CS_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        ADS_CS_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * Push button.
     */
    GPIO_InitStruct.Pin =
        PUSH_BUTTON_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        PUSH_BUTTON_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * ADS1220 DRDY.
     */
    GPIO_InitStruct.Pin =
        ADS_DRDY_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        ADS_DRDY_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * EXTI.
     */
    HAL_NVIC_SetPriority(
        EXTI0_IRQn,
        0,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI0_IRQn
    );

    HAL_NVIC_SetPriority(
        EXTI2_IRQn,
        0,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI2_IRQn
    );

    HAL_NVIC_SetPriority(
        EXTI3_IRQn,
        0,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI3_IRQn
    );

    HAL_NVIC_SetPriority(
        EXTI15_10_IRQn,
        0,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI15_10_IRQn
    );
}

/* ========================================================================= */
/* ERROR HANDLER                                                             */
/* ========================================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    /*
     * User can add assert reporting here.
     */
}

#endif
