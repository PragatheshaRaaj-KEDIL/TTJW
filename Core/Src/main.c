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
TIM_HandleTypeDef htim1;


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


/* ========================= SET VALUE / KEYPAD ============================== */

/*
 * The two keypad buttons that are physically working are used only for
 * changing the set value.
 *
 * S1 -> SV +1.0 C
 * S3 -> SV -1.0 C
 *
 * Configuration and Alarm modes are intentionally disabled for now.
 */
static float g_setValue = 0.0f;

#define KEY_SV_INCREMENT       1U
#define KEY_SV_DECREMENT       3U
#define KEYPAD_SCAN_PERIOD_MS  5U
#define KEYPAD_DEBOUNCE_MS     30U

static uint32_t g_keypadNextScanTick = 0U;
static uint8_t g_keypadRawKey = 0U;
static uint8_t g_keypadStableKey = 0U;
static uint32_t g_keypadRawChangeTick = 0U;


/* ========================= TEMPERATURE MOVING AVERAGE ==================== */

/*
 * 10-sample moving average.
 * ADS1220 is configured for 20 SPS, so 10 samples represent approximately
 * 500 ms of temperature smoothing.
 */
#define TEMPERATURE_AVERAGE_SAMPLES  10U

static float g_temperatureSamples[TEMPERATURE_AVERAGE_SAMPLES];
static float g_temperatureSampleSum = 0.0f;
static uint8_t g_temperatureSampleIndex = 0U;
static uint8_t g_temperatureAverageInitialized = 0U;


/* ========================= STARTUP ======================================= */


/*
 * Startup screen duration.
 *
 * The startup screen remains visible for exactly
 * 3.5 seconds after the frame is sent to the OLED.
 */
#define STARTUP_SCREEN_TIME_MS  1000U


/* ========================= TIMEOUT ======================================= */

#define ADS_DRDY_TIMEOUT_MS     2000U

/* ========================= HEATER / TIM1 ================================ */

/*
 * TIM1 CH1 is configured by CubeMX on PA8.
 * Timer clock = 80 MHz.
 * Prescaler = 7999 -> 10 kHz counter clock.
 * Period = 19999 -> 2 second PWM window.
 *
 * Therefore one complete PWM cycle is 2 seconds and the compare value
 * directly represents the heater ON time in that 2 second window.
 */
#define HEATER_PWM_PERIOD       19999U
#define HEATER_WINDOW_MS        2000U
#define HEATER_DUTY_STEP        10U
#define HEATER_MAX_DUTY         100U

/*
 * Initial POC proportional gain.
 * One degree below SV requests 30% heater demand before 10% quantization.
 * This value can be tuned after the first heater test.
 */
#define HEATER_KP_PERCENT_PER_C 30.0f



/* Private function prototypes ----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);

static void Heater_Init(void);
static void Heater_SetDuty(uint8_t dutyPercent);
static uint8_t Heater_CalculateDuty(float temperature);

/* KEYPAD */
static uint8_t Keypad_Scan(void);
static void Keypad_Process(void);
static void Keypad_HandleKey(uint8_t key);
static void Keypad_SetColumnsHigh(void);
static void Keypad_SelectColumn(uint8_t column);

/* OLED */

static void OLED_WriteCommand(uint8_t command);
static void OLED_WriteData(uint8_t *data, uint16_t size);
static void OLED_Init(void);
static void OLED_Clear(void);
static void OLED_FillBuffer(void);
static void OLED_UpdateScreen(void);
static void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t state);

static void OLED_SetCursor(uint8_t page, uint8_t column);
static void OLED_WriteChar(char ch);
static void OLED_WriteString(const char *str);

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

static void OLED_DrawCharNegative(
    uint8_t x,
    uint8_t y,
    char ch,
    uint8_t scale
);

static void OLED_DrawStringNegative(
    uint8_t x,
    uint8_t y,
    const char *str,
    uint8_t scale,
    uint8_t spacing
);

static void OLED_ShowStartupScreen(void);

static void OLED_DisplayTemperature(float temperature);


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
static float TemperatureMovingAverage_Update(float temperature);


/* ========================================================================= */
/* SIMPLE FONT                                                               */
/* ========================================================================= */

/*
 * 5x7 font.
 *
 * The first entries are used for numbers.
 */
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

    /* Degree symbol substitute / C is handled separately */

    /* k */
    else if (ch == 'k')
    {
        font[0] = 0x3F;
        font[1] = 0x10;
        font[2] = 0x28;
        font[3] = 0x44;
        font[4] = 0x00;
    }

    /* r */
    else if (ch == 'r')
    {
        font[0] = 0x7C;
        font[1] = 0x08;
        font[2] = 0x04;
        font[3] = 0x04;
        font[4] = 0x08;
    }

    /* m */
    else if (ch == 'm')
    {
        font[0] = 0x7C;
        font[1] = 0x04;
        font[2] = 0x18;
        font[3] = 0x04;
        font[4] = 0x78;
    }

    /* Degree symbol - internal '~' character. */
    else if (ch == '~')
    {
        font[0] = 0x06;
        font[1] = 0x09;
        font[2] = 0x09;
        font[3] = 0x06;
        font[4] = 0x00;
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


/* ========================================================================= */
/* MAIN                                                                      */
/* ========================================================================= */

int main(void)
{
    int32_t raw;
    float resistance;
    float temperature;
    float filteredTemperature;


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


    /* TIM1 / heater PWM */
    MX_TIM1_Init();
    Heater_Init();


    /* --------------------------------------------------------------------- */
    /* OLED                                                                  */
    /* --------------------------------------------------------------------- */

    OLED_Init();


    /*
     * Show startup screen.
     */
    OLED_ShowStartupScreen();


    /*
     * Keep startup screen visible for exactly 3.5 seconds.
     */
    HAL_Delay(STARTUP_SCREEN_TIME_MS);


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


    /*
     * Matrix keypad starts with both columns HIGH.
     */
    Keypad_SetColumnsHigh();


    /* --------------------------------------------------------------------- */
    /* MAIN LOOP                                                             */
    /* --------------------------------------------------------------------- */

    while (1)
    {
        /*
         * Process any keypad EXTI event from the main loop.
         * The EXTI callback itself remains very short.
         */
        Keypad_Process();


        raw = ADS1220_ReadConversion();


        /*
         * INT32_MIN means DRDY timeout.
         *
         * Safety: if the ADC does not provide a valid conversion,
         * turn the heater OFF.
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
             * Apply the 10-sample moving average after the PT100
             * resistance-to-temperature conversion.
             *
             * The first valid temperature fills all 10 samples so there
             * is no startup bias toward zero.
             */
            filteredTemperature =
                TemperatureMovingAverage_Update(temperature);


            /*
             * Heater control:
             *
             * - If SV is 0 C, heater remains OFF.
             * - Below SV, calculate heater demand.
             * - Demand is quantized in 10% steps.
             * - At or above SV, heater is forced OFF.
             */
            if (g_setValue <= 0.0f)
            {
                Heater_SetDuty(0U);
            }
            else if (filteredTemperature >= g_setValue)
            {
                Heater_SetDuty(0U);
            }
            else
            {
                Heater_SetDuty(Heater_CalculateDuty(filteredTemperature));
            }


            /*
             * Always show the live SV and filtered PV screen.
             * S1/S3 only change SV; they do not change the display mode.
             */
            OLED_DisplayTemperature(filteredTemperature);
        }
        else
        {
            /*
             * ADC timeout = heater OFF.
             */
            Heater_SetDuty(0U);
        }
    }
}


/* ========================================================================= */
/* HEATER CONTROL                                                            */
/* ========================================================================= */

/* ========================================================================= */
/* HEATER CONTROL                                                            */
/* ========================================================================= */

/* ------------------------------------------------------------------------- */
/* HEATER INITIALIZATION                                                     */
/* ------------------------------------------------------------------------- */

static void Heater_Init(void)
{
    /* Start with heater demand at 0%. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------------- */
/* SET HEATER DUTY                                                           */
/* ------------------------------------------------------------------------- */

static void Heater_SetDuty(uint8_t dutyPercent)
{
    uint32_t compare;

    /* Safety clamp. */
    if (dutyPercent > HEATER_MAX_DUTY)
    {
        dutyPercent = HEATER_MAX_DUTY;
    }

    /* Quantize demand to 10% steps. */
    dutyPercent =
        (uint8_t)((dutyPercent / HEATER_DUTY_STEP) * HEATER_DUTY_STEP);

    /*
     * 0%   -> CCR = 0
     * 10%  -> CCR = 2000 approximately
     * ...
     * 100% -> CCR = 20000, clamped to the timer period.
     */
    compare =
        ((uint32_t)HEATER_PWM_PERIOD * dutyPercent) / 100U;

    if (compare > HEATER_PWM_PERIOD)
    {
        compare = HEATER_PWM_PERIOD;
    }

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        compare
    );
}

/* ------------------------------------------------------------------------- */
/* CALCULATE HEATER DEMAND                                                   */
/* ------------------------------------------------------------------------- */

static uint8_t Heater_CalculateDuty(float temperature)
{
    float error;
    float demand;

    /* Hard safety condition. */
    if (temperature >= g_setValue)
    {
        return 0U;
    }

    error = g_setValue - temperature;

    demand = error * HEATER_KP_PERCENT_PER_C;

    if (demand <= 0.0f)
    {
        return 0U;
    }

    if (demand >= 100.0f)
    {
        return 100U;
    }

    return (uint8_t)(demand + 0.5f);
}



/* ========================================================================= */
/* KEYPAD                                                                    */
/* ========================================================================= */

static void Keypad_SetColumnsHigh(void)
{
    HAL_GPIO_WritePin(KEY_L1_GPIO_Port, KEY_L1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(KEY_L2_GPIO_Port, KEY_L2_Pin, GPIO_PIN_SET);
}

static void Keypad_SelectColumn(uint8_t column)
{
    if (column == 1U)
    {
        HAL_GPIO_WritePin(KEY_L1_GPIO_Port, KEY_L1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(KEY_L2_GPIO_Port, KEY_L2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(KEY_L1_GPIO_Port, KEY_L1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(KEY_L2_GPIO_Port, KEY_L2_Pin, GPIO_PIN_RESET);
    }
}

/*
 * The two buttons confirmed to work are S1 and S3. They are on L1:
 *
 *             L1        L2
 * R1          S1        S2
 * R2          S3        S4
 *
 * Therefore this firmware intentionally scans only L1. The unused L2
 * buttons are ignored until their hardware/wiring is investigated later.
 */
static uint8_t Keypad_Scan(void)
{
    GPIO_PinState r1;
    GPIO_PinState r2;
    uint8_t key = 0U;

    /* Drive the working column L1 LOW. */
    Keypad_SelectColumn(1U);
    HAL_Delay(1U);

    r1 = HAL_GPIO_ReadPin(KEY_R1_GPIO_Port, KEY_R1_Pin);
    r2 = HAL_GPIO_ReadPin(KEY_R2_GPIO_Port, KEY_R2_Pin);

    if (r1 == GPIO_PIN_RESET)
    {
        key = KEY_SV_INCREMENT;
    }
    else if (r2 == GPIO_PIN_RESET)
    {
        key = KEY_SV_DECREMENT;
    }

    Keypad_SetColumnsHigh();

    return key;
}

static void Keypad_Process(void)
{
    uint32_t now;
    uint8_t key;

    now = HAL_GetTick();

    if ((int32_t)(now - g_keypadNextScanTick) < 0)
    {
        return;
    }

    g_keypadNextScanTick = now + KEYPAD_SCAN_PERIOD_MS;

    key = Keypad_Scan();

    if (key != g_keypadRawKey)
    {
        g_keypadRawKey = key;
        g_keypadRawChangeTick = now;
        return;
    }

    if ((g_keypadStableKey != g_keypadRawKey) &&
        ((now - g_keypadRawChangeTick) >= KEYPAD_DEBOUNCE_MS))
    {
        g_keypadStableKey = g_keypadRawKey;

        /* One event is generated when the button becomes pressed. */
        if (g_keypadStableKey != 0U)
        {
            Keypad_HandleKey(g_keypadStableKey);
        }
    }
}

/*
 * S1 -> increment SV by 1 C
 * S3 -> decrement SV by 1 C
 */
static void Keypad_HandleKey(uint8_t key)
{
    switch (key)
    {
        case KEY_SV_INCREMENT:
            g_setValue += 1.0f;

            if (g_setValue > 100.0f)
            {
                g_setValue = 100.0f;
            }
            break;

        case KEY_SV_DECREMENT:
            g_setValue -= 1.0f;

            if (g_setValue < 0.0f)
            {
                g_setValue = 0.0f;
            }
            break;

        default:
            break;
    }
}

/* EXTI remains configured by CubeMX. The actual matrix scan is performed
 * from the main loop so no lengthy work is executed inside the ISR. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
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


    ADS1220_SPI_Transfer(
        ADS1220_CMD_RESET
    );


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
    ADS1220_SPI_Transfer(
        ADS1220_CMD_WREG_4REG
    );


    /* Register 0 */
    ADS1220_SPI_Transfer(
        ADS1220_REG0
    );


    /* Register 1 */
    ADS1220_SPI_Transfer(
        ADS1220_REG1
    );


    /* Register 2 */
    ADS1220_SPI_Transfer(
        ADS1220_REG2
    );


    /* Register 3 */
    ADS1220_SPI_Transfer(
        ADS1220_REG3
    );


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


    ADS1220_SPI_Transfer(
        ADS1220_CMD_START
    );


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
        /* Keep keypad scanning alive during the ~20 SPS conversion wait. */
        Keypad_Process();

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
     * DO NOT CHANGE THIS EQUATION.
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
/* OLED FILL                                                                 */
/* ------------------------------------------------------------------------- */

static void OLED_FillBuffer(void)
{
    /*
     * 0xFF = every OLED pixel ON.
     */
    memset(
        OLED_Buffer,
        0xFF,
        sizeof(OLED_Buffer)
    );
}


/* ------------------------------------------------------------------------- */
/* OLED SET PIXEL                                                            */
/* ------------------------------------------------------------------------- */

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


/* ------------------------------------------------------------------------- */
/* OLED UPDATE                                                               */
/* ------------------------------------------------------------------------- */

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


    OLED_GetGlyph(
        ch,
        font
    );


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

static void OLED_DrawCharScaled(
    char ch,
    uint8_t scale)
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


    OLED_GetGlyph(
        ch,
        font
    );


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
                            (uint8_t)(
                                (col * scale) + sx
                            );


                        py =
                            (uint8_t)(
                                (row * scale) + sy
                            );


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


        if ((x + (6U * scale)) >
            OLED_WIDTH)
        {
            break;
        }


        OLED_GetGlyph(
            *str,
            font
        );


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


/* ------------------------------------------------------------------------- */
/* OLED BIG STRING AT                                                        */
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


/* ------------------------------------------------------------------------- */
/* OLED NEGATIVE CHARACTER                                                   */
/* ------------------------------------------------------------------------- */

static void OLED_DrawCharNegative(
    uint8_t x,
    uint8_t y,
    char ch,
    uint8_t scale)
{
    uint8_t font[5];

    uint8_t column;
    uint8_t row;

    uint8_t dx;
    uint8_t dy;


    if (scale == 0)
    {
        scale = 1;
    }


    OLED_GetGlyph(
        ch,
        font
    );


    /*
     * Startup framebuffer is already completely ON.
     *
     * Therefore character pixels are cleared
     * to OFF.
     */
    for (column = 0;
         column < 5;
         column++)
    {
        uint8_t columnData =
            font[column];


        for (row = 0;
             row < 7;
             row++)
        {
            if ((columnData &
                 (1U << row)) != 0U)
            {
                for (dx = 0;
                     dx < scale;
                     dx++)
                {
                    for (dy = 0;
                         dy < scale;
                         dy++)
                    {
                        OLED_SetPixel(
                            (uint8_t)(
                                x +
                                (column * scale) +
                                dx
                            ),
                            (uint8_t)(
                                y +
                                (row * scale) +
                                dy
                            ),
                            0U
                        );
                    }
                }
            }
        }
    }
}


/* ------------------------------------------------------------------------- */
/* OLED NEGATIVE STRING                                                      */
/* ------------------------------------------------------------------------- */

static void OLED_DrawStringNegative(
    uint8_t x,
    uint8_t y,
    const char *str,
    uint8_t scale,
    uint8_t spacing)
{
    while (*str != '\0')
    {
        OLED_DrawCharNegative(
            x,
            y,
            *str,
            scale
        );


        x =
            (uint8_t)(
                x +
                (5U * scale) +
                spacing
            );


        str++;
    }
}


/* ------------------------------------------------------------------------- */
/* OLED STARTUP SCREEN                                                       */
/* ------------------------------------------------------------------------- */

static void OLED_ShowStartupScreen(void)
{
    /* Full-white background with dark startup text. */
    OLED_FillBuffer();

    /* Safe - BIG font. */
    OLED_DrawStringNegative(
        22,
        1,
        "Safe",
        4,
        1
    );

    /* Scientific - BIG font that fits the 128-pixel width. */
    OLED_DrawStringNegative(
        9,
        30,
        "Scientific",
        2,
        0
    );

    /* Jacket Warmer - SMALL font. */
    OLED_DrawStringNegative(
        25,
        53,
        "Jacket Warmer",
        1,
        1
    );

    OLED_UpdateScreen();
}


/* ------------------------------------------------------------------------- */
/* OLED DISPLAY TEMPERATURE                                                  */
/* ------------------------------------------------------------------------- */

static void OLED_DisplayTemperature(
    float temperature)
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
     * SV = live user-set value.
     */
    svWhole =
        (int)g_setValue;


    svDecimal =
        (int)(
            (g_setValue -
             (float)svWhole) *
            100.0f
        );


    /*
     * PV = actual measured temperature.
     */
    pvWhole =
        (int)temperature;


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
        "SV:%d.%02d~C",
        svWhole,
        svDecimal
    );


    /*
     * Prepare PV string.
     */
    snprintf(
        pvText,
        sizeof(pvText),
        "PV:%d.%02d~C",
        pvWhole,
        pvDecimal
    );


    /*
     * Clear framebuffer in RAM only.
     *
     * The physical OLED is not cleared first,
     * so there is no intentional blank frame.
     */
    OLED_Clear();


    /* Small normal-screen title. */
    OLED_DrawSmallStringAt(
        0,
        19,
        "Safe Scientific"
    );

    /* SV - LARGE 2x FONT. */
    OLED_DrawBigStringAt(
        2,
        4,
        svText
    );

    /* PV - LARGE 2x FONT with a small vertical gap after SV. */
    OLED_DrawBigStringAt(
        5,
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



/* ========================================================================= */
/* TEMPERATURE MOVING AVERAGE                                                */
/* ========================================================================= */

static float TemperatureMovingAverage_Update(float temperature)
{
    uint8_t i;
    float average;

    /*
     * On the first valid reading, initialize every sample with the same
     * temperature. This prevents the average from starting near 0 C.
     */
    if (g_temperatureAverageInitialized == 0U)
    {
        for (i = 0U; i < TEMPERATURE_AVERAGE_SAMPLES; i++)
        {
            g_temperatureSamples[i] = temperature;
        }

        g_temperatureSampleSum =
            temperature * (float)TEMPERATURE_AVERAGE_SAMPLES;

        g_temperatureSampleIndex = 0U;
        g_temperatureAverageInitialized = 1U;

        return temperature;
    }

    /* Remove the oldest sample from the running sum. */
    g_temperatureSampleSum -=
        g_temperatureSamples[g_temperatureSampleIndex];

    /* Store the new sample. */
    g_temperatureSamples[g_temperatureSampleIndex] = temperature;

    /* Add the new sample to the running sum. */
    g_temperatureSampleSum += temperature;

    /* Advance the circular buffer index. */
    g_temperatureSampleIndex++;

    if (g_temperatureSampleIndex >= TEMPERATURE_AVERAGE_SAMPLES)
    {
        g_temperatureSampleIndex = 0U;
    }

    average =
        g_temperatureSampleSum /
        (float)TEMPERATURE_AVERAGE_SAMPLES;

    return average;
}


/* ========================================================================= */
/* RESISTANCE TO TEMPERATURE                                                 */
/* ========================================================================= */

static float ResistanceToTemperature(
    float resistance)
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
     * This function is after the existing
     * resistance calculation.
     *
     * The resistance calculation itself
     * is NOT changed.
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


    RCC_OscInitStruct.PLL.PLLM =
        1;


    RCC_OscInitStruct.PLL.PLLN =
        10;


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
    hi2c1.Instance =
        I2C1;


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
    hspi1.Instance =
        SPI1;


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
/* TIM1                                                                      */
/* ========================================================================= */

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 7999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim1);
}


/* ========================================================================= */
/* GPIO                                                                      */
/* ========================================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* Initial keypad columns HIGH. */
    HAL_GPIO_WritePin(
        GPIOC,
        KEY_L1_Pin | KEY_L2_Pin,
        GPIO_PIN_SET
    );


    /* ADS1220 CS HIGH. */
    HAL_GPIO_WritePin(
        ADS_CS_GPIO_Port,
        ADS_CS_Pin,
        GPIO_PIN_SET
    );


    /* B1. */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(
        B1_GPIO_Port,
        &GPIO_InitStruct
    );


    /* Keypad columns: PC0 / PC1. */
    GPIO_InitStruct.Pin =
        KEY_L1_Pin |
        KEY_L2_Pin;

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


    /* Keypad rows: PC2 / PC3. */
    GPIO_InitStruct.Pin =
        KEY_R1_Pin |
        KEY_R2_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* ADS1220 CS. */
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


    /* Existing push button. */
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


    /* ADS1220 DRDY. */
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


    /* EXTI. */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
    /*
     * User can add assert reporting here.
     */
}

#endif
