/* TTJW configuration: tune only after checking hardware timing and safety requirements. */
#ifndef TTJW_CONFIG_H
#define TTJW_CONFIG_H

/* OLED / SH1106 */
#define OLED_ADDR               (0x3CU << 1)
#define OLED_WIDTH              128U
#define OLED_HEIGHT             64U

/* ADS1220 register configuration */
#define ADS1220_CMD_RESET       0x06U
#define ADS1220_CMD_START       0x08U
#define ADS1220_CMD_WREG_4REG   0x43U
#define ADS1220_REG0            0x66U
#define ADS1220_REG1            0x04U
#define ADS1220_REG2            0x55U
#define ADS1220_REG3            0x70U

/* PT100 analogue front-end constants */
#define RREF_OHMS               1624.0f
#define RTD_GAIN                8.0f
#define ADC_FULL_SCALE          8388608.0f
#define PT100_R0                100.0f
#define PT100_A                 0.0039083f
#define PT100_B                -0.0000005775f

/* Setpoint and adjustment ranges */
#define SV_MIN_VALUE           (-199.9f)
#define SV_MAX_VALUE            500.0f
#define SV_STEP                   1.0f
#define AL1_MIN_VALUE             0.0f
#define AL1_MAX_VALUE           999.9f
#define AL1_STEP                  0.1f
#define ERROR_MIN_VALUE        (-199.9f)
#define ERROR_MAX_VALUE         999.9f
#define ERROR_STEP                0.1f
#define PID_VALUE_MIN             0.0f
#define PID_VALUE_MAX          9999.0f

/* Keypad scan and repeat timing */
#define KEY_S1                    1U
#define KEY_S2                    2U
#define KEY_S3                    3U
#define KEY_S4                    4U
#define KEYPAD_SCAN_PERIOD_MS     5U
#define KEYPAD_DEBOUNCE_MS       30U
#define KEYPAD_HOLD_INITIAL_MS  500U
#define KEYPAD_HOLD_REPEAT_MS   150U

/* ADC fault timeout */
#define ADS_DRDY_TIMEOUT_MS    2000U

/* TIM1 CH1: 10 kHz counter, ARR 19999 => 2-second PWM time window. */
#define HEATER_PWM_PERIOD      19999U
#define HEATER_WINDOW_MS       2000U
#define HEATER_DUTY_STEP       10U
#define HEATER_MAX_DUTY        100U
#define HEATER_KP_PERCENT_PER_C 30.0f

#endif /* TTJW_CONFIG_H */
