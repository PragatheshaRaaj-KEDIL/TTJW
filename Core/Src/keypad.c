/* Matrix keypad scanning, debounce, hold-repeat and screen navigation. */
#include "main.h"
#include "ttjw_config.h"
#include "ttjw_state.h"
#include "keypad.h"

static uint32_t g_keypadNextScanTick = 0U;
static uint8_t g_keypadRawKey = 0U;
static uint8_t g_keypadStableKey = 0U;
static uint32_t g_keypadRawChangeTick = 0U;
static uint32_t g_keypadPressStartTick = 0U;
static uint32_t g_keypadNextRepeatTick = 0U;

static uint8_t Keypad_Scan(void);
static void Keypad_HandleKey(uint8_t key);
static void Keypad_SetColumnsHigh(void);
/* Put both keypad columns in their inactive HIGH state before scanning. */
void Keypad_Init(void)
{
    Keypad_SetColumnsHigh();
}

static void Keypad_SelectColumn(uint8_t column);

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

static uint8_t Keypad_Scan(void)
{
    GPIO_PinState r1;
    GPIO_PinState r2;
    uint8_t key = 0U;

    /* Scan column L1: S1 / S3. */
    Keypad_SetColumnsHigh();
    Keypad_SelectColumn(1U);
    HAL_Delay(1U);

    r1 = HAL_GPIO_ReadPin(KEY_R1_GPIO_Port, KEY_R1_Pin);
    r2 = HAL_GPIO_ReadPin(KEY_R2_GPIO_Port, KEY_R2_Pin);

    if (r1 == GPIO_PIN_RESET)
    {
        key = KEY_S1;
    }
    else if (r2 == GPIO_PIN_RESET)
    {
        key = KEY_S3;
    }

    /* Scan column L2 only if no key was found in L1. */
    if (key == 0U)
    {
        Keypad_SetColumnsHigh();
        Keypad_SelectColumn(2U);
        HAL_Delay(1U);

        r1 = HAL_GPIO_ReadPin(KEY_R1_GPIO_Port, KEY_R1_Pin);
        r2 = HAL_GPIO_ReadPin(KEY_R2_GPIO_Port, KEY_R2_Pin);

        if (r1 == GPIO_PIN_RESET)
        {
            key = KEY_S2;
        }
        else if (r2 == GPIO_PIN_RESET)
        {
            key = KEY_S4;
        }
    }

    Keypad_SetColumnsHigh();
    return key;
}

void Keypad_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t key;

    if ((int32_t)(now - g_keypadNextScanTick) < 0)
    {
        return;
    }

    g_keypadNextScanTick = now + KEYPAD_SCAN_PERIOD_MS;
    key = Keypad_Scan();

    if (key != g_keypadRawKey)
    {
        g_keypadRawKey = key;
        g_keypadRawChangeTick = HAL_GetTick();
        return;
    }

    now = HAL_GetTick();

    if ((g_keypadStableKey != g_keypadRawKey) &&
        ((uint32_t)(now - g_keypadRawChangeTick) >= KEYPAD_DEBOUNCE_MS))
    {
        g_keypadStableKey = g_keypadRawKey;

        if (g_keypadStableKey != 0U)
        {
            /* Execute once on the debounced press. */
            Keypad_HandleKey(g_keypadStableKey);
            g_keypadPressStartTick = now;
            g_keypadNextRepeatTick = now + KEYPAD_HOLD_INITIAL_MS;
        }
    }

    /*
     * Repeat only the value adjustment buttons S1 and S3.
     * S2 changes screens once per press and must not toggle repeatedly
     * while it is held.
     */
    if ((g_keypadStableKey == KEY_S1 || g_keypadStableKey == KEY_S3) &&
        (g_keypadStableKey == g_keypadRawKey) &&
        ((int32_t)(now - g_keypadNextRepeatTick) >= 0))
    {
        Keypad_HandleKey(g_keypadStableKey);
        g_keypadNextRepeatTick = now + KEYPAD_HOLD_REPEAT_MS;
    }
}

static void Keypad_HandleKey(uint8_t key)
{
    /* S1/S2/S3/S4 actions should be visible without waiting for ADC DRDY. */
    g_displayNeedsRefresh = 1U;
    if (key == KEY_S2)
    {
        if (g_displayMode == DISPLAY_MODE_OPERATION)
            g_displayMode = DISPLAY_MODE_ALARM1;
        else if (g_displayMode == DISPLAY_MODE_ALARM1)
            g_displayMode = DISPLAY_MODE_OPERATION;
        return;
    }

    if (key == KEY_S4)
    {
        switch (g_displayMode)
        {
            case DISPLAY_MODE_OPERATION:
                g_displayMode = DISPLAY_MODE_ADJUSTMENT_MENU;
                break;
            case DISPLAY_MODE_ADJUSTMENT_MENU:
                g_displayMode = DISPLAY_MODE_ERROR_ADJUSTMENT;
                break;
            case DISPLAY_MODE_ERROR_ADJUSTMENT:
                g_displayMode = DISPLAY_MODE_PID_P;
                break;
            case DISPLAY_MODE_PID_P:
                g_displayMode = DISPLAY_MODE_PID_I;
                break;
            case DISPLAY_MODE_PID_I:
                g_displayMode = DISPLAY_MODE_PID_D;
                break;
            case DISPLAY_MODE_PID_D:
                g_displayMode = DISPLAY_MODE_OPERATION;
                break;
            case DISPLAY_MODE_ALARM1:
            default:
                g_displayMode = DISPLAY_MODE_ADJUSTMENT_MENU;
                break;
        }
        return;
    }

    if (g_displayMode == DISPLAY_MODE_OPERATION)
    {
        if (key == KEY_S1) {
            g_setValue += SV_STEP;
            if (g_setValue > SV_MAX_VALUE) g_setValue = SV_MAX_VALUE;
        } else if (key == KEY_S3) {
            g_setValue -= SV_STEP;
            if (g_setValue < SV_MIN_VALUE) g_setValue = SV_MIN_VALUE;
        }
    }
    else if (g_displayMode == DISPLAY_MODE_ALARM1)
    {
        if (key == KEY_S1) {
            g_alarm1Value += AL1_STEP;
            if (g_alarm1Value > AL1_MAX_VALUE) g_alarm1Value = AL1_MAX_VALUE;
        } else if (key == KEY_S3) {
            g_alarm1Value -= AL1_STEP;
            if (g_alarm1Value < AL1_MIN_VALUE) g_alarm1Value = AL1_MIN_VALUE;
        }
    }
    else if (g_displayMode == DISPLAY_MODE_ERROR_ADJUSTMENT)
    {
        if (key == KEY_S1) {
            g_errorAdjustment += ERROR_STEP;
            if (g_errorAdjustment > ERROR_MAX_VALUE) g_errorAdjustment = ERROR_MAX_VALUE;
        } else if (key == KEY_S3) {
            g_errorAdjustment -= ERROR_STEP;
            if (g_errorAdjustment < ERROR_MIN_VALUE) g_errorAdjustment = ERROR_MIN_VALUE;
        }
    }
    else if (g_displayMode == DISPLAY_MODE_PID_P)
    {
        if (key == KEY_S1 && g_pidPValue < (float)PID_VALUE_MAX) g_pidPValue += 1.0f;
        else if (key == KEY_S3 && g_pidPValue > (float)PID_VALUE_MIN) g_pidPValue -= 1.0f;
    }
    else if (g_displayMode == DISPLAY_MODE_PID_I)
    {
        if (key == KEY_S1 && g_pidIValue < (float)PID_VALUE_MAX) g_pidIValue += 1.0f;
        else if (key == KEY_S3 && g_pidIValue > (float)PID_VALUE_MIN) g_pidIValue -= 1.0f;
    }
    else if (g_displayMode == DISPLAY_MODE_PID_D)
    {
        if (key == KEY_S1 && g_pidDValue < (float)PID_VALUE_MAX) g_pidDValue += 1.0f;
        else if (key == KEY_S3 && g_pidDValue > (float)PID_VALUE_MIN) g_pidDValue -= 1.0f;
    }
}
