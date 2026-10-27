/* Heater PWM and PID control. TIM1 is initialized by the board layer. */
#include "main.h"
#include "ttjw_config.h"
#include "ttjw_state.h"
#include "heater_control.h"

extern TIM_HandleTypeDef htim1;

/* PID history is private to the controller, not global application state. */
static float g_pidIntegral = 0.0f;
static float g_pidPreviousPv = 0.0f;
static uint8_t g_pidHasPreviousPv = 0U;
static uint32_t g_pidPreviousTick = 0U;

void Heater_Init(void)
{
    /* Start with heater demand at 0%. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void Heater_SetDuty(uint8_t dutyPercent)
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

    /* Remember the actual PWM command for the OUT display indicator. */
    g_heaterDutyPercent = dutyPercent;

    /*
     * 0%   -> CCR = 0
     * 10%  -> CCR = 2000 approximately
     * ...
     * 100% -> CCR = 20000, clamped to the timer period.
     */
    compare =
        ((uint32_t)HEATER_PWM_PERIOD * dutyPercent) / 100U;

    /* CCR = ARR + 1 gives a continuous ON output at 100% in PWM mode 1. */
    if (dutyPercent >= 100U)
    {
        compare = HEATER_PWM_PERIOD + 1U;
    }
    else if (compare > HEATER_PWM_PERIOD)
    {
        compare = HEATER_PWM_PERIOD;
    }

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        compare
    );
}

void PID_Reset(void)
{
    g_pidIntegral = 0.0f;
    g_pidPreviousPv = 0.0f;
    g_pidHasPreviousPv = 0U;
    g_pidPreviousTick = HAL_GetTick();
}

uint8_t Heater_CalculateDuty(float temperature)
{
    float error;
    float pTerm;
    float iCandidate;
    float dTerm = 0.0f;
    float demand;
    float dt;
    uint32_t now = HAL_GetTick();

    /* Retain the backend's hard shutoff condition. */
    if (temperature >= g_setValue)
    {
        PID_Reset();
        return 0U;
    }

    if (g_pidPreviousTick == 0U)
    {
        g_pidPreviousTick = now;
    }
    dt = (float)(now - g_pidPreviousTick) / 1000.0f;
    g_pidPreviousTick = now;

    /* Avoid division by zero and cap long scheduling gaps. */
    if (dt < 0.001f) dt = 0.05f;
    if (dt > 0.5f) dt = 0.5f;

    error = g_setValue - temperature;

    /* Preserve the existing P scaling: 30.0 means 30% per degree C. */
    pTerm = g_pidPValue * error;

    /* Derivative on measured PV avoids derivative kick on SV changes. */
    if (g_pidHasPreviousPv != 0U)
    {
        dTerm = -g_pidDValue * (temperature - g_pidPreviousPv) / dt;
    }

    iCandidate = g_pidIntegral + (g_pidIValue * error * dt);
    demand = pTerm + iCandidate + dTerm;

    /* Conditional integration anti-windup: integrate only if unsaturated,
       or when the current error would drive a saturated output inward. */
    if (((demand >= 0.0f) && (demand <= 100.0f)) ||
        ((demand > 100.0f) && (error < 0.0f)) ||
        ((demand < 0.0f) && (error > 0.0f)))
    {
        g_pidIntegral = iCandidate;
    }

    /* Recalculate using the accepted integral state. */
    demand = pTerm + g_pidIntegral + dTerm;
    if (demand <= 0.0f) demand = 0.0f;
    if (demand >= 100.0f) demand = 100.0f;

    g_pidPreviousPv = temperature;
    g_pidHasPreviousPv = 1U;

    return (uint8_t)(demand + 0.5f);
}
