#include "ttjw_state.h"

/* Initial setpoint requested for the current TTJW firmware. */
float g_setValue = 37.0f;
float g_alarm1Value = 0.0f;
uint8_t g_alarmActive = 0U;
uint8_t g_heaterDutyPercent = 0U;
float g_errorAdjustment = 0.0f;

/* P retains the existing proportional gain; I and D remain operator-tunable. */
float g_pidPValue = 30.0f;
float g_pidIValue = 0.0f;
float g_pidDValue = 0.0f;

DisplayMode_t g_displayMode = DISPLAY_MODE_OPERATION;
volatile uint8_t g_displayNeedsRefresh = 1U;
float g_latestTemperature = 0.0f;
