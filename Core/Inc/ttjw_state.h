/* Shared application state. Prefer module APIs over adding new shared variables. */
#ifndef TTJW_STATE_H
#define TTJW_STATE_H

#include <stdint.h>

typedef enum
{
    DISPLAY_MODE_OPERATION = 0,
    DISPLAY_MODE_ALARM1,
    DISPLAY_MODE_ADJUSTMENT_MENU,
    DISPLAY_MODE_ERROR_ADJUSTMENT,
    DISPLAY_MODE_PID_P,
    DISPLAY_MODE_PID_I,
    DISPLAY_MODE_PID_D
} DisplayMode_t;

/* Operator-adjustable values and current status shared between modules. */
extern float g_setValue;             /* SV, degrees C; default 37.0 C. */
extern float g_alarm1Value;          /* AL-1 threshold; 0.0 disables alarm. */
extern uint8_t g_alarmActive;        /* Steady alarm state derived from valid PV. */
extern uint8_t g_heaterDutyPercent;  /* Last commanded, quantized heater duty. */
extern float g_errorAdjustment;      /* Signed PV calibration offset. */
extern float g_pidPValue;
extern float g_pidIValue;
extern float g_pidDValue;
extern DisplayMode_t g_displayMode;
extern volatile uint8_t g_displayNeedsRefresh;
extern float g_latestTemperature;    /* Most recently computed PV in degrees C. */

#endif /* TTJW_STATE_H */
