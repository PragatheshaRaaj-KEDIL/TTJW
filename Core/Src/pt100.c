/* PT100 conversion. The existing resistance equation is intentionally preserved. */
#include "ttjw_config.h"
#include "pt100.h"
#include <math.h>
#include <stdint.h>

float CalculateResistance(int32_t raw)
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

float ResistanceToTemperature(
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
