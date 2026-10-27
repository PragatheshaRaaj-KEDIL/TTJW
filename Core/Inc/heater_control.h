#ifndef HEATER_CONTROL_H
#define HEATER_CONTROL_H
#include <stdint.h>
void Heater_Init(void);
void Heater_SetDuty(uint8_t dutyPercent);
uint8_t Heater_CalculateDuty(float temperature);
void PID_Reset(void);
#endif
