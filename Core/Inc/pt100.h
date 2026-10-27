#ifndef PT100_H
#define PT100_H
#include <stdint.h>
float CalculateResistance(int32_t raw);
float ResistanceToTemperature(float resistance);
#endif
