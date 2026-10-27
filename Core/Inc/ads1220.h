#ifndef ADS1220_H
#define ADS1220_H
#include <stdint.h>
void ADS1220_Reset(void);
void ADS1220_WriteRegisters(void);
void ADS1220_StartConversion(void);
int32_t ADS1220_ReadConversion(void);
#endif
