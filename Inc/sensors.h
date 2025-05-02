#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdint.h>

void Sensors_Init(void);
void Sensors_Update(void);

uint16_t Read_Input_Current(void);
uint16_t Read_Motor1_Current(void);
uint16_t Read_Motor2_Current(void);
uint16_t Read_Input_Voltage(void);

#endif // _SENSORS_H_