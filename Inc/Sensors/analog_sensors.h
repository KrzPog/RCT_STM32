#ifndef ANALOG_SENSORS_H
#define ANALOG_SENSORS_H

#include "main.h"
#include "adc.h"

// Stałe dla czujników
#define VOLTAGE_DIVIDER_RATIO   7.58f    
#define CURRENT_SENSOR_OFFSET   1.65f  
#define CURRENT_SENSOR_SENSITIVITY 0.066f 

// Deklaracje funkcji
void Start_ADC_Conversion(void);
float Get_Input_Voltage(void);
float Get_Input_Current(void);
float Get_Motor1_Current(void);
float Get_Motor2_Current(void);

/**
 * @brief Aktualizuje rejestry wejściowe Modbus wartościami odczytanymi z czujników analogowych
 */
void analogSensorsUpdate(void);

#endif /* ANALOG_SENSORS_H */