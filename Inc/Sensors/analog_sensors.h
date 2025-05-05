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

/**
 * @brief Aktualizuje rejestry wejściowe Modbus wartościami odczytanymi z czujników analogowych
 */
void analogSensorsUpdate();

// Deklaracja callback'a dla konwersji ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif /* ANALOG_SENSORS_H */