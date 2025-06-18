#ifndef ANALOG_SENSORS_H
#define ANALOG_SENSORS_H

#include "main.h"
#include "adc.h"

#define ANALOG_SAMPLING_TIME_MS 1   //!< Analog update period in ms

// Stałe dla czujników
#define VOLTAGE_DIVIDER_RATIO   7.58f    
#define CURRENT_SENSOR_OFFSET   1.65f  
#define CURRENT_SENSOR_SENSITIVITY 0.066f 

// Stałe ADC
#define ADC_RESOLUTION_BITS     12
#define ADC_MAX_VALUE          ((1 << ADC_RESOLUTION_BITS) - 1)  // 4095 dla 12-bit
#define ADC_VREF               3.3f  // Napięcie referencyjne w V

// Stałe konwersji prądu
#define CURRENT_OFFSET_BEFORE_CONVERSION  3102  // Offset w jednostkach ADC (-5A)
#define CURRENT_OFFSET_AFTER_CONVERSION   5000  // Offset w mA (-5A)

// Deklaracje funkcji
void Start_ADC_Conversion(void);
void analogSensorsUpdate(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

// Funkcje konwersji
int16_t convertCurrentToMilliamps(uint16_t adc_value);
uint16_t convertVoltageToMillivolts(uint16_t adc_value);

#endif /* ANALOG_SENSORS_H */