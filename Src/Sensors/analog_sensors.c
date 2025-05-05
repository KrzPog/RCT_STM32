#include "Sensors/analog_sensors.h"
#include "ModbusRegisters/reg_input.h"

uint16_t adc_values[4] = {0}; // Tablica na wartości z 4 kanałów

/**
 * @brief Rozpoczyna konwersję ADC z wykorzystaniem DMA
 */
void Start_ADC_Conversion(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);
}

/**
 * @brief Aktualizuje rejestry wejściowe Modbus wartościami odczytanymi z czujników analogowych
 * 
 * Ta funkcja powinna być wywoływana cyklicznie, aby zapewnić aktualne wartości w rejestrach.
 */
void analogSensorsUpdate(void)
{
  // Upewnij się, że konwersja ADC jest zakończona
  if (HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_BUSY_REG && HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_BUSY)
  {
    // Bezpośrednie przekazanie surowych wartości ADC do rejestrów
    regInput[regInpIx(REG_INPUT_MAIN_VOLTAGE)] = adc_values[0];
    regInput[regInpIx(REG_INPUT_MAIN_CURRENT)] = adc_values[1];
    regInput[regInpIx(REG_INPUT_ROT_CURRENT)] = adc_values[2];
    regInput[regInpIx(REG_INPUT_ELEV_CURRENT)] = adc_values[3];
    
    // Rozpocznij kolejną konwersję
    Start_ADC_Conversion();
  }
}