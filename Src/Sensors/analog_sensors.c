#include "Sensors/analog_sensors.h"
#include "ModbusRegisters/reg_input.h"

uint16_t adc_values[4] = {0}; // Tablica na wartości z 4 kanałów
volatile uint8_t adc_conversion_complete = 0; // Flaga informująca o zakończeniu konwersji

/**
 * @brief Rozpoczyna konwersję ADC z wykorzystaniem DMA
 */
void Start_ADC_Conversion(void)
{
  adc_conversion_complete = 0; // Resetuj flagę przed rozpoczęciem nowej konwersji
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);
}

/**
 * @brief Callback wywoływany po zakończeniu konwersji ADC przez DMA
 * @param hadc Wskaźnik na strukturę ADC
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    adc_conversion_complete = 1; // Ustaw flagę po zakończeniu konwersji
  }
}

/**
 * @brief Aktualizuje rejestry wejściowe Modbus wartościami odczytanymi z czujników analogowych
 * 
 * Ta funkcja powinna być wywoływana cyklicznie, aby zapewnić aktualne wartości w rejestrach.
 */
void analogSensorsUpdate(void)
{
  static uint16_t local_adc_values[4] = {0}; // Lokalna kopia wartości ADC
  
  // Sprawdź, czy konwersja jest zakończona
  if (adc_conversion_complete)
  {
    // Wykonaj lokalną kopię wartości ADC, aby uniknąć problemu z wyścigami
    for (int i = 0; i < 4; i++)
    {
      local_adc_values[i] = adc_values[i];
    }
    
    // Przypisz wartości do rejestrów Modbus
    regInput[regInpIx(REG_INPUT_LAMP_CURRENT)] = local_adc_values[0];
    regInput[regInpIx(REG_INPUT_RELOAD_CURRENT)] = local_adc_values[1];
    regInput[regInpIx(REG_INPUT_TRIGGER_CURRENT)] = local_adc_values[2];
    regInput[regInpIx(REG_INPUT_BATTERY_VOLTAGE)] = local_adc_values[3];
    
    // Rozpocznij kolejną konwersję
    Start_ADC_Conversion();
  }
  else if (HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_BUSY_REG && 
           HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_BUSY)
  {
    // Jeśli ADC nie jest zajęty, a flaga nie jest ustawiona, rozpocznij konwersję
    Start_ADC_Conversion();
  }
}