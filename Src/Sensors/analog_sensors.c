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
 * @brief Konwertuje wartość ADC na prąd w miliamperach
 * @param adc_value Surowa wartość z ADC (0-4095)
 * @return Prąd w miliamperach (ze znakiem, zakres zależy od czujnika)
 * 
 *  Odejmowanie offsetu PRZED konwersją
 * - Odejmuje offset 3102 (odpowiednik -5A) od surowej wartości ADC
 * - Następnie konwertuje na miliampery
 */
int16_t convertCurrentToMilliamps(uint16_t adc_value)
{
    // Odejmij offset przed konwersją
    int32_t adc_with_offset = (int32_t)adc_value - CURRENT_OFFSET_BEFORE_CONVERSION;
    
    // Konwertuj na napięcie (V)
    float voltage = ((float)adc_with_offset * ADC_VREF) / ADC_MAX_VALUE;
    
    // Konwertuj na prąd (A) używając czułości czujnika
    // I = (V - V_offset) / Sensitivity
    float current_amps = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
    
    // Konwertuj na miliampery
    int16_t current_ma = (int16_t)(current_amps * 1000.0f);
    
    return current_ma;
}

/**
 * @brief Konwertuje wartość ADC na napięcie w miliowoltach
 * @param adc_value Surowa wartość z ADC (0-4095)
 * @return Napięcie w miliowoltach (bez offsetu)
 */
uint16_t convertVoltageToMillivolts(uint16_t adc_value)
{
    // Konwertuj na napięcie przed dzielnikiem (V)
    float voltage_before_divider = ((float)adc_value * ADC_VREF) / ADC_MAX_VALUE;
    
    // Uwzględnij dzielnik napięcia
    float actual_voltage = voltage_before_divider * VOLTAGE_DIVIDER_RATIO;
    
    // Konwertuj na miliowolty
    uint16_t voltage_mv = (uint16_t)(actual_voltage * 1000.0f);
    
    return voltage_mv;
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
        
        // Konwertuj i przypisz wartości do rejestrów Modbus
        
        // Prądy - konwersja na miliampery (wybierz jedną z metod)
        regInput[regInpIx(REG_INPUT_LAMP_CURRENT)] = convertCurrentToMilliamps(local_adc_values[0]);
        regInput[regInpIx(REG_INPUT_RELOAD_CURRENT)] = convertCurrentToMilliamps(local_adc_values[1]);
        regInput[regInpIx(REG_INPUT_TRIGGER_CURRENT)] = convertCurrentToMilliamps(local_adc_values[2]);
        
        // Napięcie - konwersja na miliowolty
        regInput[regInpIx(REG_INPUT_BATTERY_VOLTAGE)] = convertVoltageToMillivolts(local_adc_values[3]);
        
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