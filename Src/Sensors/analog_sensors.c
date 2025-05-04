#include "Sensors/analog_sensors.h"
#include "ModbusRegisters/reg_input.h"

uint16_t adc_values[4] = {0}; // Tablica na wartości z 4 kanałów

void Start_ADC_Conversion(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);
}

float Get_Input_Voltage(void)
{
  // Konwersja wartości ADC na napięcie
  return (adc_values[0] * 3.3f / 4095.0f) * VOLTAGE_DIVIDER_RATIO;
}

float Get_Input_Current(void)
{
  // Konwersja wartości ADC na prąd
  return ((adc_values[1] * 3.3f / 4095.0f) - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
}

float Get_Motor1_Current(void)
{
  return ((adc_values[2] * 3.3f / 4095.0f) - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
}

float Get_Motor2_Current(void)
{
  return ((adc_values[3] * 3.3f / 4095.0f) - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
}

/**
 * @brief Aktualizuje rejestry wejściowe Modbus wartościami odczytanymi z czujników analogowych
 * 
 * Ta funkcja powinna być wywoływana cyklicznie, aby zapewnić aktualne wartości w rejestrach.
 * Odczytuje napięcie wejściowe oraz prądy i zapisuje je do odpowiednich rejestrów.
 */
void analogSensorsUpdate(void)
{
  // Odczyt wartości z czujników
  float inputVoltage = Get_Input_Voltage();
  float inputCurrent = Get_Input_Current();
  float motor1Current = Get_Motor1_Current();  
  float motor2Current = Get_Motor2_Current(); 
  
  // Konwersja wartości float na uint16_t zgodnie z definicjami z reg_input.h
  // REG_INPUT_MAIN_VOLTAGE: voltage, linear 0 to 25V (zakres 0 do 0xFFFF)
  uint16_t inputVoltageReg = (uint16_t)((inputVoltage / 25.0f) * 65535.0f);
  
  // REG_INPUT_MAIN_CURRENT: current, linear 0 to 50A (zakres 0 do 0xFFFF)
  uint16_t inputCurrentReg = (uint16_t)((inputCurrent / 50.0f) * 65535.0f);
  
  // REG_INPUT_ROT_CURRENT: current, linear 0 to 30A (zakres 0 do 0xFFFF)
  uint16_t rotCurrentReg = (uint16_t)((motor1Current / 30.0f) * 65535.0f);
  
  // REG_INPUT_ELEV_CURRENT: current, linear 0 to 20A (zakres 0 do 0xFFFF)
  uint16_t elevCurrentReg = (uint16_t)((motor2Current / 20.0f) * 65535.0f);
  
  // Zapisanie wartości do rejestrów używając istniejących definicji z reg_input.h
  regInput[regInpIx(REG_INPUT_MAIN_VOLTAGE)] = inputVoltageReg;
  regInput[regInpIx(REG_INPUT_MAIN_CURRENT)] = inputCurrentReg;
  regInput[regInpIx(REG_INPUT_ROT_CURRENT)] = rotCurrentReg;
  regInput[regInpIx(REG_INPUT_ELEV_CURRENT)] = elevCurrentReg;
}