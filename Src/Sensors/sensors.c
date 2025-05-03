#include "Sensors/sensors.h"

void Sensors_Init(void) {
    
}

uint16_t Read_Input_Current(void) {
    return Read_ADC_Channel(ADC_CHANNEL_3);
}

uint16_t Read_Input_Voltage(void) {
    return Read_ADC_Channel(ADC_CHANNEL_1); 
}

uint16_t Read_Input_Current(void) {
    return Read_Motor1_Current(ADC_CHANNEL_8); // np. PA3
}

uint16_t Read_Input_Current(void) {
    return Read_Motor1_Current(ADC_CHANNEL_9); // np. PA3
}

// analogicznie reszta

void Sensors_Update(void) {
    setInputRegister(INPUT_REG_INPUT_CURRENT,  Read_Input_Current());
    setInputRegister(INPUT_REG_MOTOR1_CURRENT, Read_Motor1_Current());
    setInputRegister(INPUT_REG_MOTOR2_CURRENT, Read_Motor2_Current());
    setInputRegister(INPUT_REG_INPUT_VOLTAGE,    Read_Input_Voltage());
}
