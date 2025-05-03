#include "Sensors/encoders.h"

volatile Encoder motor1_encoder;
volatile Encoder motor2_encoder;

void encodersUpdate(void)
{
    if (motor1_encoder.prevTime <= HAL_GetTick() || motor2_encoder.prevTime <= HAL_GetTick())
        return;

    motor1_encoder.prevRead = motor1_encoder.currRead;
    motor2_encoder.prevRead = motor2_encoder.currRead;

    motor1_encoder.currRead = __HAL_TIM_GET_COUNTER(&htim3);
    motor1_encoder.deltaTime = HAL_GetTick() - motor1_encoder.prevTime;
    motor1_encoder.prevTime = HAL_GetTick();
    motor2_encoder.currRead = __HAL_TIM_GET_COUNTER(&htim4);
    motor2_encoder.deltaTime = HAL_GetTick() - motor2_encoder.prevTime;
    motor2_encoder.prevTime = HAL_GetTick();

    regInput[regInpIx(REG_INPUT_ROT_POSITION)] = motor1_encoder.currRead * (uint32_t)360 / htim3.Instance->CCR3;  //!< lower reload value
    regInput[regInpIx(REG_INPUT_ELEV_POSITION)] = motor2_encoder.currRead * (uint32_t)360 / htim4.Instance->CCR3; //!< lower reload value

    motor1_encoder.deltaRead = motor1_encoder.currRead - motor1_encoder.prevRead;
    motor2_encoder.deltaRead = motor2_encoder.currRead - motor2_encoder.prevRead;

    motor1_encoder.speed = (motor1_encoder.deltaRead * (uint32_t)1000) / motor1_encoder.deltaTime;
    motor2_encoder.speed = (motor2_encoder.deltaRead * (uint32_t)1000) / motor2_encoder.deltaTime;

    regInput[regInpIx(REG_INPUT_ROT_SPEED)] = motor1_encoder.speed * (uint32_t)360 / htim3.Instance->CCR3;  //!< lower reload value
    regInput[regInpIx(REG_INPUT_ELEV_SPEED)] = motor2_encoder.speed * (uint32_t)360 / htim4.Instance->CCR3; //!< lower reload value
}