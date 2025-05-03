#include "Sensors/encoders.h"

Encoder motor1_encoder;
Encoder motor2_encoder;

void encodersInit(void)
{
    motor1_encoder.pTim = &htim3;
    motor2_encoder.pTim = &htim4;

    motor1_encoder.pPosReg = &regInput[regInpIx(REG_INPUT_ROT_POSITION)];
    motor1_encoder.pSpeedReg = &regInput[regInpIx(REG_INPUT_ROT_SPEED)];

    motor2_encoder.pPosReg = &regInput[regInpIx(REG_INPUT_ELEV_POSITION)];
    motor2_encoder.pSpeedReg = &regInput[regInpIx(REG_INPUT_ELEV_SPEED)];

    motor1_encoder.prevTime = HAL_GetTick();
    motor2_encoder.prevTime = HAL_GetTick();

    motor1_encoder.Position.Raw.newVal = __HAL_TIM_GET_COUNTER(motor1_encoder.pTim);
    motor2_encoder.Position.Raw.newVal = __HAL_TIM_GET_COUNTER(motor2_encoder.pTim);

    motor1_encoder.Speed.MovingAverage.bufferPtr = motor1_encoder.Speed.MovingAverage.buffer;
    motor2_encoder.Speed.MovingAverage.bufferPtr = motor2_encoder.Speed.MovingAverage.buffer;

    memset(motor1_encoder.Speed.MovingAverage.buffer, 0, ENCODER_MOVING_AVERAGE_COUNT * sizeof(int16_t));
    memset(motor2_encoder.Speed.MovingAverage.buffer, 0, ENCODER_MOVING_AVERAGE_COUNT * sizeof(int16_t));

    HAL_TIM_RegisterCallback(motor1_encoder.pTim, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, USR_TIM_MOTOR1_OC_DelayElapsedCallback);
    HAL_TIM_OC_Start_IT(motor1_encoder.pTim, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(motor1_encoder.pTim, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start_IT(motor1_encoder.pTim, TIM_CHANNEL_ALL);

    HAL_TIM_RegisterCallback(motor2_encoder.pTim, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, USR_TIM_MOTOR2_OC_DelayElapsedCallback);
    HAL_TIM_OC_Start_IT(motor2_encoder.pTim, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(motor2_encoder.pTim, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start_IT(motor2_encoder.pTim, TIM_CHANNEL_ALL);
}

bool encoderUpdate(Encoder *pEncoder)
{
    if (pEncoder == NULL)
        return 0;

    if (pEncoder->prevTime == HAL_GetTick())
        return 0;

    bool valueChanged = false;

    pEncoder->Position.Raw.oldVal = pEncoder->Position.Raw.newVal;
    pEncoder->Position.Rebased.oldVal = pEncoder->Position.Rebased.newVal;
    pEncoder->Position.Raw.newVal = __HAL_TIM_GET_COUNTER(pEncoder->pTim);
    pEncoder->Position.Rebased.newVal = (int16_t)(((int32_t)pEncoder->Position.Raw.newVal * INT16_C(3600)) / (int16_t)pEncoder->pTim->Instance->CCR3); //!< lower reload value

    if (pEncoder->Position.Rebased.newVal != *(pEncoder->pPosReg))
        valueChanged = true;
    else
        pEncoder->Speed.deltaTime += ENCODER_SAMPLING_TIME_MS;
    *(pEncoder->pPosReg) = (uint16_t)pEncoder->Position.Rebased.newVal;

    pEncoder->Speed.raw = (int16_t)(((int32_t)(pEncoder->Position.Raw.newVal - pEncoder->Position.Raw.oldVal) * INT16_C(1000)) / (int32_t)pEncoder->Speed.deltaTime);
    if (valueChanged)
        pEncoder->Speed.deltaTime = ENCODER_SAMPLING_TIME_MS;

    if (pEncoder->Speed.MovingAverage.bufferPtr == pEncoder->Speed.MovingAverage.buffer + (ENCODER_MOVING_AVERAGE_COUNT - 1))
        pEncoder->Speed.MovingAverage.bufferPtr = pEncoder->Speed.MovingAverage.buffer;
    else
        pEncoder->Speed.MovingAverage.bufferPtr++;

    *(pEncoder->Speed.MovingAverage.bufferPtr) = pEncoder->Speed.raw;

    int32_t sum = 0;
    for (int i = 0; i < ENCODER_MOVING_AVERAGE_COUNT; i++)
        sum += pEncoder->Speed.MovingAverage.buffer[i];

    pEncoder->Speed.Filtered.raw = (int16_t)(sum / (int16_t)ENCODER_MOVING_AVERAGE_COUNT);
    pEncoder->Speed.Filtered.rebased = (int16_t)(((int32_t)pEncoder->Speed.Filtered.raw * INT16_C(3600)) / (int16_t)pEncoder->pTim->Instance->CCR3); //!< lower reload value
    if (pEncoder->Speed.Filtered.rebased != *(pEncoder->pSpeedReg))
        valueChanged = true;
    *(pEncoder->pSpeedReg) = (uint16_t)pEncoder->Speed.Filtered.rebased;
    pEncoder->prevTime = HAL_GetTick();
    return valueChanged;
}

void USR_TIM_MOTOR1_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != motor1_encoder.pTim)
        return;

    switch (HAL_TIM_GetActiveChannel(htim))
    {
    case HAL_TIM_ACTIVE_CHANNEL_3:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) - htim->Instance->CCR3);
        motor1_encoder.Position.Raw.newVal -= (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) + htim->Instance->CCR3);
        motor1_encoder.Position.Raw.newVal += (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    default:
        break;
    }
}

void USR_TIM_MOTOR2_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != motor2_encoder.pTim)
        return;

    switch (HAL_TIM_GetActiveChannel(htim))
    {
    case HAL_TIM_ACTIVE_CHANNEL_3:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) - htim->Instance->CCR3);
        motor2_encoder.Position.Raw.newVal -= (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) + htim->Instance->CCR3);
        motor2_encoder.Position.Raw.newVal += (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    default:
        break;
    }
}