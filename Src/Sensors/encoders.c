#include "Sensors/encoders.h"

Encoder rot_motor_encoder;
Encoder elev_motor_encoder;

void encodersInit(void)
{
    rot_motor_encoder.pTim = &htim3;
    elev_motor_encoder.pTim = &htim4;

    rot_motor_encoder.pPosReg = (int16_t *)&regInput[regInpIx(REG_INPUT_ROT_POSITION)];
    rot_motor_encoder.pSpeedReg = (int16_t *)&regInput[regInpIx(REG_INPUT_ROT_SPEED)];

    elev_motor_encoder.pPosReg = (int16_t *)&regInput[regInpIx(REG_INPUT_ELEV_POSITION)];
    elev_motor_encoder.pSpeedReg = (int16_t *)&regInput[regInpIx(REG_INPUT_ELEV_SPEED)];

    rot_motor_encoder.prevTime = HAL_GetTick();
    elev_motor_encoder.prevTime = HAL_GetTick();

    rot_motor_encoder.Position.Raw.newVal = __HAL_TIM_GET_COUNTER(rot_motor_encoder.pTim);
    elev_motor_encoder.Position.Raw.newVal = __HAL_TIM_GET_COUNTER(elev_motor_encoder.pTim);

    rot_motor_encoder.Speed.MovingAverage.bufferPtr = rot_motor_encoder.Speed.MovingAverage.buffer;
    elev_motor_encoder.Speed.MovingAverage.bufferPtr = elev_motor_encoder.Speed.MovingAverage.buffer;

    memset(rot_motor_encoder.Speed.MovingAverage.buffer, 0, ENCODER_MOVING_AVERAGE_COUNT * sizeof(int16_t));
    memset(elev_motor_encoder.Speed.MovingAverage.buffer, 0, ENCODER_MOVING_AVERAGE_COUNT * sizeof(int16_t));

    HAL_TIM_RegisterCallback(rot_motor_encoder.pTim, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, USR_TIM_ROT_MOTOR_OC_DelayElapsedCallback);
    HAL_TIM_OC_Start_IT(rot_motor_encoder.pTim, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(rot_motor_encoder.pTim, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start_IT(rot_motor_encoder.pTim, TIM_CHANNEL_ALL);

    HAL_TIM_RegisterCallback(elev_motor_encoder.pTim, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, USR_TIM_ELEV_MOTOR_OC_DelayElapsedCallback);
    HAL_TIM_OC_Start_IT(elev_motor_encoder.pTim, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(elev_motor_encoder.pTim, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start_IT(elev_motor_encoder.pTim, TIM_CHANNEL_ALL);
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
    pEncoder->Position.Raw.newVal = (int16_t)__HAL_TIM_GET_COUNTER(pEncoder->pTim);
    if (pEncoder->Position.Raw.newVal >= (int16_t)pEncoder->pTim->Instance->CCR3)
        pEncoder->Position.Raw.newVal -= (int16_t)pEncoder->pTim->Instance->CCR3;
    else if (pEncoder->Position.Raw.newVal <= (-1 * (int16_t)pEncoder->pTim->Instance->CCR3))
        pEncoder->Position.Raw.newVal += (int16_t)pEncoder->pTim->Instance->CCR3;
    pEncoder->Position.Rebased.newVal = (int16_t)(pEncoder->Position.Raw.newVal * INT32_C(3600) / (int16_t)pEncoder->pTim->Instance->CCR3); //!< lower reload value

    if (pEncoder->Position.Rebased.newVal != *(pEncoder->pPosReg))
        valueChanged = true;
    else
        pEncoder->Speed.deltaTime += ENCODER_SAMPLING_TIME_MS;
    *(pEncoder->pPosReg) = pEncoder->Position.Rebased.newVal;

    pEncoder->Speed.raw = (int16_t)((pEncoder->Position.Raw.newVal - pEncoder->Position.Raw.oldVal) * INT32_C(1000) / (int32_t)pEncoder->Speed.deltaTime);
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
    pEncoder->Speed.Filtered.rebased = (int16_t)(pEncoder->Speed.Filtered.raw * INT32_C(3600) / (int16_t)pEncoder->pTim->Instance->CCR3); //!< lower reload value
    if (pEncoder->Speed.Filtered.rebased != *(pEncoder->pSpeedReg))
        valueChanged = true;
    *(pEncoder->pSpeedReg) = pEncoder->Speed.Filtered.rebased;
    pEncoder->prevTime = HAL_GetTick();
    return valueChanged;
}

void USR_TIM_ROT_MOTOR_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != rot_motor_encoder.pTim)
        return;

    switch (HAL_TIM_GetActiveChannel(htim))
    {
    case HAL_TIM_ACTIVE_CHANNEL_3:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) - htim->Instance->CCR3);
        rot_motor_encoder.Position.Raw.newVal -= (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) + htim->Instance->CCR3);
        rot_motor_encoder.Position.Raw.newVal += (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    default:
        break;
    }
}

void USR_TIM_ELEV_MOTOR_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != elev_motor_encoder.pTim)
        return;

    switch (HAL_TIM_GetActiveChannel(htim))
    {
    case HAL_TIM_ACTIVE_CHANNEL_3:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) - htim->Instance->CCR3);
        elev_motor_encoder.Position.Raw.newVal -= (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
        __HAL_TIM_SET_COUNTER(htim, __HAL_TIM_GET_COUNTER(htim) + htim->Instance->CCR3);
        elev_motor_encoder.Position.Raw.newVal += (int16_t)htim->Instance->CCR3; //! For better speed calculation
        break;
    default:
        break;
    }
}