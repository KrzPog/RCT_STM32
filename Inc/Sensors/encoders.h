#ifndef _ENCODERS_H
#define _ENCODERS_H

#define ENCODER_SAMPLING_TIME_MS 10   //!< Encoder update period in ms
#define ENCODER_MOVING_AVERAGE_COUNT 64 //!< Number of samples to average the speed

#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "tim.h"
#include "ModbusRegisters/reg_input.h"

typedef struct
{
    uint32_t prevTime;       //!< last update time (ms)
    uint16_t *pPosReg;       //!< pointer to position register
    uint16_t *pSpeedReg;     //!< pointer to speed register
    TIM_HandleTypeDef *pTim; //!< pointer to timer handle

    struct Position
    {
        struct Raw
        {
            int16_t oldVal;          //!< previous reading (directly from timer)
            volatile int16_t newVal; //!< current reading (directly from timer)
        } Raw;

        struct Rebased
        {
            int16_t oldVal; //!< previous reading (rebased from -3600 to 3600)
            int16_t newVal; //!< current reading (rebased from -3600 to 3600)
        } Rebased;
    } Position;

    struct Speed
    {
        uint32_t deltaTime; //!< base ENCODER_SAMPLING_TIME_MS, increased by ENCODER_SAMPLING_TIME_MS if no change in position

        int16_t raw; //!< speed as (new - last) / deltaTime in seconds [impulses/s]

        struct Filtered
        {
            int16_t raw;     //!< calculated from moving average buffer
            int16_t rebased; //!< calculated from filtered raw speed, rebased from degrees/s
        } Filtered;

        struct MovingAverage
        {
            int16_t *bufferPtr;                           //!< pointer to moving average buffer next write position
            int16_t buffer[ENCODER_MOVING_AVERAGE_COUNT]; //!< moving average buffer
        } MovingAverage;
    } Speed;
} Encoder;

extern Encoder motor1_encoder;
extern Encoder motor2_encoder;

void encodersInit(void);
bool encoderUpdate(Encoder *pEncoder);

void USR_TIM_MOTOR1_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void USR_TIM_MOTOR2_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

#endif