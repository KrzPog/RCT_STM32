#ifndef _ENCODERS_H
#define _ENCODERS_H

#include "main.h"
#include "tim.h"
#include "ModbusRegisters/reg_input.h"

typedef struct
{
    uint32_t prevTime; //!< last update time (ms)
    int16_t prevRead;  //!< last reading (directly from timer)
    int16_t currRead;  //!< current reading (directly from timer)
    int16_t deltaRead; //!< difference between last and current reading (directly from timer)
    int16_t deltaTime; //!< difference between last and current time (ms)
    int16_t speed;     //!< difference between last and current reading (directly from timer) divided by delta time
} Encoder;

extern volatile Encoder motor1_encoder;
extern volatile Encoder motor2_encoder;

void encodersUpdate(void);

#endif