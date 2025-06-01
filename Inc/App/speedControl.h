#ifndef _SPEED_CONTROL_H
#define _SPEED_CONTROL_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "App/pid_output.h"
#include "ModbusRegisters/reg_holding.h"

extern bool elevLimitMinReached;
extern bool elevLimitMaxReached;

void initRotSpeedControl(void);
void initElevSpeedControl(void);

int16_t getRotSpeedCV(void);
int16_t getElevSpeedCV(void);

void sendRotSpeedCV(void);
void sendElevSpeedCV(void);

#endif