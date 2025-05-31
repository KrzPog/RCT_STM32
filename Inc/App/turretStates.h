#ifndef _TURRET_STATES_H
#define _TURRET_STATES_H

#include "main.h"
#include "gpio.h"
#include "freertos_tasks.h"
#include "ModbusRegisters/reg_input.h"
#include "ModbusRegisters/reg_holding.h"

#define STATE_LED_SWITCHING_PERIOD_MS 100

extern volatile uint32_t lastFrameReceivedTime;

void checkControlWord(void);

void activateTurret(void);
void deactivateTurret(void);

void setTurretError(uint16_t errorCode);
void clearTurretError(void);

void reloadTurretToggle(void);

void shootTurretToggle(void);

void checkIfCommTimeout(void);

void stateLEDUpdate(void);

#endif