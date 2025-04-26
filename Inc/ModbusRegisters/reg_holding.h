#ifndef _REG_HOLDING_H
#define _REG_HOLDING_H

#define REG_HOLDING_START 0x4000
#define REG_HOLDING_COUNT 2

////////////////////////////////////////////////////////////////////////////////

#define REG_HOLDING_CONTROL_WORD 0x4000

//! Bit 0: Activate turret ( set back to 0 after activation )
#define REG_HOLDING_CONTROL_WORD_BIT_ACTIVATE 0x0001

//! Bit 1: Reset fault ( set back to 0 after reset )
#define REG_HOLDING_CONTROL_WORD_BIT_FAULT_RESET 0x0002

//! Bit 2: Reload ( impulse start reloading or stop depending on current state )
#define REG_HOLDING_CONTROL_WORD_BIT_RELOAD 0x0004

//! Bit 3: Shoot ( impulse start shooting or stop depending on current state )
#define REG_HOLDING_CONTROL_WORD_BIT_SHOOT 0x0008

//! uint16, limit on the max time turret can keep shooting ( in ms, default 0xFFFF )
#define REG_HOLDING_SHOOTING_TIME_LIMIT 0x4002

////////////////////////////////////////////////////////////////////////////////

#define regHoldIx(x) ((x - REG_HOLDING_START) / 2)

////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

#include "mb.h"
#include "main.h"
#include "ModbusRegisters/reg_input.h"

extern uint16_t regHolding[REG_HOLDING_COUNT];

void initRegHolding(void);

void checkControlWord(void);

#endif