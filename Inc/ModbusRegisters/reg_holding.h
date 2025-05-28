#ifndef _REG_HOLDING_H
#define _REG_HOLDING_H

#define REG_HOLDING_START 0x4000
#define REG_HOLDING_COUNT 1

#define REG_FLASH_START 0x5000
#define REG_FLASH_COUNT 17

////////////////////////////////////////////////////////////////////////////////

#define REG_HOLDING_CONTROL_WORD 0x4000

#define REG_HOLDING_CONTROL_WORD_BIT_ACTIVATE 0x0001     //!< Bit 0: Activate turret ( set back to 0 after activation )
#define REG_HOLDING_CONTROL_WORD_BIT_FAULT_RESET 0x0002  //!< Bit 1: Reset fault ( set back to 0 after reset )
#define REG_HOLDING_CONTROL_WORD_BIT_RELOAD 0x0004       //!< Bit 2: Reload ( impulse start reloading or stop depending on current state )
#define REG_HOLDING_CONTROL_WORD_BIT_SHOOT 0x0008        //!< Bit 3: Shoot ( impulse start shooting or stop depending on current state )
#define REG_HOLDING_CONTROL_WORD_BIT_FLASH_UNLOCK 0x0010 //!< Bit 4: Unlock flash ( set back to 0 after unlocking )

//! int16, in degrees with base 10, limited by REG_FLASH_ROT_POSITION_MIN and REG_FLASH_ROT_POSITION_MAX
#define REG_HOLDING_ROT_TARGET_POSITION 0x4002

//! int16, in degrees with base 10, limited by REG_FLASH_ELEV_POSITION_MIN and REG_FLASH_ELEV_POSITION_MAX
#define REG_HOLDING_ELEV_TARGET_POSITION 0x4004

//! int16, in degrees per second with base 10, limited by REG_FLASH_ROT_SPEED_MIN and REG_FLASH_ROT_SPEED_MAX
//! ( both signs supported )
#define REG_HOLDING_ROT_TARGET_SPEED 0x4006

//! int16, in degrees per second with base 10, limited by REG_FLASH_ELEV_SPEED_MIN and REG_FLASH_ELEV_SPEED_MAX
//! ( both signs supported )
#define REG_HOLDING_ELEV_TARGET_SPEED 0x4008

////////////////////////////////////////////////////////////////////////////////

#define REG_FLASH_CONTROL_MODE 0x5000
#define REG_FLASH_CONTROL_MODE_BIT_PID_EN 0x0001       //!< Bit 0: 1 - Open loop control, 0 - Closed loop control
#define REG_FLASH_CONTROL_MODE_BIT_PID_TYPE 0x0002     //!< Bit 1: 1 - Position control, 0 - Speed control
#define REG_FLASH_CONTROL_MODE_BIT_ROT_CONTROL 0x0004  //!< Bit 2: 1 - UART control, 0 - PWM control
#define REG_FLASH_CONTROL_MODE_BIT_ELEV_CONTROL 0x0008 //!< Bit 3: 1 - UART control, 0 - PWM control

//! int16, in degrees with base 10, min -3600, max 3600
#define REG_FLASH_ROT_POSITION_MIN 0x5002

//! int16, in degrees with base 10, min -3600, max 3600
#define REG_FLASH_ROT_POSITION_MAX 0x5004

//! int16, in degrees with base 10, min -3600, max 3600
#define REG_FLASH_ELEV_POSITION_MIN 0x5006

//! int16, in degrees with base 10, min -3600, max 3600
#define REG_FLASH_ELEV_POSITION_MAX 0x5008

//! uint16, in degrees per second with base 10, min 0
#define REG_FLASH_ROT_SPEED_MIN 0x500A

//! uint16, in degrees per second with base 10, min 0
#define REG_FLASH_ROT_SPEED_MAX 0x500C

//! uint16, in degrees per second with base 10, min 0
#define REG_FLASH_ELEV_SPEED_MIN 0x500E

//! uint16, in degrees per second with base 10, min 0
#define REG_FLASH_ELEV_SPEED_MAX 0x5010

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ROT_DUTY_PWM_MIN 0x5012

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ROT_DUTY_PWM_MAX 0x5014

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ELEV_DUTY_PWM_MIN 0x5016

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ELEV_DUTY_PWM_MAX 0x5018

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ROT_UART_SPEED_MIN 0x501A

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ROT_UART_SPEED_MAX 0x501C

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ELEV_UART_SPEED_MIN 0x501E

//! uint16, in percent with base 10, min 0, max 1000
#define REG_FLASH_ELEV_UART_SPEED_MAX 0x5020

////////////////////////////////////////////////////////////////////////////////

#define regHoldIx(x) ((x - REG_HOLDING_START) / 2)

#define regFlashIx(x) ((x - REG_FLASH_START) / 2)

////////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdint.h>

#include "mb.h"
#include "main.h"
#include "App/turretStates.h"
#include "ModbusRegisters/reg_input.h"

extern uint16_t regHolding[REG_HOLDING_COUNT];
extern uint16_t regFlash[REG_FLASH_COUNT];

void initRegHolding(void);
void initRegFlash(void);

#endif