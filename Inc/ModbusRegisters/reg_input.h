#ifndef _REG_INPUT_H
#define _REG_INPUT_H

#define REG_INPUT_START 0x3000
#define REG_INPUT_COUNT 17

////////////////////////////////////////////////////////////////////////////////

//! COMM_MODE is defined in main.h
#define REG_INPUT_COMM_MODE 0x3000

#define REG_INPUT_STATUS_WORD 0x3002
#define REG_INPUT_STATUS_WORD_BIT_ACTIVATED 0x0001      //!< Bit 0: 1 - Turret Activated, 0 - Turret Deactivated
#define REG_INPUT_STATUS_WORD_BIT_FAULT 0x0002          //!< Bit 1: 1 - Fault detected, 0 - No Fault
#define REG_INPUT_STATUS_WORD_BIT_IN_MOTION 0x0004      //!< Bit 2: 1 - Turret in motion, 0 - Turret stopped
#define REG_INPUT_STATUS_WORD_BIT_RELOADING 0x0008      //!< Bit 3: 1 - Turret reloading, 0 - Turret not reloading
#define REG_INPUT_STATUS_WORD_BIT_SHOOTING 0x0010       //!< Bit 4: 1 - Turret shooting, 0 - Turret not shooting
#define REG_INPUT_STATUS_WORD_BIT_FLASH_UNLOCKED 0x0020 //!< Bit 5: 1 - Flash access, 0 - No flash access

#define REG_INPUT_ERROR 0x3004
#define REG_INPUT_ERROR_BIT_COMM_TIMEOUT 0x0001 //!< Bit 0: 1 - No Modbus Frame detected within MODBUS_COMM_TIMEOUT_ERROR_MS

//! int16, -3600 to 3600 ( -360.0 to 360.0 degrees with base 10 )
#define REG_INPUT_ROT_POSITION 0x3006

//! int16, -1800 to 1800 ( -180.0 to 180.0 degrees with base 10 )
#define REG_INPUT_ELEV_POSITION 0x3008

//! int16, ( degrees per second with base 10, positive is clockwise, negative is counterclockwise )
#define REG_INPUT_ROT_SPEED 0x300A

//! int16, ( degrees per second with base 10, positive is up, negative is down )
#define REG_INPUT_ELEV_SPEED 0x300C

//! uint16, 0 to 0xFFFF ( voltage, linear 0 to 33V )
#define REG_INPUT_BATTERY_VOLTAGE 0x300E

//! uint16, 0 to 0xFFFF ( current, linear 0 to 5A )
#define REG_INPUT_TRIGGER_CURRENT 0x3010

//! uint16, 0 to 0xFFFF ( current, linear 0 to 5A )
#define REG_INPUT_RELOAD_CURRENT 0x3012

//! uint16, 0 to 0xFFFF ( current, linear 0 to 5A )
#define REG_INPUT_LAMP_CURRENT 0x3014

//! uint16_t, ±2g (LSB/g: 16 384)
// raw = regInput[regInpIx(REG_INPUT_ACCEL_X)]
// acc_g = raw / 16384.0f;       -> w jednostkach g
// acc_ms2 = acc_g * 9.81f;      -> w m/s²
#define REG_INPUT_ACCEL_X 0x3016
#define REG_INPUT_ACCEL_Y 0x3018
#define REG_INPUT_ACCEL_Z 0x301A

//! uint16_t, ±250°/s (LSB/°/s: 131)
// raw = regInput[regInpIx(REG_INPUT_GYRO_X)];
// gyro_dps = raw / 131.0f;      -> w stopniach na sekundę
#define REG_INPUT_GYRO_X 0x301C
#define REG_INPUT_GYRO_Y 0x301E
#define REG_INPUT_GYRO_Z 0x3020

////////////////////////////////////////////////////////////////////////////////

#define regInpIx(x) ((x - REG_INPUT_START) / 2)

////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

#include "mb.h"
#include "main.h"
#include "App/turretStates.h"

extern uint16_t regInput[REG_INPUT_COUNT];

void initRegInput(void);

#endif
