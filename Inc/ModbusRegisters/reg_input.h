#ifndef _REG_INPUT_H
#define _REG_INPUT_H

#define REG_INPUT_START 0x3000
#define REG_INPUT_COUNT 17

////////////////////////////////////////////////////////////////////////////////

//! COMM_MODE is defined in main.h
#define REG_INPUT_COMM_MODE 0x3000

#define REG_INPUT_STATUS_WORD 0x3002

//! Bit 0: 1 - Turret Activated, 0 - Turret Deactivated
#define REG_INPUT_STATUS_WORD_BIT_ACTIVATED 0x0001

//! Bit 1: 1 - Fault detected, 0 - No Fault
#define REG_INPUT_STATUS_WORD_BIT_FAULT 0x0002

//! Bit 2: 1 - Turret in motion, 0 - Turret stopped
#define REG_INPUT_STATUS_WORD_BIT_IN_MOTION 0x0004

//! Bit 3: 1 - Turret reloading, 0 - Turret not reloading
#define REG_INPUT_STATUS_WORD_BIT_RELOADING 0x0008

//! Bit 4: 1 - Turret shooting, 0 - Turret not shooting
#define REG_INPUT_STATUS_WORD_BIT_SHOOTING 0x0010

#define REG_INPUT_ERROR 0x3004

//! Bits 0-1: 0b00 - No error, 0b01 - Battery undervoltage, 0b10 - Battery overvoltage
#define REG_INPUT_ERROR_BIT_BATTER_UNDERVOLTAGE 0x0001
#define REG_INPUT_ERROR_BIT_BATTER_OVERVOLTAGE 0x0002

//! Bit 2: 1 - Rotation motor disconnected, 0 - Rotation motor connected
#define REG_INPUT_ERROR_BIT_ROT_DISCONNECTED 0x0004

//! Bit 3: 1 - Rotation motor overcurrent, 0 - Rotation motor normal
#define REG_INPUT_ERROR_BIT_ROT_OVERCURRENT 0x0008

//! Bit 4: 1 - Elevation motor disconnected, 0 - Elevation motor connected
#define REG_INPUT_ERROR_BIT_ELEV_DISCONNECTED 0x0010

//! Bit 5: 1 - Elevation motor overcurrent, 0 - Elevation motor normal
#define REG_INPUT_ERROR_BIT_ELEV_OVERCURRENT 0x0020

//! Bit 6: 1 - No Modbus Frame detected within MODBUS_COMM_TIMEOUT_ERROR_MS
#define REG_INPUT_ERROR_BIT_COMM_TIMEOUT 0x0040

//! int16, -3600 to 3600 ( -360.0 to 360.0 degrees with base 10 )
#define REG_INPUT_ROT_POSITION 0x3006

//! int16, -1800 to 1800 ( -180.0 to 180.0 degrees with base 10 )
#define REG_INPUT_ELEV_POSITION 0x3008

//! int16, ( degrees per second with base 10, positive is clockwise, negative is counterclockwise )
#define REG_INPUT_ROT_SPEED 0x300A

//! int16, ( degrees per second with base 10, positive is up, negative is down )
#define REG_INPUT_ELEV_SPEED 0x300C

//! uint16, 0 to 0xFFFF ( voltage, linear 0 to 25V )
#define REG_INPUT_MAIN_VOLTAGE 0x300E

//! uint16, 0 to 0xFFFF ( current, linear 0 to 50A )
#define REG_INPUT_MAIN_CURRENT 0x3010

//! uint16, 0 to 0xFFFF ( current, linear 0 to 30A )
#define REG_INPUT_ROT_CURRENT 0x3012

//! uint16, 0 to 0xFFFF ( current, linear 0 to 20A )
#define REG_INPUT_ELEV_CURRENT 0x3014

//! uint16_t, ±2g (LSB/g: 16 384)
#define REG_ACCEL_X 0x3016
#define REG_ACCEL_Y 0x3018
#define REG_ACCEL_Z 0x301A

//! uint16_t, ±250°/s (LSB/°/s: 131)
#define REG_GYRO_X 0x301C
#define REG_GYRO_Y 0x301E
#define REG_GYRO_Z 0x3020

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
