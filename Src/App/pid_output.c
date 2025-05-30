#include "App/pid_output.h"
#include "main.h"  // For HAL_GetTick()
#include <string.h>

// Definicja globalnej struktury
PIDOutput pidOutputs[MOTOR_COUNT];

void pidOutputInit(void)
{
    // Wyzeruj wszystkie wyjścia
    memset(pidOutputs, 0, sizeof(pidOutputs));
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pidOutputs[i].isActive = false;
        pidOutputs[i].positionControlMode = false;
        pidOutputs[i].positionIsActive = false;
        pidOutputs[i].lastUpdate = HAL_GetTick();
    }
}

// Speed PID functions
void pidOutputSetSpeed(MotorID motorId, float output, float rawOutput)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    
    pidOut->speedOutput = output;
    pidOut->speedRawOutput = rawOutput;
    pidOut->speedIsLimited = (output != rawOutput);
    pidOut->finalOutput = output;  // Speed output becomes final output
    pidOut->lastUpdate = HAL_GetTick();
}

// Position PID functions
void pidOutputSetPosition(MotorID motorId, float output, float error)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    
    pidOut->positionOutput = output;
    pidOut->positionError = error;
    pidOut->speedSetpoint = output;  // Position output becomes speed setpoint
    pidOut->lastUpdate = HAL_GetTick();
}

void pidOutputSetPositionActive(MotorID motorId, bool active)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    pidOut->positionIsActive = active;
    
    // Clear position outputs when deactivating
    if (!active) {
        pidOut->positionOutput = 0.0f;
        pidOut->positionError = 0.0f;
    }
    
    pidOut->lastUpdate = HAL_GetTick();
}

// Feedback and setpoints
void pidOutputSetFeedback(MotorID motorId, float position, float speed)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    pidOut->currentPosition = position;
    pidOut->currentSpeed = speed;
    pidOut->lastUpdate = HAL_GetTick();
}

void pidOutputSetSetpoints(MotorID motorId, float positionSP, float speedSP)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    pidOut->positionSetpoint = positionSP;
    pidOut->speedSetpoint = speedSP;
    pidOut->lastUpdate = HAL_GetTick();
}

// Final output
void pidOutputSetFinal(MotorID motorId, float output)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    pidOut->finalOutput = output;
    pidOut->lastUpdate = HAL_GetTick();
}

// Control mode
void pidOutputSetControlMode(MotorID motorId, bool positionControl)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    pidOut->positionControlMode = positionControl;
    
    // Reset outputs when changing mode
    if (!positionControl) {
        pidOut->positionOutput = 0.0f;
        pidOut->positionError = 0.0f;
        pidOut->positionIsActive = false;
    }
    
    pidOut->lastUpdate = HAL_GetTick();
}

// General control
void pidOutputSetActive(MotorID motorId, bool active)
{
    if (motorId >= MOTOR_COUNT) return;
    
    PIDOutput* pidOut = &pidOutputs[motorId];
    pidOut->isActive = active;
    
    // Clear all outputs when deactivating
    if (!active) {
        pidOut->speedOutput = 0.0f;
        pidOut->speedRawOutput = 0.0f;
        pidOut->speedIsLimited = false;
        pidOut->positionOutput = 0.0f;
        pidOut->positionError = 0.0f;
        pidOut->positionIsActive = false;
        pidOut->finalOutput = 0.0f;
    }
    
    pidOut->lastUpdate = HAL_GetTick();
}

// Getters
float pidOutputGetSpeed(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].speedOutput;
}

float pidOutputGetPosition(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].positionOutput;
}

float pidOutputGetFinal(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].finalOutput;
}

bool pidOutputIsActive(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return pidOutputs[motorId].isActive;
}

bool pidOutputIsSpeedLimited(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return pidOutputs[motorId].speedIsLimited;
}

bool pidOutputIsPositionActive(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return pidOutputs[motorId].positionIsActive;
}

bool pidOutputIsPositionMode(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return pidOutputs[motorId].positionControlMode;
}

// Debug getters
float pidOutputGetCurrentPosition(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].currentPosition;
}

float pidOutputGetCurrentSpeed(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].currentSpeed;
}

float pidOutputGetPositionSetpoint(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].positionSetpoint;
}

float pidOutputGetSpeedSetpoint(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].speedSetpoint;
}

float pidOutputGetPositionError(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return pidOutputs[motorId].positionError;
}