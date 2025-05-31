#include "App/pid_cascade.h"
#include "App/pid_parameters.h"
#include "ModbusRegisters/reg_holding.h"
#include "main.h"
#include <string.h>

// Global cascade controllers
CascadePIDController cascadeControllers[MOTOR_COUNT];

// Position PID parameters (can be moved to pid_parameters.h if needed)
static const MotorPIDParams MOTOR_POSITION_PID_PARAMS[2] = {
    // MOTOR_ROTATION - Position control
    {
        .Kp = 2.0f,     // Higher Kp for position (stiffer response)
        .Ki = 0.1f,     // Lower Ki for position (avoid integral windup)
        .Kd = 0.5f,     // Higher Kd for position (damping)
        .integralMin = -50.0f,
        .integralMax = 50.0f
    },
    // MOTOR_ELEVATION - Position control
    {
        .Kp = 2.0f,     // Higher Kp for position
        .Ki = 0.2f,     // Slightly higher Ki for elevation (gravity compensation)
        .Kd = 0.5f,     // Higher Kd for position (damping)
        .integralMin = -50.0f,
        .integralMax = 50.0f
    }
};

void cascadePidInit(void)
{
    // Initialize all cascade controllers
    for (MotorID motorId = 0; motorId < MOTOR_COUNT; motorId++) {
        cascadePidInitMotor(motorId);
    }
}

void cascadePidInitMotor(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Clear structure
    memset(cascade, 0, sizeof(CascadePIDController));
    cascade->motorId = motorId;
    
    // Initialize position PID (outer loop)
    const MotorPIDParams* posParams = &MOTOR_POSITION_PID_PARAMS[motorId];
    positionPidInit(&cascade->positionPid, 
                   posParams->Kp, 
                   posParams->Ki, 
                   posParams->Kd, 
                   motorId);
    positionPidSetIntegralLimits(&cascade->positionPid, 
                                posParams->integralMin, 
                                posParams->integralMax);
    
    // Initialize speed PID (inner loop)
    const MotorPIDParams* speedParams = &MOTOR_PID_PARAMS[motorId];
    pidInit(&cascade->speedPid, 
            speedParams->Kp, 
            speedParams->Ki, 
            speedParams->Kd);
    pidSetIntegralLimits(&cascade->speedPid, 
                        speedParams->integralMin, 
                        speedParams->integralMax);
    
    // Set default control mode based on flash register
    uint16_t controlMode = regFlash[regFlashIx(REG_FLASH_ROT_CONFIG)];
    cascade->positionControlEnabled = (controlMode & REG_FLASH_ROT_CONFIG_BIT_PID_TYPE) != 0;
    
    cascade->isActive = false;
    
    // Initialize PID output structure
    pidOutputSetControlMode(motorId, cascade->positionControlEnabled);
}

void cascadePidSetControlMode(MotorID motorId, bool positionControl)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // If switching modes, reset both controllers
    if (cascade->positionControlEnabled != positionControl) {
        cascadePidReset(motorId);
    }
    
    cascade->positionControlEnabled = positionControl;
    
    // Set active state for position controller
    positionPidSetActive(&cascade->positionPid, positionControl);
    
    // Update PID output structure
    pidOutputSetControlMode(motorId, positionControl);
    pidOutputSetPositionActive(motorId, positionControl);
}

bool cascadePidGetControlMode(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return cascadeControllers[motorId].positionControlEnabled;
}

void cascadePidSetActive(MotorID motorId, bool active)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    cascade->isActive = active;
    
    // Set PID output active state
    pidOutputSetActive(motorId, active);
    
    // Set position PID active if in position control mode
    if (cascade->positionControlEnabled) {
        positionPidSetActive(&cascade->positionPid, active);
        pidOutputSetPositionActive(motorId, active);
    }
    
    // Reset controllers when activating
    if (active) {
        cascadePidReset(motorId);
    }
}

bool cascadePidIsActive(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return cascadeControllers[motorId].isActive;
}

void cascadePidSetPositionSetpoint(MotorID motorId, float position)
{
    if (motorId >= MOTOR_COUNT) return;
    cascadeControllers[motorId].positionSetpoint = position;
    
    // Update PID output structure
    pidOutputSetSetpoints(motorId, position, cascadeControllers[motorId].speedSetpoint);
}

void cascadePidSetSpeedSetpoint(MotorID motorId, float speed)
{
    if (motorId >= MOTOR_COUNT) return;
    cascadeControllers[motorId].speedSetpoint = speed;
    
    // Update PID output structure
    pidOutputSetSetpoints(motorId, cascadeControllers[motorId].positionSetpoint, speed);
}

void cascadePidUpdateFeedback(MotorID motorId, float position, float speed)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    cascade->currentPosition = position;
    cascade->currentSpeed = speed;
    
    // Update PID output structure with feedback
    pidOutputSetFeedback(motorId, position, speed);
}

float cascadePidUpdate(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    if (!cascade->isActive) {
        return 0.0f;
    }
    
    float finalSpeedSetpoint;
    
    if (cascade->positionControlEnabled) {
        // Position control mode - cascade operation
        // Outer loop: Position PID generates speed setpoint
        finalSpeedSetpoint = positionPidUpdate(&cascade->positionPid,
                                             cascade->positionSetpoint,
                                             cascade->currentPosition,
                                             motorId);
        
        // Update position output in PID output structure
        pidOutputSetPosition(motorId, finalSpeedSetpoint, cascade->positionPid.error);
    } else {
        // Speed-only control mode
        finalSpeedSetpoint = cascade->speedSetpoint;
    }
    
    // Inner loop: Speed PID generates motor output
    float motorOutput = pidUpdate(&cascade->speedPid,
                                 finalSpeedSetpoint,
                                 cascade->currentSpeed,
                                 motorId);
    
    // Set final output in PID output structure
    pidOutputSetFinal(motorId, motorOutput);
    
    return motorOutput;
}

void cascadePidReset(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Reset both controllers
    positionPidReset(&cascade->positionPid);
    pidReset(&cascade->speedPid);
    
    // Clear setpoints
    cascade->positionSetpoint = cascade->currentPosition;
    cascade->speedSetpoint = 0.0f;
    
    // Update PID output structure
    pidOutputSetSetpoints(motorId, cascade->positionSetpoint, cascade->speedSetpoint);
}

void cascadePidLoadParametersFromFlash(void)
{
    // Update control modes from flash
    uint16_t controlMode = regFlash[regFlashIx(REG_FLASH_ROT_CONFIG)];
    bool positionControl = (controlMode & REG_FLASH_ROT_CONFIG_BIT_PID_TYPE) != 0;
    
    for (MotorID motorId = 0; motorId < MOTOR_COUNT; motorId++) {
        cascadePidSetControlMode(motorId, positionControl);
    }
}

void cascadePidUpdateFromRegisters(void)
{
    // Update setpoints from Modbus registers
    
    // Position setpoints (in degrees with base 10)
    float rotPositionSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ROT_TARGET_POSITION)]) / 10.0f;
    float elevPositionSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ELEV_TARGET_POSITION)]) / 10.0f;
    
    cascadePidSetPositionSetpoint(MOTOR_ROTATION, rotPositionSetpoint);
    cascadePidSetPositionSetpoint(MOTOR_ELEVATION, elevPositionSetpoint);
    
    // Speed setpoints (in degrees per second with base 10)
    float rotSpeedSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ROT_TARGET_SPEED)]) / 10.0f;
    float elevSpeedSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ELEV_TARGET_SPEED)]) / 10.0f;
    
    cascadePidSetSpeedSetpoint(MOTOR_ROTATION, rotSpeedSetpoint);
    cascadePidSetSpeedSetpoint(MOTOR_ELEVATION, elevSpeedSetpoint);
    
    // Update control modes if changed
    cascadePidLoadParametersFromFlash();
}